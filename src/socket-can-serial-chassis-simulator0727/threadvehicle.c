#include "threadvehicle.h"
#include "canpacket.h"
#include "coordinate.h"
#include "nmeagps.h"
#include "coordinate.h"

extern car_status_t need_car_posture;
extern can_frame_packet_t send704packet;
extern can_frame_packet_t send302packet;
extern shared_gps_data_t gps_shared_data;
extern char gps_cmd_buffer[256];
extern int epollfd;
extern int packet704fd;
extern int packet302fd;

static car_status_t car_current_posture;
static double wheelbase = 1.60;
static int speed_count = 0;

static const double car_acce_set = 2.8;     //加速度为2.8m/s
static const double car_deceleration = -1.5; //减速度为1.5m/s
static const double steer_velocity = 9;	 //前轮转角每秒可以走9度
static const double speed_tao = 32;		//车速惯性环节，tao值的设定
static const double steer_tao = 8;		//方向盘惯性环节, tao值的设定

static int init_speedandsteer(vcu_speed_feedback_t* send2speed,
							  eps_steering_feedback_t* send2steer)
{
	send2speed->vehicle_drive_mode = 0;
	send2speed->vehicle_shift = 0;
	send2speed->vehicle_brake_req = 0;
	send2speed->vehicle_velocity = 0;
	send2speed->vehicle_acceleration = 0;
	send2speed->vehicle_brake = 0; 
	send2speed->drive_motor_info = 0;
	send2speed->rolling_counter = 0;
	send2speed->drive_motor_fault = 0;
	send2speed->data_checksums = 0;

	send2steer->steering_mode = 0;
	send2steer->steering_angle = 7400;
	send2steer->steering_torque = 0;
	send2steer->steering_fault = 0;
	send2steer->data_checksums = 0;

	return 0;
}

static int init_car_current_status(void)
{
	car_current_posture.car_velocity = 0;
    car_current_posture.car_acceleration = 0;
    car_current_posture.car_steer_angle = 0;
    car_current_posture.car_steer_speed = 0;
    car_current_posture.car_steer_torque = 0;
    car_current_posture.car_drive_mode = 0;
    car_current_posture.car_steer_mode = 0;
    car_current_posture.car_drive_shift = 0;

	return 0;
}

static int adjust_vehicle_speed(double dt_s)
{
	// double dv_ms = 0.0;
	// if(car_current_posture.car_velocity < need_car_posture.car_velocity) {
	// 	dv_ms = car_acce_set * dt_s;
	// 	car_current_posture.car_acceleration = car_acce_set;
	// }
	// else if(car_current_posture.car_velocity > need_car_posture.car_velocity) {
	// 	dv_ms = car_deceleration * dt_s;
	// 	car_current_posture.car_acceleration = -car_deceleration;
	// } 
	 // debug("dv_ms:%f ", dv_ms);
    // car_current_posture.car_velocity += dv_ms;
    // if(car_current_posture.car_velocity < 0) {
    // 	car_current_posture.car_velocity = 0;
    // 	car_current_posture.car_acceleration = 0;
    // }
    // else if(car_current_posture.car_velocity > need_car_posture.car_velocity) {
    // 	car_current_posture.car_velocity = need_car_posture.car_velocity;
    	// car_current_posture.car_acceleration = 0;
    // }
    static double old_car_velocity = 0;
    //利用惯性环节来模拟车的底盘
    car_current_posture.car_drive_mode = need_car_posture.car_drive_mode;
    car_current_posture.car_drive_shift = need_car_posture.car_drive_shift;
    car_current_posture.car_velocity = (speed_tao 
    								 * car_current_posture.car_velocity 
    								 + need_car_posture.car_velocity) 
    						         / (1 + speed_tao);
    car_current_posture.car_acceleration = (car_current_posture.car_velocity
    									 - old_car_velocity) / dt_s;
    if(abs(car_current_posture.car_acceleration) > 11) {
    	car_current_posture.car_acceleration = (car_current_posture.car_acceleration / abs(car_current_posture.car_acceleration)) 
    	    * 10;
    }				         

	return 0;
}

static int adjust_vehicle_steer(double dt_s)
{
	double d_angle = 0.0;
	// if(car_current_posture.car_steer_angle < need_car_posture.car_steer_angle)
	// 	d_angle = steer_velocity * dt_s;
	// else if(car_current_posture.car_steer_angle > need_car_posture.car_steer_angle)
	// 	d_angle = (-steer_velocity) * dt_s;
	// else d_angle = 0.0;
    // car_current_posture.car_steer_angle += d_angle; 
    // if(car_current_posture.car_steer_angle < -27.0)
    // 	car_current_posture.car_steer_angle = -27.0;
    // else if(car_current_posture.car_steer_angle > 27.0)
    // 	car_current_posture.car_steer_angle = 27.0;
    car_current_posture.car_steer_speed = need_car_posture.car_steer_speed;
    car_current_posture.car_steer_torque = 0;
    car_current_posture.car_steer_mode = need_car_posture.car_steer_mode;
    car_current_posture.car_steer_angle = (steer_tao 
    									* car_current_posture.car_steer_angle 
    									+ need_car_posture.car_steer_angle) 
    									/ (1 + steer_tao);
	return 0;
}

static int adjust_vehicle_posture(double *last_time) 
{
	double timestep_ms = get_time_stamp() - *last_time;
	*last_time = get_time_stamp();
	double timestep_s = timestep_ms / 1000.0;

	vcu_speed_feedback_t send2speed;
	eps_steering_feedback_t send2steer;
	init_speedandsteer(&send2speed, &send2steer);
	adjust_vehicle_speed(timestep_s);		
	//现在是直接复值，以后根据加速度计算相应的速度大小
	adjust_vehicle_steer(timestep_s);
	car2can(&send2steer, &send2speed, &car_current_posture);
	encoding_vcu_speed(&send2speed, &send302packet);
	encoding_eps_steering(&send2steer, &send704packet);
	check_sums(send704packet.data, 7);		//第7为是校验和

	return 0;
}

static int wheel_odometry(UTMcoordinate_t* start_point,
				   UTMcoordinate_t* end_point, double dt_s)
{
	double current_steering_angle = car_current_posture.car_steer_angle;
	double current_speed = car_current_posture.car_velocity;
	double alpha_ = degree2radian(current_steering_angle);
	double v_ = current_speed;
	double R_ = 0;
	double phi_ = 0;
	if(alpha_ != 0) {
		// R_ = abs(wheelbase / sin(alpha_));
		R_ = abs(wheelbase / sin(alpha_));
		phi_ = v_ * dt_s / R_; 
	}
	
	double dx_ = R_ * sin(phi_);
	double dy_ = R_ - R_ * cos(phi_);
	double d_theta_ = v_ / wheelbase * sin(alpha_) * dt_s;

	double real_north_heading = gps_shared_data.heading_in_degrees;// - 90
	real_north_heading = fmod(real_north_heading + 360, 360); //航向角范围为0-360
	//车的航向指向二四象限
	// if((real_north_heading > 90 && real_north_heading < 180) || 
	//    (real_north_heading > 270 && real_north_heading < 360)) 
	if((real_north_heading > 0 && real_north_heading <= 90) || 
	   (real_north_heading >= 180 && real_north_heading < 270)) 
		dy_ = -dy_;
	double theta_ = degree2radian(real_north_heading);		  //转换为幅度角
	// debug("theta_: %f real_north_heading: %f\n", theta_, real_north_heading);

	end_point->x = start_point->x - (dx_ * cos(theta_) + dy_ * sin(theta_));
	end_point->y = start_point->y + dx_ * sin(theta_) + dy_ * cos(theta_); 

	theta_ += d_theta_;		//方向是反的
	gps_shared_data.heading_in_degrees = radian2degree(theta_);
	gps_shared_data.heading_in_degrees += 0;	 //90		//转换为西为正
	gps_shared_data.heading_in_degrees = fmod(gps_shared_data.heading_in_degrees + 360, 360); //航向角范围为0-360

	return 0;
}


static int caculate_gps(double *last_time)
{
	WGS84coordinate_t wgs_point;
	UTMcoordinate_t utm_point, utm_end_point;

	double timestep_ms = get_time_stamp() - *last_time;
	*last_time = get_time_stamp();
	double timestep_s = timestep_ms / 1000.0;

	wgs_point.latitude = gps_shared_data.latitude;
	wgs_point.longitude = gps_shared_data.longitude;

	WGS84_to_UTM(&wgs_point, &utm_point);

	wheel_odometry(&utm_point, &utm_end_point, timestep_s);

	UTM_to_WGS84(&utm_end_point, &wgs_point, &wgs_point);

	gps_shared_data.latitude = wgs_point.latitude;
	gps_shared_data.longitude = wgs_point.longitude;

	return 0;
}

void* thread_vehicle(void *arg)
{
	int conn_gps = *((int*) arg);
	init_packet(&send704packet, STEER_FEEDBACK);
    init_packet(&send302packet, SPEED_FEEDBACK);
	init_car_current_status();

	time_t timer302_interval = 50;	//302要延时50ms
    struct itimerspec its302;
    packet302fd = create_timerfd(&its302, timer302_interval);
    timer_param_t timer302_pm;
	timer302_pm.its = its302;
	timer302_pm.tfd = packet302fd;
	debug("packet302fd: %d\n", packet302fd);
	if(timerfd_settime(timer302_pm.tfd, TFD_TIMER_ABSTIME, 
					   &(timer302_pm.its), NULL) != 0) {
		ERR_EXIT("timerfd_settime()");
	}	//将定时器的文件描述符加入到epoll事件集合中
	add_timerfd2epoll(epollfd, &timer302_pm);

	// 初始化定时器发704的包
    time_t timer704_interval = 10;	//704要延时10ms
    struct itimerspec its704;
    packet704fd = create_timerfd(&its704, timer704_interval);
    timer_param_t timer704_pm;
	timer704_pm.its = its704;
	timer704_pm.tfd = packet704fd;
	debug("packet704fd: %d\n", packet704fd);
	if(timerfd_settime(timer704_pm.tfd, TFD_TIMER_ABSTIME, 
					   &(timer704_pm.its), NULL) != 0) {
		ERR_EXIT("timerfd_settime()");
	}	//将定时器的文件描述符加入到epoll事件集合中
	add_timerfd2epoll(epollfd, &timer704_pm);

	double time_val_car = get_time_stamp();
	double time_val_gps = get_time_stamp();

	while(1) {
		adjust_vehicle_posture(&time_val_car);
		// printf_car_status(&car_current_posture);
		// debug("\n");
		ms_sleep(0, 5);
		caculate_gps(&time_val_gps);
	}

	return NULL;
}


