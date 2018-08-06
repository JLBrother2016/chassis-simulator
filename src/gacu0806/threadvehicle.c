#include "threadvehicle.h"
#include "canpacket.h"
#include "coordinate.h"
#include "nmeagps.h"
#include "coordinate.h"

extern car_status_t need_car_posture;

extern can_frame_packet_t send704packet;
extern can_frame_packet_t send302packet;
extern can_frame_packet_t send132packet;
extern can_frame_packet_t send5A2packet; 

extern shared_gps_data_t gps_shared_data;
extern global_locks_t global_locks;
extern char enable_gacu;

extern int epollfd;
extern int packet704fd;
extern int packet302fd;
extern int packet132fd;
extern int packet5A2fd;

static car_status_t car_current_posture;
static double wheelbase = 1.60;

static const double car_acce_set = 3.0;     //加速度为2.8m/s
static double car_deceleration = 0; 	//刹车制动的时候减速度为1.5m/s
static const double steer_velocity = 9;	 //前轮转角每秒可以走9度
static const double reduce_speed_tao = 200;		//车速惯性环节，减速tao值的设定
static const double add_speed_tao = 150;		//车速惯性环节，加速tao值的设定
static const double add_steer_tao = 80;		//方向盘惯性环节, tao值的设定
static const double reduce_steer_tao = 40;		//方向盘惯性环节, tao值的设定
static const double brake_tao = 80;		//刹车盘惯性环节, tao值的设定

static int init_speed_steer_brake(vcu_speed_feedback_t* send2speed,
							  	  eps_steering_feedback_t* send2steer,
							  	  ehb_general_brake_status_t* send2brake)
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

	send2brake->ehb_status_validity = 0;
    send2brake->ehb_driving_mode = 0;
    send2brake->brake_pedal_dispalcement = 0;
    send2brake->actual_driving_brake_pressure_validity = 0;
    send2brake->actual_driving_brake_pressure = 0;
    send2brake->driving_brake_fault_code = 0;
    send2brake->rolling_counter = 0;
    send2brake->data_checksums = 0;

	return 0;
}

static int init_car_current_status(void)
{
	car_current_posture.car_velocity = 0;
    car_current_posture.car_acceleration = 0;
    car_current_posture.car_steer_angle = GACU_ANGLE_BASE;
    car_current_posture.car_steer_speed = 0;
    car_current_posture.car_steer_torque = 0;
    car_current_posture.car_drive_mode = 0;
    car_current_posture.car_steer_mode = 0;
    car_current_posture.car_drive_shift = 0;
    car_current_posture.car_ehb_status = 0;
    car_current_posture.car_ehb_driving_mode = 0;
    car_current_posture.car_brake_pedal_dispalcement = 0;
    car_current_posture.car_actual_driving_brake_pressure_validity = 0;
    car_current_posture.car_actual_driving_brake_pressure = 0;
    car_current_posture.car_driving_brake_fault_code = 0;

	return 0;
}

static int adjust_vehicle_speed(double dt_s)
{
    double old_car_velocity = car_current_posture.car_velocity;
    double speed_tao = 0.0;
    if(old_car_velocity < car_current_posture.car_velocity)
    	speed_tao = add_speed_tao;
    else speed_tao = reduce_speed_tao;
    //利用惯性环节来模拟车的底盘
    car_current_posture.car_drive_mode = need_car_posture.car_drive_mode;
    car_current_posture.car_drive_shift = need_car_posture.car_drive_shift;
    car_current_posture.car_velocity = (speed_tao 
    								 * car_current_posture.car_velocity 
    								 + need_car_posture.car_velocity) 
    						         / (1 + speed_tao);
    if(abs(old_car_velocity - car_current_posture.car_velocity) > 
    	car_acce_set * dt_s) {
    	if(old_car_velocity - car_current_posture.car_velocity > 0)
    		car_current_posture.car_velocity = old_car_velocity - car_acce_set * dt_s;
    	else car_current_posture.car_velocity = old_car_velocity + car_acce_set * dt_s;
    }
    double brake_velocity = car_deceleration * dt_s;
    car_current_posture.car_velocity += brake_velocity;
   	car_current_posture.car_vehicle_brake = (brake_tao 
    								 * car_current_posture.car_vehicle_brake 
    								 + need_car_posture.car_vehicle_brake) 
    						         / (1 + brake_tao);
    car_current_posture.car_acceleration = (car_current_posture.car_velocity
    									 - old_car_velocity) / dt_s;
    if(car_current_posture.car_velocity < 0) {
    	car_current_posture.car_velocity = 0;
    	car_current_posture.car_acceleration = 0;
    // 	car_current_posture.car_acceleration = 0;
    }
    
	return 0;
}

static int adjust_vehicle_steer(double dt_s)
{
	double steer_tao = 0.0;
	if(car_current_posture.car_steer_angle < need_car_posture.car_steer_angle)
		steer_tao = add_steer_tao;
	else steer_tao = reduce_steer_tao;
    car_current_posture.car_steer_speed = need_car_posture.car_steer_speed;
    car_current_posture.car_steer_torque = 0;
    car_current_posture.car_steer_mode = need_car_posture.car_steer_mode;
    car_current_posture.car_steer_angle = (steer_tao 
    									* car_current_posture.car_steer_angle 
    									+ need_car_posture.car_steer_angle) 
    									/ (1 + steer_tao);
	return 0;
}

static int adjust_vehicle_brake(double dt_s)
{
    car_current_posture.car_ehb_status = need_car_posture.car_ehb_status;
    car_current_posture.car_ehb_driving_mode = 
    									need_car_posture.car_ehb_driving_mode;
    car_current_posture.car_brake_pedal_dispalcement = 0;
    car_current_posture.car_actual_driving_brake_pressure_validity =  
    			need_car_posture.car_actual_driving_brake_pressure_validity;
    car_current_posture.car_driving_brake_fault_code = 
    							need_car_posture.car_driving_brake_fault_code;

    car_current_posture.car_actual_driving_brake_pressure = 
     					(brake_tao 
    					* car_current_posture.car_actual_driving_brake_pressure
    					+ need_car_posture.car_actual_driving_brake_pressure) 
    					/ (1 + brake_tao);
    car_deceleration = 
    		car_current_posture.car_actual_driving_brake_pressure * (-0.45);
    // car_current_posture.car_actual_driving_brake_pressure = 3;
    // debug("brake_pressure: %f %f\n", car_current_posture.car_actual_driving_brake_pressure, need_car_posture.car_actual_driving_brake_pressure);
    // car_current_posture.car_acceleration = 
    // 	car_current_posture.car_actual_driving_brake_pressure * (-0.45);				

	return 0;
}

static int get_vehicle_gacu(acucan_feedback_t* turn2acu)
{
	if(turn2acu == NULL) {
		debug("get_vehicle_gacu error!\n");
		return-1;
	}
	turn2acu->steer_enabled = car_current_posture.car_steer_mode;
	turn2acu->drive_enabled = car_current_posture.car_drive_mode;
	turn2acu->brake_enabled = car_current_posture.car_ehb_driving_mode;
	turn2acu->shift = car_current_posture.car_drive_shift;
	unsigned int tmp_angle = GACU_ANGLE_BASE; //方向是逆时针
	if(turn2acu->drive_enabled == ACU_SHIFT_D) {
		tmp_angle = (unsigned int)(car_current_posture.car_steer_angle
	 					   * 100 + GACU_ANGLE_BASE);
	}
	// debug("tmp_angle = %d\n", tmp_angle);
	turn2acu->wheel_angle_low = tmp_angle & TAKELOW8BIT;
	turn2acu->wheel_angle_high = (tmp_angle >> 8) & TAKELOW5BIT;
	turn2acu->steer_fault = 0;
	turn2acu->torque = GACU_TORQUE_BASE;
	unsigned int tmp_speed = car_current_posture.car_velocity * 10;

	// turn2acu->steer_enabled = need_car_posture.car_steer_mode;
	// turn2acu->drive_enabled = need_car_posture.car_drive_mode;
	// turn2acu->brake_enabled = need_car_posture.car_ehb_driving_mode;
	// // turn2acu->shift = 1;
	// turn2acu->shift = need_car_posture.car_drive_shift;
	// unsigned int tmp_angle = (unsigned int)(need_car_posture.car_steer_angle
	//  					   * 100 + GACU_ANGLE_BASE);
	// // debug("tmp_angle = %d\n", tmp_angle);
	// turn2acu->wheel_angle_low = tmp_angle & TAKELOW8BIT;
	// turn2acu->wheel_angle_high = (tmp_angle >> 8) & TAKELOW5BIT;
	// turn2acu->steer_fault = 0;
	// //力矩有一个偏移值127, 因为是一个正负值；127这个值表示是0力矩
	// turn2acu->torque = 127;		//127, 
	// unsigned int tmp_speed = need_car_posture.car_velocity * 10;
	// debug("turn2acu->shift: %d\n", turn2acu->shift);

	turn2acu->speed_low = tmp_speed & TAKELOW8BIT;
	turn2acu->speed_high = (tmp_speed >> 8) & TAKE1THBIT;
	turn2acu->engine_direction = 0;
	turn2acu->drive_fault = 0;
	turn2acu->brake_fault = 0;
	turn2acu->brake_human_intervention = 0;
	turn2acu->rolling_counter = 0;
	turn2acu->acu_state = 0;

	return 0;
}

static int adjust_vehicle_posture(double *last_time) 
{
	double timestep_ms = get_time_stamp() - *last_time;
	*last_time = get_time_stamp();
	double timestep_s = timestep_ms / 1000.0;

	if(enable_gacu == 0) {
		vcu_speed_feedback_t send2speed;
		eps_steering_feedback_t send2steer;
		ehb_general_brake_status_t send2brake;
		init_speed_steer_brake(&send2speed, &send2steer, &send2brake);
		// 这里应该加need_car_posture对应的锁
		mutex_lock(&global_locks.can_mutex);
		//现在是根据惯性环节调整速度、前轮偏角和刹车液压大小
		adjust_vehicle_speed(timestep_s);		
		adjust_vehicle_steer(timestep_s);
		adjust_vehicle_brake(timestep_s);
		car2can(&send2steer, &send2speed, &send2brake, &car_current_posture);
		mutex_unlock(&global_locks.can_mutex);

		encoding_vcu_speed(&send2speed, &send302packet);
		encoding_eps_steering(&send2steer, &send704packet);
		encoding_ehb_brake(&send2brake, &send132packet);
		// check_sums(send132packet.data, 7);
	} else if(enable_gacu == 1) {
		acucan_feedback_t send2gacu;
		memset(&send2gacu, 0, sizeof(send2gacu));
		mutex_lock(&global_locks.can_mutex);
		//现在是根据惯性环节调整速度、前轮偏角和刹车液压大小
		adjust_vehicle_speed(timestep_s);		
		adjust_vehicle_steer(timestep_s);
		adjust_vehicle_brake(timestep_s);
		// 通过车的状态编码成gacu的反馈包
		get_vehicle_gacu(&send2gacu);
		mutex_unlock(&global_locks.can_mutex);	
		encoding_gacu_feedback(&send2gacu, &send5A2packet);	
		
	}  else {
        debug("gacu command error!\n");
        return -1;
    }

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
	// if(enable_gacu == 1) {
	// 	theta_ *= (-1.0);
	// }
	end_point->x = start_point->x   //计算在x轴的增量
				 - (dx_ * cos(theta_) + dy_ * sin(theta_));
	end_point->y = start_point->y  	//计算在y轴上的增量
				 + (dx_ * sin(theta_) + dy_ * cos(theta_)); 
	if(enable_gacu == 0) {
		theta_ += d_theta_;		//方向是正的, 顺时针方向
	} else if(enable_gacu == 1) {	
		theta_ -= d_theta_;		//方向是反的, 逆时针方向
	}
	

	
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
	init_car_current_status();

	timer_param_t timer302_pm;
	timer_param_t timer704_pm;
	timer_param_t timer132_pm;
	timer_param_t timer5A2_pm;

	debug("thread_vehicle enable_gacu = %d\n", enable_gacu);
	if(enable_gacu == 0) {
		init_packet(&send704packet, STEER_FEEDBACK);
	    init_packet(&send302packet, SPEED_FEEDBACK);
	    init_packet(&send132packet, BRAKE_FEEDBACK);
		time_t timer302_interval = 20;	//302要延时50ms,but set it is 20
	    struct itimerspec its302;
	    packet302fd = create_timerfd(&its302, timer302_interval);
		timer302_pm.its = its302;
		timer302_pm.tfd = packet302fd;
		debug("packet302fd: %d\n", packet302fd);
		if(timerfd_settime(timer302_pm.tfd, TFD_TIMER_ABSTIME, 
						   &(timer302_pm.its), NULL) != 0) {
			ERR_EXIT("302 timerfd_settime()");
		}	//将定时器的文件描述符加入到epoll事件集合中
		add_timerfd2epoll(epollfd, &timer302_pm);

		// 初始化定时器发704的包
	    time_t timer704_interval = 10;	//704要延时10ms
	    struct itimerspec its704;
	    packet704fd = create_timerfd(&its704, timer704_interval);
		timer704_pm.its = its704;
		timer704_pm.tfd = packet704fd;
		debug("packet704fd: %d\n", packet704fd);
		if(timerfd_settime(timer704_pm.tfd, TFD_TIMER_ABSTIME, 
						   &(timer704_pm.its), NULL) != 0) {
			ERR_EXIT("704 timerfd_settime()");
		}	//将定时器的文件描述符加入到epoll事件集合中
		add_timerfd2epoll(epollfd, &timer704_pm);

		// 初始化定时器发132的包
	    time_t timer132_interval = 20;	//132要延时20ms
	    struct itimerspec its132;
	    packet132fd = create_timerfd(&its132, timer132_interval);
		timer132_pm.its = its132;
		timer132_pm.tfd = packet132fd;
		debug("packet132fd: %d\n", packet132fd);
		if(timerfd_settime(timer132_pm.tfd, TFD_TIMER_ABSTIME, 
						   &(timer132_pm.its), NULL) != 0) {
			ERR_EXIT("132 timerfd_settime()");
		}	//将定时器的文件描述符加入到epoll事件集合中
		add_timerfd2epoll(epollfd, &timer132_pm);
	} else if(enable_gacu == 1) {
		init_packet(&send5A2packet, ACU_ID_FDBK);
		time_t timer5A2_interval = 50;	//5A2要延时50ms
		struct itimerspec its5A2;
	    packet5A2fd = create_timerfd(&its5A2, timer5A2_interval);
		timer5A2_pm.its = its5A2;
		timer5A2_pm.tfd = packet5A2fd;
		debug("packet5A2fd: %d\n", packet5A2fd);
		if(timerfd_settime(timer5A2_pm.tfd, TFD_TIMER_ABSTIME, 
						   &(timer5A2_pm.its), NULL) != 0) {
			ERR_EXIT("5A2 timerfd_settime()");
		}	//将定时器的文件描述符加入到epoll事件集合中
		add_timerfd2epoll(epollfd, &timer5A2_pm);
	} else {
        debug("gacu command error!\n");
        return NULL;
    }
		

	double time_val_car = get_time_stamp();
	double time_val_gps = get_time_stamp();

	while(1) {
		adjust_vehicle_posture(&time_val_car);
		// printf_car_status(&car_current_posture);
		// debug("\n");
		ms_sleep(0, 3);
		// 这里应该加一把计算gps坐标的锁
		mutex_lock(&global_locks.gps_mutex);
		caculate_gps(&time_val_gps);
		mutex_unlock(&global_locks.gps_mutex);
	}

	return NULL;
}


