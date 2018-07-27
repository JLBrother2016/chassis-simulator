#include "threadchassiscan.h"
#include "pthreadutil.h"
#include "canpacket.h"

car_status_t need_car_posture;
static can_shared_use_t shared_information;
//shared_information只是解包，还没有做数据转换
static int init_need_car_status(void)
{
	need_car_posture.car_velocity = 0;
    need_car_posture.car_acceleration = 0;
    need_car_posture.car_steer_angle = 0;
    need_car_posture.car_steer_speed = 0;
    need_car_posture.car_steer_torque = 0;
    need_car_posture.car_drive_mode = 0;
    need_car_posture.car_steer_mode = 0;
    need_car_posture.car_drive_shift = 0;

	return 0;
}

static int acquire_vehicle_speed(vcu_speed_control_t* recv4speed)
{
	shared_information.vehicle_drive_mode = recv4speed->vehicle_drive_mode;
	shared_information.vehicle_shift = recv4speed->vehicle_shift;   //档位
														//刹车请求
   	shared_information.vehicle_brake_req = recv4speed->vehicle_brake_req; 
														//期望速度
	shared_information.vehicle_velocity = recv4speed->vehicle_velocity;		
														//期望加速度
	shared_information.vehicle_acceleration = recv4speed->vehicle_acceleration;
	shared_information.vehicle_brake = recv4speed->vehicle_brake; //刹车量

	return 0;
}

static int acquire_vehicle_steer(eps_steering_control_t* recv4steer)
{
	shared_information.steering_mode = recv4steer->steering_mode;//驾驶模式
	shared_information.steering_angle = recv4steer->steering_angle; //前轮转角
										//前轮转角变换速度
	shared_information.steering_speed = recv4steer->steering_speed;

	return 0;
}

static int acquire_vehicle_posture(can_frame_packet_t* recvframe)
{
	vcu_speed_control_t recv4speed;
    eps_steering_control_t recv4steer;
    if(recvframe->can_id == SPEED_CONTROL) { //收到的是速度控制的报文，就调整速度
    	decoding_vcu_speed(recvframe, &recv4speed);
		acquire_vehicle_speed(&recv4speed);
		// debug("receive 301");
		// printf_packet(recvframe);
		// debug("\n"); 
		// ms_sleep(0, 2); 
	} else if(recvframe->can_id == STEER_CONTROL) {	//是方向盘的报文就调整方向盘
		decoding_eps_steering(recvframe, &recv4steer);
		acquire_vehicle_steer(&recv4steer);
		// debug("receive 703");
		// printf_packet(recvframe);
		// debug("\n"); 
		// ms_sleep(0, 2); 
	} else if(recvframe->can_id == BRAKE_CONTROL) {
		// ++recvframe->can_id;
		// check_sums(recvframe, 7);
		// debug("this is brake control: ");
		// printf_packet(recvframe);
		// debug("\n");
		// ms_sleep(0, 2); 
		return 0;
	} else {
		debug("this is unknown packet: ");
		printf_packet(recvframe);
		debug("\n");
		return -1;
	}  //把原始解包的数据转换为需要车运动的数据
	can2car(&need_car_posture, &shared_information); 

	return 0;
}

void* thread_chassis_can(void *arg)
{
    can_frame_packet_t recvframe;
    int conn_can = *((int*) arg);
    init_need_car_status();
    double start = get_time_stamp();
    // int file_fd = open("read.txt", O_APPEND | O_CREAT | O_RDWR);
    // char write_buff[64];
    // int len = 0;
    // int res = 0;
    while(1) {
    	fflush(stdout);
    	// debug("read_can_packet!\n");
    	int ret = read_packet(conn_can, &recvframe);
    	double end = get_time_stamp();
		if(ret == 0) {
			close(conn_can);
			thread_exit();
			break;
		}
		// usleep(500*1000);
		acquire_vehicle_posture(&recvframe);
		debug("receive ");
		printf_packet(&recvframe);
		// res = sprintf(write_buff, "ID = 0X%X ", recvframe.can_id); 
		// len += res;
	 //    res = sprintf(write_buff + len, "data: ");
	 //    len += res;
	 //    for(int i = 0; i < 8; ++i) {
	 //        res = sprintf(write_buff + len, "0X%02x ", recvframe.data[i]);
	 //        len += res;
	 //    }
	 //    res = sprintf(write_buff + len, "\n");
	 //    len += res;
		// write(file_fd, write_buff, len);
		// len = 0;
		// res = 0;
		// memset(write_buff, 0, sizeof(write_buff));
		// debug("\n");     
		debug("can time: %f ", end - start);
		printf("\n");
    	start = end;
		if(recvframe.can_id == BRAKE_CONTROL) {
			++recvframe.can_id;
			check_sums(recvframe.data, 7);
			// write_serial_packet(conn_can, &recvframe);
		} 
		// ms_sleep(0, 15); 	//delay 20 ms		
    }
    //close(file_fd);

    return NULL;
}


