#include "threadchassiscan.h"
#include "pthreadutil.h"
#include "canpacket.h"

extern global_locks_t global_locks;
extern char enable_gacu;

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
    need_car_posture.car_ehb_status = 0;
    need_car_posture.car_ehb_driving_mode = 0;
    need_car_posture.car_brake_pedal_dispalcement = 0;
    need_car_posture.car_actual_driving_brake_pressure_validity = 0;
    need_car_posture.car_actual_driving_brake_pressure = 0;
    need_car_posture.car_driving_brake_fault_code = 0;

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

static int acquire_vehicle_brake(idu_general_brake_status_t* recv4brake)
{
	// 0, 6, 10, 16, 22, 30, 40, 56        //*1 bar                  
	// 0, 3,  5,  7, 10, 13, 17, 20    //dec   *10 m/s^2 acc = brake * (-0.45) 
	if(recv4brake->brake_pressure_target) {
		// while(1) {;}
		debug("\n\nrecv4brake->brake_pressure_target:%d\n\n", 
			   recv4brake->brake_pressure_target);
	}
	shared_information.vehicle_drive_mode_request_validity = 
							recv4brake->vehicle_drive_mode_request_validity;
    shared_information.vehicle_drive_mode_request = 
    						recv4brake->vehicle_drive_mode_request;
    shared_information.brake_pressure_target_validity =
    						recv4brake->brake_pressure_target_validity;
    shared_information.brake_pressure_target = 
    						recv4brake->brake_pressure_target;
    shared_information.idu_fault_code = recv4brake->idu_fault_code;
    shared_information.vehicle_brake_request = 
    						recv4brake->vehicle_brake_request;

	return 0;
}

static int acquire_vehicle_gacu(const acucan_ctrl_t* gacu4ctrl)
{
    // printf("steer_enable = %hhu, drive_enable = %hhu, brake_enable = %hhu, shift = %hhu, estop = %hhu, clear_error = %hhu, wheel_angle_low = %hhu, wheel_angle_high = %hhu, angle_rate = %hhu, speed_low = %hhu, speed_high = %hhu, acc = %hhu, rolling_counter = %hhu, uos_state = %hhu, checksum = %hhu\n",
    //                                         gacu4ctrl->steer_enable,
    //                                         gacu4ctrl->drive_enable,
    //                                         gacu4ctrl->brake_enable,
    //                                         gacu4ctrl->shift,
    //                                         gacu4ctrl->estop,
    //                                         gacu4ctrl->clear_error,
    //                                         gacu4ctrl->wheel_angle_low,
    //                                         gacu4ctrl->wheel_angle_high,
    //                                         gacu4ctrl->angle_rate,
    //                                         gacu4ctrl->speed_low,
    //                                         gacu4ctrl->speed_high,
    //                                         gacu4ctrl->acc,
    //                                         gacu4ctrl->rolling_counter,
    //                                         gacu4ctrl->uos_state,
    //                                         gacu4ctrl->checksum);   //it's ok
    // 针对gacu这个报文直接把它转换为车的位姿
    need_car_posture.car_drive_mode = gacu4ctrl->drive_enable;
    need_car_posture.car_steer_mode = gacu4ctrl->steer_enable;
	need_car_posture.car_ehb_driving_mode = gacu4ctrl->brake_enable;
	need_car_posture.car_drive_shift = gacu4ctrl->shift;
	need_car_posture.car_driving_brake_fault_code = gacu4ctrl->estop; //即停命令	
	need_car_posture.car_ehb_status = gacu4ctrl->clear_error;		  //清楚错误？
	need_car_posture.car_steer_angle = (((gacu4ctrl->wheel_angle_low | 
									   (gacu4ctrl->wheel_angle_high << 8))
                                     - GACU_ANGLE_BASE)) * 0.01;
	need_car_posture.car_steer_speed = gacu4ctrl->angle_rate * 0.1;
	need_car_posture.car_velocity =  (gacu4ctrl->speed_low |
									 (gacu4ctrl->speed_high << 8)) * 0.1;
	need_car_posture.car_acceleration = gacu4ctrl->acc * 0.1; //刹车对应的加速度
	need_car_posture.car_actual_driving_brake_pressure_validity = gacu4ctrl->uos_state; //uos状态
    
	printf_car_status(&need_car_posture);
    debug("\n");
	// int tmp = gacu4ctrl->steer_enable;
    // printf("steer_enable: %d\n", tmp);
    // tmp = gacu4ctrl->drive_enable;
    // printf("drive_enable: %d\n", tmp);
    // tmp = gacu4ctrl->brake_enable;
    // printf("brake_enable: %d\n", tmp);
    // int tmp = gacu4ctrl->shift;
    // printf("read shift: %d ", tmp);    
    // tmp = gacu4ctrl->estop;
    // printf("estop: %d\n", tmp);
    // tmp = gacu4ctrl->clear_error;
    // printf("clear_error: %d\n", tmp);
    // tmp = gacu4ctrl->wheel_angle_low;
    // printf("wheel_angle_low: %d\n", tmp);
    // tmp = gacu4ctrl->wheel_angle_high;
    // printf("wheel_angle_high: %d\n", tmp);
    // tmp = gacu4ctrl->angle_rate;
    // printf("angle_rate: %d\n", tmp);
    // tmp = gacu4ctrl->speed_low;
    // printf("speed_low: %d\n", tmp);
    // tmp = gacu4ctrl->speed_high;
    // printf("speed_high: %d\n", tmp);
    // tmp = gacu4ctrl->acc;
    // printf("acc: %d\n", tmp);
    // tmp = gacu4ctrl->rolling_counter;
    // printf("rolling_counter: %d\n", tmp);
    // tmp = gacu4ctrl->uos_state;
    // printf("uos_state: %d\n", tmp);
    // tmp = gacu4ctrl->checksum;
    // printf("checksum: %d\n", tmp);

    return 0;
}

static int acquire_vehicle_posture(can_frame_packet_t* recvframe)
{
    int check_sum = 0;

    if(enable_gacu == 0) {
        if(recvframe->can_id == SPEED_CONTROL) { //收到的是速度控制的报文，就调整速度
            check_sum = get_check_sums(recvframe->data, 6);
            if(check_sum != recvframe->data[6]) {
                debug("301 packet error\n");
                printf_packet(recvframe);
                debug("\n");
                while(1) {
                    ;
                }
            }
            vcu_speed_control_t recv4speed;

            decoding_vcu_speed(recvframe, &recv4speed);
            acquire_vehicle_speed(&recv4speed);
        } else if(recvframe->can_id == STEER_CONTROL) { //是方向盘的报文就调整方向盘
            check_sum = get_check_sums(recvframe->data, 7);
            if(check_sum != recvframe->data[7]) {
                debug("703 packet error\n");
                printf_packet(recvframe);
                debug("\n");
                while(1) {
                    ;
                }
            }
            eps_steering_control_t recv4steer;

            decoding_eps_steering(recvframe, &recv4steer);
            acquire_vehicle_steer(&recv4steer);
        } else if(recvframe->can_id == BRAKE_CONTROL) {
            check_sum = get_check_sums(recvframe->data, 7);
            if(check_sum != recvframe->data[7]) {
                debug("131 packet error\n");
                printf_packet(recvframe);
                debug("\n");
                while(1) {
                    ;
                }
            }
            idu_general_brake_status_t recv4brake;

            decoding_idu_brake(recvframe, &recv4brake);
            acquire_vehicle_brake(&recv4brake);
        } else {
            debug("this is unknown packet: ");
            printf_packet(recvframe);
            debug("\n");
            return 0;
        }

        //把原始解包的数据转换为需要车运动的数据, 这里应该加一把锁
        mutex_lock(&global_locks.can_mutex);
        can2car(&need_car_posture, &shared_information); 
        mutex_unlock(&global_locks.can_mutex);
    } else if(enable_gacu == 1) {
        if(recvframe->can_id == ACU_ID_CTRL){
            check_sum = get_check_sums(recvframe->data, 7);
            if(check_sum != recvframe->data[7]) {
                debug("ACU_ID_CTRL packet error\n");
                printf_packet(recvframe);
                debug("\n");
                while(1) {
                    ;
                }
            }
            debug("ACU_ID_CTRL packet: ");
            printf_packet(recvframe);
            static double start = 0;
            double end = get_time_stamp();
            debug("time: %f\n", end - start);
            start = end;
            debug("\n");

            // debug("decoding_gacu_ctrl!\n");
            acucan_ctrl_t recv4gacu;
            decoding_gacu_ctrl(recvframe, &recv4gacu);
            mutex_lock(&global_locks.can_mutex);
            acquire_vehicle_gacu(&recv4gacu);
            mutex_unlock(&global_locks.can_mutex);
        } else if(recvframe->can_id == ACU_ID_CTRL_EXT){
            // debug("this is ACU_ID_CTRL_EXT packet: ");
            // printf_packet(recvframe);
            // debug("\n");
            return 0;
        } else {
            debug("this is unknown packet: ");
            printf_packet(recvframe);
            debug("\n");
            return 0;
        }
    } else {
        debug("gacu command error!\n");
        return -1;
    }

	return 0;
}

void* thread_chassis_can(void *arg)
{
    can_frame_packet_t recvframe;
    int conn_can = *((int*) arg);
    init_need_car_status();
    while(1) {
    	fflush(stdout);
    	int ret = read_packet(conn_can, &recvframe);
        // int ret = read(conn_can, &gapu_frame, sizeof(gapu_frame));
		if(ret == 0) {
			close(conn_can);
			thread_exit();
			break;
		}
       
		acquire_vehicle_posture(&recvframe);		
    }

    return NULL;
}


