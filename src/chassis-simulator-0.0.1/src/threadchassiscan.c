/************************************************************************
*
*   file name：threadchassiscan.c
*
*   file description：the thread to read raw can data.
*
*   creator： Jiang Long, 2018.08.07
*
*   version：0.0.1
*
*   Modify record：
*
************************************************************************/

#include "threadchassiscan.h"
#include "pthreadutil.h"
#include "canpacket.h"

extern global_locks_t global_locks;
extern char enable_gacu;

car_status_t need_car_posture;
//shared_information just for decoding can packet
static can_shared_use_t shared_information;

/*======================================================================
 * function name： init_need_car_status
 * parameter：
 *      void
 * descriptions:
 *      initialize the need status of car
 * return：0
 =====================================================================*/
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

/*======================================================================
 * function name： acquire_vehicle_speed
 * parameter：
 *      vcu_speed_control_t* recv4speed [IN] : speed data
 * descriptions:
 *      convert speed data to shared_information
 * return：success --- 0, failure --- -1
 =====================================================================*/
static int acquire_vehicle_speed(vcu_speed_control_t* recv4speed)
{
    if(recv4speed == NULL) {
        return -1;
    }
    
	shared_information.vehicle_drive_mode = recv4speed->vehicle_drive_mode;
	shared_information.vehicle_shift = recv4speed->vehicle_shift;  
   	shared_information.vehicle_brake_req = recv4speed->vehicle_brake_req; 
	shared_information.vehicle_velocity = recv4speed->vehicle_velocity;		
	shared_information.vehicle_acceleration = recv4speed->vehicle_acceleration;
	shared_information.vehicle_brake = recv4speed->vehicle_brake;

	return 0;
}

/*======================================================================
 * function name： acquire_vehicle_steer
 * parameter：
 *      vcu_speed_control_t* recv4speed [IN] : steer data
 * descriptions:
 *      convert steer data to shared_information
 * return：success --- 0, failure --- -1
 =====================================================================*/
static int acquire_vehicle_steer(eps_steering_control_t* recv4steer)
{
    if(recv4steer == NULL) {
        return -1;
    }

	shared_information.steering_mode = recv4steer->steering_mode;
	shared_information.steering_angle = recv4steer->steering_angle; 
	shared_information.steering_speed = recv4steer->steering_speed;

	return 0;
}

/*======================================================================
 * function name： acquire_vehicle_brake
 * parameter：
 *     idu_general_brake_status_t* recv4brake [IN] : brake data
 * descriptions:
 *      convert brake data to shared_information
 * return：success --- 0, failure --- -1
 =====================================================================*/
static int acquire_vehicle_brake(idu_general_brake_status_t* recv4brake)
{
    if(recv4brake == NULL) {
        return -1;
    }
	// 0, 6, 10, 16, 22, 30, 40, 56    //*1 bar                  
	// 0, 3,  5,  7, 10, 13, 17, 20    //dec   *10 m/s^2 acc = brake * (-0.45) 
	if(recv4brake->brake_pressure_target) { // when braking to debug
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

/*======================================================================
 * function name： acquire_vehicle_gacu
 * parameter：
 *     const acucan_ctrl_t* gacu4ctrl [IN] : gacu struct data
 * descriptions:
 *      convert gacu struct data to the need status of car
 * return：success --- 0, failure --- -1
 =====================================================================*/
static int acquire_vehicle_gacu(const acucan_ctrl_t* gacu4ctrl)
{
    if(gacu4ctrl == NULL)
        return -1;
    // directly convert the gacu message into a pose of the car
    need_car_posture.car_drive_mode = gacu4ctrl->drive_enable;
    need_car_posture.car_steer_mode = gacu4ctrl->steer_enable;
	need_car_posture.car_ehb_driving_mode = gacu4ctrl->brake_enable;
	need_car_posture.car_drive_shift = gacu4ctrl->shift;
	need_car_posture.car_driving_brake_fault_code = gacu4ctrl->estop;//for stop
	need_car_posture.car_ehb_status = gacu4ctrl->clear_error; //for clear error
	need_car_posture.car_steer_angle = (((gacu4ctrl->wheel_angle_low | 
									   (gacu4ctrl->wheel_angle_high << 8))
                                     - GACU_ANGLE_BASE)) * 0.01;
	need_car_posture.car_steer_speed = gacu4ctrl->angle_rate * 0.1;
	need_car_posture.car_velocity =  (gacu4ctrl->speed_low |
									 (gacu4ctrl->speed_high << 8)) * 0.1;
	need_car_posture.car_acceleration = gacu4ctrl->acc * 0.1;//deceleration
	need_car_posture.car_actual_driving_brake_pressure_validity = 
                                        gacu4ctrl->uos_state; //uos  state

    return 0;
}

/*======================================================================
 * function name： acquire_vehicle_posture
 * parameter：
 *     can_frame_packet_t* recvframe [IN] : the raw can packet
 * descriptions:
 *     convert the raw can packet to the need status of car
 * return：success --- 0, failure --- -1
 =====================================================================*/
static int acquire_vehicle_posture(can_frame_packet_t* recvframe)
{
    if(recvframe == NULL)
        return -1;
    int check_sum = 0;

    if(enable_gacu == 0) {      // 3 packet format
        if(recvframe->can_id == SPEED_CONTROL) { //get speed
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
        } else if(recvframe->can_id == STEER_CONTROL) {//get steer
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
        } else if(recvframe->can_id == BRAKE_CONTROL) {//get brake
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

        //convert raw can data to need status of car, add a lock
        mutex_lock(&global_locks.can_mutex);
        can2car(&need_car_posture, &shared_information); 
        mutex_unlock(&global_locks.can_mutex);
    } else if(enable_gacu == 1) {       //gacu format
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

/*======================================================================
 * function name： thread_chassis_can
 * parameter：
 *     void *arg [IN] : the file descriptor of connection for can
 * descriptions:
 *     the thread blocking read raw data from protocols(gacu, 3 packets format)
 * return：NULL
 =====================================================================*/
void* thread_chassis_can(void *arg)
{
    can_frame_packet_t recvframe;
    int conn_can = *((int*) arg);
    init_need_car_status();
    while(1) {
    	fflush(stdout);
    	int ret = read_packet(conn_can, &recvframe);
		if(ret == 0) {
            debug("read peer close!\n");
            while(1) {
                ;   // busy waiting
            }
			// close(conn_can);
			// thread_exit();
			// break;
		}
       
		acquire_vehicle_posture(&recvframe);		
    }

    return NULL;
}


