#ifndef _CAN_PACKET_H
#define _CAN_PACKET_H

#ifdef __cplusplus
extern "C" {
#endif
#include "utilconfig.h"
#include "pthreadutil.h"

#define SPEED_CONTROL  0x301
#define SPEED_FEEDBACK 0x302
#define STEER_CONTROL  0x703
#define STEER_FEEDBACK 0x704
#define BRAKE_CONTROL  0x131
#define BRAKE_FEEDBACK 0x132

#define TAKE8THBIT     0x80
#define TAKE7THBIT     0x40
#define TAKE2THBIT     0x02
#define TAKE1THBIT     0x01
#define TAKELOW2BIT    0x03
#define TAKELOW4BIT    0x0F
#define TAKEHIGH4BIT   0xF0
#define TAKEHIGH7BIT   0xFE
#define TAKELOW7BIT    0x7F
#define TAKELOW8BIT    0xFF

#define SPEED_RATION 64.0
#define CONST_CONVERT 3.6
#define STEER_RATION 16.2
#define STEER_DEVIATION 0.123457
#define WEIGHT_COEFFICIENT 0.1
#define ANGLE_BASE  740

typedef struct can_frame_packet {
    unsigned short int can_id;
    unsigned char   data[8];
} can_frame_packet_t;

typedef struct can_frame linux_can_frame;

typedef struct vcu_speed_feedback {     //0x302的报文
    char vehicle_drive_mode;
    char vehicle_shift;
    char vehicle_brake_req;
    short int vehicle_velocity;
    char vehicle_acceleration;
    char vehicle_brake; 
    char drive_motor_info;
    char rolling_counter;
    char drive_motor_fault;
    char data_checksums;
} vcu_speed_feedback_t;

typedef struct vcu_speed_control {      //0x301的报文
    char vehicle_drive_mode;
    char vehicle_shift;
    char vehicle_brake_req;
    short int vehicle_velocity;
    char vehicle_acceleration;
    char vehicle_brake; 
    char rolling_counter;
    char data_checksums;
} vcu_speed_control_t;

typedef struct eps_steering_feedback {  //0x704的报文
    char steering_mode;
    short int steering_angle;
    char steering_torque;
    char steering_fault;
    char data_checksums;
} eps_steering_feedback_t;

typedef struct eps_steering_control {   //0x703的报文
    char steering_mode;
    short int steering_angle;
    char steering_speed;
    char dcu_timeout_valid;
    char dcu_timeout_counter;
    char tgt_svu_plimit_valid;
    char tgt_svu_plimit;
    char data_checksums;
} eps_steering_control_t;

typedef struct idu_general_status {    //0x131的报文
    char vehicle_drive_mode_request_validity;
    char vehicle_drive_mode_request;
    char brake_pressure_target_validity;
    char brake_pressure_target;
    char idu_fault_code;
    char vehicle_brake_request;
    char rolling_counter;
    char data_checksums;
} idu_general_brake_status_t;

typedef struct ehb_general_status {     //0x132的报文
    char ehb_status_validity;
    char ehb_driving_mode;
    char brake_pedal_dispalcement;
    char actual_driving_brake_pressure_validity;
    char actual_driving_brake_pressure;
    char driving_brake_fault_code;
    char rolling_counter;
    char data_checksums;
} ehb_general_brake_status_t;

typedef struct car_status {
    double car_velocity;
    double car_vehicle_brake;
    double car_acceleration;
    double car_steer_angle;
    double car_steer_speed;
    double car_steer_torque;
    char   car_drive_mode;
    char   car_steer_mode;
    char   car_drive_shift;
    char   car_ehb_status;
    char   car_ehb_driving_mode;
    char   car_brake_pedal_dispalcement;
    char   car_actual_driving_brake_pressure_validity;
    double car_actual_driving_brake_pressure;
    char   car_driving_brake_fault_code;
} car_status_t;

// typedef struct shared_car_status {
//     pthread_mutex_t pmutex; //锁
//     car_status_t shared_car_posture;
// } shared_car_status_t;

//要同步的话应该和信号量，或者互斥量配合使用   
typedef struct can_shared_use {  //只是用来做全局变量的缓存      
    char vehicle_drive_mode;
    char vehicle_shift;
    char vehicle_brake_req;
    short int vehicle_velocity;
    char vehicle_acceleration;
    char vehicle_brake;
    char steering_mode;
    short int steering_angle;
    char steering_speed;
    char vehicle_drive_mode_request_validity;
    char vehicle_drive_mode_request;
    char brake_pressure_target_validity;
    char brake_pressure_target;
    char idu_fault_code;
    char vehicle_brake_request;
} can_shared_use_t;

int check_sums(unsigned char data[], const int len);
int get_check_sums(unsigned char data[], const int len); 
int decoding_vcu_speed(const can_frame_packet_t *packet_301, 
                       vcu_speed_control_t *recv_message); 
int encoding_vcu_speed(const vcu_speed_feedback_t *send_message,
                       can_frame_packet_t *packet_302);
int decoding_eps_steering(const can_frame_packet_t *packet_703, 
                          eps_steering_control_t *recv_message);
int encoding_eps_steering(const eps_steering_feedback_t *send_message,
                          can_frame_packet_t *packet_704);
int decoding_idu_brake(const can_frame_packet_t *packet_131, 
                       idu_general_brake_status_t *recv_message);
int encoding_ehb_brake(const ehb_general_brake_status_t *send_message,
                       can_frame_packet_t *packet_132);

int write_can_packet(const int conn, can_frame_packet_t *sendframe);
int read_can_packet(const int conn, can_frame_packet_t *recvframe);
int write_packet(const int conn, can_frame_packet_t *sendframe);
int read_packet(const int conn, can_frame_packet_t *recvframe);
int write_serial_packet(const int conn, can_frame_packet_t *sendframe);
int read_serial_packet(const int conn, can_frame_packet_t *recvframe);
int printf_packet(can_frame_packet_t *frame_packet);
int init_packet(can_frame_packet_t* packet, unsigned short int can_id);

int can2car(car_status_t* car_status, 
            const can_shared_use_t* srv_msg);
int car2can(eps_steering_feedback_t* steer,
            vcu_speed_feedback_t* speed,
            ehb_general_brake_status_t* brake,
            const car_status_t* car_status);

void printf_car_status(const car_status_t* car_status);

#ifdef __cplusplus
}
#endif

#endif //_CAN_PACKET_H
