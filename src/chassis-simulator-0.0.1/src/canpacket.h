#ifndef _CAN_PACKET_H
#define _CAN_PACKET_H

#ifdef __cplusplus
extern "C" {
#endif
#include "utilconfig.h"
#include "pthreadutil.h"
#include <stdint.h>

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
#define TAKELOW3BIT    0x07
#define TAKELOW4BIT    0x0F
#define TAKELOW5BIT    0x1F
#define TAKEHIGH4BIT   0xF0
#define TAKEHIGH7BIT   0xFE
#define TAKELOW7BIT    0x7F
#define TAKELOW8BIT    0xFF

#define SPEED_RATION 64.0
#define CONST_CONVERT 3.6
#define STEER_RATION 16.2               //steering gear ratio
#define STEER_DEVIATION 0.0             //added a deviation
#define WEIGHT_COEFFICIENT 0.1
#define ANGLE_BASE  740                 //initial steer angle of 3 packets
#define GACU_ANGLE_BASE 4000.0          //initial steer angle of gacu
#define GACU_TORQUE_BASE 127
// GACU
#define GACU_VERSION          (1)
#define GACU_CAN_DATA_LEN     (8)
#define GACU_SPEED_MAX        (500)     // * 10 m/s
#define GACU_SPEED_MIN        (0)
#define GACU_STEERING_MAX     (4000)    // * 100 deg
#define GACU_STEERING_MIN     (-4000)
#define GACU_BRAKE_MAX        (16)
#define GACU_BRAKE_MIN        (0)

#define acu_can_set_id(u, id) \
    do {                       \
        u.id_high = (id & 0xff00) >> 8;\
        u.id_low = id & 0xff;\
    } while(0)

#define acu_is_can_id(id, exp_id)  \
        (((id >> 8) == (exp_id&0xff)) && ((id&0xff) == (exp_id >> 8)))

#define acu_can_id(id) \
    ((id.id_high <<8) | (id.id_low))

// reserved from 0x5A0 to 0x5A7
typedef enum ACU_Id {
    ACU_ID_CTRL = 0x5A0,        // command from CCU
    ACU_ID_CTRL_EXT = 0x5A1,    // extended command from CCU
    ACU_ID_FDBK = 0x5A2,        // feedback from ACU
    ACU_ID_FDBK_EXT = 0x5A3,    // extended feedback from ACU
    ACU_ID_CTRL2 = 0x5A4,       // command from RCU, with higher priority
    ACU_ID_CTRL_EXT2 = 0x5A5,   // extended command from RCU, with higher priority
    ACU_ID_RESERVED = 0x5A6,
    ACU_ID_INFO = 0x5A7,        // reserve
    ACU_ID_REQ = 0x5A8,         // config request
    ACU_ID_RSP = 0x5A9,         // config response
    ACU_ID_CTRL_PS = 0x5AA,     // project-specific command from CCU
    ACU_ID_FDBK_PS = 0x5AB,     // project-specific feedback from ACU
    ACU_ID_CTRL_PCU = 0x5AC,    // power-related
    ACU_ID_FDBK_PCU = 0x5AD,    // power-related

    ACU_ID_DIAG_CMD = 0x7A0,    // diag command
    ACU_ID_DIAG_RET = 0x7A1,    // diag return
    ACU_ID_PROG_CMD = 0x7A2,    // programming command
    ACU_ID_PROG_RET = 0x7A3,    // programming return
} acu_id_t;

typedef enum ACU_Shift {
    ACU_SHIFT_N = 0,
    ACU_SHIFT_D = 1,
    ACU_SHIFT_R = 2,
    ACU_SHIFT_P = 3,
} acu_shift_t;

typedef enum ACU_Error {
    ACU_ERROR_NONE = 0,         // -
    ACU_ERROR_OH = 1,           // H    Over Heat
    ACU_ERROR_OV = 2,           // V    Over Voltage
    ACU_ERROR_UV = 3,           // v    Under Voltage
    ACU_ERROR_MOTOR = 4,        // M    Motor Error
    ACU_ERROR_SAS = 5,          // A    Steering Angle Sensor Error
    ACU_ERROR_TQ = 6,           // T    Torque Sensor Error
    ACU_ERROR_MISC = 7,         // X    Misc
} acu_error_t;

typedef struct ACU_CAN_Id {
    uint8_t id_high;
    uint8_t id_low;
} acu_can_id_t;
// 5A0 packet
typedef struct ACU_Can_Ctrl {
    /* Byte 0 */
    // From LSb to MSb
    uint8_t steer_enable:1;
    uint8_t drive_enable:1;
    uint8_t brake_enable:1;
    uint8_t shift:2;            // N D R P
    uint8_t byte0_reserved:1;
    uint8_t estop:1;            // E Ji Ting Ming Ling
    uint8_t clear_error:1;

    /* Byte 1,2,3 */
    uint8_t wheel_angle_low:8;  // deg = V*0.01 [-40 left, 40 right]
    uint8_t wheel_angle_high:5;
    uint8_t byte2_reserved:3;
    uint8_t angle_rate:8;       // deg/s = V*0.1   0 ~ 25.5 deg/s [optional]

    /* Byte 4 & 5 */
    uint8_t speed_low:8;        // vel = V*0.1, m/s, [0 - 50]
    uint8_t speed_high:1;
    uint8_t acc:7;              // acc = V*0.1, m^2/s, [-6.3 - 6.3], -6.4 == max brake value

    /* Byte 6 */
    uint8_t rolling_counter:4;
    uint8_t uos_state:3;        //0 = OFF, 1=PAUSE, 2-6 = ON, 7 = reserved
    uint8_t byte5_reserved:1;

    /* Byte 7 */
    uint8_t checksum:8;
} acucan_ctrl_t;
// 5A1 packet
typedef struct ACU_Can_Ctrl_Ext {
    /* Byte 0 */
    uint8_t left_light:1;       // L Zuo Zhuan Xiang Deng
    uint8_t right_light:1;      // R You Zhuan Xiang Deng
    uint8_t dual_light:1;       // D Shuang Shan Deng
    uint8_t park_light:1;       // P Zhu Che Deng
    uint8_t low_light:1;        // L Jin  Guang Deng
    uint8_t high_light:1;       // H Yuan Guang Deng
    uint8_t front_fog_light:1;  // f Qian Wu Deng
    uint8_t rear_fog_light:1;   // r Hou  Wu Deng

    /* Byte 1 */
    uint8_t brake_light:1;      // B Sha Che Deng
    uint8_t back_light:1;       // b Dao Che Deng
    uint8_t speaker:1;          // S La Ba
    uint8_t wiper:1;            // W Yu Shua
    uint8_t day_light:1;        // w Gong Zuo Deng
    uint8_t poweroff_ccu:1;     // O CCU asks APU to power off CCU 1 minute later
    uint8_t byte1_reserved:2;

    /* Byte 2~6 */
    uint8_t byte2_reserved:8;
    uint8_t byte3_reserved:8;
    uint8_t byte4_reserved:8;
    uint8_t byte5_reserved:8;
    uint8_t rolling_counter:4;
    uint8_t byte6_reserved:4;

    /* Byte 7 */
    uint8_t check_sum:8;
} acucan_ctrl_ext_t;
    // 5A2 packet
typedef struct ACU_Can_Feedback {
    /* Byte 0 */
    uint8_t steer_enabled:1;
    uint8_t drive_enabled:1;
    uint8_t brake_enabled:1;
    uint8_t shift:2;            // N D R P
    uint8_t byte0_reserved:3;   // reserved for furture use

    /* Byte 1,2,3 */
    uint8_t wheel_angle_low:8;  // deg = V*0.01 [-40 left, 40 right]
    uint8_t wheel_angle_high:5;
    uint8_t byte2_reserved:1;
    uint8_t steer_fault:2;      // 0 = no fault, 1-2 = warning, 3 = fault
    uint8_t torque:8;           // Nm = V*0.1, [-12.7, 12.7], -12.8 means torque is not supported
    //torque should initialize 127, 127 indicates 0 torque
    /* Byte 4 & 5 */
    uint8_t speed_low:8;        // vel = V*0.1, m/s, [0 - 50]
    uint8_t speed_high:1;
    uint8_t engine_direction:1; // 0: forward, 1: backword
    uint8_t drive_fault:2;      //shuold set 0
    uint8_t brake_fault:2;
    uint8_t brake_human_intervention:1;
    uint8_t byte5_reserved:1;

    /* Byte 6 */
    uint8_t rolling_counter:4;
    /* apu_state */
    /*  apu_state_init_error = 0,
     *  apu_state_veh_stopped = 1,
     *  apu_state_auto_ready = 2,
     *  apu_state_veh_softstopping  = 3,
     *  apu_state_veh_error     = 4,
     *  apu_state_standby  = 5,
     *  apu_state_chassis_control = 6,
     *  apu_state_joystick_control = 7,
     */
    uint8_t acu_state:3;
    uint8_t byte6_reserved:1;

    /* Byte 7 */
    uint8_t check_sum:8;
} acucan_feedback_t;

typedef struct ACU_Can_Feedback_Ext {
    /* Byte 0 */
    uint8_t left_light:1;
    uint8_t right_light:1;
    uint8_t dual_light:1;
    uint8_t park_light:1;
    uint8_t low_light:1;
    uint8_t high_light:1;
    uint8_t front_fog_light:1;
    uint8_t rear_fog_light:1;

    /* Byte 1 */
    uint8_t brake_light:1;
    uint8_t back_light:1;
    uint8_t speaker:1;
    uint8_t wiper:1;
    uint8_t day_light:1;
    uint8_t shutdown_ccu:1;     // APU asks CCU to shutdown itself,
    // APU will power off CCU 1 minute later
    uint8_t byte1_reserved:2;

    /* Byte 2: status only, no control */
    uint8_t brake_pedal:1;
    uint8_t hand_brake:1;
    uint8_t driver_mode_button:1;   // 1: auto; 0: manual
    uint8_t stop_button:1;      // 1: stop button active; 0: stop button inactive;
    uint8_t byte2_reserved:4;
    uint8_t byte3_reserved:8;

    uint8_t byte4_reserved:8;
    uint8_t byte5_reserved:8;
    uint8_t rolling_counter:4;
    uint8_t byte6_reserved:4;

    /* Byte 7 */
    uint8_t check_sum:8;
} acucan_feedback_ext_t;

typedef struct gacu_can_feedback_packet {
    unsigned short int can_id;
    acucan_feedback_t   data;
} gacu_can_feedback_packet_t;

typedef struct gacu_can_ctrl_packet {
    unsigned short int can_id;
    acucan_ctrl_t   data;
} gacu_can_ctrl_packet_t;

typedef struct can_frame_packet {
    unsigned short int can_id;
    unsigned char   data[8];
} can_frame_packet_t;

typedef struct can_frame linux_can_frame;

typedef struct vcu_speed_feedback {     //0x302 raw can packet
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

typedef struct vcu_speed_control {      //0x301 raw can packet
    char vehicle_drive_mode;
    char vehicle_shift;
    char vehicle_brake_req;
    short int vehicle_velocity;
    char vehicle_acceleration;
    char vehicle_brake; 
    char rolling_counter;
    char data_checksums;
} vcu_speed_control_t;

typedef struct eps_steering_feedback {  //0x704 raw can packet
    char steering_mode;
    short int steering_angle;
    char steering_torque;
    char steering_fault;
    char data_checksums;
} eps_steering_feedback_t;

typedef struct eps_steering_control {   //0x703 raw can packet
    char steering_mode;
    short int steering_angle;
    char steering_speed;
    char dcu_timeout_valid;
    char dcu_timeout_counter;
    char tgt_svu_plimit_valid;
    char tgt_svu_plimit;
    char data_checksums;
} eps_steering_control_t;

typedef struct idu_general_status {    //0x131 raw can packet
    char vehicle_drive_mode_request_validity;
    char vehicle_drive_mode_request;
    char brake_pressure_target_validity;
    char brake_pressure_target;
    char idu_fault_code;
    char vehicle_brake_request;
    char rolling_counter;
    char data_checksums;
} idu_general_brake_status_t;

typedef struct ehb_general_status {     //0x132 raw can packet
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

// Just used to cache global variables
typedef struct can_shared_use {     
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

int decoding_gacu_ctrl(const can_frame_packet_t *packet_gacu, 
                       acucan_ctrl_t *recv_message);
int encoding_gacu_feedback(const acucan_feedback_t *packet_gacu_feedback,
                           can_frame_packet_t *send_message);

int write_can_packet(const int conn, can_frame_packet_t *sendframe);
int read_can_packet(const int conn, can_frame_packet_t *recvframe);
int write_packet(const int conn, can_frame_packet_t *sendframe);
int read_packet(const int conn, can_frame_packet_t *recvframe);
int write_serial_packet(const int conn, can_frame_packet_t *sendframe);
int read_serial_packet(const int conn, can_frame_packet_t *recvframe);
int printf_packet(const can_frame_packet_t *frame_packet);
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
