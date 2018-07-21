#include "canpacket.h"

int init_packet(can_frame_packet_t* packet, unsigned short int can_id)
{
    packet->can_id = can_id;
    for(int i = 0; i < 8; ++i)
        packet->data[i] = 0;

    return 0;
}

int check_sums(unsigned char data[], const int len) 
{
    int tmp = 0;
    for(int i = 0; i < len; ++i) {
        tmp += data[i];
    }
    data[len] = (tmp & 0xff);

    return 0;
}

int decoding_vcu_speed(const can_frame_packet_t *packet_301, 
                       vcu_speed_control_t *recv_message)
{
    if(packet_301->can_id != SPEED_CONTROL) {
        debug("packet error for speed control\n");
        return -1;
    }
    recv_message->vehicle_drive_mode = (packet_301->data[0] & TAKE8THBIT) >> 7;
    recv_message->vehicle_shift = (packet_301->data[0] >> 5) & TAKELOW2BIT;
    recv_message->vehicle_brake_req = (packet_301->data[0] >> 3) & TAKELOW2BIT;
    recv_message->vehicle_velocity = (packet_301->data[1] << 8) + packet_301->data[2];
    recv_message->vehicle_acceleration = packet_301->data[3];
    recv_message->vehicle_brake = packet_301->data[4];
    recv_message->rolling_counter = (packet_301->data[5] >> 4) & TAKELOW4BIT;
    recv_message->data_checksums = packet_301->data[6];

    return 0;
}

int encoding_vcu_speed(const vcu_speed_feedback_t *send_message,
                       can_frame_packet_t *packet_302)
{
    if(packet_302->can_id != SPEED_FEEDBACK) {
        packet_302->can_id = SPEED_FEEDBACK;
    }
    packet_302->data[0] = (send_message->vehicle_drive_mode << 7) |
                          (send_message->vehicle_shift << 5) | 
                          (send_message->vehicle_brake_req << 3);
    packet_302->data[1] = (send_message->vehicle_velocity >> 8) & TAKELOW8BIT;
    packet_302->data[2] = send_message->vehicle_velocity & TAKELOW8BIT;
    packet_302->data[3] = send_message->vehicle_acceleration;
    packet_302->data[4] = send_message->vehicle_brake;          //为什么不一致?
    packet_302->data[5] = send_message->drive_motor_info;
    packet_302->data[6] = (send_message->rolling_counter << 4) |
                          (send_message->drive_motor_fault);
    packet_302->data[7] = send_message->data_checksums;

    return 0;
}

int decoding_eps_steering(const can_frame_packet_t *packet_703, 
                          eps_steering_control_t *recv_message)
{
    if(packet_703->can_id != STEER_CONTROL) {
        debug("packet error for steer control\n");
        return -1;
    }
    recv_message->steering_mode = packet_703->data[0] & TAKE1THBIT;
    recv_message->steering_angle = (packet_703->data[1] << 8) + packet_703->data[2];
    recv_message->steering_speed = packet_703->data[3];
    recv_message->dcu_timeout_valid = (packet_703->data[4] >> 7) & TAKE1THBIT;
    recv_message->dcu_timeout_counter = (packet_703->data[4] & TAKELOW7BIT);
    recv_message->tgt_svu_plimit_valid = (packet_703->data[5] >> 7)
                                          & TAKE1THBIT;
    recv_message->tgt_svu_plimit = (packet_703->data[5] & TAKELOW7BIT)
                                    + packet_703->data[6];
    recv_message->data_checksums = packet_703->data[7];

    return 0;
}

int encoding_eps_steering(const eps_steering_feedback_t *send_message,
                          can_frame_packet_t *packet_704)
{
    if(packet_704->can_id != STEER_FEEDBACK) {
        packet_704->can_id = STEER_FEEDBACK;
    }
    packet_704->data[0] = (send_message->steering_mode << 7);
    // packet_704->data[0] = (send_message->steering_mode);
    packet_704->data[1] = (send_message->steering_angle >> 8) & TAKELOW8BIT;
    packet_704->data[2] = (send_message->steering_angle) & TAKELOW8BIT;
    packet_704->data[3] = send_message->steering_torque;
    packet_704->data[4] = send_message->steering_fault << 6;
    packet_704->data[5] = 0;
    packet_704->data[6] = 0;
    packet_704->data[7] = send_message->data_checksums;

    return 0;
}

int write_packet(const int conn, can_frame_packet_t *sendframe)
{
    // check_sums(sendframe->data, 7);

    sendframe->can_id = htons(sendframe->can_id);
    int res = write(conn, sendframe, sizeof(can_frame_packet_t));
    if(res < 0) {
        debug("write failed ");
        debug("should delete timers!\n");
        return -1;
    } 

    return res;
}

int read_packet(const int conn, can_frame_packet_t *recvframe)
{
    memset(recvframe, 0, sizeof(can_frame_packet_t));
    //堵塞地读数据
    int ret = read(conn, recvframe, sizeof(can_frame_packet_t));    
    if(ret < 0) {
        debug("read failed ");
        close(conn);
        // condition_broadcast(&shared_information.g_cond);
        thread_exit();
    }
    else if(ret == 0) {
        debug("peer close\n");
    } else {
        recvframe->can_id = ntohs(recvframe->can_id);     //转换为小端字节序
    }

    return ret;
}

int printf_packet(can_frame_packet_t *frame_packet)
{
    debug("ID = 0X%X ", frame_packet->can_id); 
    debug("data: ");
    for(int i = 0; i < 8; ++i) 
        debug("0X%02x ", frame_packet->data[i]);

    return 0;
}

int can2car(car_status_t* car_status,
            const can_shared_use_t* srv_msg)
{
    car_status->car_velocity = (srv_msg->vehicle_velocity / CONST_CONVERT)
                             / SPEED_RATION;
    car_status->car_acceleration = srv_msg->vehicle_acceleration
                                 * WEIGHT_COEFFICIENT;
    car_status->car_steer_angle = (srv_msg->steering_angle * WEIGHT_COEFFICIENT
                                 - ANGLE_BASE) / STEER_RATION
                                 + STEER_DEVIATION;//16.04, 16.2
    car_status->car_steer_speed = srv_msg->steering_speed;
    car_status->car_drive_mode =  srv_msg->vehicle_drive_mode;
    car_status->car_steer_mode =  srv_msg->steering_mode;
    car_status->car_drive_shift =  srv_msg->vehicle_shift;

    return 0;
}

int car2can(eps_steering_feedback_t* steer,
            vcu_speed_feedback_t* speed,
            const car_status_t* car_status)
{
    steer->steering_angle = (((car_status->car_steer_angle - STEER_DEVIATION)
                          * STEER_RATION) + ANGLE_BASE) / WEIGHT_COEFFICIENT;
    steer->steering_torque = car_status->car_steer_torque / WEIGHT_COEFFICIENT;
    speed->vehicle_velocity = car_status->car_velocity * 
                              SPEED_RATION * CONST_CONVERT;
    speed->vehicle_acceleration = car_status->car_acceleration 
                                / WEIGHT_COEFFICIENT;
    steer->steering_torque = car_status->car_steer_torque;
    steer->steering_mode = car_status->car_steer_mode;
    speed->vehicle_drive_mode = car_status->car_drive_mode;
    speed->vehicle_shift = car_status->car_drive_shift;

    return 0;
}

void printf_car_status(const car_status_t* car_status)
{
    printf("car_status: ");
    printf("v = %f, a = %f, sa = %f, ss = %f, st = %f ", \
            car_status->car_velocity, car_status->car_acceleration,
            car_status->car_steer_angle, car_status->car_steer_speed, 
            car_status->car_steer_torque);
}

