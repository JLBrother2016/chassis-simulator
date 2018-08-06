#include "canpacket.h"
#include <termios.h>

extern int select_protocol; //0--serial; 1--tcp/ip; 2--raw can

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
int get_check_sums(unsigned char data[], const int len) 
{
    int tmp = 0;
    for(int i = 0; i < len; ++i) {
        tmp += data[i];
    }

    return (tmp & 0xff);
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
    packet_704->data[1] = (send_message->steering_angle >> 8) & TAKELOW8BIT;
    packet_704->data[2] = (send_message->steering_angle) & TAKELOW8BIT;
    packet_704->data[3] = send_message->steering_torque;
    packet_704->data[4] = send_message->steering_fault << 6;
    packet_704->data[5] = 0;
    packet_704->data[6] = 0;
    packet_704->data[7] = send_message->data_checksums;

    return 0;
}
// decoding brake
int decoding_idu_brake(const can_frame_packet_t *packet_131, 
                       idu_general_brake_status_t *recv_message)
{
    if(packet_131->can_id != BRAKE_CONTROL) {
        debug("IDU packet error for steer control\n");
        return -1;
    }
    recv_message->vehicle_drive_mode_request_validity = (packet_131->data[0] & 
                                                        TAKE8THBIT) >> 7;
    recv_message->vehicle_drive_mode_request = (packet_131->data[0] & 
                                                TAKE7THBIT) >> 6;
    recv_message->brake_pressure_target_validity = (packet_131->data[0] & 
                                                    TAKE2THBIT) >> 1;
    recv_message->brake_pressure_target = packet_131->data[2];
    recv_message->idu_fault_code = packet_131->data[5];
    recv_message->vehicle_brake_request = (packet_131->data[6]
                                        & TAKE8THBIT) >> 7;
    recv_message->rolling_counter = packet_131->data[6] & TAKELOW4BIT;
    recv_message->data_checksums = packet_131->data[7];

    return 0;
}
// encoding brake
int encoding_ehb_brake(const ehb_general_brake_status_t *send_message,
                       can_frame_packet_t *packet_132)
{
    if(packet_132->can_id != BRAKE_FEEDBACK) {
        debug("EHB packet error for steer control\n");
        packet_132->can_id = BRAKE_FEEDBACK;
    }
    packet_132->data[0] = (send_message->ehb_status_validity << 3) |
                          (send_message->ehb_driving_mode << 2);
    packet_132->data[1] = (send_message->brake_pedal_dispalcement << 1) |
                        (send_message->actual_driving_brake_pressure_validity);
    packet_132->data[2] = send_message->actual_driving_brake_pressure;
    packet_132->data[3] = 0;
    packet_132->data[4] = 0;
    packet_132->data[5] = send_message->driving_brake_fault_code;
    packet_132->data[6] = send_message->rolling_counter;
    packet_132->data[7] = send_message->data_checksums;

    return 0;
}

// decoding gacu
int decoding_gacu_ctrl(const can_frame_packet_t *packet_gacu, 
                       acucan_ctrl_t *recv_message)
{
    if(packet_gacu->can_id != ACU_ID_CTRL) {
        debug("gacu ctrl packet error for control ...\n");
        debug("this is unknown packet in ctrl: ");
        printf_packet(recv_message);
        debug("\n");

        return -1;
    }
    memset(recv_message, 0, sizeof(acucan_ctrl_t));
    // recv_message->steer_enable = packet_gacu->data[0] & TAKE1THBIT;
    // recv_message->drive_enable = (packet_gacu->data[0] >> 1) & TAKE1THBIT;
    // recv_message->brake_enable = (packet_gacu->data[0] >> 2) & TAKE1THBIT;
    // recv_message->shift = (packet_gacu->data[0] >> 3) & TAKELOW2BIT;
    // recv_message->estop = (packet_gacu->data[0] >> 6) & TAKELOW2BIT;
    // recv_message->clear_error = (packet_gacu->data[0] >> 7) & TAKELOW2BIT;
    // recv_message->wheel_angle_low = packet_gacu->data[1];
    // recv_message->wheel_angle_high = packet_gacu->data[2] & TAKELOW5BIT;
    // recv_message->angle_rate = packet_gacu->data[3];
    // recv_message->speed_low = packet_gacu->data[4];
    // recv_message->speed_high = packet_gacu->data[5] & TAKE1THBIT;
    // recv_message->acc = (packet_gacu->data[5] >> 1) & TAKELOW7BIT;
    // recv_message->rolling_counter = packet_gacu->data[6] & TAKELOW4BIT;
    // recv_message->uos_state = (packet_gacu->data[6] >> 4) & TAKELOW3BIT;
    // recv_message->checksum = packet_gacu->data[7]; 
    memcpy(recv_message, packet_gacu->data, 8);

    return 0;
}
// encoding gacu
int encoding_gacu_feedback(const acucan_feedback_t *packet_gacu_feedback,
                       	   can_frame_packet_t *send_message)
{
    if(send_message->can_id != ACU_ID_FDBK) {
        debug("gacu feedback packet error for control ...\n");
        debug("this is unknown packet in feedback: ");
        printf_packet(send_message);
        debug("\n");
        send_message->can_id = ACU_ID_FDBK;
    }
    // packet_132->data[0] = (send_message->ehb_status_validity << 3) |
    //                       (send_message->ehb_driving_mode << 2);
    // packet_132->data[1] = (send_message->brake_pedal_dispalcement << 1) |
    //                     (send_message->actual_driving_brake_pressure_validity);
    // packet_132->data[2] = send_message->actual_driving_brake_pressure;
    // packet_132->data[3] = 0;
    // packet_132->data[4] = 0;
    // packet_132->data[5] = send_message->driving_brake_fault_code;
    // packet_132->data[6] = send_message->rolling_counter;
    // packet_132->data[7] = send_message->data_checksums;
    memcpy(send_message->data, packet_gacu_feedback, 
    						sizeof(acucan_feedback_t));

    return 0;
}

static int tcp_write_packet(const int conn, can_frame_packet_t *sendframe)
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

static int tcp_read_packet(const int conn, can_frame_packet_t *recvframe)
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

static int can_write_packet(const int conn, can_frame_packet_t *sendframe)
{
    linux_can_frame temp_can_frame;
    temp_can_frame.can_id = sendframe->can_id;
    temp_can_frame.can_dlc = 8;
    for (int i = 0; i < temp_can_frame.can_dlc; ++i) {
        temp_can_frame.data[i] = sendframe->data[i];
    }
    int res = write(conn, &temp_can_frame, sizeof(linux_can_frame));
    if(res < 0) {
        debug("write failed ");
        debug("should delete timers!\n");
        return -1;
    } 

    return res;
}

static int can_read_packet(const int conn, can_frame_packet_t *recvframe)
{
    linux_can_frame temp_can_frame;
    memset(&temp_can_frame, 0, sizeof(linux_can_frame));
    memset(recvframe, 0, sizeof(can_frame_packet_t));
    //堵塞地读数据
    int ret = read(conn, &temp_can_frame, sizeof(linux_can_frame));    
    if(ret < 0) {
        debug("read failed ");
        close(conn);
        // condition_broadcast(&shared_information.g_cond);
        thread_exit();
    }
    else if(ret == 0) {
        debug("peer close\n");
    } else {
        recvframe->can_id = temp_can_frame.can_id;
        for (int i = 0; i < temp_can_frame.can_dlc; ++i) {
            recvframe->data[i] = temp_can_frame.data[i];
        }
    }

    return ret;
}

static int write_n(int fd, unsigned char* pchar, int n) 
{
    size_t  nleft = n;
    ssize_t  nread;
    while (nleft > 0) {
        if ((nread = write(fd, pchar, nleft)) < 0) {
            if(errno == EAGAIN){
                // usleep(1*1000);
                continue;
            }

            printf("write_n error read n\n");
            if (nleft == n)
                return(-1); /* error, return -1 */
            else
                break;      /* error, return amount read so far */
        } else if (nread == 0) {
            // break;          /* EOF */
            // printf("O_NDELAY\n");
            // usleep(1*1000);
            continue;
        }
        nleft -= nread;
        pchar   += nread;
    }
    
    return(n - nleft);      /* return >= 0 */
}

static int serial_write_packet(const int conn, can_frame_packet_t *sendframe)
{
    unsigned char write_buffer[16];
    memset(write_buffer, 0, sizeof(write_buffer));
    // printf_packet(sendframe);
    write_buffer[0] = (sendframe->can_id >> 8) & 0xff;
    write_buffer[1] = sendframe->can_id & 0xff;
    for(int i = 0; i < 8; ++i)
        write_buffer[2 + i] = sendframe->data[i]; 
    int res = write_n(conn, write_buffer, 10);  //only write 10 bytes
    // int res = write(conn, write_buffer, 10); 
    printf("write fd:%d :%d:", conn, res);
    for (int i = 0; i < res; ++i)  //it will change rdlen
        printf(" 0x%02x", write_buffer[i]);
    printf("\n");
    if(res < 0) {

        int val = fcntl(conn, F_GETFL, 0);
        printf("file status = 0x%x\n", val);
        printf("res error: ");
        for (int i = 0; i < 10; ++i)  //it will change rdlen
            printf(" 0x%02x", write_buffer[i]);
        if(errno == EAGAIN)
            printf("EAGAIN ");
        perror("write serail_fd~~~~~~~~");
        while(1) {
            ;
        }
        debug("should delete timers!\n");
        return -1;
    } /*else if(res == 10) {
        tcflush(conn, TCOFLUSH);
    }*/

    return res;
}

static inline int is_canid(unsigned char* buff, int index) 
{
    if(index > 10) 
        printf("index too long err\n");
    unsigned short int can_id = buff[index] << 8 | buff[index + 1];
    if(can_id == 0x131 ||
       can_id == 0x703 || 
       can_id == 0x301 ||
       can_id == 0x132) return 1;

    return 0;
}

static int shift_to_canid(unsigned char* buff) 
{
    int index = -1;
    for(int i = 0; i < 10; ++i) {
        if(is_canid(buff, i)) {
            index = i;
            break;
        }
    }

    return index;
}

/* Read "n" bytes from a descriptor  */
ssize_t readn(int fd, void *ptr, size_t n)
{
    size_t  nleft;
    ssize_t  nread;
    nleft = n;
    while (nleft > 0) {
        if ((nread = read(fd, ptr, nleft)) < 0) {
            printf("readn error read n\n");
            if (nleft == n)
                return(-1); /* error, return -1 */
            else
                break;      /* error, return amount read so far */
        } else if (nread == 0) {
            // break;          /* EOF */
            // printf("O_NDELAY\n");
            // usleep(1*1000);
            continue;
        }
        nleft -= nread;
        ptr   += nread;
    }
    
    return(n - nleft);      /* return >= 0 */
}

static int read_a_packet(const int serail_fd, 
                         unsigned char* read_buffer, int n)
{
    int length = readn(serail_fd, read_buffer, n);
    int res = shift_to_canid(read_buffer);
    if(res == 0) //don't need deal
        return length - res;
    else if(res > 0 && res < 10) {
        unsigned char swap_buff[16];
        memset(swap_buff, 0, sizeof(swap_buff)); 
        int new_len = readn(serail_fd, swap_buff, res);
        if(new_len == res) {
            memcpy(read_buffer, swap_buff, res);
            memcpy(swap_buff, read_buffer+res, length - res);
            memcpy(swap_buff+length-res, read_buffer, res);
            memcpy(read_buffer, swap_buff, length);
        } else {
            printf("error: have can id but no data!\n");
            return -1;
        }
    }
    else if(res == -1) { //no can_id
        unsigned char tmp_buff[16];
        memset(tmp_buff, 0, sizeof(tmp_buff));
        readn(serail_fd, tmp_buff, 1);
        tmp_buff[1] = tmp_buff[0];
        tmp_buff[0] = read_buffer[n - 1];
        if(is_canid(tmp_buff, 0)) {
            readn(serail_fd, tmp_buff + 2, length - 2);
            memcpy(read_buffer, tmp_buff, length);
        } else {
            printf("error: no can id !\n");
            readn(serail_fd, read_buffer, n);
            printf("read_buffer: ");
            for(int i = 0; i < length; ++i) {
                printf("0x%02x ", read_buffer[i]);
            }
            printf("\n");

            while(1) {
                ;
            }
            return -1;
        }
    } else if(res == 10) {
        tcflush(serail_fd, TCIFLUSH);
    }
    
    return length;
}

static int serial_read_packet(const int conn, can_frame_packet_t *recvframe)
{
    unsigned char read_buffer[11];
    memset(read_buffer, 0, sizeof(read_buffer));
    int read_len = 0;
    int length = 0;
    // unsigned char msg[11] = {0};
    // tcflush(conn, TCIFLUSH);    //clear serail read buffer
    
    read_len = read_a_packet(conn, read_buffer, sizeof(read_buffer) - 1);
    // read_len =  read(conn, read_buffer, sizeof(read_buffer) - 1);
    if(read_len < 0) {
        printf("read error!\n");
        return -1;
    }
    // printf("Read %d:", read_len);
    // for (int i = 0; i < read_len; ++i)  //it will change rdlen
    //     printf(" 0x%02x", read_buffer[i]);
    length = read_len;
    if(length == 10){
        memset(recvframe, 0, sizeof(can_frame_packet_t));
        recvframe->can_id = read_buffer[0] << 8 | read_buffer[1];
        for(int i = 0; i < 8; ++i) {
            recvframe->data[i] = read_buffer[2 + i];
        }
        // tcflush(conn, TCIFLUSH);
    }
    // printf(" ");
    // printf_packet(recvframe);
    // printf("\n");
     
    return length;
}

int write_packet(const int conn, can_frame_packet_t *sendframe)
{
	int res;
    // check_sums(sendframe->data, 7);//第7位是校验和, 三个报文应该分开计算校验和  
	if(select_protocol == serial_protocol) 
		res = serial_write_packet(conn, sendframe);
	else if(select_protocol == tcp_ip_protocol)
		res = tcp_write_packet(conn, sendframe);
	else if(select_protocol == raw_can_protocol)
		res = can_write_packet(conn, sendframe);
	else {
		res = -1;
		debug("write %d this protocol is not defined!\n", select_protocol);
	}
    // debug("write: %d: ", res);
    // printf_packet(sendframe);
    // printf("\n");

    return res;
}

int read_packet(const int conn, can_frame_packet_t *recvframe)
{
	int res;
	if(select_protocol == serial_protocol) 
		res = serial_read_packet(conn, recvframe);
	else if(select_protocol == tcp_ip_protocol)
		res = tcp_read_packet(conn, recvframe);
	else if(select_protocol == raw_can_protocol)
		res = can_read_packet(conn, recvframe);
	else {
		res = -1;
		debug("read %d this protocol is not defined!\n", select_protocol);
	}

    return res;
}

int printf_packet(can_frame_packet_t *frame_packet)
{
    debug("ID = 0X%X ", frame_packet->can_id); 
    debug("data: ");
    for(int i = 0; i < 8; ++i) 
        debug("0X%02x ", frame_packet->data[i]);

    return 0;
}
// 这个函数会把收到的所有can包信息转换为车的姿态信息
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
    car_status->car_vehicle_brake = srv_msg->vehicle_brake;
    if(srv_msg->vehicle_brake_request) {    //刹车有效才进行改变
        car_status->car_ehb_status = 
                                srv_msg->vehicle_drive_mode_request_validity;
        car_status->car_ehb_driving_mode = srv_msg->vehicle_drive_mode_request;
        car_status->car_actual_driving_brake_pressure_validity = 
                                    srv_msg->brake_pressure_target_validity;
        car_status->car_actual_driving_brake_pressure = 
                                    srv_msg->brake_pressure_target * 0.5;
        car_status->car_driving_brake_fault_code = srv_msg->idu_fault_code; 
    }
    
    return 0;
}

int car2can(eps_steering_feedback_t* steer,
            vcu_speed_feedback_t* speed,
            ehb_general_brake_status_t* brake,
            const car_status_t* car_status)
{
    steer->steering_angle = (short int)(
                            (((car_status->car_steer_angle - STEER_DEVIATION)
                          * STEER_RATION) + ANGLE_BASE) / WEIGHT_COEFFICIENT);
    steer->steering_torque = (char) 
                        (car_status->car_steer_torque / WEIGHT_COEFFICIENT);
    steer->steering_mode = car_status->car_steer_mode;

    speed->vehicle_velocity = (short int)(car_status->car_velocity * 
                              SPEED_RATION * CONST_CONVERT);
    speed->vehicle_acceleration = (char) (car_status->car_acceleration 
                                / WEIGHT_COEFFICIENT);
    speed->vehicle_brake = (char) (car_status->car_vehicle_brake);
    speed->vehicle_drive_mode = car_status->car_drive_mode;
    speed->vehicle_shift = car_status->car_drive_shift;

    brake->ehb_status_validity = car_status->car_ehb_status;
    brake->ehb_driving_mode = car_status->car_ehb_driving_mode;
    brake->brake_pedal_dispalcement = car_status->car_brake_pedal_dispalcement;
    brake->actual_driving_brake_pressure_validity = 
                        car_status->car_actual_driving_brake_pressure_validity;
    brake->actual_driving_brake_pressure = 
                        (char)car_status->car_actual_driving_brake_pressure;
    brake->driving_brake_fault_code = car_status->car_driving_brake_fault_code;

    return 0;
}

void printf_car_status(const car_status_t* car_status)
{
    printf("car_status: ");
    printf("v = %f, a = %f, sa = %f, ss = %f, st = %f, sh = %d", \
            car_status->car_velocity, car_status->car_acceleration,
            car_status->car_steer_angle, car_status->car_steer_speed, 
            car_status->car_steer_torque, car_status->car_drive_shift);
}

