/************************************************************************
*
*   file name：canpacket.c
*
*   file description：read, write can packet
*
*   creator： Jiang Long, 2018.08.07
*
*   version：0.0.1
*
*   Modify record：
*
************************************************************************/

#include "canpacket.h"
#include <termios.h>

extern int select_protocol; //0--serial; 1--tcp/ip; 2--raw can
/*======================================================================
 * function name： init_packet
 * parameter：
 *      can_frame_packet_t* packet [IN] : to initialize can packet
 *      unsigned short int can_id  [IN] : can id of the can packet
 * descriptions:
 *      initialization a can packet
 * return：success --- 0, failure --- -1
 =====================================================================*/
int init_packet(can_frame_packet_t* packet, unsigned short int can_id)
{
    if(packet == NULL)
        return -1;
    packet->can_id = can_id;
    for(int i = 0; i < 8; ++i)
        packet->data[i] = 0;

    return 0;
}

/*======================================================================
 * function name： get_check_sums
 * parameter：
 *      unsigned char data[] [IN] : the data of a can packet
 *      const int len  [IN] : the position of the caculated check sums
 * descriptions:
 *      to get the check sums of a can packet, but not assignment
 * return：success --- the check sums, failure --- -1
 =====================================================================*/
int get_check_sums(unsigned char data[], const int len) 
{
    if(len < 0 || data == NULL || len > 8)
        return -1;
    int tmp = 0;
    for(int i = 0; i < len; ++i) {
        tmp += data[i];
    }

    return (tmp & 0xff);
}

/*======================================================================
 * function name： check_sums
 * parameter：
 *      unsigned char data[] [IN] : the data of a can packet
 *      const int len  [IN] : the position of the caculated check sums
 * descriptions:
 *      to caculate the check sums of a can packet
 * return：success --- 0, failure --- -1
 =====================================================================*/
int check_sums(unsigned char data[], const int len) 
{
    int tmp = get_check_sums(data, len);
    if(tmp < 0)
        return -1;
    data[len] = tmp;

    return 0;
}

/*======================================================================
 * function name： decoding_vcu_speed
 * parameter：
 *      const can_frame_packet_t *packet_301 [IN] : the can packet of 301
 *      vcu_speed_control_t *recv_message  [OUT] : the states of vcu
 * descriptions:
 *      convert a raw can packet to the states of vcu
 * return：success --- 0, failure --- -1
 =====================================================================*/
int decoding_vcu_speed(const can_frame_packet_t *packet_301, 
                       vcu_speed_control_t *recv_message)
{
    if(packet_301 == NULL || recv_message == NULL) 
        return -1;
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

/*======================================================================
 * function name： encoding_vcu_speed
 * parameter：
 *      const vcu_speed_feedback_t *send_message [IN] : the states of vcu
 *      can_frame_packet_t *packet_302  [OUT] : the can packet of 302
 * descriptions:
 *      convert  the states of vcu(speed) to a raw can packet
 * return：success --- 0, failure --- -1
 =====================================================================*/
int encoding_vcu_speed(const vcu_speed_feedback_t *send_message,
                       can_frame_packet_t *packet_302)
{
    if(send_message == NULL || packet_302 == NULL)
        return -1;
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

/*======================================================================
 * function name： decoding_eps_steering
 * parameter：
 *      const can_frame_packet_t *packet_703 [IN] : the can packet of 703
 *      eps_steering_control_t *recv_message  [OUT] : the states of EPS
 * descriptions:
 *      convert a raw can packet to the states of EPS(steering)
 * return：success --- 0, failure --- -1
 =====================================================================*/
int decoding_eps_steering(const can_frame_packet_t *packet_703, 
                          eps_steering_control_t *recv_message)
{
    if(packet_703 == NULL || recv_message == NULL)
        return -1;
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

/*======================================================================
 * function name： encoding_eps_steering
 * parameter：
 *      const eps_steering_feedback_t *send_message [IN] : the states of EPS
 *      can_frame_packet_t *packet_704  [OUT] : the can packet of 704
 * descriptions:
 *      convert  the states of EPS(steering) to a raw can packet
 * return：success --- 0, failure --- -1
 =====================================================================*/
int encoding_eps_steering(const eps_steering_feedback_t *send_message,
                          can_frame_packet_t *packet_704)
{
    if(send_message == NULL || packet_704 == NULL)
        return -1;
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

/*======================================================================
 * function name： decoding_idu_brake
 * parameter：
 *      const can_frame_packet_t *packet_131 [IN] : the can packet of 131
 *      idu_general_brake_status_t *recv_message [OUT] : the states of IDU
 * descriptions:
 *      convert a raw can packet to the states of IDU(brake)
 * return：success --- 0, failure --- -1
 =====================================================================*/
int decoding_idu_brake(const can_frame_packet_t *packet_131, 
                       idu_general_brake_status_t *recv_message)
{
    if(packet_131 == NULL || recv_message == NULL)
        return -1;
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

/*======================================================================
 * function name： encoding_ehb_brake
 * parameter：
 *      const ehb_general_brake_status_t *send_message [IN] : the states of EHB
 *      can_frame_packet_t *packet_132  [OUT] : the can packet of 132
 * descriptions:
 *      convert  the states of EHB(brake) to a raw can packet
 * return：success --- 0, failure --- -1
 =====================================================================*/
int encoding_ehb_brake(const ehb_general_brake_status_t *send_message,
                       can_frame_packet_t *packet_132)
{
    if(send_message == NULL || packet_132 == NULL)
        return -1;
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

/*======================================================================
 * function name： decoding_gacu_ctrl
 * parameter：
 *      const can_frame_packet_t *packet_gacu [IN] : the can packet of gacu
 *      acucan_ctrl_t *recv_message [OUT] : the states of acu can
 * descriptions:
 *      convert a raw can packet to the states of acu can
 * return：success --- 0, failure --- -1
 =====================================================================*/
int decoding_gacu_ctrl(const can_frame_packet_t *packet_gacu, 
                       acucan_ctrl_t *recv_message)
{
    if(packet_gacu == NULL || recv_message == NULL)
        return -1;
    if(packet_gacu->can_id != ACU_ID_CTRL) {
        debug("gacu ctrl packet error for control ...\n");
        debug("this is unknown packet in ctrl: ");
        printf_packet(packet_gacu);
        debug("\n");

        return -1;
    }
    memset(recv_message, 0, sizeof(acucan_ctrl_t));
    memcpy(recv_message, packet_gacu->data, 8);     //using memcpy assignment

    return 0;
}

/*======================================================================
 * function name： encoding_gacu_feedback
 * parameter：
 *      const acucan_feedback_t *packet_gacu_feedback [IN] : the states of gacu
 *     can_frame_packet_t *send_message  [OUT] : the can packet of feedback acu
 * descriptions:
 *      convert the states of feedback gacu to a raw can packet
 * return：success --- 0, failure --- -1
 =====================================================================*/
int encoding_gacu_feedback(const acucan_feedback_t *packet_gacu_feedback,
                       	   can_frame_packet_t *send_message)
{
    if(packet_gacu_feedback == NULL || send_message == NULL)
        return -1;
    if(send_message->can_id != ACU_ID_FDBK) {
        debug("gacu feedback packet error for control ...\n");
        debug("this is unknown packet in feedback: ");
        printf_packet(send_message);
        debug("\n");
        send_message->can_id = ACU_ID_FDBK;
    }
    memcpy(send_message->data, packet_gacu_feedback, 
    						sizeof(acucan_feedback_t));

    return 0;
}

/*======================================================================
 * function name： tcp_write_packet
 * parameter：
 *     const int conn [IN] : the file descriptor of established connection
 *     can_frame_packet_t *sendframe  [IN] : the can packet will be written
 * descriptions:
 *      write a raw can packet by tcp-ip socket
 * return：success --- the number of written, failure --- -1
 =====================================================================*/
static int tcp_write_packet(const int conn, can_frame_packet_t *sendframe)
{
    // check_sums(sendframe->data, 7);//the position of packet is different
    if(conn < 0 || sendframe == NULL)
        return -1;
    //convert local endian to network byte order
    sendframe->can_id = htons(sendframe->can_id);
    int res = write(conn, sendframe, sizeof(can_frame_packet_t));
    if(res < 0) {
        debug("write failed ");
        debug("should delete timers!\n");
        return -1;
    } 

    return res;
}

/*======================================================================
 * function name： tcp_read_packet
 * parameter：
 *     const int conn [IN] : the file descriptor of established connection
 *     can_frame_packet_t *recvframe  [OUT] : the can packet will be read
 * descriptions:
 *      read a raw can packet from tcp-ip socket
 * return：success --- the number of read, failure --- -1
 =====================================================================*/
static int tcp_read_packet(const int conn, can_frame_packet_t *recvframe)
{
    if(conn < 0 || recvframe == NULL)
        return -1;
    memset(recvframe, 0, sizeof(can_frame_packet_t));
    //blocking read data from socket(TCP-IP)
    int ret = read(conn, recvframe, sizeof(can_frame_packet_t));    
    if(ret < 0) {
        debug("read failed ");
        // close(conn);
        // thread_exit();
    }
    else if(ret == 0) {
        debug("peer close\n");
    } else {    //convert  network byte order to local endian
        recvframe->can_id = ntohs(recvframe->can_id);
    }

    return ret;
}

/*======================================================================
 * function name： can_write_packet
 * parameter：
 *     const int conn [IN] : the file descriptor of SocketCan
 *     can_frame_packet_t *sendframe  [IN] : the can packet will be written
 * descriptions:
 *      write a raw can packet by socket-can
 * return：success --- the number of written, failure --- -1
 =====================================================================*/
static int can_write_packet(const int conn, can_frame_packet_t *sendframe)
{
    if(conn < 0 || sendframe == NULL)
        return -1;
    linux_can_frame temp_can_frame;
    temp_can_frame.can_id = sendframe->can_id;
    temp_can_frame.can_dlc = 8;
    // corvet struct of Linux SocketCan to raw can packet
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

/*======================================================================
 * function name： can_read_packet
 * parameter：
 *     const int conn [IN] : the file descriptor of SocketCan
 *     can_frame_packet_t *recvframe  [OUT] : the can packet will be read
 * descriptions:
 *      read a raw can packet from SocketCan
 * return：success --- the number of read, failure --- -1
 =====================================================================*/
static int can_read_packet(const int conn, can_frame_packet_t *recvframe)
{
    if(conn < 0 || recvframe == NULL)
        return -1;
    linux_can_frame temp_can_frame;
    memset(&temp_can_frame, 0, sizeof(linux_can_frame));
    memset(recvframe, 0, sizeof(can_frame_packet_t));
    //blocking read data from socket-can
    int ret = read(conn, &temp_can_frame, sizeof(linux_can_frame));    
    if(ret < 0) {
        debug("read failed ");
        close(conn);
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

/*======================================================================
 * function name： write_n
 * parameter：
 *     const int fd [IN] : the file descriptor of serial
 *     unsigned char* pchar [IN] : the pointer of buffer will be written
 *     const int n [IN] : the number of chars will be written
 * descriptions:
 *     write n bytes char to serial
 * return：success --- the number of written, failure --- -1
 =====================================================================*/
static int write_n(const int fd, unsigned char* pchar, const int n) 
{
    if(pchar == NULL || n < 0)
        return -1;
    size_t  nleft = n;
    ssize_t  nread;
    // blocking write util to n bytes
    while (nleft > 0) {
        if ((nread = write(fd, pchar, nleft)) < 0) {
            if(errno == EAGAIN){
                continue;
            }
            printf("write_n error write n\n");
            if (nleft == n)
                return(-1); /* error, return -1 */
            else
                break;      /* error, return amount read so far */
        } else if (nread == 0) {
            continue;
        }
        nleft -= nread;
        pchar   += nread;
    }
    
    return(n - nleft);      /* return >= 0 */
}

/*======================================================================
 * function name： serial_write_packet
 * parameter：
 *     const int conn [IN] : the file descriptor of serial
 *     can_frame_packet_t *sendframe [IN] : the can packet will be written
 * descriptions:
 *     write a raw can packet by serial
 * return：success --- the number of written, failure --- -1
 =====================================================================*/
static int serial_write_packet(const int conn, can_frame_packet_t *sendframe)
{
    if(conn < 0 || sendframe == NULL)
        return -1;
    unsigned char write_buffer[16];
    memset(write_buffer, 0, sizeof(write_buffer));
    write_buffer[0] = (sendframe->can_id >> 8) & 0xff;
    write_buffer[1] = sendframe->can_id & 0xff;
    for(int i = 0; i < 8; ++i)
        write_buffer[2 + i] = sendframe->data[i]; 
    int res = write_n(conn, write_buffer, 10);  //only write 10 bytes
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
    } 

    return res;
}

/*======================================================================
 * function name： is_canid
 * parameter：
 *     const unsigned char* buff [IN] : the buffer of a 10 bytes data
 *     int index [IN] : the first address of can id
 * descriptions:
 *     check if it is valid can id of serial
 * return：success --- 1, failure --- 0
 =====================================================================*/
static inline int is_canid(const unsigned char* buff, int index) 
{
    if(buff == NULL || index < 0)
        return 0;
    if(index > 10) 
        printf("index too long err\n");
    unsigned short int can_id = buff[index] << 8 | buff[index + 1];
    if(can_id == SPEED_CONTROL ||
       can_id == STEER_CONTROL || 
       can_id == BRAKE_CONTROL ||
       can_id == ACU_ID_CTRL ||
       can_id == ACU_ID_CTRL_EXT) return 1;

    return 0;
}

/*======================================================================
 * function name： shift_to_canid
 * parameter：
 *     const unsigned char* buff [IN] : the buffer of a 10 bytes data
 * descriptions:
 *     to find a valid can_id form a 10 bytes buffer
 * return：success --- the first address of can id, failure --- -1 or 0
 =====================================================================*/
static int shift_to_canid(const unsigned char* buff) 
{
    if(buff == NULL)
        return -1;
    int index = -1;
    for(int i = 0; i < 10; ++i) {
        if(is_canid(buff, i)) {
            index = i;
            break;
        }
    }

    return index;
}

/*======================================================================
 * function name： readn
 * parameter：
 *     const int fd [IN] : the file descriptor of serial
 *     void *ptr [IN] : the pointer of buffer will be read
 *     const int n [IN] : the number of chars will be read
 * descriptions:
 *     read n bytes char from serial
 * return：success --- the number of read, failure --- -1
 =====================================================================*/
static ssize_t readn(const int fd, void *ptr, size_t n)
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
            continue;
        }
        nleft -= nread;
        ptr   += nread;
    }
    
    return (n - nleft);      /* return >= 0 */
}

/*======================================================================
 * function name： read_a_packet
 * parameter：
 *     const int serail_fd [IN] : the file descriptor of serial
 *     unsigned char* read_buffer [OUT] : the buffer will be read
 *     int n [IN] : the number to read
 * descriptions:
 *     read a raw can packet by serial --- 10 bytes
 * return：success --- the number of read, failure --- -1
 =====================================================================*/
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

/*======================================================================
 * function name： serial_read_packet
 * parameter：
 *     const int conn [IN] : the file descriptor of serial
 *     can_frame_packet_t *recvframe [OUT] : the raw can packet will be read
 * descriptions:
 *     read a raw can packet by serial
 * return：success --- the number of read, failure --- -1
 =====================================================================*/
static int serial_read_packet(const int conn, can_frame_packet_t *recvframe)
{
    if(conn < 0 || recvframe == NULL)
        return -1;
    unsigned char read_buffer[11];
    memset(read_buffer, 0, sizeof(read_buffer));
    int read_len = 0;
    int length = 0;
    
    read_len = read_a_packet(conn, read_buffer, sizeof(read_buffer) - 1);
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

/*======================================================================
 * function name： write_packet
 * parameter：
 *     const int conn [IN] : the file descriptor of established connection
 *     can_frame_packet_t *sendframe [IN] : the raw can packet to write
 * descriptions:
 *     write a raw can packet to serial, tcp-ip or socket-can
 * return：success --- the number of written, failure --- -1
 =====================================================================*/
int write_packet(const int conn, can_frame_packet_t *sendframe)
{
    int res;   //Three packet should be calculated separately.
    if(conn < 0 || sendframe == NULL) {
        return -1;
    }
    // check_sums(sendframe->data, 7);  
	if(select_protocol == serial_protocol)         //0
		res = serial_write_packet(conn, sendframe);
	else if(select_protocol == tcp_ip_protocol)    //1
		res = tcp_write_packet(conn, sendframe);
	else if(select_protocol == raw_can_protocol)   //2
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

/*======================================================================
 * function name： read_packet
 * parameter：
 *     const int conn [IN] : the file descriptor of established connection
 *     can_frame_packet_t *recvframe [OUT] : the raw can packet will be read
 * descriptions:
 *     read a raw can packet from serial, tcp-ip or socket-can
 * return：success --- the number of read, failure --- -1
 =====================================================================*/
int read_packet(const int conn, can_frame_packet_t *recvframe)
{
    int res;
    if(conn < 0 || recvframe == NULL) {
        return -1;
    }

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

/*======================================================================
 * function name： printf_packet
 * parameter：
 *    const can_frame_packet_t *frame_packet [IN] : the raw can packet 
 * descriptions:
 *     print the raw can packet
 * return：success --- the number of read, failure --- -1
 =====================================================================*/
int printf_packet(const can_frame_packet_t *frame_packet)
{
    debug("ID = 0X%X ", frame_packet->can_id); 
    debug("data: ");
    for(int i = 0; i < 8; ++i) 
        debug("0X%02x ", frame_packet->data[i]);

    return 0;
}

/*======================================================================
 * function name： can2car
 * parameter：
 *    car_status_t* car_status [OUT] : the car status from can packet
 *    const can_shared_use_t* srv_msg [IN] : message from can packet
 * descriptions:
 *     convert the message from can packet to the need car status
 * return：success --- 0, failure --- -1
 =====================================================================*/
// 这个函数会把收到的所有can包信息转换为车的姿态信息
int can2car(car_status_t* car_status,
            const can_shared_use_t* srv_msg)
{
    if(car_status == NULL || srv_msg == NULL)
        return -1;
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
    if(srv_msg->vehicle_brake_request) {    //valid of brake
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

/*======================================================================
 * function name： car2can
 * parameter：
 *    eps_steering_feedback_t* steer [OUT] : the car steering from car status
 *    vcu_speed_feedback_t* speed [OUT] : the car speed from car status
 *    ehb_general_brake_status_t* brake [OUT] : the car brake from car status
 *    const car_status_t* car_status [IN] : the current car status 
 * descriptions:
 *     convert the car status to 3 feedback can frame
 * return：success --- 0, failure --- -1
 =====================================================================*/
int car2can(eps_steering_feedback_t* steer,
            vcu_speed_feedback_t* speed,
            ehb_general_brake_status_t* brake,
            const car_status_t* car_status)
{
    if(steer == NULL ||
       speed == NULL ||
       brake == NULL ||
       car_status == NULL)
        return -1;
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

/*======================================================================
 * function name： printf_car_status
 * parameter：
 *    const car_status_t* car_status [IN] : the car status will be printf
 * descriptions:
 *     printf the car status: velocity, acceleration, steer angle, 
 *                            steer speed, torque, shift
 * return：success --- 0, failure --- -1
 =====================================================================*/
void printf_car_status(const car_status_t* car_status)
{
    printf("car_status: ");
    printf("v = %f, a = %f, sa = %f, ss = %f, st = %f, sh = %d", \
            car_status->car_velocity, car_status->car_acceleration,
            car_status->car_steer_angle, car_status->car_steer_speed, 
            car_status->car_steer_torque, car_status->car_drive_shift);
}

