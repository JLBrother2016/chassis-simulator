/************************************************************************
*
*   file name：threadepollsrv.c
*
*   file description：the thread to read raw can data.
*
*   creator： Jiang Long, 2018.08.08
*
*   version：0.0.1
*
*   Modify record：
*
************************************************************************/

#include "utilconfig.h"
#include "pthreadutil.h"
#include "canpacket.h"
#include "initialsrv.h"

#include "threadchassiscan.h"
#include "threadvehicle.h"
#include "threadchassisgps.h"

static const char can_listen_name[8] = "can1";
static const char serial_port_name[16] = "/dev/ttyUSB0";

global_locks_t global_locks;

can_frame_packet_t send704packet;
can_frame_packet_t send302packet;
can_frame_packet_t send132packet;
can_frame_packet_t send5A2packet;     //it would should be 50ms
char gps_cmd_buffer[256] = {0};

char enable_gacu = 0;

int epollfd;
int packet704fd;
int packet302fd;
int packet132fd;
int packet5A2fd;
int gpspacketfd;
int select_protocol;                  //0--serial; 1--tcp/ip; 2--raw can
uint8_t brake_rolling_counter = 0;
uint8_t gacu_rolling_counter = 0;

/*======================================================================
 * function name： run_epoll
 * parameter：
 *     int listencanfd [IN] : the listening file descriptor for raw can
 *     int listengpsfd [IN] : the listening file descriptor for gps command
 * descriptions:
 *     convert the raw can packet to the need status of car
 * return：success --- 0
 =====================================================================*/
int run_epoll(int listencanfd, int listengpsfd)
{
	struct epoll_event events[MAX_EPOLL_EVENTS];
	int number = 0;
	int *p_gps = malloc(sizeof(int));
    int *p_can = malloc(sizeof(int));
    //the listening file descriptor is also the connection file descriptor
    int chassis_conn_fd = listencanfd;  //serial and socket-can
    int gps_conn_fd = 0;
    int thread_count = 0;
    pthread_t tid_can, tid_vehicle, tid_gps;

    while(1) {
        number = epoll_wait_conn(epollfd, events);
        if(number > 0) {
            for(int i = 0; i < number; ++i) {
                if (events[i].events & EPOLLIN) {
                    int* fdptr = (int*)(events[i].data.ptr);
                    int newfd = *fdptr;
                    if(select_protocol == tcp_ip_protocol 
                       && newfd == listencanfd) {
                        //tcp-ip will create new connection
                        int connfd = connect2srv(listencanfd);
                        *p_can = connfd;
                        chassis_conn_fd = connfd;
                        thread_start(&tid_can, &thread_chassis_can, p_can);
                        thread_start(&tid_vehicle, &thread_vehicle, p_can);
                        brake_rolling_counter = 0;
                        gacu_rolling_counter = 0;
                    } else if(newfd == listengpsfd) {
                        int connfd = connect2srv(listengpsfd);
                        *p_gps = connfd;
                        gps_conn_fd = connfd;
                        debug("listengpsfd = %d, conngpsfd = %d\n", 
                               listengpsfd, connfd);
                        thread_start(&tid_gps, &thread_chassis_gps, p_gps);
                        // serial and socket-can need gps start vehicle thread
                        if(select_protocol != tcp_ip_protocol
                           && thread_count == 0) {
                            *p_can = listencanfd;
                            debug("listencanfd = %d\n", listencanfd);
                            thread_start(&tid_can, &thread_chassis_can, p_can);
                            thread_start(&tid_vehicle, &thread_vehicle, p_can);
                            brake_rolling_counter = 0;
                            gacu_rolling_counter = 0;
                            ++thread_count;
                        }
                    } else {   //data ptr cannot coexit, they are union
                        struct epoll_event ev = events[i];
                        timer_param_t *pm = (timer_param_t *)(ev.data.ptr);
                        if(pm->tfd == packet302fd) {
                            if(is_file_descriptor(packet302fd)) {
                                update4timer(epollfd, packet302fd, &events[i]);
                                if(write_packet(chassis_conn_fd, 
                                                &send302packet) < 0 ) {
                                    debug("write_packet 302!\n");
                                	delfd4epoll(epollfd, packet302fd);
                                    delfd4epoll(epollfd, packet132fd);
                                    delfd4epoll(epollfd, packet704fd);

                                    thread_cancel(&tid_vehicle);
                                    thread_cancel(&tid_can);

                                    close(chassis_conn_fd);
                                }
                            }        
                        } else if(pm->tfd == packet704fd) {
                            if(is_file_descriptor(packet704fd)) {
                                update4timer(epollfd, packet704fd, &events[i]);
                                check_sums(send704packet.data, 7);      
                                if(write_packet(chassis_conn_fd,
                                                &send704packet) < 0) {
                                    debug("write_packet 704!\n");
                                	delfd4epoll(epollfd, packet302fd);
                                    delfd4epoll(epollfd, packet132fd);
                                    delfd4epoll(epollfd, packet704fd);

                                    thread_cancel(&tid_vehicle);
                                    thread_cancel(&tid_can);

                                    close(chassis_conn_fd);
                                }
                            }        
                        } else if(pm->tfd == packet132fd) {
                            if(is_file_descriptor(packet132fd) && //tcp-ip no 
                               select_protocol != tcp_ip_protocol) {
                                update4timer(epollfd, packet132fd, &events[i]);
                                send132packet.data[6] = brake_rolling_counter;
                                ++brake_rolling_counter;
                                if(brake_rolling_counter > 15)
                                    brake_rolling_counter = 0;
                                check_sums(send132packet.data, 7);
                                if(write_packet(chassis_conn_fd,
                                                &send132packet) < 0) {
                                    debug("write_packet 302!\n");
                                    delfd4epoll(epollfd, packet302fd);
                                    delfd4epoll(epollfd, packet132fd);
                                    delfd4epoll(epollfd, packet704fd);

                                    thread_cancel(&tid_vehicle);
                                    thread_cancel(&tid_can);

                                    close(chassis_conn_fd);
                                }
                            }
                                
                        } else if(pm->tfd == gpspacketfd) {
                            if(is_file_descriptor(gpspacketfd)) {
                                update4timer(epollfd, gpspacketfd, &events[i]);
                                int ret = write(gps_conn_fd, gps_cmd_buffer,
                                                strlen(gps_cmd_buffer));
                                if(ret < 0) {
                                    debug("gps write error! ");
                                    delfd4epoll(epollfd, gpspacketfd);
                                    thread_cancel(&tid_gps);
                                    close(gps_conn_fd);

                                    if(thread_count) {
                                        if(enable_gacu == 0) {
                                            delfd4epoll(epollfd, packet302fd);
                                            delfd4epoll(epollfd, packet132fd);
                                            delfd4epoll(epollfd, packet704fd);
                                        } else if(enable_gacu == 1) {
                                            delfd4epoll(epollfd, packet5A2fd);
                                        }
                                        thread_cancel(&tid_vehicle);
                                        thread_cancel(&tid_can);
                                        thread_count = 0;
                                    }
                                }
                            }
                        } else if(pm->tfd == packet5A2fd) {
                            if(is_file_descriptor(packet5A2fd)) {
                                update4timer(epollfd, packet5A2fd, &events[i]);
                                send5A2packet.data[6] &= TAKEHIGH4BIT;
                                send5A2packet.data[6] |= gacu_rolling_counter;
                                ++gacu_rolling_counter;
                                if(gacu_rolling_counter > 15)
                                    gacu_rolling_counter = 0;
                                send5A2packet.data[7] = get_check_sums(
                                                    send5A2packet.data, 7);
                                if(write_packet(chassis_conn_fd,
                                                &send5A2packet) < 0) {
                                    debug("gacu write error!\n");
                                    delfd4epoll(epollfd, packet5A2fd);
                                    thread_cancel(&tid_vehicle);
                                    close(chassis_conn_fd);
                                }
                            }
                                
                        } else {
                            debug("this fd id %d not added, continue!\n",
                                                                 pm->tfd);
                            delfd4epoll(epollfd, pm->tfd);
                            continue;
                        }
                    }
                }
            }
        } else if(number == 0) {
            continue;
        } else {
            perror("epoll_wait_conn error");
            continue;
        }
    }

    wait_thread(&tid_can);
    wait_thread(&tid_vehicle);
    wait_thread(&tid_gps);

    free(p_gps);
    free(p_can);

	return 0;
}

/*======================================================================
 * function name： main
 * parameter：
 *     int argc [IN] : the number of argument command
 *     char const *argv[] [IN] : argument command
 * descriptions:
 *     convert the raw can packet to the need status of car
 * return：success --- 0, failure --- -1
 =====================================================================*/
int main(int argc, char const *argv[])
{
    if (argc < 4) {
        fprintf(stderr, "if use '-s', usage:%s ip_address gps_port_number, default:ttyUSB0\n", argv[0]);
        fprintf(stderr, "if use '-t', usage:%s ip_address gps_port_number can_port_number\n", argv[0]);
        fprintf(stderr, "if use '-c', usage:%s ip_address gps_port_number, default:can1\n", argv[0]);
        printf("please restart and right input attributes\n");
        exit(1);
    }
    epollfd = create_epollfd();
    int listencanfd = 0;
    int opt;
    while ((opt = getopt(argc, (char *const *)argv, "s::t::c::")) != -1) {
        switch (opt) {
            case 's':  //using serial for chassis raw can data
                printf("use serial to link!\n");
                if(optarg && (strcmp(optarg, "g") == 0 || 
                              strcmp(optarg, "G") == 0)) {
                    printf("usage gacu can packets!\n"); 
                    enable_gacu = 1;
                }
                listencanfd = initialize_link_4serial(serial_port_name);
                select_protocol = serial_protocol;
                break;
            case 't':  //using TCP-IP for chassis raw can data 
                printf("use tcp ip to link!\n");
                if(optarg && (strcmp(optarg, "g") == 0 || 
                              strcmp(optarg, "G") == 0)) {
                    printf("usage gacu can packets!\n"); 
                    enable_gacu = 1;
                }
                printf("ip addr:%s ", argv[2]);
                printf("can port number:%s\n", argv[4]);
                listencanfd = initialize_link_4sorver(argv[2], atoi(argv[4]));
                addfd2epoll(epollfd, &listencanfd);  
                select_protocol = tcp_ip_protocol;
                break;
            case 'c':  //using SocketCan for chassis raw can data
                printf("use raw can to link!\n");
                // printf("optopt = %c, optarg = %s\n", optopt, optarg);
                if(optarg && (strcmp(optarg, "g") == 0 || 
                              strcmp(optarg, "G") == 0)) {
                    printf("usage gacu can packets!\n"); 
                    enable_gacu = 1;
                }
                if(argc == 4)
                    listencanfd = initialize_link_4canbus(can_listen_name);
                else if(argc == 5)
                    listencanfd = initialize_link_4canbus(argv[4]);
                select_protocol = raw_can_protocol;
                break;
            case '?':
                printf("please see usage:%s\n", argv[0]);
                break;
            default:
                fprintf(stderr, "Unknown option %c\n", opt);
                printf("please restart and right input attributes\n");
                exit(1);
                break;
        }
    }

    printf("ip addr:%s ", argv[2]);
    printf("gps port number:%s\n", argv[3]);

    int listengpsfd = initialize_link_4sorver(argv[2], atoi(argv[3]));
    addfd2epoll(epollfd, &listengpsfd); 

    global_locks_init(&global_locks);

    debug("listencanfd:%d listengpsfd:%d\n", listencanfd, listengpsfd);
    debug("enable_gacu = %d\n", enable_gacu);

    run_epoll(listencanfd, listengpsfd);

    global_locks_destroy(&global_locks);
    close(listencanfd);
    close(listengpsfd);
    
    return 0;
}

