#include "utilconfig.h"
#include "pthreadutil.h"
#include "canpacket.h"
#include "initialsrv.h"

#include "threadchassiscan.h"
#include "threadvehicle.h"
#include "threadchassisgps.h"

// static const char can_listen_name[8] = "can1";
static const char serial_port_name[16] = "/dev/ttyUSB0";

global_locks_t global_locks;

can_frame_packet_t send704packet;
can_frame_packet_t send302packet;
char gps_cmd_buffer[256] = {0};

int epollfd;
int packet704fd;
int packet302fd;
int gpspacketfd;

int run_epoll(int listencanfd, int listengpsfd)
{
	struct epoll_event events[MAX_EPOLL_EVENTS];
	int number = 0;
	int *p_gps = malloc(sizeof(int));
    int *p_can = malloc(sizeof(int));
    int chassis_conn_fd = listencanfd;  //监听文件描述符也是连接文件描述符
    int gps_conn_fd = 0;
    int thread_count = 0;

    pthread_t tid_can, tid_vehicle, tid_gps;
    // *p = conn;
    while(1) {
        number = epoll_wait_conn(epollfd, events);
        if(number > 0) {
            for(int i = 0; i < number; ++i) {

                if (events[i].events & EPOLLIN) {
                    int* fdptr = (int*)(events[i].data.ptr);
                    int newfd = *fdptr;
                    // if(newfd == listencanfd) {
                    //     // int connfd = connect2srv(listencanfd); //创建新的连接
                    //     // *p = connfd;
                    //     // chassis_conn_fd = connfd;
                    //     // SocketCan不用连接，直接监听就可以了
                    //     *p = listencanfd;
                    //     chassis_conn_fd = listencanfd;
                    //     thread_start(&tid_can, &thread_chassis_can, p);
                    //     thread_start(&tid_vehicle, &thread_vehicle, p);
                    // } else if(newfd == listengpsfd) {
                    if(newfd == listengpsfd) {
                        int connfd = connect2srv(listengpsfd);
                        *p_gps = connfd;
                        gps_conn_fd = connfd;
                        debug("listengpsfd = %d, conngpsfd = %d\n", 
                               listengpsfd, connfd);
                        //起GPS的线程
                        thread_start(&tid_gps, &thread_chassis_gps, p_gps);
                        // 起底盘的线程
                        *p_can = listencanfd;
                        if(thread_count == 0) {
                            thread_start(&tid_can, &thread_chassis_can, p_can);
                            thread_start(&tid_vehicle, &thread_vehicle, p_can);
                        }
                        ++thread_count;

                    } else {   //data和ptr不能共存
                        // debug("this is timer event %d ", newfd);
                        struct epoll_event ev = events[i];
                        timer_param_t *pm = (timer_param_t *)(ev.data.ptr);
                        if(pm->tfd == packet302fd) {
                            // debug("epoll packet302fd ");
                           	//线程死了对应的定时器也会被清楚
                            if(is_file_descriptor(packet302fd)) {
                                update4timer(epollfd, packet302fd, &events[i]);
                                if(write_serial_packet(chassis_conn_fd, &send302packet) < 0 ) {
                                	delfd4epoll(epollfd, packet302fd);
                                	thread_cancel(&tid_vehicle);
                                }
                                // printf_packet(&send302packet);
                                // debug("\n");
                            }
                                
                        } else if(pm->tfd == packet704fd) {
                            // debug("epoll packet704fd ");
                            //线程死了对应的定时器也会被清楚
                            if(is_file_descriptor(packet704fd)) {
                                update4timer(epollfd, packet704fd, &events[i]);
                                if(write_serial_packet(chassis_conn_fd, &send704packet) < 0) {
                                	delfd4epoll(epollfd, packet704fd);
                                	thread_cancel(&tid_vehicle);
                                }
                                // printf_packet(&send704packet);
                                // debug("\n");
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
                                }
                            }
                        } else {
                            debug("this fd id %d not added, continue!\n", pm->tfd);
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

int main(int argc, char const *argv[])
{
    // if (argc < 3) {
    //     fprintf(stderr, "usage:%s ip_address gps_port_number chassis_can_port || ip_address gps_port_number, default:can1\n", argv[0]);
    //     exit(1);
    // }

	// int listencanfd = initialize_link_4sorver(argv[1], atoi(argv[2]));
    // int listengpsfd = initialize_link_4sorver(argv[1], atoi(argv[2]));
    // int listencanfd;
    // if(argc == 3)   //使用默认的can端口--can1
    //     listencanfd = initialize_link_4canbus(can_listen_name);
    // else    
    //     listencanfd = initialize_link_4canbus(argv[3]);
    if (argc < 3) {
        fprintf(stderr, "usage:%s ip_address gps_port_number, default:ttyUSB0\n", argv[0]);
        exit(1);
    }
    int listengpsfd = initialize_link_4sorver(argv[1], atoi(argv[2]));
    int listencanfd = initialize_link_4serial(serial_port_name);

    epollfd = create_epollfd();
    // addfd2epoll(epollfd, &listencanfd);  //将监听套接字加入epoll机制
    // SocketCan不能加入epoll机制，因为每次读写都会响应
    addfd2epoll(epollfd, &listengpsfd); 
    global_locks_init(&global_locks);
    debug("listencanfd:%d listengpsfd:%d\n", listencanfd, listengpsfd);

    // double time_val = get_time_stamp();
    run_epoll(listencanfd, listengpsfd);

    global_locks_destroy(&global_locks);
    close(listencanfd);
    close(listengpsfd);
    
    return 0;
}

