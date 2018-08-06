#include "threadchassisgps.h"
#include "nmeagps.h"
#include "pthreadutil.h"
#include <signal.h>

extern int epollfd;
extern int gpspacketfd;
extern char gps_cmd_buffer[256];
extern global_locks_t global_locks;

shared_gps_data_t gps_shared_data;

static int init_write(gga_data_t *read_gga,
			   		  rmc_data_t *read_rmc,
			   		  hdt_data_t *read_hdt)
 {
 	char sentence_gga[] = "$GPGGA,103956.85,3036.99137803,N,12030.57125800,E,4,20,0.7,6.537,M,7.868,M,0.8,1688*7A\r\n"; //GPGGA
 	//GPRMC, 多的最后一位是什么意思呢？,S已被省略
    char sentence_rmc[] = "$GPRMC,103956.95,A,3036.99137801,N,12030.57125765,E,0.03,218.53,270618,5.7,W,D*2A\r\n";   
    // char sentence_hdt[] = "$GPHDT,174.050,T*32\r\n";
    //航向角为180度应该指向的是正南方向
    char sentence_hdt[] = "$GPHDT,180.0,T*3C\r\n";
    // // 谷歌地图：39.6556100069,116.1068131256 公司经纬度 
    // 谷歌地球：39.6542389362,116.1005206633
    // 北纬N39°39′15.26″ 东经E116°06′1.87″

    // 谷歌地图：39.6559610590,116.1052252578
    // 百度地图：39.6616399057,116.1118417455
    // 腾讯高德：39.6559563183,116.1052161455
    // 图吧地图：39.6561799883,116.0970827955
    // 谷歌地球：39.6545899883,116.0989327955
    // 北纬N39°39′16.52″ 东经E116°05′56.16″

    // 可用坐标
    // 谷歌地球：39.6545899883,116.0989327955   //用的是谷歌地球的坐标

    // 在地图中的点
    read_gga_sentence(sentence_gga, read_gga);
    read_rmc_sentence(sentence_rmc, read_rmc);
    read_hdt_sentence(sentence_hdt, read_hdt);

    get_position(read_gga, read_rmc, read_hdt, &gps_shared_data);

    //初始化经度 , 这个坐标刚开始还可以呀
    // gps_information.latitude = 39.6542389362;   
    // gps_information.longitude = 116.1005206633; //初始化纬度, 有地图
    gps_shared_data.latitude = 39.6545899883;   
    gps_shared_data.longitude = 116.0989327955; //初始化纬度, 有地图
    // gps_information.latitude = 45;   
    // gps_information.longitude = 45; //初始化纬度, 有地图
    // gps_information.heading_in_degrees = 0;  //0度朝向是西
    // gps_information.heading_in_degrees = 180; //180度朝向是东
    gps_shared_data.heading_in_degrees = 90; //90度朝向是北
    // gps_information.heading_in_degrees = 270; //270度朝向是南
    // bearing_in_degrees = gps_information.heading_in_degrees - 90;

    return 0;
}

void handle_sigpipe(int sig) 
{
    printf("recv a sig = %d\n", sig);
    // SIGPIPE 13 A 管道破裂: 写一个没有读端口的管道
    // if(sig == SIGPIPE)
    //     thread_exit();
}

void* thread_chassis_gps(void *arg)
{
	gga_data_t read_gga;
    rmc_data_t read_rmc;
    hdt_data_t read_hdt;
    
    int connfd = *((int*) arg);
    init_write(&read_gga, &read_rmc, &read_hdt);
    // printf_gga(&read_gga);

    // 初始化定时器发GPS的包
    time_t timer_GPS_interval = 20;  //GPS要延时50ms
    struct itimerspec its_gps;
    gpspacketfd = create_timerfd(&its_gps, timer_GPS_interval);
    timer_param_t timer_gps_pm;
    timer_gps_pm.its = its_gps;
    timer_gps_pm.tfd = gpspacketfd;
    debug("gpspacketfd: %d\n", gpspacketfd);
    if(timerfd_settime(timer_gps_pm.tfd, TFD_TIMER_ABSTIME, 
                       &(timer_gps_pm.its), NULL) != 0) {
        ERR_EXIT("timerfd_settime()");
    }   //将定时器的文件描述符加入到epoll事件集合中
    add_timerfd2epoll(epollfd, &timer_gps_pm);

    char cmd_buffer[256] = {0};

    signal(SIGPIPE, handle_sigpipe);    //信号处理函数
    time_caculation(&gps_shared_data.date, &gps_shared_data.time);
    set_position(&gps_shared_data, &read_gga, &read_rmc, &read_hdt);
    get_gps_cmd(cmd_buffer, sizeof(cmd_buffer),
                &read_gga, &read_rmc, &read_hdt);
    
    int ret = write(connfd, cmd_buffer, strlen(cmd_buffer));
    if(ret < 0) {
        debug("thread gps write %d error! ", connfd);
        
        // thread_cancel(&tid_caculate);
        thread_exit();
    } 


    while(1) {
        // 这里应该加一把计算gps坐标的锁
        mutex_lock(&global_locks.gps_mutex);
    	time_caculation(&gps_shared_data.date, &gps_shared_data.time);
    	set_position(&gps_shared_data, &read_gga, &read_rmc, &read_hdt);
        mutex_unlock(&global_locks.gps_mutex);

    	get_gps_cmd(cmd_buffer, sizeof(cmd_buffer),
    				&read_gga, &read_rmc, &read_hdt);
    	
        memcpy(gps_cmd_buffer, cmd_buffer, sizeof(cmd_buffer));
        // debug("gps_cmd_buffer: %s", gps_cmd_buffer);

        ms_sleep(0, 1);
    }

    return NULL;
}




