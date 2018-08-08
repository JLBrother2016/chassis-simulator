/************************************************************************
*
*   file name：threadchassisgps.c
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

#include "threadchassisgps.h"
#include "nmeagps.h"
#include "pthreadutil.h"
#include <signal.h>

extern int epollfd;
extern int gpspacketfd;
extern char gps_cmd_buffer[256];
extern global_locks_t global_locks;

shared_gps_data_t gps_shared_data;

/*======================================================================
 * function name： init_need_car_status
 * parameter：
 *      gga_data_t *read_gga [IN] : the struct data of GGA
 *      rmc_data_t *read_rmc [IN] : the struct data of RMC
 *      hdt_data_t *read_hdt [IN] : the struct data of HDT
 * descriptions:
 *      initialize the GPS start position of the car
 * return：success --- 0, failure --- -1
 =====================================================================*/
static int init_write(gga_data_t *read_gga,
			   		  rmc_data_t *read_rmc,
			   		  hdt_data_t *read_hdt)
 {
    char sentence_gga[] = "$GPGGA,103956.85,3036.99137803,N,12030.57125800,E,4,20,0.7,6.537,M,7.868,M,0.8,1688*7A\r\n";          //GPGGA
    char sentence_rmc[] = "$GPRMC,103956.95,A,3036.99137801,N,12030.57125765,E,0.03,218.53,270618,5.7,W,D*2A\r\n";               //GPRMC  
    char sentence_hdt[] = "$GPHDT,180.0,T*3C\r\n";    //GPHDT
    
    if(read_gga == NULL ||
       read_rmc == NULL ||
       read_hdt == NULL) {
        return -1;
    }

    read_gga_sentence(sentence_gga, read_gga);
    read_rmc_sentence(sentence_rmc, read_rmc);
    read_hdt_sentence(sentence_hdt, read_hdt);

    get_position(read_gga, read_rmc, read_hdt, &gps_shared_data);
    // using google earth coordinate
    gps_shared_data.latitude = 39.6545899883;   //initialize start latitude  
    gps_shared_data.longitude = 116.0989327955; //initialize start longitude 
    gps_shared_data.heading_in_degrees = 90;    //90 degrees toward the north

    return 0;
}

/*======================================================================
 * function name： handle_sigpipe
 * parameter：
 *      int sig [IN] : a signal(SIGPIPE) when gps connection 
 * descriptions:
 *      handle the signal when repeating writing
 * return： void
 =====================================================================*/
void handle_sigpipe(int sig) 
{
    printf("recv a sig = %d\n", sig);
    // SIGPIPE 13
    // if(sig == SIGPIPE)
    //     thread_exit();
}

/*======================================================================
 * function name： thread_chassis_gps
 * parameter：
 *     void *arg [IN] : the file descriptor of connection for gps
 * descriptions:
 *     the thread for update the status of gps, once every 1 millisecond
 * return：NULL
 =====================================================================*/
void* thread_chassis_gps(void *arg)
{
	gga_data_t read_gga;
    rmc_data_t read_rmc;
    hdt_data_t read_hdt;
    
    int connfd = *((int*) arg);
    init_write(&read_gga, &read_rmc, &read_hdt);

    // initialize the timer for sending GPS data packet初
    time_t timer_GPS_interval = 20;  //timer interval is 20ms
    struct itimerspec its_gps;
    gpspacketfd = create_timerfd(&its_gps, timer_GPS_interval);
    timer_param_t timer_gps_pm;
    timer_gps_pm.its = its_gps;
    timer_gps_pm.tfd = gpspacketfd;
    debug("gpspacketfd: %d\n", gpspacketfd);
    if(timerfd_settime(timer_gps_pm.tfd, TFD_TIMER_ABSTIME, 
                       &(timer_gps_pm.its), NULL) != 0) {
        ERR_EXIT("timerfd_settime()");
    }   //add the timer into epoll manage
    add_timerfd2epoll(epollfd, &timer_gps_pm);

    char cmd_buffer[256] = {0};

    signal(SIGPIPE, handle_sigpipe);    //registered signal handling function
    time_caculation(&gps_shared_data.date, &gps_shared_data.time);
    set_position(&gps_shared_data, &read_gga, &read_rmc, &read_hdt);
    get_gps_cmd(cmd_buffer, sizeof(cmd_buffer),
                &read_gga, &read_rmc, &read_hdt);
    
    int ret = write(connfd, cmd_buffer, strlen(cmd_buffer));
    if(ret < 0) {
        debug("thread gps write %d error! ", connfd);
        thread_exit();
    } 


    while(1) {
        // add a lock for gps shared data
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




