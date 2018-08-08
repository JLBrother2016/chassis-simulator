/************************************************************************
*
*   file name：pthreadutil.c
*
*   file description：package thread related functions.
*
*   creator： Jiang Long, 2018.08.07
*
*   version：0.0.1
*
*   Modify record：
*
************************************************************************/

#include "pthreadutil.h"

/*======================================================================
 * function name： thread_start
 * parameter：
 *      pthread_t *tid [IN] : thread id
 *      void *(*func) (void *) [IN] : function pointer
 *      void *arg [IN] : argument of the fuction
 * descriptions:
 *      to create a thread
 * return：success --- 0, failure --- -1
 =====================================================================*/
int thread_start(pthread_t *tid, void *(*func) (void *), void *arg)
{
    if(func == NULL || tid == NULL)
        return -1;
    int res = pthread_create(tid, NULL, func, arg);
    if(res != 0) {
        handle_error_en(res, "pthread create");
    }

    return 0;
}

/*======================================================================
 * function name： wait_thread
 * parameter：
 *      pthread_t *tid [IN] : thread id
 * descriptions:
 *      wait a thread
 * return：success --- 0, failure --- -1
 =====================================================================*/
int wait_thread(pthread_t *tid)
{
    if(tid == NULL)
        return -1;
    int s = pthread_join(*tid, NULL);
   if (s != 0)
       handle_error_en(s, "pthread_join");

    return 0;
}

/*======================================================================
 * function name： thread_exit
 * parameter：
 *      void
 * descriptions:
 *      a thread exit by self
 * return：success --- 0, failure --- -1
 =====================================================================*/
int thread_exit(void)
{
    printf("ID:%lu exit\n", pthread_self());  
    pthread_exit((void *)0);

    return 0;
}

/*======================================================================
 * function name： thread_detach
 * parameter：
 *      pthread_t *tid [IN] : thread id
 * descriptions:
 *      detach from a thread
 * return：success --- 0, failure --- -1
 =====================================================================*/
// all resources are automatically released when the thread finishes running.
int thread_detach(pthread_t *tid)
{
    if(tid == NULL)
        return -1;
    printf("ID:%lu detach\n", *tid);
    int s = pthread_detach(*tid);
    if (s != 0)
       handle_error_en(s, "pthread_detach");

    return 0;
}

/*======================================================================
 * function name： thread_cancel
 * parameter：
 *      pthread_t *tid [IN] : thread id
 * descriptions:
 *      initiatively to cancel a thread
 * return：success --- 0, failure --- -1
 =====================================================================*/
int thread_cancel(pthread_t *tid)
{
    if(tid == NULL)
        return -1;
    printf("ID:%lu cancel\n", *tid);
    int s = pthread_cancel(*tid);
    if (s != 0)
       handle_error_en(s, "pthread_cancel");

    return 0;
}

/*======================================================================
 * function name： mutex_init
 * parameter：
 *      pthread_mutex_t *pmutex [IN] : the mutex pointer
 * descriptions:
 *      initial a mutex
 * return：success --- 0, failure --- -1
 =====================================================================*/
int mutex_init(pthread_mutex_t *pmutex)
{
    if(pmutex == NULL)
        return -1;
    int status;
    if ((status = pthread_mutex_init(pmutex, NULL)))
        handle_error_en(status, "pthread_mutex_init");

    return 0;
}
int mutex_lock(pthread_mutex_t *pmutex)
{
   return pthread_mutex_lock(pmutex); 
}
int mutex_unlock(pthread_mutex_t *pmutex)
{
    return pthread_mutex_unlock(pmutex);
}
int mutex_destroy(pthread_mutex_t *pmutex)
{
    int status;
    if ((status = pthread_mutex_destroy(pmutex)))
        handle_error_en(status, "pthread_mutex_destroy");

    return 0;
}

/*======================================================================
 * function name： condition_init
 * parameter：
 *      condition_t *cond [IN] : the struct pointer include mutex and condition
 * descriptions:
 *      initial a mutex and a condition
 * return：success --- 0, failure --- -1
 =====================================================================*/
// pthread_cond_wait() must be used with pthread_mutex
// initial the condition --> pcond and pmutex
int condition_init(condition_t *cond)
{
    if(cond == NULL)
        return -1;
    int status;
    if ((status = pthread_mutex_init(&cond->pmutex, NULL)))
        handle_error_en(status, "pthread_mutex_init");

    if ((status = pthread_cond_init(&cond->pcond, NULL)))
        handle_error_en(status, "pthread_cond_init");

    return 0;
}
//lock the mutex of the condition
int condition_lock(condition_t *cond)
{
    return pthread_mutex_lock(&cond->pmutex);
}
//unlock the mutex of the condition
int condition_unlock(condition_t *cond)
{
    return pthread_mutex_unlock(&cond->pmutex);
}
//set blocking wait on the condition
int condition_blocking_wait(condition_t *cond)
{
    return pthread_cond_wait(&cond->pcond, &cond->pmutex);
}
//set time out wait on the condition, 15s
int condition_wait(condition_t *cond)
{
    struct timeval now;
    struct timespec timeout;
    int retcode = 0;
    gettimeofday(&now, NULL);
    timeout.tv_sec = now.tv_sec + 15;
    timeout.tv_nsec = now.tv_usec * 1000;

    retcode = pthread_cond_timedwait(&cond->pcond, &cond->pmutex, &timeout);
    if(retcode == ETIMEDOUT) {
        debug("condition wait time out ");
    }

    return retcode;
}
////set time of time out wait on the condition
int condition_timedwait(condition_t *cond, const struct timespec *abstime)
{
    return pthread_cond_timedwait(&cond->pcond, &cond->pmutex, abstime);
}
//send a notify to a condition
int condition_signal(condition_t *cond)
{
    return pthread_cond_signal(&cond->pcond);
}
//broadcast to all waiting thread
int condition_broadcast(condition_t* cond)
{
    return pthread_cond_broadcast(&cond->pcond);
}

/*======================================================================
 * function name： condition_destroy
 * parameter：
 *      condition_t* cond [IN] : the struct pointer include mutex and condition
 * descriptions:
 *      destroy a mutex and a condition
 * return：success --- 0, failure --- -1
 =====================================================================*/
int condition_destroy(condition_t* cond)
{
    if(cond == NULL)
        return -1;
    int status;
    if ((status = pthread_mutex_destroy(&cond->pmutex)))
        handle_error_en(status, "pthread_mutex_destroy");

    if ((status = pthread_cond_destroy(&cond->pcond)))
        handle_error_en(status, "pthread_cond_destroy");

    return 0;
}

/*======================================================================
 * function name： global_locks_init
 * parameter：
 *      global_locks_t *locks [IN] : the global three locks and a condition
 * descriptions:
 *      initial the global three locks and a condition
 * return：success --- 0, failure --- -1
 =====================================================================*/
int global_locks_init(global_locks_t *locks)
{
    if(locks == NULL)
        return -1;
    int status;
    if ((status = pthread_mutex_init(&locks->can_mutex, NULL)))
        handle_error_en(status, "pthread_mutex_init");
    if ((status = pthread_mutex_init(&locks->gps_mutex, NULL)))
        handle_error_en(status, "pthread_mutex_init");
    if ((status = pthread_mutex_init(&locks->car_mutex, NULL)))
        handle_error_en(status, "pthread_mutex_init");

    if ((status = pthread_cond_init(&locks->global_cond, NULL)))
        handle_error_en(status, "pthread_cond_init");

    return 0;
}

/*======================================================================
 * function name： global_locks_destroy
 * parameter：
 *      global_locks_t *locks [IN] : the global three locks and a condition
 * descriptions:
 *      destroy the global three locks and a condition
 * return：success --- 0, failure --- -1
 =====================================================================*/
int global_locks_destroy(global_locks_t *locks)
{
    if(locks == NULL)
        return -1;
    int status;
    if ((status = pthread_mutex_destroy(&locks->can_mutex)))
        handle_error_en(status, "pthread_mutex_destroy");
    if ((status = pthread_mutex_destroy(&locks->gps_mutex)))
        handle_error_en(status, "pthread_mutex_destroy");
    if ((status = pthread_mutex_destroy(&locks->car_mutex)))
        handle_error_en(status, "pthread_mutex_destroy");

    if ((status = pthread_cond_destroy(&locks->global_cond)))
        handle_error_en(status, "pthread_cond_destroy");

    return 0;
}

/*======================================================================
 * function name： set_nonblock
 * parameter：
 *      int fd [IN] : a file descriptor 
 * descriptions:
 *      set a file descriptor to nonblocking
 * return：success --- 0, failure --- -1
 =====================================================================*/
static int set_nonblock(int fd)
{
    if(fd < 0)
        return -1;
    int old_flags = fcntl(fd, F_GETFL);
    if(old_flags == -1)
        ERR_EXIT("fcntl F_GETFL");
    if(fcntl(fd, F_SETFL, old_flags | O_NONBLOCK) == -1)
        ERR_EXIT("fcntl F_SETFL");
    return old_flags;
}

/*======================================================================
 * function name： addfd_to_epoll
 * parameter：
 *      int epollfd [IN] : a file descriptor of epoll
 *      int* fd [IN] : a file descriptor will be added epoll
 *      int epoll_type [IN] : the tppe of epoll (EPOLLET or EPOLLLT) 
 *      int block_type [IN] : epoll is blocking  
 * descriptions:
 *      add a file descriptor(fd) to epoll mechanism
 * return：success --- 0, failure --- -1
 =====================================================================*/
static int addfd_to_epoll(int epollfd, int* fd, 
                          int epoll_type, int block_type)
{
    if(fd == NULL)
        return -1;
    struct epoll_event ep_event;
    memset(&ep_event, 0, sizeof(ep_event));
    ep_event.events = EPOLLIN;
    ep_event.data.ptr = (void *)fd;
    /* 如果是ET模式，设置EPOLLET */
    if (epoll_type == EPOLL_ET)
        ep_event.events |= EPOLLET;
    /* 设置是否阻塞 */
    if (block_type == FD_NONBLOCK)
        set_nonblock(*fd);
    if(epoll_ctl(epollfd, EPOLL_CTL_ADD, *fd, &ep_event) == -1) {
        close(*fd);
        ERR_EXIT("epoll_ctl add");
    }

    return 0;
}

/*======================================================================
 * function name： create_epollfd
 * parameter：
 *      void
 * descriptions:
 *      create a file descriptor of epoll mechanism
 * return：the file descriptor
 =====================================================================*/
int create_epollfd(void)
{
    int epollfd = epoll_create1(0);
    if(epollfd == -1)
        ERR_EXIT("epoll_create1");

    return epollfd;
}

/*======================================================================
 * function name： epoll_wait_conn
 * parameter：
 *      int epollfd [IN] ： the file descriptor of epoll
 *      struct epoll_event *events [IN] : epoll event
 * descriptions:
 *      create a file descriptor of epoll mechanism
 * return：the number of active events
 =====================================================================*/
int epoll_wait_conn(int epollfd, struct epoll_event *events)
{   
    if(events == NULL)
        return -1;
    //-1 is busy waiting
    int number = epoll_wait(epollfd, events, MAX_EPOLL_EVENTS, -1);
    if (number < 0)       //every time will wait for the connect fd
        handle_error_en(number, "epoll_wait");

    return number;
}

void addfd2epoll(int epollfd, int* fd)
{
    addfd_to_epoll(epollfd, fd, EPOLL_LT, FD_NONBLOCK);
}

int delfd4epoll(int epollfd, int fd)
{
    if(epoll_ctl(epollfd, EPOLL_CTL_DEL, fd, NULL) == -1)
        ERR_EXIT("epoll_ctl del");

    close(fd);      //close the file descriptor

    return 0;
}

static int movefd4epoll(int epollfd, int fd)
{   //only move the file descriptor from epoll deque
    if(epoll_ctl(epollfd, EPOLL_CTL_DEL, fd, NULL) == -1)
        debug("epoll_ctl move \n"); 

    return 0;
}

/*======================================================================
 * function name： create_timerfd
 * parameter：
 *      struct itimerspec *its [IN] ： timer struct
 *      time_t interval [IN] : timer interval
 * descriptions:
 *      create a ms-level timer of file descriptor and can be managed by epoll
 * return：the file descriptor of new timer
 =====================================================================*/
int create_timerfd(struct itimerspec *its, time_t interval)
{
    if(its == NULL)
        return -1;
    int tfd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK);
    if(tfd < 0){
        handle_error_en(tfd, "timerfd_create fail");
    }
    struct timespec nw;
    if(clock_gettime(CLOCK_MONOTONIC, &nw) != 0){
        ERR_EXIT("clock_gettime");
    }
    its->it_value.tv_sec = nw.tv_sec;// + interval;
    its->it_value.tv_nsec = nw.tv_nsec + interval * 1000 * 1000;//0;
    its->it_interval.tv_sec = 0;//interval;
    its->it_interval.tv_nsec = interval * 1000 * 1000;//0;
    if(its->it_value.tv_nsec >= 1000*1000*1000){
        ++its->it_value.tv_sec;
        its->it_value.tv_nsec -= 1000*1000*1000;
    }

    return tfd;
}

/*======================================================================
 * function name： add_timerfd2epoll
 * parameter：
 *      const int epoll_fd [IN] ： the file descriptor of a timer
 *      timer_param_t* ptimer_pm [IN] : timer struct
 * descriptions:
 *      add the file descriptora of ms-level timer to epoll
 * return：success --- 0, failure --- -1
 =====================================================================*/
int add_timerfd2epoll(const int epoll_fd,  
                      timer_param_t* ptimer_pm)
{
    if(ptimer_pm == NULL)
        return -1;
    struct epoll_event ev;
    ev.events = EPOLLIN | EPOLLET;
    ev.data.ptr = ptimer_pm;        
    //struct epoll_event is union
    if(epoll_ctl(epoll_fd, EPOLL_CTL_ADD, ptimer_pm->tfd, &ev) != 0){
        ERR_EXIT("epoll_ctl() error");
    }

    return 0;
}

/*======================================================================
 * function name： update4timer
 * parameter：
 *      const int epoll_fd [IN] ： the file descriptor of epoll manager
 *      const int timer_fd [IN] : the file descriptor of a timer
 *      struct epoll_event* timer_event [IN] : timer out event of epoll 
 * descriptions:
 *      add the file descriptora of ms-level timer to epoll
 * return：success --- 0, failure --- -1
 =====================================================================*/
int update4timer(const int epoll_fd,
                 const int timer_fd,
                 struct epoll_event* timer_event)
{
    if(timer_event == NULL)
        return -1;
    timer_param_t *pm = (timer_param_t *)(timer_event->data.ptr);
    movefd4epoll(epoll_fd, timer_fd);   //移除文件描述符
    //更新定时器事件和时间
    pm->its.it_value.tv_sec = pm->its.it_value.tv_sec 
                            + pm->its.it_interval.tv_sec;
    pm->its.it_value.tv_nsec = pm->its.it_value.tv_nsec 
                             + pm->its.it_interval.tv_nsec;
    if(pm->its.it_value.tv_nsec >= 1000*1000*1000){
        ++pm->its.it_value.tv_sec;
        pm->its.it_value.tv_nsec -= 1000*1000*1000;
    }
    if(timerfd_settime(pm->tfd, TFD_TIMER_ABSTIME, &(pm->its), NULL) != 0){
        ERR_EXIT("timerfd_settime");
    }
    add_timerfd2epoll(epoll_fd, pm);

    return 0;
}

