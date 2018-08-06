#include "pthreadutil.h"

// 创建一个线程
int thread_start(pthread_t *tid, void *(*func) (void *), void *arg)
{
    int res = pthread_create(tid, NULL, func, arg);
    if(res != 0) {
        handle_error_en(res, "pthread create");
    }

    return 0;
}

int wait_thread(pthread_t *tid)
{
    int s = pthread_join(*tid, NULL);
   if (s != 0)
       handle_error_en(s, "pthread_join");

    return 0;
}

int thread_exit(void)
{
    printf("ID:%lu exit\n", pthread_self());  
    pthread_exit((void *)0);

    return 0;
}
// 该线程运行结束后会自动释放所有资源
int thread_detach(pthread_t *tid)
{
    printf("ID:%lu detach\n", *tid);
    int s = pthread_detach(*tid);
    if (s != 0)
       handle_error_en(s, "pthread_detach");

    return 0;
}

int thread_cancel(pthread_t *tid)
{
    printf("ID:%lu cancel\n", *tid);
    int s = pthread_cancel(*tid);
    if (s != 0)
       handle_error_en(s, "pthread_cancel");

    return 0;
}

int mutex_init(pthread_mutex_t *pmutex)
{
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

// pthread_cond_wait() 必须与pthread_mutex 配套使用
//初使化条件变量，可想而知是对互斥锁和条件变量进行初始化
int condition_init(condition_t *cond)
{
    int status;
    if ((status = pthread_mutex_init(&cond->pmutex, NULL)))
        handle_error_en(status, "pthread_mutex_init");

    if ((status = pthread_cond_init(&cond->pcond, NULL)))
        handle_error_en(status, "pthread_cond_init");

    return 0;
}
//对互斥锁进行锁定
int condition_lock(condition_t *cond)
{
    return pthread_mutex_lock(&cond->pmutex);
}
//对互斥锁进行解锁
int condition_unlock(condition_t *cond)
{
    return pthread_mutex_unlock(&cond->pmutex);
}
//在条件变量上阻塞等待条件
int condition_blocking_wait(condition_t *cond)
{
    return pthread_cond_wait(&cond->pcond, &cond->pmutex);
}
//在条件变量上等待条件
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
        // condition_unlock(&shared_information.g_cond);
        debug("condition wait time out ");

        // delfd4epoll(epollfd, packet302fd);
        // delfd4epoll(epollfd, packet704fd);

        // thread_exit();
    }

    return retcode;
    // return pthread_cond_wait(&cond->pcond, &cond->pmutex);
}
//具有超时功能的等待功能
int condition_timedwait(condition_t *cond, const struct timespec *abstime)
{
    return pthread_cond_timedwait(&cond->pcond, &cond->pmutex, abstime);
}
//向等待线程发起一个通知
int condition_signal(condition_t *cond)
{
    return pthread_cond_signal(&cond->pcond);
}
//向等待线程发起广播
int condition_broadcast(condition_t* cond)
{
    return pthread_cond_broadcast(&cond->pcond);
}
//销毁条件变量
int condition_destroy(condition_t* cond)
{
    int status;
    if ((status = pthread_mutex_destroy(&cond->pmutex)))
        handle_error_en(status, "pthread_mutex_destroy");

    if ((status = pthread_cond_destroy(&cond->pcond)))
        handle_error_en(status, "pthread_cond_destroy");

    return 0;
}

int global_locks_init(global_locks_t *locks)
{
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

int global_locks_destroy(global_locks_t *locks)
{
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

/* 设置文件为非阻塞 */
static int set_nonblock(int fd)
{
    int old_flags = fcntl(fd, F_GETFL);
    if(old_flags == -1)
        ERR_EXIT("fcntl F_GETFL");
    if(fcntl(fd, F_SETFL, old_flags | O_NONBLOCK) == -1)
        ERR_EXIT("fcntl F_SETFL");
    return old_flags;
}

/* 注册文件描述符到epoll，并设置其事件为EPOLLIN(可读事件) */
static void addfd_to_epoll(int epollfd, int* fd, 
                           int epoll_type, int block_type)
{
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
}

int create_epollfd(void)
{
    int epollfd = epoll_create1(0);
    if(epollfd == -1)
        ERR_EXIT("epoll_create1");

    return epollfd;
}

int epoll_wait_conn(int epollfd, struct epoll_event *events)
{   //-1是忙等
    int number = epoll_wait(epollfd, events, MAX_EPOLL_EVENTS, -1);
    if (number < 0)       //每次在等待的connect fd中轮询
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

    close(fd);      //关闭文件描述符

    return 0;
}

static int movefd4epoll(int epollfd, int fd)
{
    if(epoll_ctl(epollfd, EPOLL_CTL_DEL, fd, NULL) == -1)
        debug("epoll_ctl move \n"); //只从epoll队列中移除文件描述符

    return 0;
}

// 创建一个ms级的定时器，返回的是文件描述符，可以用epoll来进行管理
int create_timerfd(struct itimerspec *its, time_t interval)
{
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

int add_timerfd2epoll(const int epoll_fd,  
                       timer_param_t* ptimer_pm)
{
    struct epoll_event ev;
    ev.events = EPOLLIN | EPOLLET;
    ev.data.ptr = ptimer_pm;        
    //struct epoll_event 被定义为了一个联合体
    // ev.data.fd = ptimer_pm->tfd;
    if(epoll_ctl(epoll_fd, EPOLL_CTL_ADD, ptimer_pm->tfd, &ev) != 0){
        ERR_EXIT("epoll_ctl() error");
    }

    
    return 0;
}

int update4timer(const int epoll_fd,
                 const int timer_fd,
                 struct epoll_event* timer_event)
{
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

