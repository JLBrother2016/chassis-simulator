#ifndef _PTHREAD_UTIL_H
#define _PTHREAD_UTIL_H

#ifdef __cplusplus
extern "C" {
#endif
#include "utilconfig.h"

/* 最大缓存区大小 */
#define MAX_BUFFER_SIZE 64
/* epoll最大监听数 */
#define MAX_EPOLL_EVENTS 20
/* LT模式 */
#define EPOLL_LT 0
/* ET模式 */
#define EPOLL_ET 1
/* 文件描述符设置阻塞 */
#define FD_BLOCK 0
/* 文件描述符设置非阻塞 */
#define FD_NONBLOCK 1

typedef struct condition {
    pthread_mutex_t pmutex;
    pthread_cond_t pcond;
} condition_t;

typedef struct global_locks {
    pthread_mutex_t can_mutex;
    pthread_mutex_t gps_mutex;
    pthread_mutex_t car_mutex;
    pthread_cond_t 	global_cond;
} global_locks_t;

typedef struct timer_param {
    struct itimerspec its;
    int tfd;
} timer_param_t;

int thread_start(pthread_t *tid, void *(*func) (void *), void *arg);
int wait_thread(pthread_t *tid);
int thread_exit(void);
int thread_detach(pthread_t *tid);
int thread_cancel(pthread_t *tid);

int mutex_init(pthread_mutex_t *pmutex);
int mutex_lock(pthread_mutex_t *pmutex);
int mutex_unlock(pthread_mutex_t *pmutex);
int mutex_destroy(pthread_mutex_t *pmutex);

int condition_init(condition_t *cond);
int condition_lock(condition_t *cond);
int condition_unlock(condition_t *cond);
int condition_blocking_wait(condition_t *cond);
int condition_wait(condition_t *cond);
int condition_timedwait(condition_t *cond, 
                        const struct timespec *abstime);
int condition_signal(condition_t *cond);
int condition_broadcast(condition_t* cond);
int condition_destroy(condition_t* cond);

int global_locks_init(global_locks_t *locks);
int global_locks_destroy(global_locks_t *locks);

int create_epollfd(void);
int epoll_wait_conn(int epollfd, struct epoll_event *events);
void addfd2epoll(int epollfd, int* fd);
int delfd4epoll(int epollfd, int fd);

int create_timerfd(struct itimerspec *its, time_t interval);
int add_timerfd2epoll(const int epollfd, 
                       timer_param_t* ptimer_pm);
int update4timer(const int epoll_fd,
                 const int timer_fd,
                 struct epoll_event* timer_event);

#ifdef __cplusplus
}
#endif

#endif //_PTHREAD_UTIL_H
