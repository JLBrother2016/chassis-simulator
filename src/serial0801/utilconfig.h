#ifndef _UTIL_CONFIG_H
#define _UTIL_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdbool.h>
#include <math.h>
#include <pthread.h>
#include <ctype.h>

#include <sys/time.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/epoll.h>
#include <sys/timerfd.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <arpa/inet.h>
#include <net/if.h>

#include <linux/can.h>

// #define debug

#ifndef debug
#define debug printf
#endif //debug

#define ERR_EXIT(m) \
        do \
        {  \
            perror(m); \
            exit(EXIT_FAILURE); \
        } while(0)

#define handle_error_en(en, msg) \
       	do \
       	{  \
       		errno = en;\
       		perror(msg);\
       		exit(EXIT_FAILURE);\
       	} while (0)

typedef enum protocol {
  serial_protocol = 0,
  tcp_ip_protocol = 1,
  raw_can_protocol = 2,
} protocol_num;

//获取当前时间戳
static inline double get_time_stamp(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec * 1000 + (double)(tv.tv_usec)/1000.0);
}
//select做的延时函数
static inline int ms_sleep(int sec, int msec)
{
    struct timeval tv;
    tv.tv_sec = sec;
    tv.tv_usec = msec * 1000;

    select(0, NULL, NULL, NULL, &tv);

    return 0;
}
// 判断文件描述符是否有效
static inline bool is_file_descriptor(int fd)
{
    return !(-1 == fcntl(fd, F_GETFD, NULL) && errno == EBADF);
}

#ifdef __cplusplus
}
#endif

#endif //_UTIL_CONFIG_H
