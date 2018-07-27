#ifndef _INITIAL_SRV_H
#define _INITIAL_SRV_H

#ifdef __cplusplus
extern "C" {
#endif

#include <sys/types.h>          /* See NOTES */
#include <sys/socket.h>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "utilconfig.h"

int initialize_link_4sorver(const char *ip, const int port_number);
int initialize_link_4client(const char *ip, const int port_number);
int connect2srv(int listenfd);
int initialize_link_4canbus(const char* can_name);
int initialize_link_4serial(const char* serial_portname);

#ifdef __cplusplus
}
#endif

#endif //_INITIAL_SRV_H
