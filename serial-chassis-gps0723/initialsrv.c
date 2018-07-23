#include "initialsrv.h"
#include <termios.h>

int initialize_link_4sorver(const char *ip, const int port_number)
{
    // listenfd = socket(PF_INET, SOCK_STREAM, 0);
    struct  sockaddr_in servaddr;
    int listenfd, on = 1;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port_number);
    if(inet_pton(PF_INET, ip, &servaddr.sin_addr) < 0)
        ERR_EXIT("inet_pton failed");

    if((listenfd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0 )
        ERR_EXIT("socket failed");
    // servaddr.sin_addr.s_addr = htonl(INADDR_ANY);   //INADDR_ANY表示本机的任意地址, 网络字节序
    // servaddr.sin_addr.s_addr = inet_addr("127.0.0.1"); //显示指定本机的IP地址
    // inet_aton("127.0.0.1", &servaddr.sin_addr)
    //允许关闭后马上重启
    if(setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) < 0)  
        ERR_EXIT("setsockopt failed");
    if(bind(listenfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) < 0)
        ERR_EXIT("bind failed");
    if(listen(listenfd, SOMAXCONN) < 0)
        ERR_EXIT("listen failed");  // SOMAXCONN是队列允许的最大值

	return listenfd;
}

int initialize_link_4client(const char *ip, const int port_number)
{
    struct  sockaddr_in servaddr;
    int sock;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port_number);
    if(inet_pton(PF_INET, ip, &servaddr.sin_addr) < 0)
        ERR_EXIT("inet_pton failed");

    if((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0 )
        ERR_EXIT("socket failed");

    if(connect(sock, (struct sockaddr*)&servaddr, sizeof(servaddr)) < 0)
        ERR_EXIT("connect failed");

    return sock;
}

int connect2srv(int listenfd)
{
    struct sockaddr_in peeraddr;
    socklen_t peerlen = sizeof(peeraddr);  //peerlen一定要有初始值
    int conn;
    if((conn = accept(listenfd, (struct sockaddr*)&peeraddr, &peerlen)) < 0)
        ERR_EXIT("accept failed");
    printf("ip = %s port = %d\n", inet_ntoa(peeraddr.sin_addr),
                                  ntohs(peeraddr.sin_port));

    return conn;
}

int initialize_link_4canbus(const char* can_name)
{
    int family = PF_CAN, type = SOCK_RAW, proto = CAN_RAW;
    struct sockaddr_can addr;   //can的socket地址
    struct ifreq ifr;   //配置socket的ip
    int sock_can;

    printf("interface = %s, family = %d, type = %d, proto = %d\n",
           can_name, family, type, proto);

    if ((sock_can = socket(family, type, proto)) < 0) {
        perror("socket");   //创建can的类套接字
        return -1;
    }

    addr.can_family = family;
    strcpy(ifr.ifr_name, can_name);
    ioctl(sock_can, SIOCGIFINDEX, &ifr);
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock_can, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("can bind");
        return -1;
    }

    return sock_can;
}

static int set_interface_attribs (int fd, int speed, int parity) {
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0) {
        printf ("error %d from tcgetattr", errno);
        return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 11;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0) {
        printf ("error %d from tcsetattr", errno);
        return -1;
    }
    return 0;
}

static void set_blocking (int fd, int should_block) {
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0) {
        printf ("error %d from tggetattr", errno);
        return;
    }

    tty.c_cc[VMIN]  = should_block;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
        printf ("error %d setting term attributes", errno);
}

int initialize_link_4serial(const char* serial_portname)
{
    int serial_fd = open (serial_portname, O_RDWR | O_NOCTTY | O_SYNC);
    // int fd = open (portname, O_RDWR | O_NOCTTY);
    if (serial_fd < 0) {
        printf ("error %d opening %s: %s", errno, serial_portname, strerror (errno));
        return -1;
    }
    // set speed to 115,200 bps, 8n1 (no parity)
    set_interface_attribs (serial_fd, B115200, 0);  
    // set_blocking (fd, 0);                // set no blocking
    set_blocking (serial_fd, 11);                // set blocking read

    return serial_fd;
}
