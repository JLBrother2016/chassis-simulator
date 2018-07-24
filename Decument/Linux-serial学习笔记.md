####Linux-serial学习笔记
[原链接来源于这个网页](http://www.faqs.org/docs/Linux-HOWTO/Serial-Programming-HOWTO.html)
1. Linux中的串口设备在`/dev/ttyUSB*`目录下，a raw device
2. 在`<asm/termbits.h>`头文件中定义了一个结构体 

    ```cpp
     #define NCCS 19
        struct termios {
            tcflag_t c_iflag;       /* input mode flags */  //用于读
            tcflag_t c_oflag;       /* output mode flags */ //用于写
            //c_cflag配置端口，波特率，每位字节数，停止位等
            tcflag_t c_cflag;       /* control mode flags */
            //c_lflag决定数据是否回环，信号是否被接收
            tcflag_t c_lflag;       /* local mode flags */
            cc_t c_line;            /* line discipline *///不被POSIX系统使用
            //c_cc主要是用来控制文件描述符的结尾，停止等
            cc_t c_cc[NCCS];        /* control characters */
    };
    ```
3. 一共有三种不同的输入的concepts(read)----不要循环读取单个字符以获取整个字符串
    * 规范输入处理(Canonical Input Processing)---文件结束符不会被认为是有效的数据，Canonical input processing can also handle the erase, delete word, and reprint characters, translate CR to NL, etc
    * 非规范输入处理(Non-Canonical Input Processing)---Non-Canonical Input Processing will handle a fixed amount of characters per read, and allows for a character timer. This mode should be used if your application will always read a fixed number of characters, or if the connected device sends bursts of characters.(可以处理固定字数的处理，和接收突发字符)
    * 异步输入(Asynchronous Input)---规范和非规范输入都可以用于同步和异步模式，默认是同步的，即读取数据时会堵塞；在异步模式下，read语句会立即返回，并在完成时向调用程序发送信号，信号可以由信号处理函数处理。
    * 有多个设备输入设备时，可以选用select来等待输入
4. 最大输入输出长度为255个字符, 一般serial port要超级用户才能打开，所以可以利用`chmod a+rw /dev/ttyUSB*`来改变正确的权限。
5. 规范输入的例子
    ```cpp
    #include <sys/types.h>
    #include <sys/stat.h>
    #include <fcntl.h>
    #include <termios.h>
    #include <stdio.h>
    /* baudrate settings are defined in <asm/termbits.h>, which is
    included by <termios.h> */
    #define BAUDRATE B38400            
    /* change this definition for the correct port */
    #define MODEMDEVICE "/dev/ttyS1"
    #define _POSIX_SOURCE 1 /* POSIX compliant source */

    #define FALSE 0
    #define TRUE 1

    volatile int STOP=FALSE; 

    main()
    {
      int fd,c, res;
      struct termios oldtio, newtio;
      char buf[255];
    /* 
      Open modem device for reading and writing and not as controlling tty
      because we don't want to get killed if linenoise sends CTRL-C.
    */  //将串口设置为non-controlling，这样就不会得到ctrl+c的结束数据
     fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY ); 
     if (fd <0) {perror(MODEMDEVICE); exit(-1); }
    
     tcgetattr(fd, &oldtio); /* save current serial port settings */
     bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */
    
    /* 
      BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
      CRTSCTS : output hardware flow control (only used if the cable has
                all necessary lines. See sect. 7 of Serial-HOWTO)
      CS8     : 8n1 (8bit,no parity,1 stopbit)  8位数据位, 没有优先级, 1位停止位
      CLOCAL  : local connection, no modem contol 无调制解调控制
      CREAD   : enable receiving characters 使能接收字符
    */
     newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
     
    /*
      IGNPAR  : ignore bytes with parity errors 忽略字节序的奇偶检验
      ICRNL   : map CR to NL (otherwise a CR input on the other computer
                will not terminate input)                    
      otherwise make device raw (no other input processing)
    */
     newtio.c_iflag = IGNPAR | ICRNL;
     
    /*
     Raw output.
    */
     newtio.c_oflag = 0;    //原始输出
     
    /*
      ICANON  : enable canonical input
      disable all echo functionality, and don't send signals to calling program
    */
     newtio.c_lflag = ICANON;   //使能规范输入
     
    /* 
      initialize all control characters 
      default values can be found in /usr/include/termios.h, and are given
      in the comments, but we don't need them here
    */
     newtio.c_cc[VINTR]    = 0;     /* Ctrl-c */    //控制字符
     newtio.c_cc[VQUIT]    = 0;     /* Ctrl-\ */
     newtio.c_cc[VERASE]   = 0;     /* del */
     newtio.c_cc[VKILL]    = 0;     /* @ */
     newtio.c_cc[VEOF]     = 4;     /* Ctrl-d */
     newtio.c_cc[VTIME]    = 0;     /* inter-character timer unused */
     newtio.c_cc[VMIN]     = 1;     /* blocking read until 1 character arrives */
     newtio.c_cc[VSWTC]    = 0;     /* '\0' */
     newtio.c_cc[VSTART]   = 0;     /* Ctrl-q */ 
     newtio.c_cc[VSTOP]    = 0;     /* Ctrl-s */
     newtio.c_cc[VSUSP]    = 0;     /* Ctrl-z */
     newtio.c_cc[VEOL]     = 0;     /* '\0' */
     newtio.c_cc[VREPRINT] = 0;     /* Ctrl-r */
     newtio.c_cc[VDISCARD] = 0;     /* Ctrl-u */
     newtio.c_cc[VWERASE]  = 0;     /* Ctrl-w */
     newtio.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
     newtio.c_cc[VEOL2]    = 0;     /* '\0' */
    
    /* 
      now clean the modem line and activate the settings for the port
    */
     tcflush(fd, TCIFLUSH);             //先把串口的缓存清除
     tcsetattr(fd, TCSANOW, &newtio);   //再将参数注册
    
    /*
      terminal settings done, now handle input
      In this example, inputting a 'z' at the beginning of a line will 
      exit the program.
    */
     while (STOP == FALSE) {     /* loop until we have a terminating condition */
     /* read blocks program execution until a line terminating character is 
        input, even if more than 255 chars are input. If the number
        of characters read is smaller than the number of chars available,
        subsequent reads will return the remaining chars. res will be set
        to the actual number of characters actually read */
        res = read(fd, buf, 255); 
        buf[res]=0;             /* set end of string, so we can printf */
        printf(":%s:%d\n", buf, res);
        if (buf[0]=='z') STOP=TRUE;
     }
     /* restore the old port settings */
     tcsetattr(fd, TCSANOW, &oldtio);
    }
    ```
6. 非规范输入模式(Non-Canonical Input Processing)，`c_cc[VTIME]` 设置超时时间,  `c_cc[VMIN]`设置最小读取的字符数
    * 情况1： If MIN > 0 and TIME = 0, MIN sets the number of characters to receive before the read is satisfied. As TIME is zero, the timer is not used.
    * 情况2： If MIN = 0 and TIME > 0, TIME serves as a timeout value. The read will be satisfied if a single character is read, or TIME is exceeded (t = TIME *0.1 s). If TIME is exceeded, no character will be returned.
    * 情况3： If MIN = 0 and TIME = 0, read will be satisfied immediately(立即读). The number of characters currently available, or the number of characters requested will be returned. According to Antonino (see contributions), you could issue a `fcntl(fd, F_SETFL, FNDELAY)`; before reading to get the same result.
    ```cpp
      #include <sys/types.h>
      #include <sys/stat.h>
      #include <fcntl.h>
      #include <termios.h>
      #include <stdio.h>
        
      #define BAUDRATE B38400
      #define MODEMDEVICE "/dev/ttyS1"
      #define _POSIX_SOURCE 1 /* POSIX compliant source */
      #define FALSE 0
      #define TRUE 1
        
      volatile int STOP=FALSE; 
       
      main()
      {
        int fd,c, res;
        struct termios oldtio,newtio;
        char buf[255];
        
        fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY ); 
        if (fd <0) {perror(MODEMDEVICE); exit(-1); }
        
        tcgetattr(fd,&oldtio); /* save current port settings */
        
        bzero(&newtio, sizeof(newtio));
        newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
        newtio.c_iflag = IGNPAR;
        newtio.c_oflag = 0;
        
        /* set input mode (non-canonical, no echo,...) */
        newtio.c_lflag = 0;
         
        newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
        newtio.c_cc[VMIN]     = 5;   /* blocking read until 5 chars received */
        
        tcflush(fd, TCIFLUSH);
        tcsetattr(fd,TCSANOW,&newtio);
        
        
        while (STOP==FALSE) {       /* loop for input */
          res = read(fd,buf,255);   /* returns after 5 chars have been input */
          buf[res]=0;               /* so we can printf... */
          printf(":%s:%d\n", buf, res);
          if (buf[0]=='z') STOP=TRUE;
        }
        tcsetattr(fd,TCSANOW,&oldtio);
      }

    ```
7. 异步输入模式的例子
    ```cpp
      #include <termios.h>
      #include <stdio.h>
      #include <unistd.h>
      #include <fcntl.h>
      #include <sys/signal.h>
      #include <sys/types.h>
        
      #define BAUDRATE B38400
      #define MODEMDEVICE "/dev/ttyS1"
      #define _POSIX_SOURCE 1 /* POSIX compliant source */
      #define FALSE 0
      #define TRUE 1
        
      volatile int STOP=FALSE; 
      //异步输入模式就是有一个信号处理函数
      void signal_handler_IO (int status);   /* definition of signal handler */
      int wait_flag=TRUE;                    /* TRUE while no signal received */
        
      main()
      {
        int fd,c, res;
        struct termios oldtio,newtio;
        struct sigaction saio;           /* definition of signal action */
        char buf[255];
        
        /* open the device to be non-blocking (read will return immediatly) */
        fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd <0) {perror(MODEMDEVICE); exit(-1); }
        
        /* install the signal handler before making the device asynchronous */
        saio.sa_handler = signal_handler_IO;
        saio.sa_mask = 0;
        saio.sa_flags = 0;
        saio.sa_restorer = NULL;
        sigaction(SIGIO, &saio, NULL);
          
        /* allow the process to receive SIGIO */
        fcntl(fd, F_SETOWN, getpid());
        /* Make the file descriptor asynchronous (the manual page says only 
           O_APPEND and O_NONBLOCK, will work with F_SETFL...) */
        fcntl(fd, F_SETFL, FASYNC);
        
        tcgetattr(fd,&oldtio); /* save current port settings */
        /* set new port settings for canonical input processing */
        newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
        newtio.c_iflag = IGNPAR | ICRNL;
        newtio.c_oflag = 0;
        newtio.c_lflag = ICANON;
        newtio.c_cc[VMIN]=1;
        newtio.c_cc[VTIME]=0;
        tcflush(fd, TCIFLUSH);
        tcsetattr(fd,TCSANOW,&newtio);
         
        /* loop while waiting for input. normally we would do something
           useful here */ 
        while (STOP==FALSE) {
          printf(".\n");usleep(100000);
          /* after receiving SIGIO, wait_flag = FALSE, input is available
             and can be read */
          if (wait_flag==FALSE) { 
            res = read(fd,buf,255);
            buf[res]=0;
            printf(":%s:%d\n", buf, res);
            if (res==1) STOP=TRUE; /* stop loop if only a CR was input */
            wait_flag = TRUE;      /* wait for new input */
          }
        }
        /* restore old port settings */
        tcsetattr(fd,TCSANOW,&oldtio);
      }
        
      /***************************************************************************
      * signal handler. sets wait_flag to FALSE, to indicate above loop that     *
      * characters have been received.                                           *
      ***************************************************************************/
        
      void signal_handler_IO (int status)
      {
        printf("received SIGIO signal.\n");
        wait_flag = FALSE;
      }
    ```
8. 基于select的IO复用的例子
    ```cpp
      #include <sys/time.h>
      #include <sys/types.h>
      #include <unistd.h>
        
      main()
      {
        int    fd1, fd2;  /* input sources 1 and 2 */
        fd_set readfs;    /* file descriptor set */
        int    maxfd;     /* maximum file desciptor used */
        int    loop=1;    /* loop while TRUE */ 
        
        /* open_input_source opens a device, sets the port correctly, and
           returns a file descriptor */
        fd1 = open_input_source("/dev/ttyS1");   /* COM2 */
        if (fd1<0) exit(0);
        fd2 = open_input_source("/dev/ttyS2");   /* COM3 */
        if (fd2<0) exit(0);
        maxfd = MAX (fd1, fd2)+1;  /* maximum bit entry (fd) to test */
        
        /* loop for input */
        while (loop) {
          FD_SET(fd1, &readfs);  /* set testing for source 1 */
          FD_SET(fd2, &readfs);  /* set testing for source 2 */
          /* block until input becomes available */
          select(maxfd, &readfs, NULL, NULL, NULL);
          if (FD_ISSET(fd1))         /* input from source 1 available */
            handle_input_from_source1();
          if (FD_ISSET(fd2))         /* input from source 2 available */
            handle_input_from_source2();
        }
      } 
        //slect设置读超时
        int res;
        struct timeval Timeout;

        /* set timeout value within input loop */
        Timeout.tv_usec = 0;  /* milliseconds */
        Timeout.tv_sec  = 1;  /* seconds */
        res = select(maxfd, &readfs, NULL, NULL, &Timeout);
        if (res==0)
        /* number of file descriptors with input = 0, timeout occurred. */
    ```