###socket-can-serial-chassis-simulator0727程序的使用
* 一共可以有三种不同的启动方式
1. serial 串口启动, `./threadepollsrv -s 192.168.100.246 5017`---默认使用的`"/dev/ttyUSB0"`设备---192.168.100.246是gps的ip地址，5017是GPS的端口号
2. tcp-ip 套接字启动, `./threadepollsrv -t 192.168.100.246 5017 5016`---192.168.100.246即是gps的ip地址也是socket模拟can的ip地址，5017是GPS的端口号，5016是模拟can的端口号
3. 使用raw-can启动, `./threadepollsrv -c 192.168.100.246 5017` 或 `./threadepollsrv -c 192.168.100.246 5017 can0`, 默认是利用can1进行发送，但也可以指定can的端口号---192.168.100.246是gps的ip地址，5017是GPS的端口号