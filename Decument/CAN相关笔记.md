每个CAN-Bus数据包包括四个主要部分：

Arbitration ID ：
    用于标识发起通信请求的设备ID广播消息，并且任何一个设备组件都能发起多个Arbitration ID，如果两个CAN数据包同一时间在总线Bus内传输，则Arbitration ID较小的数据包先获得总线使用权，先被传输。
Identifier extension（标识符扩展，IDE） ：
    对标准CAN协议来说，这个数据位总是为o；
Data length code （数据长度代码，DLC）： 
    代表数据的大小，从0到8字节不等；
Data（数据）： 
    传输数据本身，标准CAN总线可以承载最大数据包为8字节，但有些系统也强制使用8字节进行数据包填充。