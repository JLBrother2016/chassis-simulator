./threadepollsrv -c 192.168.100.100 5017
use raw can to link!
interface = can1, family = 29, type = 3, proto = 1
ip addr:192.168.100.100 gps port number:5017
listencanfd:4 listengpsfd:5
enable_gacu = 0
ip = 192.168.100.100 port = 60170
listengpsfd = 5, conngpsfd = 6
listencanfd = 4
thread_vehicle enable_gacu = 0
packet302fd: 7
packet704fd: 8
packet132fd: 9
gpspacketfd: 10

./threadepollsrv -cg 192.168.100.100 5017
use raw can to link!
usage gacu can packets!
interface = can1, family = 29, type = 3, proto = 1
ip addr:192.168.100.100 gps port number:5017
listencanfd:4 listengpsfd:5
enable_gacu = 1
ip = 192.168.100.100 port = 60188
listengpsfd = 5, conngpsfd = 6
listencanfd = 4
gpspacketfd: 7
thread_vehicle enable_gacu = 1
packet5A2fd: 8


