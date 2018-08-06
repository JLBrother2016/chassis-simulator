.PHONY: clean all
# CC = gcc, CC有软链接
# CC := aarch64-linux-gun-gcc
CFLAGS = -Wall -g
BIN = threadepollsrv
all: $(BIN)
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@
# pthread.o: pthread.c
# 	$(CC) $(CFLAGS) -c pthread.c -o $@ -lpthread
threadepollsrv: threadepollsrv.o canpacket.o nmeagps.o pthreadutil.o coordinate.o threadchassiscan.o threadchassisgps.o threadvehicle.o initialsrv.o -lpthread -lm
	$(CC) $(CFLAGS) $^ -o $@

clean:
	rm -f *.o $(BIN)