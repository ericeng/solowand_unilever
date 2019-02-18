
ifdef DEBUG
    CFLAGS = -Wall -pthread -g -D_GNU_SOURCE
else
    CFLAGS = -Wall -pthread -O2 -D_GNU_SOURCE
endif

PREFIX = /usr/local

all: streameye

streameye.o: streameye.c streameye.h client.h common.h
	$(CC) $(CFLAGS) -c -o streameye.o streameye.c

wandeye.o: wandeye.c 
	$(CC) $(CFLAGS) -c -o wandeye.o wandeye.c

client.o: client.c client.h streameye.h common.h
	$(CC) $(CFLAGS) -c -o client.o client.c

auth.o: auth.c auth.h  common.h
	$(CC) $(CFLAGS) -c -o auth.o auth.c

streameye: streameye.o client.o auth.o wandeye.o
	$(CC) $(CFLAGS) -o streameye streameye.o client.o auth.o wandeye.o $(LDFLAGS)

install: streameye
	cp streameye $(PREFIX)/bin

clean:
	rm -f *.o
	rm -f streameye
