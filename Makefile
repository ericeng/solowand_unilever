
ifdef DEBUG
    CFLAGS = -Wall -pthread -g -D_GNU_SOURCE
else
    CFLAGS = -Wall -pthread -O2 -D_GNU_SOURCE
endif

PREFIX = /usr/local

all: wand launcheye

launcheye: launcheye.c 
	$(CC) $(CFLAGS) -o launcheye launcheye.c

wand: solohwapi.c 
	$(CC) $(CFLAGS) -D__DEMO__ -o wand solohwapi.c

install: wand launcheye
	cp wand $(PREFIX)/bin
	cp launcheye $(PREFIX)/bin

clean:
	rm -f *.o
	rm -f wand
	rm -f launcheye

