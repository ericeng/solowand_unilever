
ifdef DEBUG
    CFLAGS = -Wall -pthread -g -D_GNU_SOURCE
else
    CFLAGS = -Wall -pthread -O2 -D_GNU_SOURCE
endif

PREFIX = /usr/local

all: wand launcheye wand_sim

launcheye: launcheye.c 
	$(CC) $(CFLAGS) -o launcheye launcheye.c

wand: solohwapi.c 
	$(CC) $(CFLAGS) -D__DEMO__ -o wand solohwapi.c

wand_sim: wand_sim.c
	$(CC) $(CFLAGS) -D__DEMO_ -o wand_sim -lwiringPi wand_sim.c

install: wand launcheye
	cp wand $(PREFIX)/bin
	cp wand_sim $(PREFIX)/bin
	cp launcheye $(PREFIX)/bin

clean:
	rm -f *.o
	rm -f wand
	rm -f wand_sim
	rm -f launcheye

