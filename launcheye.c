//
// launcheye.c
// 
// desc: launches raspijmpeg | streameye
//       after waiting for boot to complete, supports relaunching if the service dies 
//
// Written by: G. Eric Engstrom
//
// (C) Copyright 2019, Solomomo LCC, All Rights Reserved
//


#include "e_ventures.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define BILLION               1000000000
#define MILLION                  1000000
#define THOUSAND                    1000

struct timespec now;

ulong now_sec;
ulong now_nsec;  

static void _current_time( void )
{
    clock_gettime( CLOCK_REALTIME, &now );

    now_sec = (ulong)now.tv_sec;
    now_nsec = now.tv_nsec;
}

// Add to_add nanoseconds to the timer defined by t_sec/t_nsec
static void _add_time( ulong *t_sec, ulong *t_nsec, int to_add_msec, int units )
{
    if( units ) // milliseconds
      {
      *t_sec += to_add_msec / THOUSAND;
      *t_nsec += to_add_msec % THOUSAND * MILLION;
      }
    else
      {
      *t_nsec += to_add_msec;
      }

    if (*t_nsec > BILLION)
      {
      *t_nsec -= BILLION;
      ++*t_sec;
      }
}

int main( int argc, char** argv )
{
    ulong boot_delay_sec = 0;
    ulong boot_delay_nsec = 0;

    _current_time();

    boot_delay_sec = now_sec;
    boot_delay_nsec = now_nsec;

    _add_time( &boot_delay_sec, &boot_delay_nsec, 9000, 1 );

    while( 1 )
      {
      _current_time();

      if( now_sec > boot_delay_sec )
	{
	printf( "now %ld boot %ld\n", now_sec, boot_delay_sec );
	break;
	}
      }

    system( "/usr/bin/sudo /usr/bin/python /home/pi/streameye/extras/raspimjpeg.py -h 480 -w 640 -r 15 | /usr/bin/sudo /usr/local/bin/streameye" );

    exit( 0 );
}

