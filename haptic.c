//
// haptic.c
// 
// desc: test and example code for haptic sub system 
//
// Written by: Alexei
// Adapted by: G. Eric Engstrom
//
// (C) Copyright 2019, Solomomo LCC, All Rights Reserved
//


#include "e_ventures.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <time.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include "e_ventures_specific.h"

#define LEVEL_OFFSET                0x34
#define SET_OFFSET                  0x1C
#define CLEAR_OFFSET                0x28

#define FG_PWR_MSBYTE_REG             25
#define FG_PWR_LSBYTE_REG             24

#define GPIO_BUTTONLED                 4
#define GPIO_LE                        5        // Latch enable
#define GPIO_SDI                       6        // Data input
#define GPIO_OE                        7        // Output enable
#define GPIO_CLK                       8        // Clock
#define GPIO_HAPTIC                   23
#define GPIO_HALL                     24        
#define GPIO_BUTTON                   25
#define GPIO_BASE             0x20200000        //Raspberry Pi 0

#define MAP_SIZE                  4096UL
#define MAP_MASK           (MAP_SIZE - 1)
#define BUS                  "/dev/i2c-1"

#define BUTTON_DEBOUNCE_MS           200        // Time to wait for button debounce 
#define NOTIFICATION_MS            10000        // Time delay for demo notification 

#define ADC_ADDRESS                 0x1D
#define FUELGAUGE_ADDRESS           0x55

#define DRIVER_CAP                    16
#define LEDS                          20
#define LEVEL_OFFSET                0x34
#define SET_OFFSET                  0x1C
#define CLEAR_OFFSET                0x28
#define CLK_FREQ                     100        // Clock frequency
#define LE_TO_OE                   10000
#define BETWEEN_CYCLES             10000

#define BILLION               1000000000
#define MILLION                  1000000
#define THOUSAND                    1000

void* level_addr;
void* set_addr;
void* clear_addr;

int fd;

struct timespec now;

ulong now_sec;
ulong now_nsec;  

ulong timer_ring_sec;
ulong timer_ring_nsec;  

const long clock_period = BILLION / CLK_FREQ;
const long number_of_clock_pulses = DRIVER_CAP * (( LEDS - 1 ) / DRIVER_CAP + 1 ) * 1;
ulong clock_pulse;
ulong between_clock_pulses;

static int _button( void );
static int _wand_in_cradle( void );

static void _current_time( void );
static void _add_time( ulong *t_sec, ulong *t_nsec, uint to_add_msec, uint units );
static void _change_addr_to( uchar new_addr );
static void _set_level( uchar gpio, uchar value );
void _change_level( uchar gpio, uchar level, ulong ns_to_add );
static int _ring_led( uchar led_number, ulong oe_pulse, uint scan );
static void _ring_reset( void );
static void _ring_beg( void );
static void _preview( void );
static int _ring_capture( void );
static inline char _get_level( char gpio );
static int _notification( int start );

static void _current_time( void )
{
    clock_gettime( CLOCK_REALTIME, &now );

    now_sec = (ulong)now.tv_sec;
    now_nsec = now.tv_nsec;
}

static void _button_led( uint status )
{
    if( status )
      {
      *((ulong*)set_addr) = 1 << GPIO_BUTTONLED;
      }
    else
      {
      *((ulong*)clear_addr) = 1 << GPIO_BUTTONLED;
      }
}

// Add to_add nanoseconds to the timer defined by t_sec/t_nsec
static void _add_time( ulong *t_sec, ulong *t_nsec, uint to_add_msec, uint units )
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

    if( *t_nsec > BILLION )
      {
      *t_nsec -= BILLION;
      ++*t_sec;
      }
}

// Connect to I2C address new_addr
static void _change_addr_to( uchar new_addr )
{
    static char current_addr = 0;

    if( current_addr != new_addr )
      {
      ioctl( fd, I2C_SLAVE, new_addr );
      current_addr = new_addr;
      }
}

// Set gpio level to value (1 or 0)
static void _set_level( uchar gpio, uchar value )
{
    if( value )
      {
      *((ulong*)set_addr) = 1 << gpio;
      }
    else
      {
      *((ulong*)clear_addr) = 1 << gpio;
      }
}

void _change_level( uchar gpio, uchar level, ulong ns_to_add )
{
    while( 1 )
      {
      _current_time();

      if(( now_sec == timer_ring_sec ) ? ( now_nsec >= timer_ring_nsec )
                                       : ( now_sec > timer_ring_sec ))
        {
        break;
        }
      }

    if( gpio == GPIO_SDI )
      {
      _set_level( GPIO_CLK, 0 );
      }

    _set_level( gpio, level );
    _add_time( &timer_ring_sec, &timer_ring_nsec, ns_to_add, 0 );
}

static int _ring_led( uchar led_number, ulong oe_pulse, uint scan )
{
    int i;

    _current_time();

    timer_ring_sec = now_sec;
    timer_ring_nsec = now_nsec;

    for( int j = 0;  j < 10 ; j++ )
      {
      if( _wand_in_cradle() )
        {
        return 0;
        }

      for( int i = 0; i < number_of_clock_pulses; ++i )
        {
        if( i == ( 31 - led_number ))
          {
          _change_level( GPIO_SDI, 1, between_clock_pulses );
	  }
        else if( i == ( 31 - ( led_number + 10 )))
          {
          _change_level( GPIO_SDI, 1, between_clock_pulses );
	  }
        else if( i == ( 31 - led_number + 1 ))
          {
          _change_level( GPIO_SDI, 0, between_clock_pulses );
          }
        else if( i == ( 31 - ( led_number + 10 ) + 1 ))
          {
          _change_level( GPIO_SDI, 0, between_clock_pulses );
          }
        else
          {
          _change_level( GPIO_CLK, 0, between_clock_pulses );
          }

        _change_level( GPIO_CLK, 1, clock_pulse );
        }

      _change_level( GPIO_CLK, 0, between_clock_pulses );
      _change_level( GPIO_LE, 1, clock_pulse );
      _change_level( GPIO_LE, 0, LE_TO_OE );
      _change_level( GPIO_OE, 0, oe_pulse );

      if( !scan )
        {
	break;
        }
      }

    if( scan )
      {
      _change_level( GPIO_OE, 1, BETWEEN_CYCLES );
      }

    return 1;
}

static void _ring_reset( void )
{
    _set_level (GPIO_LE, 0);
    _set_level (GPIO_SDI, 0);
    _set_level (GPIO_OE, 1);
    _set_level (GPIO_CLK, 0);
}

static void _ring_beg( void )
{
    clock_pulse = (clock_period < 40 ? 20 : clock_period / 2);
    between_clock_pulses = clock_period - clock_pulse;
    
    _ring_reset();
}

static void _preview( void )
{
    _ring_beg();

    while( !_wand_in_cradle() )
    {
        _ring_led( 1, 10, 0 ); 
        if( _button() )
	{
	    _ring_capture();
	}
    }

    _ring_reset();
}

static int _ring_capture( void )
{
    int i; 
    int mem;

    _ring_beg();

    if( _ring_led( 5, 20000000, 1 )	// R1
     && _ring_led( 6, 20000000, 1 )	// R2
     && _ring_led( 7, 20000000, 1 )	// R3
     && _ring_led( 4, 1000000, 1 )	// IR
     && _ring_led( 8, 50000000, 1 )	// G1
     && _ring_led( 9, 100000000, 1 )	// G2
     && _ring_led( 10, 100000000, 1 )	// POLARIZED H
     && _ring_led( 1, 1000000, 1 )	// PREVIEW
     && _ring_led( 2, 1000000, 1 )	// POLARIZED V
     && _ring_led( 3, 1000000, 1 ))	// UV
    {
        ;  // structure allows the preview to exit if the wand is returned to the cradle
    }

    _ring_reset();
}

static inline char _get_level( char gpio )
{
    return *((volatile unsigned long *)level_addr) >> gpio & 1;
}

static int _notification( int start )
{
    static int notification_pending = 0;
    static unsigned long notification_sec = 0;
    static unsigned long notification_nsec = 0;

    if(( start == 1 )
     &&( !notification_pending ))
    {
        notification_sec = now_sec;
        notification_nsec = now_nsec;
        _add_time( &notification_sec, &notification_nsec, NOTIFICATION_MS, 1 );

        notification_pending = 1;
    }
    else if( start == -1 )
    {
        notification_pending = 0;
    }

    if( notification_pending )
    {
    	/* checking notifcation demo delay timer */
        if( now_sec > notification_sec )
	{
	    notification_pending = 0;
	    return 1;
	}
    }

    return 0;
}

static int _button( void )
{
    static int button_engaged = 0;
    static unsigned long timer_button_sec = 0;
    static unsigned long timer_button_nsec = 0;

    int button_pressed;

    button_pressed = !_get_level( GPIO_BUTTON );

    if( !button_engaged
     && button_pressed )
    {
        timer_button_sec = now_sec;
        timer_button_nsec = now_nsec;
        _add_time( &timer_button_sec, &timer_button_nsec, BUTTON_DEBOUNCE_MS, 1 );

        button_engaged = 1;
    }
    
    if( button_engaged )
    {
    	/* checking debounce timer */
        if(( now_sec == timer_button_sec )
         ? ( now_nsec >= timer_button_nsec )
         : ( now_sec > timer_button_sec ))
	{
	    if( button_pressed )
	    {
	        return 1;
	    }
            else
	    {
	        button_engaged = 0;
	        return 0;
	    }
	}
    }

    return 0;
}

static void _frame_notify( void )
{
}

static void _frame_preview( void )
{
}

static void _frame_capture( void )
{
}

static int _haptic( int status )
{
    if( status )
    {
        *((unsigned long *)set_addr) = 1 << GPIO_HAPTIC;
	return 1;
    }
    else
    {
        *((unsigned long *)clear_addr) = 1 << GPIO_HAPTIC;
	return 0;
    }
}

static void _off( void )
{
    _haptic( 0 );
    _button_led( 0 );
    _ring_reset();
}

static int _wand_in_cradle_backwards( void )
{
    return( _haptic( !_get_level( GPIO_HALL )));
}

static int _wand_in_cradle( void )
{
    static unsigned long backwards_sec = 0;

    __s32 pwr_lsbyte = 0;
    __s32 pwr_msbyte = 0;

    if( _wand_in_cradle_backwards() )
    {
	_current_time();
	backwards_sec = now_sec;
	return 1;
    }
    else if( backwards_sec != 0 )
    {
        if( now_sec < backwards_sec + 5 )
        {
            return 1;
        }

        backwards_sec = 0;
    }

    _change_addr_to( FUELGAUGE_ADDRESS );

    pwr_lsbyte = _i2c_smbus_read_byte_data( fd, FG_PWR_LSBYTE_REG );
    pwr_msbyte = _i2c_smbus_read_byte_data( fd, FG_PWR_MSBYTE_REG );

    if( pwr_msbyte >> 7 )
    {
	return 0;
    }
    else if( pwr_lsbyte | pwr_msbyte )
    {
	// fprintf( stderr, "C\n");  // Avg power > 0, i.e. Charging
	return 1;
    }
    else
    {
	// fprintf( stderr, "?\n");  // Avg power == 0
	return 1;
    }
}

static int _notification_demo( void )
{
    static int hall_on = 0;

    _current_time();

    if( _button() )
    {
        _button_led( 0 );
        hall_on = 0;
        return 1;
    }
    else if( _wand_in_cradle_backwards() ) 
    {
        hall_on = 1;

        /* reset timer for notification led, in case of demo misbehavior */
        _notification( -1 );
    }
    else
    {
        if( hall_on )
        {
	    hall_on = 0;

	    /* start timer for notification led */
	    _notification( 1 );

        }
	else if( _notification( 0 ))
	{
	    _button_led( 1 ); 	/* turn notification LED on */
	    _frame_notify();
	}
    }

    return 0;
}

int main( int argc, char** argv )
{
    int mem, value, i;
    void *map_base;
    
    if ((mem = open ("/dev/mem", O_RDWR | O_SYNC)) == -1)
    {
        fprintf(stderr, "Cannot open /dev/mem: Try sudo before the command (sudo haptic).\n");
        exit( -1 );
    }
    else if ((map_base = (long *)mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem, GPIO_BASE & ~MAP_MASK)) == NULL)
    {
        fprintf(stderr, "Cannot open Memory Map: Try sudo before the command (sudo haptic).\n");
        exit( -1 );
    }
    else if(( fd = open( BUS, O_RDWR )) == -1 )
    {
        fprintf(stderr, "Cannot open I2C: Try sudo before the command (sudo haptic).\n");
	exit( -1 );
    }

    ioctl( fd, I2C_SLAVE, ADC_ADDRESS );

    // Addresses of GPIO registers
    level_addr = (void *)(map_base + (GPIO_BASE & MAP_MASK) + LEVEL_OFFSET);
    set_addr = (void *)(map_base + (GPIO_BASE & MAP_MASK) + SET_OFFSET);
    clear_addr = (void *)(map_base + (GPIO_BASE & MAP_MASK) + CLEAR_OFFSET);

    _ring_reset();

    if( argc == 2 )
    {
        if( (char)*argv[1] == '1' )
        {
	    _haptic( 1 );
	    exit( 0 );
	}

        if( (char)*argv[1] == 'b' )
	{
	    _button_led( 1 );
	    exit( 0 );
	}

        if( (char)*argv[1] == 'h' )
        {
	    while( 1 )
	    {
		if( _notification_demo() )
		{
		    break;
		}
	    }
        }

        if( (char)*argv[1] == 'L' )
        {
	    /* LED and picture test section */
	    int i;

	    fprintf( stderr, "LED ring\n" );

	    for( i = 0; i < 10; i++ )
	    {
		_ring_capture();
	    }
	}

        if( (char)*argv[1] == 'p' )
        {
	    while( 1 ) 
	    {
		_preview();
	    }
        }

        if( (char)*argv[1] == 'a' )
	{
	    while( 1 )
	    {
		_preview();
		_notification_demo();
	    }
	}
    }

    _off();

    exit( 0 );
}

