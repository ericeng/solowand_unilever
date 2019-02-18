/*
Test code and example code for haptic and hall sub system.
*/

#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <time.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#inclide "wandeye.h"

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

unsigned long now_sec;
unsigned long now_nsec;  

unsigned long timer_ring_sec;
unsigned long timer_ring_nsec;  

const long clock_period = BILLION / CLK_FREQ;
const long number_of_clock_pulses = DRIVER_CAP * ((LEDS - 1) / DRIVER_CAP + 1) * 1;
long clock_pulse;
long between_clock_pulses;

static int _button( void );
static int _ring_capture( void );
static int _wand_in_cradle( void );

/*
The following four function declarations are included because they are missing
from standard library header file i2c-dev.h
*/
static inline __s32 _i2c_smbus_access( int file, char read_write, __u8 command, int size, union i2c_smbus_data *data )
{
	struct i2c_smbus_ioctl_data args;

	args.read_write = read_write;
	args.command = command;
	args.size = size;
	args.data = data;

	return ioctl( file, I2C_SMBUS, &args );
}

static inline __s32 _i2c_smbus_read_byte_data( int file, __u8 command )
{
	union i2c_smbus_data data;

	if( _i2c_smbus_access( file, I2C_SMBUS_READ, command, I2C_SMBUS_BYTE_DATA, &data ))
        {
	    return -1;
        }
	else
	{
   	    return 0x0FF & data.byte;
	}
}

static inline __s32 _i2c_smbus_write_byte_data( int file, __u8 command, __u8 value )
{
	union i2c_smbus_data data;

	data.byte = value;
	return _i2c_smbus_access( file, I2C_SMBUS_WRITE, command, I2C_SMBUS_BYTE_DATA, &data );
}

static inline __s32 _i2c_smbus_read_word_data( int file, __u8 command )
{
	union i2c_smbus_data data;

	if( _i2c_smbus_access( file, I2C_SMBUS_READ, command, I2C_SMBUS_WORD_DATA, &data ))
        {
	    return -1;
        }
	else
	{
	    return 0x0FFFF & data.word;
	}
}

static void _current_time( void )
{
    clock_gettime(CLOCK_REALTIME, &now);
    now_sec = (unsigned long)now.tv_sec;
    now_nsec = now.tv_nsec;
}

static void _button_led( int status )
{
    if( status )
    {
        *((unsigned long *)set_addr) = 1 << GPIO_BUTTONLED;
    }
    else
    {
        *((unsigned long *)clear_addr) = 1 << GPIO_BUTTONLED;
    }
}

// Add to_add nanoseconds to the timer defined by t_sec/t_nsec
static void _add_time( unsigned long *t_sec, unsigned long *t_nsec, int to_add_msec, int units )
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

// Connect to I2C address new_addr
static void _change_addr_to( char new_addr )
{
    static char current_addr = 0;

    if (current_addr != new_addr)
    {
        ioctl( fd, I2C_SLAVE, new_addr );
        current_addr = new_addr;
    }
}

// Set gpio level to value (1 or 0)
static void _set_level (char gpio, char value)
{
    if (value)
    {
        *((unsigned long *)set_addr) = 1 << gpio;
    }
    else
    {
        *((unsigned long *)clear_addr) = 1 << gpio;
    }
}

void _change_level( char gpio, char level, long ns_to_add )
{
    while( 1 )
    {
	_current_time();

        if(( now_sec == timer_ring_sec )
         ?( now_nsec >= timer_ring_nsec )
         :( now_sec > timer_ring_sec ))
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

static int _ring_led( char led_number, long oe_pulse, int scan )
{
    int i;
    int j;

    _current_time();

    timer_ring_sec = now_sec;
    timer_ring_nsec = now_nsec;

    for( j = 0;  j < 10 ; j++ )
    {
        if( _wand_in_cradle() )
        {
	    return 0;
        }

        for( i = 0; i < number_of_clock_pulses; ++i )
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
    static int status = 0;

    if( !status )
    {
        _ring_beg();
    }

    if( !_wand_in_cradle() )
    {
        if( status == 2 )
        {
            if( !_ring_capture() )
	    {
	        status = 1;
            }
        }
        else
        {
            status = 1;
            _ring_led( 1, 10, 0 ); 
            if( _button() )
	    {
                status = 2;
	        _ring_capture();
            }
	}
    }
    else
    {
        _ring_reset();
        status = 0;
    }
}

static int _ring_capture( void )
{
    int i; 
    int mem;
typedef struct _RING_TIME
{
    int _r1;
    int _r2;
    int _r3;
    int _ir;
    int _g1;
    int _g2;
    int _ph;
    int _w;
    int _pv; 
    int _uv;
}
_ring_time;

    if( _ring_time._r1 > 0 )
    {
        if( _ring_led( 5, 20000000, 1 )	// R1
        {
	    _ring_time.r1--;
        }
        else
        {
	    return 0;
        }
    }
    else if(
    
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

void wandeye_open()
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
}

void wandeye( void )
{
    _preview();
    _notification_demo();
}

void wandeye_close( void )
{
    _off();
}

