//
// SoloHwApi.c
//
// Written by: Alexei for test purposes
// Adapted by: G. Eric Engstrom for beta deploymnent
//
// Copyright Solomomo LLC 2018, 2019 All Rights Reserved
//
// API provides access to:
// - two LED drivers TLC59116
// - accelerometer/gyroscope BMI160 
// - analog-to-digital converter ADC128D818
// - battery fuel gauge bq27421-G1 
// - haptic motor 
// - Hall sensor 
//

#include "e_ventures.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <wchar.h>
#include <locale.h>
#include <poll.h>
#include <linux/types.h>
#include "e_ventures_specific.h"

#define BUTTON_TRIGGER 10
#define PREVIEW_CAROUSEL_CURSOR 2

#define ADC_ADDRESS                 0x1D
#define FUELGAUGE_ADDRESS           0x55
#define LED_DRIVER1_ADDRESS         0x60
#define LED_DRIVER2_ADDRESS         0x61
#define SENSOR_ADDRESS              0x69
#define BRIGHTNESS                    99   
#define LED_ON_MSEC                  500        // Duration of period when a LED is on
#define LED_OFF_MSEC                   5	// Duration of period when all LEDs are off
#define LED_MODE1_REG                  0        // Mode register 1 for LED driver
#define LED_NORMAL                     0        // Normal value for mode register 1
#define NUMBER_OF_LEDS                12
#define PWM_START_REG                  2        // First brightness register for LED driver
#define LEDOUT_START_REG              20        // First output state register for LED driver
#define INDIV_CTRL                   170        // Mode of individual brightness controlling
#define FG_PWR_MSBYTE_REG             25
#define FG_PWR_LSBYTE_REG             24
#define ADC_CONFIG_REG                 0
#define ADC_CONFIG_NORMAL              2
#define ADC_ADVANCED_REG              11
#define ADC_ADVANCED_NORMAL            3
#define ADC_INTERRUPT_REG              1
#define FIRST_ADC_CHANNEL_REG         32
#define FIRST_ADC_LIMIT_REG           42
#define ADC_CHANNELS                   8        // Number of channel reading registers
#define ADC_VOLTAGE                 2500
#define ADC_TOLERANCE                  5        // Value for setting limits
#define INT_EN                      0x50        // Register for enabling interrupts
#define ANYMOT_SINTAP_EN            0x27        // Enable ANY MOTION and SINGLE TAP interrupts
#define INT_OUT_CTRL                0x53        // Register for controlling interrupt pins
#define CONF_PINS                   0xAA        // Enable, push-pull, active = 1, level triggered
#define INT_LATCH                   0x54        // Register for setting latch mode
#define LATCH                       0x8         // Disable input for pin1 and pin2, latch = 40 ms
#define INT_MAP1                    0x55        // Register for mapping interrupts to pin1
#define ANYMOT_MAP                  0x04        // Map ANY MOTION interrupt to pin1
#define INT_MAP2                    0x57        // Register for mapping interrupts to pin2
#define SINTAP_MAP                  0x20        // Map SINGLE TAP interrupt to pin2
#define CMD                         0x7E        // Register for changing power mode
#define ADC_DISABLE_CHANNELS_REG     0x8
#define ACC_TO_NORM                 0x11        // Change accelerometer power mode to normal
#define GYR_TO_NORM                 0x15        // Change gyroscope power mode to normal
#define ACC_TIME_TO_NORM_NS      3800000        // Max time for changing accelerometer power mode to normal
#define GYR_TIME_TO_NORM_NS     80000000        // Max time for changing gyroscope power mode to normal
#define CAPTURE_END_NS          90000000        // Time for physical signal at the end of capture. 
#define CAROUSEL_FRAME_NS      140000000	// Time to capture each image from the camera, 6.5 fps
#define BUS                  "/dev/i2c-1"
#define LEVEL_OFFSET                0x34
#define SET_OFFSET                  0x1C
#define CLEAR_OFFSET                0x28
#define FREQ                           5        // Samplings per second
#define GYR_X_LSB_REG                0xC
#define GYR_X_MSB_REG                0xD
#define GYR_Y_LSB_REG                0xE
#define GYR_Y_MSB_REG                0xF
#define GYR_Z_LSB_REG               0x10
#define GYR_Z_MSB_REG               0x11
#define ACC_X_LSB_REG               0x12
#define ACC_X_MSB_REG               0x13
#define ACC_Y_LSB_REG               0x14
#define ACC_Y_MSB_REG               0x15
#define ACC_Z_LSB_REG               0x16
#define ACC_Z_MSB_REG               0x17
#define GYR_RESOLUTION             0.061        // Angular velocity in deg/s corresponding to LSB if range is set to +/-2000 deg/s (default)
#define ACC_RESOLUTION          0.000061        // Acceleration in g corresponding to LSB if range is set to +/-2g (default)
#define DEGREE_SYMBOL               0xB0        // Unicode for degree symbol
#define GPIO_INT1                     17        // Interrupt 1 from accelerometer/gyroscope
#define GPIO_INT2                     27        // Interrupt 2 from accelerometer/gyroscope
#define GPIO_INT3                     22        // Interrupt from Fuel Gauge (change from charging to discharging or vice versa)
#define GPIO_INT4                      5        // Interrupt from ADC (output values exceed set limits)
#define GPIO_HALL                     24        
#define GPIO_HAPTIC                   23
#define GPIO_BUTTONLED                 4
#define GPIO_BUTTON                   25
#define MAP_SIZE                    4096UL
#define MAP_MASK           (MAP_SIZE - 1)
#define GPIO_BASE             0x20200000        //Raspberry Pi 0
#define BILLION               1000000000
#define MILLION                  1000000
#define THOUSAND                    1000
#define LINES_BETWEEN_HEADINGS        20
#define NOTIFICATION_DELAY_MS	   20000       // Notification should show up in 20 seconds
#define BUTTON_DOWNTIME_MS           200       // Time that the button must be pressed to engage the LED
#define BUTTONLED_PERIOD_MS         1500       // Period for toggling LED after pressing button
#define HAPTIC_PERIOD_MS             300       // Period for toggling the haptic motor
#define HAPTIC_TIMES                   5       // Haptic motor goes on/off this number of times when pressing Enter

#define SENSOR_PERIOD ( THOUSAND / FREQ )
#define PWM_END_REG PWM_START_REG + NUMBER_OF_LEDS

static uint _cradle_in_flag = 1;
static uint _notification_pending = 0;

typedef struct C_NOTIFICATION
{
    ulong sec;
    ulong nsec;
} c_notification;

typedef struct C_HW
{
    void* addr_level;
    void* addr_set;
    void* addr_clear;
} c_hw;

typedef struct C_HAPTIC
{
    char timer_expired;
    char engaged;
    char on;
    char input;
    char count;
} c_haptic;

typedef struct C_BUTTON
{
    char timer_expired;
    char pressed_was;
    char pressed;
    char engaged;
    char led;
} c_button;

typedef struct C_TIMER
{
    ulong sampling_sec;
    ulong sampling_nsec;
    ulong haptic_sec;
    ulong haptic_nsec;
    ulong button_sec;
    ulong button_nsec;
    ulong button_led_sec;
    ulong button_led_nsec;
} c_timer;

typedef struct C_IMU
{
      // gyroscope
    uchar gyr_x_lsb;
    uchar gyr_x_msb;
    uchar gyr_y_lsb;
    uchar gyr_y_msb;
    uchar gyr_z_lsb;
    uchar gyr_z_msb;

    ulong gyr_x;
    ulong gyr_y;
    ulong gyr_z;

      // accelerometer
    uchar acc_x_lsb;
    uchar acc_x_msb;
    uchar acc_y_lsb;
    uchar acc_y_msb;
    uchar acc_z_lsb;
    uchar acc_z_msb;

    ulong acc_x;
    ulong acc_y;
    ulong acc_z;
} c_imu;

static struct timespec _now;
static ulong _now_sec;
static ulong _now_nsec;
static uchar _carousel_cursor = 0;

static void _clock_now_get( struct timespec* _scratch );
static void _change_addr_to( uchar new_addr );
static c_haptic* _c_haptic_new( void );
static void _c_haptic_delete( c_haptic* d );
static c_button* _c_button_new( void );
static void _c_button_delete( c_button* d );
static c_timer* _c_timer_new( void );
static void _c_timer_delete( c_timer* d );
static c_imu* _c_imu_new( void );
static void _c_imu_delete( c_imu* d );
static c_hw* _c_hw_new( void );
static void _c_hw_delete( c_hw* _this );
static void _capture_beg( void );
static void _capture_end( void );
static void _preview_on( void );
static void _preview_off( void );
static void _wand_hw_init( void );
static void _pwm_init( int fd, int address );
static void _haptics_on( void );
static void _haptics_off( void );
static void _button_led_on( void );
static void _button_led_off( void );
static void __carousel_led_set( int file, int cursor, int state );

uchar WandHwOpen( void );
void WandHwClose( void );
uchar GetGPIO( uchar gpio );
void SetGPIO( uchar gpio, uchar value );
void AddTime( ulong* t_sec, ulong* t_nsec, int to_add_msec );
uchar TimersExpired( void );
uchar Button( void );
uchar ButtonLEDSet( uchar );
int Preview( void );
int Capture( void );
int CradleIn( void );
int CradleInBackwards( void );
void Notification( void );

static c_haptic* _h = NULL;
static c_button* _b = NULL;
static c_timer* _t = NULL;
static c_imu* _m = NULL;
static c_hw* _p = NULL;
static c_notification _n;
static void* _map_base = NULL;
static int _mem;
static int _fd;
static struct timespec _wake_acc;
static struct timespec _wake_gyr;
static struct timespec _capture_end_haptic;
static struct timespec _carousel_frame;

/*
 * API
 *
 * uchar	WandHwOpen( void )
 * void		WandHwClose( void )
 * char		GetGPIO( uchar gpio )
 * void		SetGPIO( uchar gpio, uchar value )
 * void		AddTime( ulong* t_sec, ulong* t_nsec, int to_add_msec )
 * uchar	TimersExpired( void )
 * uchar        Button( void )
 * uchar        ButtonLEDSet( uchar )
 * int		Preview( void )
 * int		Capture( void )
 * int		CradleIn( void )
 * int		CradleInBackwards( void )
 * void		Notification( void )
 *
 */

#define LEDOUT_END_REG LEDOUT_START_REG + (NUMBER_OF_LEDS - 1) / 4 + 1

static void _clock_now_get( struct timespec* _scratch )
{
    clock_gettime( CLOCK_REALTIME, _scratch );

    _now_sec = (ulong)_scratch->tv_sec;
    _now_nsec = _scratch->tv_nsec;
}
    
// Connect to I2C address new_addr
static void _change_addr_to( uchar new_addr )
{
    static uchar current_addr = 0;

    if( current_addr != new_addr )
      {
      ioctl( _fd, I2C_SLAVE, new_addr );
      current_addr = new_addr;
      }
}

static c_haptic* _c_haptic_new( void )
{
    return calloc( sizeof( c_haptic ), 1 );
}

static void _c_haptic_delete( c_haptic* d )
{
    free( d );
}

static c_button* _c_button_new( void )
{
    return calloc( sizeof( c_button ), 1 );
}

static void _c_button_delete( c_button* d )
{
    free( d );
}

static c_timer* _c_timer_new( void )
{
    return calloc( sizeof( c_timer ), 1 );
}

static void _c_timer_delete( c_timer* d )
{
    free( d );
}

static c_imu* _c_imu_new( void )
{
    return calloc( sizeof( c_imu ), 1 );
}

static void _c_imu_delete( c_imu* d )
{
    free( d );
}


static c_hw* _c_hw_new( void )
{
    return calloc( sizeof( c_hw ), 1 );
}

static void _c_hw_delete( c_hw* _this )
{
    free( _this );
}

uchar WandHwOpen( void )
{
    if(( _mem = open( "/dev/mem", O_RDWR | O_SYNC )) == -1 )
      {
      fprintf( stderr, "Cannot open /dev/mem\n");
      exit(1);
      }
        
    if(( _map_base = (long*) mmap( 0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, _mem, ( GPIO_BASE & ~MAP_MASK ))) == NULL )
      {
      fprintf( stderr, "Cannot open Memory Map\n" );
      exit(1);
      }

    _fd = open( BUS, O_RDWR );
    ioctl( _fd, I2C_SLAVE, ADC_ADDRESS );
    
    _p = _c_hw_new();
    _h = _c_haptic_new();
    _b = _c_button_new();
    _t = _c_timer_new();
    _m = _c_imu_new();

    // Addresses of GPIO registers
    _p->addr_level = (void*)( _map_base + ( GPIO_BASE & MAP_MASK ) + LEVEL_OFFSET );
    _p->addr_set = (void*)( _map_base + ( GPIO_BASE & MAP_MASK ) + SET_OFFSET );
    _p->addr_clear = (void*)( _map_base + ( GPIO_BASE & MAP_MASK ) + CLEAR_OFFSET );

    if( _p
      && _h
      && _b
      && _t
      && _m )
      {
      _wand_hw_init();

      return 1;
      }
    else
      {
      return 0;
      }
}

void WandHwClose( void )
{
    _c_imu_delete( _m );
    _c_timer_delete( _t );
    _c_button_delete( _b );
    _c_haptic_delete( _h );
    _c_hw_delete( _p );

    close( _fd );

    if( munmap( _map_base, MAP_SIZE ) == -1 )
      {
      fprintf( stderr, "Cannot close Memory Map");
      exit( 1 );
      }
    
    close( _mem );
}

uchar GetGPIO( uchar gpio )
{
    return *((volatile ulong*)_p->addr_level ) >> gpio & 1;
}

void SetGPIO( uchar gpio, uchar value )
{
    if( value )
      {
      *((ulong*)_p->addr_set ) = 1 << gpio;
      }
    else
      {
      *((ulong*)_p->addr_clear ) = 1 << gpio;
      }
}

// Add to_add_msec milliseconds to the timer defined by t_sec/t_nsec
void AddTime( ulong* t_sec, ulong* t_nsec, int to_add_msec )
{
    *t_sec += to_add_msec / THOUSAND;
    *t_nsec += ( to_add_msec % THOUSAND ) * MILLION;

    if( *t_nsec > BILLION )
      {
      *t_nsec -= BILLION;
      ++*t_sec;
      }
}

/* Return integer value coding the information of timers expired:
 * 1 - sampling timer expired (time to get readings);
 * 2 - haptic motor timer expired (time to toggle haptic motor);
 * 4 - button timer expired.
 * For example, 6 (= 2 + 4) means that haptic motor timer and button timer are expired.
 */
uchar TimersExpired( void )
{
    static struct pollfd mypoll =
      {
      STDIN_FILENO,
      POLLIN | POLLPRI
      };

    static char result = 1;
    static char sampling_timer_expired = 1;
    static char initializing = 1;

    _clock_now_get( &_now );

    _h->input = 0;
    result = 0;

    if( initializing )
      {
      _t->sampling_sec = _now_sec;
      _t->sampling_nsec = _now_nsec;
      initializing = 0;
      }
    
    Button();

    if( !_b->timer_expired
      && !_b->pressed
      && !_b->led )
      {
      _b->engaged = 0; 
      }
    
    if( poll( &mypoll, 1, 0 ))
      {    // User input buffer is not empty
      scanf( "%c", &( _h->input ));

      if( _h->input == BUTTON_TRIGGER )
        {    // Pressed Enter
        _t->haptic_sec = _now_sec;
        _t->haptic_nsec = _now_nsec;
        _h->engaged = 1;        // Haptic motor is in state of toggling periodically 
        _h->count = 0;
        _h->on = 0;
        *((ulong*)_p->addr_set ) = 1 << GPIO_HAPTIC;
        }
    }
    
    sampling_timer_expired = ( _now_sec == _t->sampling_sec
                             ? _now_nsec >= _t->sampling_nsec
                             : _now_sec > _t->sampling_sec );

    if( sampling_timer_expired )
      {
      AddTime( &( _t->sampling_sec ), &( _t->sampling_nsec ), SENSOR_PERIOD );
      result += 1;
      }
    
    _b->timer_expired = _b->engaged
                       && ( _now_sec == _t->button_nsec 
                          ? _now_nsec >= _t->button_nsec
                          : _now_sec > _t->button_sec );

    if( _b->timer_expired )
      {
      if( _b->led )
        {
        _b->led = 0;
        _button_led_off();
        // *((ulong*)_p->addr_clear ) = 1 << GPIO_BUTTONLED;
        _b->engaged = 0;
        }
      else
        {
        _b->led = 1;
        _button_led_on();
        //*((ulong*)_p->addr_set ) = 1 << GPIO_BUTTONLED;
        AddTime( &( _t->button_sec ), &( _t->button_nsec ), BUTTONLED_PERIOD_MS );
        }
      }
    
    if( _b->engaged )
      {
      result += 4;
      }
    
    _h->timer_expired = _h->engaged
                       && ( _now_sec == _t->haptic_sec
                          ? _now_nsec >= _t->haptic_nsec
                          : _now_sec > _t->haptic_sec );

    if( _h->timer_expired )
      {
      AddTime( &( _t->haptic_sec ), &( _t->haptic_nsec ), HAPTIC_PERIOD_MS);

      _h->on = !_h->on;

      if( _h->on)
        {
        *((ulong*)_p->addr_set ) = 1 << GPIO_HAPTIC;
        ++_h->count;
        }
      else
        {
        *((ulong*)_p->addr_clear ) = 1 << GPIO_HAPTIC;

        if( _h->count >= HAPTIC_TIMES )
          {
          _h->engaged = 0;
          }
        }

      result += 2;
    }

    return result;
}

static void _pwm_init( int file, int address )
{
    _change_addr_to( address );
    _i2c_smbus_write_byte_data( file, LED_MODE1_REG, LED_NORMAL );

    for ( int i = LEDOUT_START_REG; i < LEDOUT_END_REG; i++ )
      {
      _i2c_smbus_write_byte_data( file, i, INDIV_CTRL );
      }

    for( int i = PWM_END_REG - 1; i >= PWM_START_REG; i-- )
      {
      _i2c_smbus_write_byte_data( file, i, 0 );
      }
}
    
static void _wand_hw_init( void )
{
    _wake_acc.tv_sec = 0;
    _wake_acc.tv_nsec = ACC_TIME_TO_NORM_NS;

    _wake_gyr.tv_sec = 0;
    _wake_gyr.tv_nsec = GYR_TIME_TO_NORM_NS;

    _capture_end_haptic.tv_sec = 0;
    _capture_end_haptic.tv_nsec = CAPTURE_END_NS;

    _carousel_frame.tv_sec = 0;
    _carousel_frame.tv_nsec = CAROUSEL_FRAME_NS;

    setlocale( LC_CTYPE, "" );   // For printing Unicode characters
    
    // Setting default parameters for ADC
    _i2c_smbus_write_byte_data( _fd, ADC_CONFIG_REG, ADC_CONFIG_NORMAL );
    _i2c_smbus_write_byte_data( _fd, ADC_ADVANCED_REG, ADC_ADVANCED_NORMAL );
    _i2c_smbus_write_byte_data( _fd, ADC_DISABLE_CHANNELS_REG, 0 ); // Reset disabling channels, if any
    
    // Setting limits for ADC readings (exceeding these limits causes an interrupt)

    for( int i = 0; i < ADC_CHANNELS; i++ )
      {
      int scratch;

      scratch = ADC_VOLTAGE * ( ADC_CHANNELS - i ) / ADC_CHANNELS / 16;
      scratch += ADC_TOLERANCE;

      _i2c_smbus_write_byte_data( _fd, FIRST_ADC_LIMIT_REG + 2 * i, scratch );

      scratch -= 2 * ADC_TOLERANCE;

      _i2c_smbus_write_byte_data( _fd, FIRST_ADC_LIMIT_REG + 1 + 2 * i, scratch );
      }
    
    // Setting default parameters for accelerometer/gyroscope
    _change_addr_to( SENSOR_ADDRESS );
    _i2c_smbus_write_byte_data( _fd, CMD, ACC_TO_NORM );
    nanosleep( &_wake_acc, NULL );
    _i2c_smbus_write_byte_data( _fd, CMD, GYR_TO_NORM );
    nanosleep( &_wake_gyr, NULL );
    _i2c_smbus_write_byte_data( _fd, INT_EN, ANYMOT_SINTAP_EN );
    _i2c_smbus_write_byte_data( _fd, INT_OUT_CTRL, CONF_PINS );
    _i2c_smbus_write_byte_data( _fd, INT_LATCH, LATCH );
    _i2c_smbus_write_byte_data( _fd, INT_MAP1, ANYMOT_MAP );
    _i2c_smbus_write_byte_data( _fd, INT_MAP2, SINTAP_MAP );
    
    // Setting default/initial parameters for LED drivers
    _pwm_init( _fd, LED_DRIVER1_ADDRESS );
    _pwm_init( _fd, LED_DRIVER2_ADDRESS );
}

    // is Button depressed passed debounce
uchar Button( void )
{
    _b->pressed = !GetGPIO( GPIO_BUTTON );

    _clock_now_get( &_now );

    if( !_b->pressed_was
      && _b->pressed )
      {
      _t->button_sec = _now_sec;
      _t->button_nsec = _now_nsec;
      _b->engaged = 1;

      AddTime( &( _t->button_sec ), &( _t->button_nsec ), BUTTON_DOWNTIME_MS );
      }

    _b->pressed_was = _b->pressed;

    _b->timer_expired = ( _b->engaged
                       && ( _now_sec == _t->button_sec 
                        ? _now_nsec >= _t->button_nsec
                        : _now_sec > _t->button_sec )); // debounce timer expired

    return _b->timer_expired;
}

uchar ButtonLEDSet( uchar on )
{
    // set LED status to equal on
    return on;
}

int Preview( void )
{
    if( !CradleIn() )
      {

      // turn preview lights on here

      for( ;; )
        {
        if( CradleIn())
          {
          return 1;
          }
        else if( CradleInBackwards() )
          {
          return -1;
          }

        Capture();
        }
      }

    return 0;
}

int Capture( void )
{
    if( Button())
      {
      // start light carousel
      while( Button())
        {
        // if( check moisture sensor )
        //   remove instructions (if any) from frame
        //   if( frame capture time complete )
        //     advance carousel
        //     if( carousel complete )
        //       send frame complete capture message
        //       stop light carousel
        //       return 1;
        // else
        //   instructions to frame
        //   reset light carousel
        //
        // if( CradleIn )
        //   send frame incomplete capture message
        //   stop carousel
        //   return -1
        //
        // if( CradleInBackwards )
        //   send frame incomplete capture message
        //   stop carousel
        //   return -2
        }
      // send frame incomplete capture message
      // stop carousel
      return 0;
      }

    return 0;
}

int CradleIn( void )
{
    return _cradle_in_flag;
}

int CradleInBackwards( void )
{
    static int backwards = 0;

    if( !GetGPIO( GPIO_HALL ))
      {
      if( backwards == 0 )
        {
        _haptics_on();
        backwards = 1;
        }

      return 1;
      }
    else
      {
      if( backwards == 1 )
        {
        _clock_now_get( &_now );

        _n.sec = _now_sec;
        _n.nsec = _now_nsec;

        AddTime( &_n.sec, &_n.nsec, NOTIFICATION_DELAY_MS );
        _notification_pending = 1;
        }

      backwards = 0;
      return 0;
      }

}

void Notification( void )
{
    if( _notification_pending )
      {
      /* if time, then _notification_pending stopped, notification visible set, next button press clears it */
      }
}

void Haptic( uchar type )
{
/*
    switch( type )
    {
    case HAPTIC_PREVIEW_END:
    case HAPTIC_CRADLE_IN_BACKWARDS:
    default:
      _haptic_clear();
      *((ulong*)p->addr_clear ) = 1 << GPIO_HAPTIC;
    }
        {    // Pressed Enter
        t->haptic_sec = _now_sec;
        t->haptic_nsec = _now_nsec;
        h->engaged = 1;        // Haptic motor is in state of toggling periodically 
        h->count = 0;
        h->on = 0;
        *((ulong*)p->addr_set ) = 1 << GPIO_HAPTIC;
        }
 */
}

static void _haptics_on( void )
{
    *((ulong*)_p->addr_set ) = 1 << GPIO_HAPTIC;
}

static void _haptics_off( void )
{
    *((ulong*)_p->addr_clear ) = 1 << GPIO_HAPTIC;
}

static void __carousel_led_set( int file, int cursor, int state )
{
    if( PWM_START_REG + cursor < PWM_END_REG )
      {
      _change_addr_to( LED_DRIVER1_ADDRESS );
      _i2c_smbus_write_byte_data( file, cursor, state ? BRIGHTNESS
                                                      : 0 );

      _change_addr_to( LED_DRIVER2_ADDRESS );
      _i2c_smbus_write_byte_data( file, cursor, state ? BRIGHTNESS
                                                      : 0 );
      }
}

static void _carousel_led_on( int file, int cursor )
{
    __carousel_led_set( file, cursor, 1 );
}

static void _carousel_led_off( int file, int cursor )
{
    __carousel_led_set( file, cursor, 0 );
}

static void _carousel_reset( void )
{
    _carousel_cursor = 0;
    __carousel_led_set( _fd, _carousel_cursor, 0 );
}

static uchar _carousel_next( void )
{
    if( PWM_START_REG + _carousel_cursor + 1 < PWM_END_REG )
      {
      ++_carousel_cursor;
      return 1;
      }
    else
      {
      return 0; 
      }
}

static void _button_led_on( void )
{
    *((ulong*)_p->addr_clear ) = 1 << GPIO_BUTTONLED;
}

static void _button_led_off( void )
{
    *((ulong*)_p->addr_set ) = 1 << GPIO_BUTTONLED;
}

static void _preview_on( void )
{
    _carousel_led_on( _fd, PREVIEW_CAROUSEL_CURSOR );
}

static void _preview_off( void )
{
    _carousel_led_off( _fd, PREVIEW_CAROUSEL_CURSOR );
}

static void _capture_beg( void )
{
    _carousel_reset();

    do
      {
      _carousel_led_on( _fd, _carousel_cursor );
      nanosleep( &_carousel_frame, NULL );
      _carousel_led_off( _fd, _carousel_cursor );
      }
    while( _carousel_next() );

    _carousel_led_off( _fd, _carousel_cursor );
    _capture_end();
}

static void _capture_end( void )
{
    _haptics_on();
    nanosleep( &_capture_end_haptic, NULL );
    _haptics_off();
}

#ifdef __DEMO__
int main( int argc, char** argv );

int main( int argc, char** argv )
{
    WandHwOpen();

    for( ;; )
      {
      if( CradleInBackwards() == 0 )
        { 
        _haptics_off();
        if( CradleIn() == 0 )
          {
          int just = 1;

          _preview_on();
          for( ;; )
            {
            Button();

            if( _b->timer_expired
              && _b->pressed )
              {
              if( just )
                {
                _preview_off();
                _capture_beg();
                just = 0;
                }
              }
            else
              {
              if( !just )
                {
                _capture_end();
                _preview_on();
                just = 1;
                }
              }

            if( CradleIn()
              || CradleInBackwards() )
              {
              _preview_off();
              _button_led_off();
              break;
              }
            }
          }
        else
          {
          _haptics_off();
          _preview_off();
          _button_led_off();

          while( Button() )
            {
            if( _b->timer_expired
              && _b->pressed )
              {
              _cradle_in_flag = 0;
              }
            }
          }
        }
      else
        {
        _preview_off();
        _button_led_off();
        _cradle_in_flag = 1;
        }
      }

    WandHwClose();
}
#endif

#ifdef __TEST__

int main( int argc, char** argv );
static void _t_imu_read( c_imu* _this );
static void _t_imu_clear( c_imu* _this );
static void _t_imu_average( c_imu* _this );
static int _t_swap_bytes( ushort n );
static void _t_print_unicode( ushort code );
static void _t_pwm_read( int fd, int address );
static void _t_read_data( ulong duration_ns, char current_led_address );

// Output readings during a period between changes of LED brightness 
static void _t_read_data( ulong duration_ns, char current_led_address )
{
    static struct timespec led_time;

    static ulong start_sec;
    static ulong start_nsec;
    static ulong elapsed_ns;

    static uchar int1;
    static uchar int2;
    static uchar int3;
    static uchar int4;

    static uchar buttonled_state;
    static uchar haptic_state;
    static uchar hall_state;
    static uchar timer_state;
    static uchar pwr_msbyte;
    static uchar pwr_lsbyte;
    static char charging_state[8];
    static ushort in[ADC_CHANNELS]; 
    static int line_count = 0;
    static int sampling_count = 0;

    uchar i;

    clock_gettime( CLOCK_REALTIME, &led_time );

    start_nsec = led_time.tv_nsec;
    start_sec = (long)led_time.tv_sec;
    elapsed_ns = 0;

    while( elapsed_ns < duration_ns )
      {
      _change_addr_to( current_led_address );

      clock_gettime( CLOCK_REALTIME, &led_time );

      if( !int1
        && GetGPIO( GPIO_INT1 ))
        {
        int1 = 1;
        }

      if( !int2
        && GetGPIO( GPIO_INT2 ))
        {
        int2 = 1;
        }

      if( !int3 
        && !GetGPIO( GPIO_INT3 ))
        {
        int3 = 1;
        }

      if( !int4 
        && !GetGPIO( GPIO_INT4 ))
        {
        int4 = 1;
        }

      if( !GetGPIO( GPIO_HALL ))
        {
        hall_state = 1;
        }

      timer_state = TimersExpired();

      if(( timer_state >> 2 ) & 1 )
        {  // Buttonled timer expired
        buttonled_state = 1;
        }

      if(( timer_state >> 1 ) & 1 )
        {  // Haptic timer expired
        haptic_state = 1;
        }

      if( timer_state & 1 )
        {      // Sampling timer expired
        ++sampling_count;
        sampling_count %= FREQ;  // Number of sampling within the current second

        _t_imu_read( _m );

        if( !sampling_count )
          {    // After one second passed
          if( !line_count )
            {
            printf("------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
                    printf("   IN0   IN1   IN2   IN3   IN4   IN5   IN6   IN7      GYR_X     GYR_Y     GYR_Z    ACC_X   ACC_Y   ACC_Z    INT1  INT2  INT3  INT4 Haptic Hall Button  Batt \n");
                    printf("------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
            }

          _t_imu_average( _m );

          _change_addr_to(ADC_ADDRESS);
          for( int i = 0; i < ADC_CHANNELS; i++ )
            {
            in[i] = _i2c_smbus_read_word_data( _fd, 32 + i );
            printf( "%6d", _t_swap_bytes( in[i] ));
            }

          if( int4 )
            {
            _i2c_smbus_read_word_data( _fd, ADC_INTERRUPT_REG ); // Clear interrupt register
            }

          _change_addr_to( FUELGAUGE_ADDRESS );

//        if( int3 )
            {
            pwr_lsbyte = _i2c_smbus_read_byte_data( _fd, FG_PWR_LSBYTE_REG );
            pwr_msbyte = _i2c_smbus_read_byte_data( _fd, FG_PWR_MSBYTE_REG );

            if( pwr_msbyte >> 7 )
              {
              strcpy( charging_state, "    D " );   // Average power < 0, i.e. Discharging
              }
            else if( pwr_lsbyte | pwr_msbyte )
              {
              strcpy( charging_state, "    C " );   // Average power > 0, i.e. Charging
              }
            else
              {
              strcpy( charging_state, "    - " );   // Average power == 0
              }
            }
          Button();

          printf( "%9.1f", _m->gyr_x * GYR_RESOLUTION );
          _t_print_unicode( DEGREE_SYMBOL );
          printf( "/s%7.1f", _m->gyr_y * GYR_RESOLUTION );
          _t_print_unicode( DEGREE_SYMBOL );
          printf( "/s%7.1f", _m->gyr_z * GYR_RESOLUTION );
          _t_print_unicode( DEGREE_SYMBOL );
          printf( "/s%7.2fg", _m->acc_x * ACC_RESOLUTION );
          printf( "%7.2fg", _m->acc_y * ACC_RESOLUTION );
          printf( "%7.2fg", _m->acc_z * ACC_RESOLUTION );
          printf( "%s", int1 ? "      1 " : "      0 " );
          printf( "%s", int2 ? "    1 " : "    0 " );
          printf( "%s", int3 ? "    1 " : "    0 " );
          printf( "%s", int4 ? "    1 " : "    0 " );
          printf( "%s", haptic_state ? "    1 " : "    0 " );
          printf( "%s", hall_state ? "    1 " : "    0 " );
          printf( "%s", _b->pressed ? "    1 " : "    0 " );
          printf( "%s", charging_state );
          printf( "\n" );
          fflush( stdout );

          ++line_count;
          line_count %= LINES_BETWEEN_HEADINGS;

          _t_imu_clear( _m );

          int1 = 0;
          int2 = 0;
          int3 = 0;
          int4 = 0;

          if( haptic_state )
            {
            haptic_state = 0;
            hall_state = 0;
            buttonled_state = 0;
            }
          }

        elapsed_ns = BILLION * (((long)led_time.tv_sec - start_sec ) + led_time.tv_nsec - start_nsec );
        }
      }
}                                           

static void _t_pwm_read( int file, int address )
{
    _change_addr_to( address );

    for( int i = PWM_START_REG; i < PWM_END_REG; i++ )
      {
      _i2c_smbus_write_byte_data( file, i, BRIGHTNESS );
      _t_read_data( LED_ON_MSEC * MILLION, address );
      _i2c_smbus_write_byte_data( file, i, 0 );
      _t_read_data( LED_OFF_MSEC * MILLION, address );
      }
}

// For printing the degree symbol
static void _t_print_unicode( ushort code )
{
    fclose( stdout );
    freopen( "/dev/tty", "a", stdout );
    wprintf( L"%lc", code );
    fclose( stdout );
    freopen( "/dev/tty", "a", stdout );
}

// Swap bytes in a two-byte number n
static int _t_swap_bytes( ushort n )
{
    ushort a = n >> 8;
    ushort b = n << 8;

    return (( a << 8 ) + ( b >> 8 )); 
}

static void _t_imu_read( c_imu* _this )
{
    _change_addr_to( SENSOR_ADDRESS );

    _this->gyr_x_lsb = _i2c_smbus_read_byte_data( _fd, GYR_X_LSB_REG );
    _this->gyr_x_msb = _i2c_smbus_read_byte_data( _fd, GYR_X_MSB_REG );
    _this->gyr_y_lsb = _i2c_smbus_read_byte_data( _fd, GYR_Y_LSB_REG );
    _this->gyr_y_msb = _i2c_smbus_read_byte_data( _fd, GYR_Y_MSB_REG );
    _this->gyr_z_lsb = _i2c_smbus_read_byte_data( _fd, GYR_Z_LSB_REG );
    _this->gyr_z_msb = _i2c_smbus_read_byte_data( _fd, GYR_Z_MSB_REG );

    _this->acc_x_lsb = _i2c_smbus_read_byte_data( _fd, ACC_X_LSB_REG );
    _this->acc_x_msb = _i2c_smbus_read_byte_data( _fd, ACC_X_MSB_REG );
    _this->acc_y_lsb = _i2c_smbus_read_byte_data( _fd, ACC_Y_LSB_REG );
    _this->acc_y_msb = _i2c_smbus_read_byte_data( _fd, ACC_Y_MSB_REG );
    _this->acc_z_lsb = _i2c_smbus_read_byte_data( _fd, ACC_Z_LSB_REG );
    _this->acc_z_msb = _i2c_smbus_read_byte_data( _fd, ACC_Z_MSB_REG );

    _this->gyr_x += (short)(( _this->gyr_x_msb << 8 ) | _this->gyr_x_lsb );
    _this->gyr_y += (short)(( _this->gyr_y_msb << 8 ) | _this->gyr_y_lsb );
    _this->gyr_z += (short)(( _this->gyr_z_msb << 8 ) | _this->gyr_z_lsb );

    _this->acc_x += (short)(( _this->acc_x_msb << 8 ) | _this->acc_x_lsb );
    _this->acc_y += (short)(( _this->acc_y_msb << 8 ) | _this->acc_y_lsb );
    _this->acc_z += (short)(( _this->acc_z_msb << 8 ) | _this->acc_z_lsb );
}

static void _t_imu_clear( c_imu* _this )
{
    _this->gyr_x = 0;
    _this->gyr_y = 0;
    _this->gyr_z = 0;

    _this->acc_x = 0;
    _this->acc_y = 0;
    _this->acc_z = 0;
}

static void _t_imu_average( c_imu* _this )
{
    _this->gyr_x /= FREQ;  // Averaging
    _this->gyr_y /= FREQ;
    _this->gyr_z /= FREQ;

    _this->acc_x /= FREQ;
    _this->acc_y /= FREQ;
    _this->acc_z /= FREQ;
}

int main( int argc, char** argv )
{
    int c = 1;  // just to track log lines

    WandHwOpen();

    /* 
    LEDs switch on/off or change brightness in a certain (cyclic) pattern;
    between these changes, the control is passed to the _test_read_data() function
    */
    while( 1 )
      {
      printf( "Counter %d\n", c++ );
      _test_pwm_read( _fd, LED_DRIVER1_ADDRESS );
      _test_pwm_read( _fd, LED_DRIVER2_ADDRESS );
      }


    WandHwClose();
}
#endif

