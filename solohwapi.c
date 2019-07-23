/*
 * SoloHwApi.c
 *
 * Copyright Solomomo LLC 2018, 2019
 * Written by: Alexei for test purposes
 * Adapted by: G. Eric Engstrom for beta deploymnet
 *
 * API provides access to:
 * - two LED drivers TLC59116
 * - accelerometer/gyroscope BMI160 
 * - analog-to-digital converter ADC128D818
 * - battery fuel gauge bq27421-G1 
 * - haptic motor 
 * - Hall sensor 
 */

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

#define BUTTON_TRIGGER 10

#define ADC_ADDRESS                 0x1D
#define FUELGAUGE_ADDRESS           0x55
#define LED_DRIVER1_ADDRESS         0x60
#define LED_DRIVER2_ADDRESS         0x61
#define SENSOR_ADDRESS              0x69
#define BRIGHTNESS                    10   
#define LED_ON_MSEC                  500        // Duration of period when a LED is on
#define LED_OFF_MSEC                 500        // Duration of period when all LEDs are off
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
#define BUTTON_DOWNTIME_MS           200       // Time that the button must be pressed to engage the LED
#define BUTTONLED_PERIOD_MS         1500       // Period for toggling LED after pressing button
#define HAPTIC_PERIOD_MS             300       // Period for toggling the haptic motor
#define HAPTIC_TIMES                   5       // Haptic motor goes on/off this number of times when pressing Enter

#define SENSOR_PERIOD ( THOUSAND / FREQ )
#define PWM_END_REG PWM_START_REG + NUMBER_OF_LEDS

typedef unsigned long ulong;
typedef unsigned int uint;
typedef unsigned short ushort;
typedef unsigned char uchar;

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

void change_addr_to( uchar new_addr );
c_haptic* c_haptic_new( void );
void c_haptic_delete( c_haptic* d );
c_button* c_button_new( void );
void c_button_delete( c_button* d );
c_timer* c_timer_new( void );
void c_timer_delete( c_timer* d );
c_imu* c_imu_new( void );
void c_imu_delete( c_imu* d );
void c_imu_read( c_imu* _this );
void c_imu_clear( c_imu* _this );
void c_imu_average( c_imu* _this );
c_hw* c_hw_new( void );
c_hw* c_hw_delete( c_hw* _this );
uchar WandHwOpen( void );
void WandHwClose( void );
uchar GetGPIO( char gpio );
void SetGPIO( char gpio, char value );
void AddTime( ulong* t_sec, ulong* t_nsec, int to_add_msec );
uchar TimersExpired( void );

#ifdef __TEST__
int main( int argc, char** argv );
int test_swap_bytes( ushort n );
void test_print_unicode( ushort code );
void test_pwm_read( int fd, int address );
void test_pwm_write( int fd, int address );
void test_read_data( ulong duration_ns, char current_led_address );
#endif

static c_haptic* h = NULL;
static c_button* b = NULL;
static c_timer* t = NULL;
static c_imu* m = NULL;
static c_hw* p = NULL;
static void* map_base = NULL;
static int mem;
static int fd;

/*
 * API
 *
 * uchar WandHwOpen( void )
 * void WandHwClose( void )
 * char GetGPIO( char gpio )
 * void SetGPIO( char gpio, char value )
 * void AddTime( ulong* t_sec, ulong* t_nsec, int to_add_msec )
 * uchar TimersExpired( void )
 */

#define LEDOUT_END_REG LEDOUT_START_REG + (NUMBER_OF_LEDS - 1) / 4 + 1

// Connect to I2C address new_addr
void change_addr_to( uchar new_addr )
{
    static uchar current_addr = 0;

    if( current_addr != new_addr )
      {
      ioctl( fd, I2C_SLAVE, new_addr );
      current_addr = new_addr;
      }
}

/*
The following four function declarations are included because they are missing
from standard library header file i2c-dev.h
*/
static inline __s32 i2c_smbus_access( int file, char read_write, __u8 command, int size, union i2c_smbus_data *data )
{
    struct i2c_smbus_ioctl_data args;

    args.read_write = read_write;
    args.command = command;
    args.size = size;
    args.data = data;

    return ioctl( file, I2C_SMBUS, &args );
}

static inline __s32 i2c_smbus_read_byte_data( int file, __u8 command )
{
    union i2c_smbus_data data;

    if ( i2c_smbus_access( file, I2C_SMBUS_READ, command, I2C_SMBUS_BYTE_DATA, &data ))
      {
      return -1;
      }
    else
      {
      return ( 0xFF & data.byte );
      }
}

static inline __s32 i2c_smbus_write_byte_data( int file, __u8 command, 
                                           __u8 value )
{
    union i2c_smbus_data data;

    data.byte = value;

    return i2c_smbus_access( file, I2C_SMBUS_WRITE, command, I2C_SMBUS_BYTE_DATA, &data );
}

static inline __s32 i2c_smbus_read_word_data(int file, __u8 command)
{
    union i2c_smbus_data data;

    if( i2c_smbus_access( file, I2C_SMBUS_READ, command, I2C_SMBUS_WORD_DATA, &data ))
      {
      return -1;
      }
    else
      {
      return 0xFFFF & data.word;
      }
}

static inline __s32 i2c_smbus_write_word_data( int file, __u8 command, __u16 value )
{
    union i2c_smbus_data data;

    data.word = value;
    return i2c_smbus_access( file, I2C_SMBUS_WRITE, command, I2C_SMBUS_WORD_DATA, &data );
}

c_haptic* c_haptic_new( void )
{
    return calloc( sizeof( c_haptic ), 1 );
}

void c_haptic_delete( c_haptic* d )
{
    free( d );
}

c_button* c_button_new( void )
{
    return calloc( sizeof( c_button ), 1 );
}

void c_button_delete( c_button* d )
{
    free( d );
}

c_timer* c_timer_new( void )
{
    return calloc( sizeof( c_timer ), 1 );
}

void c_timer_delete( c_timer* d )
{
    free( d );
}

c_imu* c_imu_new( void )
{
    return calloc( sizeof( c_imu ), 1 );
}

void c_imu_delete( c_imu* d )
{
    free( d );
}

void c_imu_read( c_imu* _this )
{
    change_addr_to( SENSOR_ADDRESS );

    _this->gyr_x_lsb = i2c_smbus_read_byte_data( fd, GYR_X_LSB_REG );
    _this->gyr_x_msb = i2c_smbus_read_byte_data( fd, GYR_X_MSB_REG );
    _this->gyr_y_lsb = i2c_smbus_read_byte_data( fd, GYR_Y_LSB_REG );
    _this->gyr_y_msb = i2c_smbus_read_byte_data( fd, GYR_Y_MSB_REG );
    _this->gyr_z_lsb = i2c_smbus_read_byte_data( fd, GYR_Z_LSB_REG );
    _this->gyr_z_msb = i2c_smbus_read_byte_data( fd, GYR_Z_MSB_REG );

    _this->acc_x_lsb = i2c_smbus_read_byte_data( fd, ACC_X_LSB_REG );
    _this->acc_x_msb = i2c_smbus_read_byte_data( fd, ACC_X_MSB_REG );
    _this->acc_y_lsb = i2c_smbus_read_byte_data( fd, ACC_Y_LSB_REG );
    _this->acc_y_msb = i2c_smbus_read_byte_data( fd, ACC_Y_MSB_REG );
    _this->acc_z_lsb = i2c_smbus_read_byte_data( fd, ACC_Z_LSB_REG );
    _this->acc_z_msb = i2c_smbus_read_byte_data( fd, ACC_Z_MSB_REG );

    _this->gyr_x += (short)(( _this->gyr_x_msb << 8 ) | _this->gyr_x_lsb );
    _this->gyr_y += (short)(( _this->gyr_y_msb << 8 ) | _this->gyr_y_lsb );
    _this->gyr_z += (short)(( _this->gyr_z_msb << 8 ) | _this->gyr_z_lsb );

    _this->acc_x += (short)(( _this->acc_x_msb << 8 ) | _this->acc_x_lsb );
    _this->acc_y += (short)(( _this->acc_y_msb << 8 ) | _this->acc_y_lsb );
    _this->acc_z += (short)(( _this->acc_z_msb << 8 ) | _this->acc_z_lsb );
}

void c_imu_clear( c_imu* _this )
{
    _this->gyr_x = 0;
    _this->gyr_y = 0;
    _this->gyr_z = 0;

    _this->acc_x = 0;
    _this->acc_y = 0;
    _this->acc_z = 0;
}

void c_imu_average( c_imu* _this )
{
   _this->gyr_x /= FREQ;  // Averaging
   _this->gyr_y /= FREQ;
   _this->gyr_z /= FREQ;
   _this->acc_x /= FREQ;
   _this->acc_y /= FREQ;
   _this->acc_z /= FREQ;
}

c_hw* c_hw_new( void )
{
    return calloc( sizeof( c_hw ), 1 );
}

c_hw* c_hw_delete( c_hw* _this )
{
    free( _this );
}

uchar WandHwOpen( void )
{
    if(( mem = open( "/dev/mem", O_RDWR | O_SYNC )) == -1 )
      {
      fprintf( stderr, "Cannot open /dev/mem\n");
      exit(1);
      }
        
    if(( map_base = (long*) mmap( 0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem, ( GPIO_BASE & ~MAP_MASK ))) == NULL )
      {
      fprintf( stderr, "Cannot open Memory Map\n" );
      exit(1);
      }

    fd = open( BUS, O_RDWR );
    ioctl( fd, I2C_SLAVE, ADC_ADDRESS );
    
    p = c_hw_new();
    h = c_haptic_new();
    b = c_button_new();
    t = c_timer_new();
    m = c_imu_new();

    // Addresses of GPIO registers
    p->addr_level = (void*)( map_base + ( GPIO_BASE & MAP_MASK ) + LEVEL_OFFSET );
    p->addr_set = (void *)( map_base + ( GPIO_BASE & MAP_MASK ) + SET_OFFSET );
    p->addr_clear = (void *)( map_base + ( GPIO_BASE & MAP_MASK ) + CLEAR_OFFSET );

    return ( p
           && h
           && b
           && t
           && m );
}

void WandHwClose( void )
{
    c_imu_delete( m );
    c_timer_delete( t );
    c_button_delete( b );
    c_haptic_delete( h );
    c_hw_delete( p );

    close( fd );

    if( munmap( map_base, MAP_SIZE ) == -1 )
      {
      fprintf( stderr, "Cannot close Memory Map");
      exit( 1 );
      }
    
    close( mem );
}

uchar GetGPIO( char gpio )
{
    return *((volatile ulong*)p->addr_level ) >> gpio & 1;
}

void SetGPIO( char gpio, char value )
{
    if (value)
      {
      *((ulong*)p->addr_set ) = 1 << gpio;
      }
    else
      {
      *((ulong*)p->addr_clear ) = 1 << gpio;
      }
}

// Add to_add_msec milliseconds to the timer defined by t_sec/t_nsec
void AddTime( ulong* t_sec, ulong* t_nsec, int to_add_msec )
{
    *t_sec += to_add_msec / THOUSAND;
    *t_nsec += to_add_msec % THOUSAND * MILLION;

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

    struct timespec now;
    ulong now_sec;
    ulong now_nsec;

    clock_gettime(CLOCK_REALTIME, &now);

    now_sec = (ulong)now.tv_sec;
    now_nsec = now.tv_nsec;
    
    h->input = 0;
    result = 0;

    if( initializing )
      {
      t->sampling_sec = now_sec;
      t->sampling_nsec = now_nsec;
      initializing = 0;
      }
    
    b->pressed = !GetGPIO( GPIO_BUTTON );

    if( !b->pressed_was
      && b->pressed )
      {
      t->button_sec = now_sec;
      t->button_nsec = now_nsec;
      b->engaged = 1;

      AddTime( &( t->button_sec ), &( t->button_nsec ), BUTTON_DOWNTIME_MS );
      }

    b->pressed_was = b->pressed;

    if( !b->timer_expired
      && !b->pressed
      && !b->led )
      {
      b->engaged = 0; 
      }
    
    if( poll( &mypoll, 1, 0 ))
      {    // User input buffer is not empty
      scanf( "%c", &( h->input ));

      if( h->input == BUTTON_TRIGGER )
        {    // Pressed Enter
        t->haptic_sec = now_sec;
        t->haptic_nsec = now_nsec;
        h->engaged = 1;        // Haptic motor is in state of toggling periodically 
        h->count = 0;
        h->on = 0;
        *((ulong*)p->addr_set ) = 1 << GPIO_HAPTIC;
        }
    }
    
    sampling_timer_expired = ( now_sec == t->sampling_sec
                             ? now_nsec >= t->sampling_nsec
                             : now_sec > t->sampling_sec );

    if( sampling_timer_expired )
      {
      AddTime( &( t->sampling_sec ), &( t->sampling_nsec ), SENSOR_PERIOD );
      result += 1;
      }
    
    b->timer_expired = b->engaged
                       && ( now_sec == t->button_nsec 
                          ? now_nsec >= t->button_nsec
                          : now_sec > t->button_sec );

    if( b->timer_expired )
      {
      if (b->led)
        {
        b->led = 0;
        *((ulong*)p->addr_clear ) = 1 << GPIO_BUTTONLED;
        b->engaged = 0;
        }
      else
        {
        b->led = 1;
        *((ulong*)p->addr_set ) = 1 << GPIO_BUTTONLED;
        AddTime( &( t->button_sec ), &( t->button_nsec ), BUTTONLED_PERIOD_MS );
        }
      }
    
    if( b->engaged )
      {
      result += 4;
      }
    
    h->timer_expired = h->engaged
                       && ( now_sec == t->haptic_sec
                          ? now_nsec >= t->haptic_nsec
                          : now_sec > t->haptic_sec );

    if( h->timer_expired )
      {
      AddTime( &( t->haptic_sec ), &( t->haptic_nsec ), HAPTIC_PERIOD_MS);

      h->on = !h->on;

      if( h->on)
        {
        *((ulong*)p->addr_set ) = 1 << GPIO_HAPTIC;
        ++h->count;
        }
      else
        {
        *((ulong*)p->addr_clear ) = 1 << GPIO_HAPTIC;

        if( h->count >= HAPTIC_TIMES )
          {
          h->engaged = 0;
          }
        }

      result += 2;
    }

    return result;
}

#ifdef __TEST__
// Output readings during a period between changes of LED brightness 
void test_read_data( ulong duration_ns, char current_led_address )
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
        c_imu_read( m );

        if( !sampling_count )
          {    // After one second passed
          if( !line_count )
            {
            printf("------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
                    printf("   IN0   IN1   IN2   IN3   IN4   IN5   IN6   IN7      GYR_X     GYR_Y     GYR_Z    ACC_X   ACC_Y   ACC_Z    INT1  INT2  INT3  INT4 Haptic Hall Button  Batt \n");
                    printf("------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
            }

          c_imu_average( m );

          change_addr_to(ADC_ADDRESS);
          for( int i = 0; i < ADC_CHANNELS; i++ )
            {
            in[i] = i2c_smbus_read_word_data( fd, 32 + i );
            printf( "%6d", test_swap_bytes( in[i] ));
            }

          if( int4 )
            {
            i2c_smbus_read_word_data( fd, ADC_INTERRUPT_REG ); // Clear interrupt register
            }

          change_addr_to( FUELGAUGE_ADDRESS );

//        if( int3 )
            {
            pwr_lsbyte = i2c_smbus_read_byte_data( fd, FG_PWR_LSBYTE_REG );
            pwr_msbyte = i2c_smbus_read_byte_data( fd, FG_PWR_MSBYTE_REG );

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

          printf( "%9.1f", m->gyr_x * GYR_RESOLUTION );
          test_print_unicode( DEGREE_SYMBOL );
          printf( "/s%7.1f", m->gyr_y * GYR_RESOLUTION );
          test_print_unicode( DEGREE_SYMBOL );
          printf( "/s%7.1f", m->gyr_z * GYR_RESOLUTION );
          test_print_unicode( DEGREE_SYMBOL );
          printf( "/s%7.2fg", m->acc_x * ACC_RESOLUTION );
          printf( "%7.2fg", m->acc_y * ACC_RESOLUTION );
          printf( "%7.2fg", m->acc_z * ACC_RESOLUTION );
          printf( "%s", int1 ? "      1 " : "      0 " );
          printf( "%s", int2 ? "    1 " : "    0 " );
          printf( "%s", int3 ? "    1 " : "    0 " );
          printf( "%s", int4 ? "    1 " : "    0 " );
          printf( "%s", haptic_state ? "    1 " : "    0 " );
          printf( "%s", hall_state ? "    1 " : "    0 " );
          printf( "%s", buttonled_state ? "    1 " : "    0 " );
          printf( "%s", charging_state );
          printf( "\n" );
          fflush( stdout );

          ++line_count;
          line_count %= LINES_BETWEEN_HEADINGS;

          c_imu_clear( m );

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

      change_addr_to(current_led_address);
      }
}                                           

void test_pwm_write( int fd, int address )
{
    change_addr_to( address );
    i2c_smbus_write_byte_data( fd, LED_MODE1_REG, LED_NORMAL );

    for ( int i = LEDOUT_START_REG; i < LEDOUT_END_REG; i++ )
      {
      i2c_smbus_write_byte_data( fd, i, INDIV_CTRL );
      }

    for( int i = PWM_END_REG - 1; i >= PWM_START_REG; i-- )
      {
      i2c_smbus_write_byte_data( fd, i, 0 );
      }
}
    
void test_pwm_read( int fd, int address )
{
    change_addr_to( address );

    for( int i = PWM_START_REG; i < PWM_END_REG; i++ )
      {
      i2c_smbus_write_byte_data( fd, i, BRIGHTNESS );
      test_read_data( LED_ON_MSEC * MILLION, address );
      i2c_smbus_write_byte_data( fd, i, 0 );
      test_read_data( LED_OFF_MSEC * MILLION, address );
      }
}

// For printing the degree symbol
void test_print_unicode( ushort code )
{
    fclose( stdout );
    freopen( "/dev/tty", "a", stdout );
    wprintf( L"%lc", code );
    fclose( stdout );
    freopen( "/dev/tty", "a", stdout );
}

// Swap bytes in a two-byte number n
int test_swap_bytes( ushort n )
{
    ushort a = n >> 8;
    ushort b = n << 8;
    return (( a << 8 ) + ( b >> 8 )); 
}

int main( int argc, char** argv )
{
    int value;
    int c = 1;

    struct timespec wake_acc;
    struct timespec wake_gyr;

    WandHwOpen();

    wake_acc.tv_sec = 0;
    wake_acc.tv_nsec = ACC_TIME_TO_NORM_NS;

    wake_gyr.tv_sec = 0;
    wake_gyr.tv_nsec = GYR_TIME_TO_NORM_NS;

    setlocale( LC_CTYPE, "" );   // For printing Unicode characters
    
    // Setting default parameters for ADC
    i2c_smbus_write_byte_data( fd, ADC_CONFIG_REG, ADC_CONFIG_NORMAL );
    i2c_smbus_write_byte_data( fd, ADC_ADVANCED_REG, ADC_ADVANCED_NORMAL );
    i2c_smbus_write_byte_data( fd, ADC_DISABLE_CHANNELS_REG, 0 ); // Reset disabling channels, if any
    
    // Setting limits for ADC readings (exceeding these limits causes an interrupt)

    for( int i = 0; i < ADC_CHANNELS; i++ )
      {
      value = ADC_VOLTAGE * ( ADC_CHANNELS - i ) / ADC_CHANNELS / 16;
      value += ADC_TOLERANCE;
      i2c_smbus_write_byte_data( fd, FIRST_ADC_LIMIT_REG + 2 * i, value );
      value -= 2 * ADC_TOLERANCE;
      i2c_smbus_write_byte_data( fd, FIRST_ADC_LIMIT_REG + 1 + 2 * i, value );
      }
    
    // Setting default parameters for accelerometer/gyroscope
    change_addr_to( SENSOR_ADDRESS );
    i2c_smbus_write_byte_data( fd, CMD, ACC_TO_NORM );
    nanosleep( &wake_acc, NULL );
    i2c_smbus_write_byte_data( fd, CMD, GYR_TO_NORM );
    nanosleep( &wake_gyr, NULL );
    i2c_smbus_write_byte_data( fd, INT_EN, ANYMOT_SINTAP_EN );
    i2c_smbus_write_byte_data( fd, INT_OUT_CTRL, CONF_PINS );
    i2c_smbus_write_byte_data( fd, INT_LATCH, LATCH );
    i2c_smbus_write_byte_data( fd, INT_MAP1, ANYMOT_MAP );
    i2c_smbus_write_byte_data( fd, INT_MAP2, SINTAP_MAP );
    
    // Setting default/initial parameters for LED drivers
    test_pwm_write( fd, LED_DRIVER1_ADDRESS );
    test_pwm_write( fd, LED_DRIVER2_ADDRESS );
    /* 
    LEDs switch on/off or change brightness in a certain (cyclic) pattern;
    between these changes, the control is passed to the test_read_data() function
    */
    while( 1 )
      {
      printf( "Counter %d\n", c++ );
      test_pwm_read( fd, LED_DRIVER1_ADDRESS );
      test_pwm_read( fd, LED_DRIVER2_ADDRESS );
      }


    WandHwClose();
}
#endif

