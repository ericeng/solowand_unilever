/*
This program demonstrates controlling the following devices: 
- two LED drivers TLC59116; 
- accelerometer/gyroscope BMI160; 
- analog-to-digital converter ADC128D818;
- battery fuel gauge bq27421-G1 
- haptic motor; 
- Hall sensor. 
While LEDS are being switched on/off
and their brightness is changing in a certain pattern, the display is updated
every second with ACD data (IN1 through IN8), averaged (for the last second) 
accelerometer and giroscope data on X, Y, and Z axes, interrupt states, and 
haptic motor and Hall sensor states. Haptic motor is activated by user pressing 
Enter.
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

#define ADC_ADDRESS                 0x1D
#define FUELGAUGE_ADDRESS           0x55
#define LED_DRIVER1_ADDRESS         0x60
#define LED_DRIVER2_ADDRESS         0x61
#define SENSOR_ADDRESS              0x69
#define BRIGHTNESS0                   10       
#define BRIGHTNESS1                  100
#define BRIGHTNESS2                    0
#define BRIGHTNESS3                   50
#define BRIGHTNESS_CHANGE_PER_SEC     60
#define BRIGHTNESS_INCREMENT           1
#define LED_ON_MSEC                  500        // Time between switching consequtive LEDs on
#define LED_OFF_MSEC                 500        // Time between switching consequtive LEDs off
#define LED_MODE1_REG                  0        // Mode register 1 for LED driver
#define LED_NORMAL                     0        // Normal value for mode register 1
#define NUMBER_OF_LEDS                10
#define PWM_START_REG                  2        // First brightness register for LED driver
#define LEDOUT_START_REG              20        // First output state register for LED driver
#define INDIV_CTRL                   170        // Mode of individual brightness controlling
#define FG_PWR_MSBYTE_REG             25
#define FG_PWR_LSBYTE_REG             24
#define ADC_CONFIG_REG                 0
#define ADC_CONFIG_NORMAL              3
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
#define LATCH                       0x08        // Disable input for pin1 and pin2, latch = 40 ms
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

int fd;
const unsigned long sensor_period = THOUSAND / FREQ;                         // Period between samplings (msec)
const unsigned char pwm_end_reg = PWM_START_REG + NUMBER_OF_LEDS;           // First register after brightness registers for LED driver
const unsigned char ledout_end_reg = LEDOUT_START_REG + (NUMBER_OF_LEDS - 1) / 4 + 1; // First register after output state registers for LED driver
void *level_addr, *set_addr, *clear_addr;

/*
The following four function declarations are included because they are missing
from standard library header file i2c-dev.h
*/
static inline __s32 i2c_smbus_access(int file, char read_write, __u8 command, 
                                  int size, union i2c_smbus_data *data)
{
	struct i2c_smbus_ioctl_data args;

	args.read_write = read_write;
	args.command = command;
	args.size = size;
	args.data = data;
	return ioctl(file,I2C_SMBUS,&args);
}

static inline __s32 i2c_smbus_read_byte_data(int file, __u8 command)
{
	union i2c_smbus_data data;
	if (i2c_smbus_access(file,I2C_SMBUS_READ,command,
	                     I2C_SMBUS_BYTE_DATA,&data))
		return -1;
	else
		return 0x0FF & data.byte;
}

static inline __s32 i2c_smbus_write_byte_data(int file, __u8 command, 
                                           __u8 value)
{
	union i2c_smbus_data data;
	data.byte = value;
	return i2c_smbus_access(file,I2C_SMBUS_WRITE,command,
	                        I2C_SMBUS_BYTE_DATA, &data);
}

static inline __s32 i2c_smbus_read_word_data(int file, __u8 command)
{
	union i2c_smbus_data data;
	if (i2c_smbus_access(file,I2C_SMBUS_READ,command,
	                     I2C_SMBUS_WORD_DATA, &data))
		return -1;
	else
		return 0x0FFFF & data.word;
}

// Get gpio level
char get_level (char gpio) {
    return *((volatile unsigned long *)level_addr) >> gpio & 1;
}

// Set gpio level to value (1 or 0)
void set_level (char gpio, char value) {
    if (value) *((unsigned long *)set_addr) = 1 << gpio;
    else *((unsigned long *)clear_addr) = 1 << gpio;
}

// Add to_add nanoseconds to the timer defined by t_sec/t_nsec
void add_time (unsigned long *t_sec, unsigned long *t_nsec, int to_add_msec) {
    *t_sec += to_add_msec / THOUSAND;
    *t_nsec += to_add_msec % THOUSAND * MILLION;
    if (*t_nsec > BILLION) {
        *t_nsec -= BILLION;
        ++*t_sec;
    }
}

/* Return integer value coding the information of timers expired:
0 - none of the timers expired;
1 - sampling timer expired (time to get readings);
2 - haptic motor timer expired (time to toggle haptic motor);
3 - both timers expired;
*/
char timers_expired() {
    static struct pollfd mypoll = {STDIN_FILENO, POLLIN | POLLPRI};
    static char rslt, sampling_timer_expired, initializing = 1,
        haptic_timer_expired, haptic_engaged = 0, haptic_on = 0,
        haptic_input, haptic_count,
        button_timer_expired = 0, button_was_pressed = 0, button_pressed = 0,
        button_engaged = 0, buttonled_on = 0;
    static struct timespec now;
    static unsigned long now_sec, now_nsec,
        timer_sampling_sec, timer_sampling_nsec,
        timer_haptic_sec, timer_haptic_nsec,
        timer_button_sec, timer_button_nsec,
        timer_buttonled_sec, timer_buttonled_nsec;
        
    //Get current time
    clock_gettime(CLOCK_REALTIME, &now);
    now_sec = (unsigned long)now.tv_sec;
    now_nsec = now.tv_nsec;
    
    haptic_input = 0;
    rslt = 0;
    if (initializing) {
        timer_sampling_sec = now_sec;
        timer_sampling_nsec = now_nsec;
        initializing = 0;
    }
    
    button_pressed = !get_level(GPIO_BUTTON);
    if (!button_was_pressed && button_pressed) {
        timer_button_sec = now_sec;
        timer_button_nsec = now_nsec;
        button_engaged = 1;
        add_time(&timer_button_sec, &timer_button_nsec, BUTTON_DOWNTIME_MS);
    }
    button_was_pressed = button_pressed;
    if (!button_timer_expired && !button_pressed && !buttonled_on) button_engaged = 0; 
    
    if (poll(&mypoll, 1, 0)) {    // User input buffer is not empty
        scanf("%c", &haptic_input);
        if (haptic_input == 10) {    // Pressed Enter
            timer_haptic_sec = now_sec;
            timer_haptic_nsec = now_nsec;
            haptic_engaged = 1;        // Haptic motor is in state of toggling periodically 
            haptic_count = 0;
            haptic_on = 0;
            *((unsigned long *)set_addr) = 1 << GPIO_HAPTIC;
        }
    }
    
    sampling_timer_expired = (now_sec == timer_sampling_sec ? now_nsec >= timer_sampling_nsec : now_sec > timer_sampling_sec);
    if (sampling_timer_expired) {
        add_time(&timer_sampling_sec, &timer_sampling_nsec, sensor_period);
        rslt += 1;
    }
    
    button_timer_expired = button_engaged && (now_sec == timer_button_nsec ? now_nsec >= timer_button_nsec : now_sec > timer_button_sec);
    if (button_timer_expired) {
        if (buttonled_on) {
            buttonled_on = 0;
            *((unsigned long *)clear_addr) = 1 << GPIO_BUTTONLED;
            button_engaged = 0;
        } else {
            buttonled_on = 1;
            *((unsigned long *)set_addr) = 1 << GPIO_BUTTONLED;
            add_time(&timer_button_sec, &timer_button_nsec, BUTTONLED_PERIOD_MS);
        }
//        rslt += 4;
    }
    
    if (button_engaged) rslt += 4;
    
    haptic_timer_expired = haptic_engaged && (now_sec == timer_haptic_sec ? now_nsec >= timer_haptic_nsec : now_sec > timer_haptic_sec);
    if (haptic_timer_expired) {
        add_time(&timer_haptic_sec, &timer_haptic_nsec, HAPTIC_PERIOD_MS);
        haptic_on = !haptic_on;
        if (haptic_on) {
            *((unsigned long *)set_addr) = 1 << GPIO_HAPTIC;
            ++haptic_count;
        } else {
            *((unsigned long *)clear_addr) = 1 << GPIO_HAPTIC;
            if (haptic_count >= HAPTIC_TIMES) haptic_engaged = 0;
        }
        rslt += 2;
    }
    return rslt;
}

// Connect to I2C address new_addr
void change_addr_to (char new_addr) {
    static char current_addr = 0;
    if (current_addr != new_addr) {
        ioctl(fd, I2C_SLAVE, new_addr);
        current_addr = new_addr;
    }
}

// Swap bytes in a two-byte number n
int swap_bytes (unsigned short n) {
    return (n % 256) * 16 + n / 4096;
}

// For printing the degree symbol
void print_unicode(unsigned short code) {
    fclose(stdout);
    freopen("/dev/tty", "a", stdout);
    wprintf(L"%lc", code);
    fclose(stdout);
    freopen("/dev/tty", "a", stdout);
}

// Output readings during a period between changes of LED brightness 
void read_data (unsigned long duration_ns, char current_led_driver_address) {
    static struct timespec led_time;
    static unsigned long start_sec, start_nsec, elapsed_ns;
    static char int1, int2, int3, int4, 
        buttonled_state, haptic_state, hall_state, timer_state;
    static unsigned char gyr_x_lsb, gyr_x_msb, gyr_y_lsb, gyr_y_msb,
    gyr_z_lsb, gyr_z_msb, acc_x_lsb, acc_x_msb,
    acc_y_lsb, acc_y_msb, acc_z_lsb, acc_z_msb,
    pwr_msbyte, pwr_lsbyte;
    static signed long gyr_x, gyr_y, gyr_z, acc_x, acc_y, acc_z;
    unsigned char i;
    static char charging_state[8];
    static int in[ADC_CHANNELS]; 
    static int line_count = 0, sampling_count = 0;
    clock_gettime(CLOCK_REALTIME, &led_time);
    start_nsec = led_time.tv_nsec;
    start_sec = (long)led_time.tv_sec;
    elapsed_ns = 0;
    while(elapsed_ns < duration_ns) {
        clock_gettime(CLOCK_REALTIME, &led_time);
        if (!int1 && get_level(GPIO_INT1)) int1 = 1;
        if (!int2 && get_level(GPIO_INT2)) int2 = 1;
        if (!int3 && !get_level(GPIO_INT3)) int3 = 1;
        if (!int4 && !get_level(GPIO_INT4)) int4 = 1;
        if (!get_level(GPIO_HALL)) hall_state = 1;
        timer_state = timers_expired();
        if (timer_state >> 2 & 1) {  // Buttonled timer expired
            buttonled_state = 1;
        }
        if (timer_state >> 1 & 1) {  // Haptic timer expired
            haptic_state = 1;
        }
        if (timer_state & 1) {      // Sampling timer expired
            ++sampling_count;
            sampling_count %= FREQ;  // Nimber of sampling within the current second
            change_addr_to(SENSOR_ADDRESS);
            gyr_x_lsb = i2c_smbus_read_byte_data(fd, GYR_X_LSB_REG);
            gyr_x_msb = i2c_smbus_read_byte_data(fd, GYR_X_MSB_REG);
            gyr_y_lsb = i2c_smbus_read_byte_data(fd, GYR_Y_LSB_REG);
            gyr_y_msb = i2c_smbus_read_byte_data(fd, GYR_Y_MSB_REG);
            gyr_z_lsb = i2c_smbus_read_byte_data(fd, GYR_Z_LSB_REG);
            gyr_z_msb = i2c_smbus_read_byte_data(fd, GYR_Z_MSB_REG);
            acc_x_lsb = i2c_smbus_read_byte_data(fd, ACC_X_LSB_REG);
            acc_x_msb = i2c_smbus_read_byte_data(fd, ACC_X_MSB_REG);
            acc_y_lsb = i2c_smbus_read_byte_data(fd, ACC_Y_LSB_REG);
            acc_y_msb = i2c_smbus_read_byte_data(fd, ACC_Y_MSB_REG);
            acc_z_lsb = i2c_smbus_read_byte_data(fd, ACC_Z_LSB_REG);
            acc_z_msb = i2c_smbus_read_byte_data(fd, ACC_Z_MSB_REG);
            gyr_x += (signed short)(gyr_x_msb << 8 | gyr_x_lsb);
            gyr_y += (signed short)(gyr_y_msb << 8 | gyr_y_lsb);
            gyr_z += (signed short)(gyr_z_msb << 8 | gyr_z_lsb);
            acc_x += (signed short)(acc_x_msb << 8 | acc_x_lsb);
            acc_y += (signed short)(acc_y_msb << 8 | acc_y_lsb);
            acc_z += (signed short)(acc_z_msb << 8 | acc_z_lsb);
            if (!sampling_count) {    // After one second passed
                if (!line_count) {
                    printf("------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
                    printf("   IN0   IN1   IN2   IN3   IN4   IN5   IN6   IN7      GYR_X     GYR_Y     GYR_Z    ACC_X   ACC_Y   ACC_Z    INT1  INT2  INT3  INT4 Haptic Hall Button  Batt \n");
                    printf("------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
                }
                gyr_x /= FREQ;  // Averaging
                gyr_y /= FREQ;
                gyr_z /= FREQ;
                acc_x /= FREQ;
                acc_y /= FREQ;
                acc_z /= FREQ;
                change_addr_to(ADC_ADDRESS);
                for (i = 0; i < ADC_CHANNELS; i++) {
                    in[i] = i2c_smbus_read_word_data(fd, 32 + i);
                    printf ("%6d", swap_bytes(in[i]));
                }
                if (int4) i2c_smbus_read_word_data(fd, ADC_INTERRUPT_REG); // Clear interrupt register
                change_addr_to(FUELGAUGE_ADDRESS);
//                if (int3) {
                    pwr_lsbyte = i2c_smbus_read_byte_data(fd, FG_PWR_LSBYTE_REG);
                    pwr_msbyte = i2c_smbus_read_byte_data(fd, FG_PWR_MSBYTE_REG);
                    if (pwr_msbyte >> 7) strcpy(charging_state, "    D ");   // Average power < 0, i.e. Discharging
                    else if (pwr_lsbyte | pwr_msbyte) strcpy(charging_state, "    C ");   // Average power > 0, i.e. Charging
                    else strcpy(charging_state, "    - ");   // Average power == 0
//                }
                printf ("%9.1f", gyr_x * GYR_RESOLUTION);
                print_unicode(DEGREE_SYMBOL);
                printf ("/s%7.1f", gyr_y * GYR_RESOLUTION);
                print_unicode(DEGREE_SYMBOL);
                printf ("/s%7.1f", gyr_z * GYR_RESOLUTION);
                print_unicode(DEGREE_SYMBOL);
                printf ("/s%7.2fg", acc_x * ACC_RESOLUTION);
                printf ("%7.2fg", acc_y * ACC_RESOLUTION);
                printf ("%7.2fg", acc_z * ACC_RESOLUTION);
                printf ("%s", int1 ? "      1 " : "      0 ");
                printf ("%s", int2 ? "    1 " : "    0 ");
                printf ("%s", int3 ? "    1 " : "    0 ");
                printf ("%s", int4 ? "    1 " : "    0 ");
                printf ("%s", haptic_state ? "    1 " : "    0 ");
                printf ("%s", hall_state ? "    1 " : "    0 ");
                printf ("%s", buttonled_state ? "    1 " : "    0 ");
                printf ("%s", charging_state);
                printf ("\n");
                fflush(stdout);
                ++line_count;
                line_count %= LINES_BETWEEN_HEADINGS;
                gyr_x = 0;
                gyr_y = 0;
                gyr_z = 0;
                acc_x = 0;
                acc_y = 0;
                acc_z = 0;
                int1 = 0;
                int2 = 0;
                int3 = 0;
                int4 = 0;
                if (haptic_state) haptic_state = 0;
                if (buttonled_state) buttonled_state = 0;
                hall_state = 0;
            }
        }
        elapsed_ns = BILLION * ((long)led_time.tv_sec - start_sec) + led_time.tv_nsec - start_nsec;
    }
    change_addr_to(current_led_driver_address);
}                                           

// Change LEDs brightness gradually
void gradual_change(unsigned char start, unsigned char end,
                 unsigned char increment, int rate) {
    unsigned long led_period = BILLION / rate * increment;
    unsigned char i, brightness = start;
    while (1) {
        if (end > start) {
            brightness += increment;
            if (brightness > end) brightness = end;
        } else {
            brightness -= increment;
            if (brightness < end) brightness = end;
        }
        change_addr_to(LED_DRIVER2_ADDRESS);
        for (i = PWM_START_REG; i < pwm_end_reg; i++) {
            i2c_smbus_write_byte_data(fd, i, brightness);
        }
        change_addr_to(LED_DRIVER1_ADDRESS);
        for (i = PWM_START_REG; i < pwm_end_reg; i++) {
            i2c_smbus_write_byte_data(fd, i, brightness);
        }
        if (brightness == end) break;
        read_data(led_period, LED_DRIVER1_ADDRESS);
    }
}

int main() {
    int mem, value, i;
    void *map_base;
    struct timespec wake_acc, wake_gyr;
    wake_acc.tv_sec = 0;
    wake_acc.tv_nsec = ACC_TIME_TO_NORM_NS;
    wake_gyr.tv_sec = 0;
    wake_gyr.tv_nsec = GYR_TIME_TO_NORM_NS;
    setlocale(LC_CTYPE, "");   // For printing Unicode characters
    
    if ((mem = open ("/dev/mem", O_RDWR | O_SYNC)) == -1)
        fprintf(stderr, "Cannot open /dev/mem\n"), exit(1);
        
    if ((map_base = (long *)mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem, GPIO_BASE & ~MAP_MASK)) == NULL)
        fprintf(stderr, "Cannot open Memory Map\n"), exit(1);
    
    // Addresses of GPIO registers
    level_addr = (void *)(map_base + (GPIO_BASE & MAP_MASK) + LEVEL_OFFSET);
    set_addr = (void *)(map_base + (GPIO_BASE & MAP_MASK) + SET_OFFSET);
    clear_addr = (void *)(map_base + (GPIO_BASE & MAP_MASK) + CLEAR_OFFSET);

    fd = open(BUS, O_RDWR);
    ioctl(fd, I2C_SLAVE, ADC_ADDRESS);
    
    // Setting default parameters for ADC
    i2c_smbus_write_byte_data(fd, ADC_CONFIG_REG, ADC_CONFIG_NORMAL);
    i2c_smbus_write_byte_data(fd, ADC_ADVANCED_REG, ADC_ADVANCED_NORMAL);
    i2c_smbus_write_byte_data(fd, ADC_DISABLE_CHANNELS_REG, 0); // Reset disabling channels, if any
    
    // Setting limits for ADC readings (exceeding these limits causes an interrupt)
    for (i = 0; i < ADC_CHANNELS; i++) {
        value = ADC_VOLTAGE * (ADC_CHANNELS - i) / ADC_CHANNELS / 16;
        value += ADC_TOLERANCE;
        i2c_smbus_write_byte_data(fd, FIRST_ADC_LIMIT_REG + 2 * i, value);
        value -= 2 * ADC_TOLERANCE;
        i2c_smbus_write_byte_data(fd, FIRST_ADC_LIMIT_REG + 1 + 2 * i, value);
    }
    
    // Setting default parameters for accelerometer/gyroscope
    change_addr_to(SENSOR_ADDRESS);
    i2c_smbus_write_byte_data(fd, CMD, ACC_TO_NORM);
    nanosleep(&wake_acc, NULL);
    i2c_smbus_write_byte_data(fd, CMD, GYR_TO_NORM);
    nanosleep(&wake_gyr, NULL);
    i2c_smbus_write_byte_data(fd, INT_EN, ANYMOT_SINTAP_EN);
    i2c_smbus_write_byte_data(fd, INT_OUT_CTRL, CONF_PINS);
    i2c_smbus_write_byte_data(fd, INT_LATCH, LATCH);
    i2c_smbus_write_byte_data(fd, INT_MAP1, ANYMOT_MAP);
    i2c_smbus_write_byte_data(fd, INT_MAP2, SINTAP_MAP);
    
    // Setting default/initial parameters for LED drivers
    change_addr_to(LED_DRIVER1_ADDRESS);
    i2c_smbus_write_byte_data(fd, LED_MODE1_REG, LED_NORMAL);
    for (i = LEDOUT_START_REG; i < ledout_end_reg; i++) {
        i2c_smbus_write_byte_data(fd, i, INDIV_CTRL);
    }
    for (i = pwm_end_reg - 1; i >= PWM_START_REG; i--) {
        i2c_smbus_write_byte_data(fd, i, 0);
    }
    change_addr_to(LED_DRIVER2_ADDRESS);
    i2c_smbus_write_byte_data(fd, LED_MODE1_REG, LED_NORMAL);
    for (i = LEDOUT_START_REG; i < ledout_end_reg; i++) {
        i2c_smbus_write_byte_data(fd, i, INDIV_CTRL);
    }
    for (i = pwm_end_reg - 1; i >= PWM_START_REG; i--) {
        i2c_smbus_write_byte_data(fd, i, 0);
    }
    
    /* 
    LEDs switch on/off or change brightness in a certain (cyclic) pattern;
    between these changes, the control is passed to the read_data() function
    */
    while (1) {
        change_addr_to(LED_DRIVER2_ADDRESS);
        for (i = pwm_end_reg - 1; i >= PWM_START_REG; i--) {
            i2c_smbus_write_byte_data(fd, i, BRIGHTNESS0);
            read_data(LED_ON_MSEC * MILLION, LED_DRIVER2_ADDRESS);
        }
        change_addr_to(LED_DRIVER1_ADDRESS);
        for (i = pwm_end_reg - 1; i >= PWM_START_REG; i--) {
            i2c_smbus_write_byte_data(fd, i, BRIGHTNESS0);
            read_data(LED_ON_MSEC * MILLION, LED_DRIVER1_ADDRESS);
        }
        gradual_change (BRIGHTNESS0, BRIGHTNESS1, BRIGHTNESS_INCREMENT, BRIGHTNESS_CHANGE_PER_SEC);
        gradual_change (BRIGHTNESS1, BRIGHTNESS2, BRIGHTNESS_INCREMENT, BRIGHTNESS_CHANGE_PER_SEC);
        gradual_change (BRIGHTNESS2, BRIGHTNESS3, BRIGHTNESS_INCREMENT, BRIGHTNESS_CHANGE_PER_SEC);
        for (i = PWM_START_REG; i < pwm_end_reg; i++) {
            i2c_smbus_write_byte_data(fd, i, 0);
            read_data(LED_OFF_MSEC * MILLION, LED_DRIVER1_ADDRESS);
        }
        change_addr_to(LED_DRIVER2_ADDRESS);
        for (i = PWM_START_REG; i < pwm_end_reg; i++) {
            i2c_smbus_write_byte_data(fd, i, 0);
            read_data(LED_OFF_MSEC * MILLION, LED_DRIVER2_ADDRESS);
        }
    }
    close(fd);
    if (munmap(map_base, MAP_SIZE) == -1)
        fprintf(stderr, "Cannot close Memory Map"), exit(1);
    
    close(mem);
}
