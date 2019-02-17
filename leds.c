#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <linux/types.h>

#define DRIVER_CAP                    16
#define LEDS                          20
#define LEVEL_OFFSET                0x34
#define SET_OFFSET                  0x1C
#define CLEAR_OFFSET                0x28
#define CLK_FREQ                     100        // Clock frequency
#define LE_TO_OE                   10000
#define BETWEEN_CYCLES             10000
#define GPIO_LE                        5        // Latch enable
#define GPIO_SDI                       6        // Data input
#define GPIO_OE                        7        // Output enable
#define GPIO_CLK                       8        // Clock
#define MAP_SIZE                    4096UL
#define MAP_MASK           (MAP_SIZE - 1)
#define GPIO_BASE             0x20200000        //Raspberry Pi 0
#define BILLION               1000000000

void *level_addr, *set_addr, *clear_addr;
const long clock_period = BILLION / CLK_FREQ;
const char number_of_clock_pulses = DRIVER_CAP * ((LEDS - 1) / DRIVER_CAP + 1);
long clock_pulse, between_clock_pulses;

struct timespec now;
unsigned long now_sec, now_nsec, timer_sec, timer_nsec;

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
void add_time (unsigned long *t_sec, unsigned long *t_nsec, long nsec_to_add) {
    *t_nsec += nsec_to_add;
    if (*t_nsec > BILLION) {
        *t_nsec -= BILLION;
        ++*t_sec;
    }
}

void change_level (char gpio, char level, long ns_to_add) {
    while(1) {
        clock_gettime(CLOCK_REALTIME, &now);
        now_sec = (unsigned long)now.tv_sec;
        now_nsec = now.tv_nsec;
        if (now_sec == timer_sec ? now_nsec >= timer_nsec : now_sec > timer_sec) break;
    }
    if (gpio == GPIO_SDI) set_level (GPIO_CLK, 0);
    set_level (gpio, level);
    add_time(&timer_sec, &timer_nsec, ns_to_add);
}

void light (char led_number, long oe_pulse) {
    int i, j;
    clock_gettime(CLOCK_REALTIME, &now);
    now_sec = (unsigned long)now.tv_sec;
    now_nsec = now.tv_nsec;
    timer_sec = now_sec;
    timer_nsec = now_nsec;
    for (i = 0; i < number_of_clock_pulses; ++i) {
        if (i == 31 - led_number) change_level(GPIO_SDI, 1, between_clock_pulses);
        else if (i == 31 - led_number + 1) change_level(GPIO_SDI, 0, between_clock_pulses);
        else change_level(GPIO_CLK, 0, between_clock_pulses);
        change_level(GPIO_CLK, 1, clock_pulse);
    }
    change_level(GPIO_CLK, 0, between_clock_pulses);
    change_level(GPIO_LE, 1, clock_pulse);
    change_level(GPIO_LE, 0, LE_TO_OE);
    change_level(GPIO_OE, 0, oe_pulse);
    change_level(GPIO_OE, 1, BETWEEN_CYCLES);
}

int main() {
    int i, mem;
    void *map_base;
    if ((mem = open ("/dev/mem", O_RDWR | O_SYNC)) == -1)
        fprintf(stderr, "Cannot open /dev/mem\n"), exit(1);
        
    if ((map_base = (long *)mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem, GPIO_BASE & ~MAP_MASK)) == NULL)
        fprintf(stderr, "Cannot open Memory Map\n"), exit(1);
        
    // Addresses of GPIO registers
    level_addr = (void *)(map_base + (GPIO_BASE & MAP_MASK) + LEVEL_OFFSET);
    set_addr = (void *)(map_base + (GPIO_BASE & MAP_MASK) + SET_OFFSET);
    clear_addr = (void *)(map_base + (GPIO_BASE & MAP_MASK) + CLEAR_OFFSET);
    
    clock_pulse = (clock_period < 40 ? 20 : clock_period / 2);
    between_clock_pulses = clock_period - clock_pulse;
    
    set_level (GPIO_LE, 0);
    set_level (GPIO_SDI, 0);
    set_level (GPIO_OE, 1);
    set_level (GPIO_CLK, 0);

    while (1) {
/*        for (i = 0; i < 20; i++) {
            light(i, 10000000);
        }  */
    
        
       light (0, 1000000);
       light (1, 1000000);
       light (2, 1000000);
       light (3, 1000000);
       light (4, 1000000);
       light (5, 20000000);
       light (6, 20000000);
       light (7, 20000000);
       light (8, 50000000);
       light (9, 100000000);
       light (10, 1000000);
       light (11, 1000000);
       light (12, 1000000);
       light (13, 1000000);
       light (14, 1000000);
       light (15, 20000000);
       light (16, 20000000);
       light (17, 20000000);
       light (18, 50000000);
       light (19, 100000000);
}    
    if (munmap(map_base, MAP_SIZE) == -1)
    fprintf(stderr, "Cannot close Memory Map"), exit(1);
    
    close(mem);

}
