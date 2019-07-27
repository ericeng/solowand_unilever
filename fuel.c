
#include "e_ventures.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/types.h>
#include "e_ventures_specific.h"

#define I2C_ADDRESS                 0x55
#define BUS                  "/dev/i2c-1"
#define REGS                          50
#define GPIO_BASE             0x20200000
#define LEVEL_OFFSET                0x34
#define GPIO                          22
#define MAP_SIZE                    4096UL
#define MAP_MASK           (MAP_SIZE - 1)
#define I2C_SMBUS_BLOCK_MAX	          32
#define I2C_SLAVE                 0x0703
#define I2C_SMBUS                 0x0720
#define I2C_SMBUS_READ	               1
#define I2C_SMBUS_WRITE	               0
#define I2C_SMBUS_BYTE_DATA	           2 

uchar yesno[4];
uchar bin_str[9];
uchar reg[REGS];
int fd;

void print_state();
int signed_int( uint i );
void is_one( uchar val, uchar bit );
void to_bin( uchar x );

int main()
{
    void *map_base, *level_addr;
    unsigned long value;
    int mem;
    char bit = 0, new_bit;
    
    if ((mem = open ("/dev/mem", O_RDWR | O_SYNC)) == -1)
        fprintf(stderr, "Cannot open /dev/mem\n"), exit(1);
        
    if ((map_base = (long *)mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem, GPIO_BASE & ~MAP_MASK)) == NULL)
        fprintf(stderr, "Cannot open Memory Map\n"), exit(1);
    
    fd = open(BUS, O_RDWR);
    ioctl(fd, I2C_SLAVE, I2C_ADDRESS);

    level_addr = (void *)(map_base + (GPIO_BASE & MAP_MASK) + LEVEL_OFFSET);
    
    while(1) {
        new_bit = *((volatile unsigned long *)level_addr) >> GPIO & 1;
        if (!bit && new_bit) print_state();
        bit = new_bit;
    }
    
    close(fd);
    
    if (munmap(map_base, MAP_SIZE) == -1)
        fprintf(stderr, "Cannot close Memory Map"), exit(1);
    
    close(mem);
}

void print_state() {
    char soh_ready = 1;
    int i;
    float t;
    
    for (i = 0; i < REGS; i++) {
      reg[i] = i2c_smbus_read_byte_data(fd, i);
    }
    
    is_one(reg[1], 7);
    printf ("SHUTDOWN_ENABLE subcommand received:  %s\n", yesno);
    is_one(reg[1],6);
    printf ("Watchdog Reset performed:             %s\n", yesno);
    is_one(reg[1],5);
    printf ("SEALED state:                         %s\n", yesno);
    is_one(reg[1],4);
    printf ("Calibration mode:                     %s\n", yesno);
    is_one(reg[1],3);
    printf ("Coulomb counter autocalibr active:    %s\n", yesno);
    is_one(reg[1],2);
    printf ("Board calibration active:             %s\n", yesno);
    is_one(reg[1],1);
    printf ("Qmax updated:                         %s\n", yesno);
    is_one(reg[1],0);
    printf ("Resistance updated:                   %s\n", yesno);
    is_one(reg[0],7);
    printf ("Initialization complete:              %s\n", yesno);
    is_one(reg[0],6);
    printf ("Request for SLEEP->HIBERNATE issued:  %s\n", yesno);
    is_one(reg[0],4);
    printf ("SLEEP mode:                           %s\n", yesno);
    is_one(reg[0],3);
    printf ("Constant-power model:                 %s\n", yesno);
    is_one(reg[0],2);
    printf ("Ra table updates disabled:            %s\n", yesno);
    is_one(reg[0],1);
    printf ("Cell voltages OK for Qmax updates:    %s\n", yesno);
    printf ("Temperature:                          %.1f\n", .1 * (reg[3] << 8 | reg[2]) - 273.2);
    printf ("Voltage:                              %d mV\n", reg[5] << 8 | reg[4]);
    is_one(reg[7], 7);
    printf ("Over-Temperature condition:           %s\n", yesno);
    is_one(reg[7],6);
    printf ("Under-Temperature condition:          %s\n", yesno);
    is_one(reg[7],1);
    printf ("Full charge:                          %s\n", yesno);
    is_one(reg[7],0);
    printf ("Fast charging allowed:                %s\n", yesno);
    is_one(reg[6],7);
    printf ("OCV measurement performed:            %s\n", yesno);
    is_one(reg[6],5);
    printf ("POR or RESET command occurred:        %s\n", yesno);
    is_one(reg[6],4);
    printf ("CONFIG UPDATE mode:                   %s\n", yesno);
    is_one(reg[6],3);
    printf ("Battery insertion:                    %s\n", yesno);
    is_one(reg[6],2);
    printf ("State of Charge >= SOC1 Set Treshold: %s\n", yesno);
    is_one(reg[6],1);
    printf ("State of Charge >= SOCF Set Treshold: %s\n", yesno);
    is_one(reg[6],0);
    printf ("Discharging:                          %s\n", yesno);
    printf ("Nominal Available Capacity:           %d mAh\n", reg[9] << 8 | reg[8]);
    printf ("Full Available Capacity:              %d mAh\n", reg[11] << 8 | reg[10]);
    printf ("Remaining Capacity:                   %d mAh\n", reg[13] << 8 | reg[12]);
    printf ("Full Charge Capacity:                 %d mAh\n", reg[15] << 8 | reg[14]);
    printf ("Average Current:                      %d mA\n", signed_int(reg[17] << 8 | reg[16]));
    printf ("Standby Current:                      %d mA\n", signed_int(reg[19] << 8 | reg[18]));
    printf ("MaxLoad Current:                      %d mA\n", signed_int(reg[21] << 8 | reg[20]));
    printf ("Average Power:                        %d mW", signed_int(reg[25] << 8 | reg[24]));
    if (reg[25] >> 7) printf (" (discharging)\n");
    else if (reg[25] | reg[24]) printf (" (charging)\n");
    else printf ("\n");
    printf ("State of Charge:                      %d %\n", reg[29] << 8 | reg[28]);
    printf ("Internal Temperature:                 %.1f\n", .1 * (reg[31] << 8 | reg[30]) - 273.2);
    printf ("State of Health readiness:            ");
    switch (reg[33]) {
      case 0: {
        soh_ready = 0;
        printf("not valid\n");
        break;
      }
      case 1: {
        printf("instant\n");
        break;
      }
      case 2: {
        printf("initial\n");
        break;
      }
      case 3: {
        printf("most accurate\n");
        break;
      }
    }
    if (soh_ready) printf ("State of Health:                      %d %\n", reg[32]);
    printf ("Remaining Capacity Unfiltered:        %d mAh\n", reg[41] << 8 | reg[40]);
    printf ("Remaining Capacity Filtered:          %d mAh\n", reg[43] << 8 | reg[42]);
    printf ("Full Charge Capacity Unfiltered:      %d mAh\n", reg[45] << 8 | reg[44]);
    printf ("Full Charge Capacity Filtered:        %d mAh\n", reg[47] << 8 | reg[46]);
    printf ("State of Charge Unfiltered:           %d %\n", reg[49] << 8 | reg[48]);
    printf ("__________________________________________________\n");
}

void to_bin(unsigned char x)
{
    for( int i = 0; i < 8; i++)
    {
        bin_str[i] = ( x >> ( 7 - i ) & 1 ) ? 49
                                            : 48; // ASCII code: 48 for '0', '49 for '1'
    }

    bin_str[8] = 0;
}

int signed_int( uint i )
{
    return( i >> 15 ? i - 65536
                    : i );
}

void is_one( uchar val, uchar bit )
{
    strcpy( yesno, ( val >> bit & 1 ? "yes"
                                    : "no" ));
}
