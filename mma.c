// MMA7660 example c code for the Raspberry pi.
//
// Reads X, Y & Z data from MMA7660
// and then prints to the screen.
//
// DGtal 2015.

#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

int main(int argc, char **argv)
{
    printf("**** MMA7660 test program ****,\n");
    int fd;                                                     // File descrition
    char *fileName = "/dev/i2c-1";                              // Name of the port we will be using
    int  address = 0x4c;                                        // Address of TPA81 shifted right 1 bit
    unsigned char buf[3];                                       // Buffer for data being read/ written on the i2c bus

    if ((fd = open(fileName, O_RDWR)) < 0) {                    // Open port for reading and writing
        printf("Failed to open i2c port\n");
        exit(1);
    }

    if (ioctl(fd, I2C_SLAVE, address) < 0) {                    // Set the port options and set the address of the device we wish to speak to
        printf("Unable to get bus access to talk to slave\n");
        exit(1);
    }

    buf[0] = 0;                                                 // This is the register we wish to read from

    if ((write(fd, buf, 1)) != 1) {                             // Send register to read from
        printf("Error writing to i2c slave\n");
        exit(1);
    }

    do {
        if (read(fd, buf, 3) != 3) {                              // Read back data into buf[]
            printf("Unable to read from slave\n");
            exit(1);
        }
        else {
            printf("X: %u \n",buf[0]);
            printf("Y: %u \n",buf[1]);
            printf("Z: %u \n",buf[2]);
        }
    } while (1);

    return 0;
}
