//
// hall.c
// 
// desc: exercise the hall effect sensor on the solowand daughter board
//
// Written by: Alexei
// Adapted by: G. Eric Engstrom
//
// (C) Copyright 2019, Solomomo LCC, All Rights Reserved
//

#include "e_ventures.h"
#include <stdio.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>

#define MAP_SIZE        4096UL
#define MAP_MASK        ( MAP_SIZE - 1 )
#ifdef PI3
#define BASE            0x3F200000     // Raspberry Pi 3
#else
#define BASE            0x20200000     // Raspberry Pi 0
#endif
#define OFFSET          0x34           // GPIO Pin Level 0 register
#define GPIO            24

void print_state( uchar val )
{
    printf( "%s\n", val ? "OFF"
                        : "ON" );
}

int main()
{
    int mem;
    uchar value;
    uchar new_value;

    void* map_base;
    void* virtual_addr;

    if(( mem = open( "/dev/mem", O_RDWR | O_SYNC )) == -1 )
      {
      fprintf( stderr, "Cannot open /dev/mem\n" );
      exit( 1 );
      }

    if(( map_base = (long*)mmap( 0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mem, BASE & ~MAP_MASK )) == NULL )
      {
      fprintf( stderr, "Cannot open Memory Map\n" );
      exit( 1 );
      }

    virtual_addr = (void*)( map_base + ( BASE & MAP_MASK ) + OFFSET );

    value = *((volatile ulong*)virtual_addr) >> GPIO & 1;
    print_state( value );
  
    while( 1 )
      {
      new_value = *((volatile ulong*)virtual_addr) >> GPIO & 1;

      if( new_value != value )
        {
        print_state( new_value );
        value = new_value;
        }
      }
  
    if( munmap( map_base, MAP_SIZE ) == -1 )
      {
      fprintf( stderr, "Cannot close Memory Map" );
      exit( 1 );
      }

    close( mem );
}

