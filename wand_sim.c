
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <wiringPi.h>
#include <signal.h>

#define POWER_BLINK_DELAY_ON	10
#define POWER_BLINK_DELAY_OFF	10

#define GPIO_LED        21
#define GPIO_BUTTON_0   26
#define GPIO_BUTTON_1   20
#define GPIO_RED        19
#define GPIO_GREEN      16
#define GPIO_BLUE       13
#define GPIO_CRADLE_0   17
#define GPIO_CRADLE_1   27
#define GPIO_CHARGING_0 04
#define GPIO_CHARGING_1 18

void sigintHandler( int sig_num )
  {
  digitalWrite( GPIO_BUTTON_0, 0 );
  digitalWrite( GPIO_LED, 0 );
  digitalWrite( GPIO_RED, 0 );
  digitalWrite( GPIO_GREEN, 0 );
  digitalWrite( GPIO_BLUE, 0 );
  digitalWrite( GPIO_CRADLE_0, 0 );
  digitalWrite( GPIO_CHARGING_0, 0 );

  printf( "Wand Simulator Exiting with all GPIOs cleared.\n" );

  fflush( stdout );

  exit( 0 );
  }

int main( int argc, char** argv )
  {
  int on = POWER_BLINK_DELAY_ON;
  int off = POWER_BLINK_DELAY_OFF;

  signal( SIGINT, sigintHandler );

  wiringPiSetupGpio();

  pinMode( GPIO_LED, OUTPUT );
  pinMode( GPIO_BUTTON_0, OUTPUT );
  pinMode( GPIO_BUTTON_1, INPUT );
  pinMode( GPIO_RED, OUTPUT );
  pinMode( GPIO_GREEN, OUTPUT );
  pinMode( GPIO_BLUE, OUTPUT );
  pinMode( GPIO_CRADLE_0, OUTPUT );
  pinMode( GPIO_CRADLE_1, INPUT );
  pinMode( GPIO_CHARGING_0, OUTPUT );
  pinMode( GPIO_CHARGING_1, INPUT );

  printf( "Wand Simulator\n");

  digitalWrite( GPIO_BUTTON_0, 1 );
  digitalWrite( GPIO_CRADLE_0, 1 );
  digitalWrite( GPIO_CHARGING_0, 1 );

  while( 1 )
    {
    // wand in charger backwards
    while( !digitalRead( GPIO_BUTTON_1 ))
      {
      digitalWrite( GPIO_GREEN, 0 );
      digitalWrite( GPIO_LED, 1 );
      sleep( 1 );
      }
    digitalWrite( GPIO_LED, 0 );

    // wand removed from cradle, preparing for capture
    while( digitalRead( GPIO_CRADLE_1 ))
      {
      digitalWrite( GPIO_GREEN, 0 );
      digitalWrite( GPIO_BLUE, 1 );
      delay( 50 );
      digitalWrite( GPIO_BLUE, 0 );
      delay( 50 );
      }

    // wand charging
    while( digitalRead( GPIO_CHARGING_1 ))
      {
      digitalWrite( GPIO_GREEN, 0 );
      digitalWrite( GPIO_RED, 1 );
      delay( 50 );
      digitalWrite( GPIO_RED, 0 );
      delay( 50 );
      }

    // wand standing by to capture
    sleep( 1 );
    if( on )
      {
      digitalWrite( GPIO_GREEN, 1 );
      delay( 25 );
      on--;
      }
    else if( off )
      {
      digitalWrite( GPIO_GREEN, 0 );
      delay( 25 );
      off--;
      }
    else
      {
      on = POWER_BLINK_DELAY_ON;
      off = POWER_BLINK_DELAY_OFF;
      }
    }

  digitalWrite( GPIO_BUTTON_0, 0 );
  digitalWrite( GPIO_CRADLE_0, 0 );
  }

