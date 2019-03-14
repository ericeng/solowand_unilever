// Server side C/C++ program to demonstrate Socket programming 
#include <unistd.h> 
#include <stdio.h> 
#include <sys/socket.h> 
#include <stdlib.h> 
#include <netinet/in.h> 
#include <string.h> 
#include "wandsocketeye.h"

static wand_socket* _ws; 

static void _socket_exit( char* msg )
{
    perror( msg ); 
    exit( EXIT_FAILURE ); 
}

static wand_socket* _wand_constructor( void )
{
    wand_socket* ws;

    ws = (wand_socket*)calloc( sizeof( wand_socket ), 1 );

    ws->opt = 1;
    ws->addrlen = sizeof( ws->address );
    memset( ws->buffer, 0, sizeof( ws->buffer ));
    ws->hello = "Wand is online.\n";

    return ws;
}

static int _socket_send( char c )
{
    send( _ws->new_socket,  &c, 1, 0 );
    _ws->valread = read( _ws->new_socket, _ws->buffer, 1024 ); 
    if( _ws->valread == 0 )
    {
	return -1;
    }
    else
    {
	return _ws->buffer[0];
    }
}

#ifdef __UNIT_TEST
void main( int argc, char** argv )
{
    if( socket_wand_open() )
    {
	while( 1 )
	{
	    unsigned int c;

	    c = getchar();

	    if( c == 'q' )
	    {
		break;
	    }

            printf( "\tCommand: '%c'\n", c );

	    switch( c )
	    {
	    case 'E':
	        c = socket_wand_in_cradle();
	        break;

	    case 'W':
	        c = socket_wand_in_cradle_wrong();
	        break;

	    case 'P':
	        c = socket_wand_in_preview();
	        break;

	    case 'C':
	        c = socket_wand_in_capture();
	        break;

	    case 'D':
	        c = socket_wand_done_capturing();
	        break;

	    case 'A':
	        c = socket_wand_aborted_capture();
	        break;

	    case 'N':
	        c = socket_wand_notification_confirmed();
	        break;

	    case 'B':
	        c = socket_wand_notification_accepted();
	        break;

	    default:
	        printf("\nCommand not understood.\n\n");
	        break;
	    }


	    switch( c )
	    {
	    case 'E':
		printf("Wand is in the (E)asel Cradle\n\n");
		break;

	    case 'W':
		printf("Wand is oriented the (W)rong way in the Easel Cradle\n\n");
		break;

	    case 'P':
		printf("Wand is in (P)review mode (Removed from the Easel Cradle)\n\n");
		break;

	    case 'C':
		printf("Wand is in (C)apture mode (The lights will begin to cycle)\n\n");
		break;

	    case 'D':
		printf("Wand is (D)one capturing a post (The lights will return to Preview mode and the active spot should be advanced)\n\n");
		break;

	    case 'A':
		printf("Wand Capture has been (A)borted (The Wand was returned to the Easel Cradle)\n\n");
		break;

	    case 'N':
		printf("Wand is showing a (N)otification indicator (The Blue button light is on)\n\n");
		break;

	    case 'B':
		printf("(B)lue lit button in notification mode has been pressed\n\n");
		break;

            case -1:
	        goto exit;

	    default:
		printf("\nResponse not understood.\n\n");
		break;
	    }
	}
    }

exit:
    socket_wand_close();
}
#endif

int socket_wand_open( void ) 
{ 
    _ws = _wand_constructor();

    // Creating socket file descriptor 
    if(( _ws->server_fd = socket( AF_INET, SOCK_STREAM, 0 )) == 0 ) 
    { 
	_socket_exit( "socket failed" ); 
    } 
	
    // Forcefully attaching socket to SOCKET_PORT 
    if( setsockopt( _ws->server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &(_ws->opt), sizeof( _ws->opt ))) 
    { 
	_socket_exit( "socket setsockopt failed\n" ); 
    } 

    _ws->address.sin_family = AF_INET; 
    _ws->address.sin_addr.s_addr = INADDR_ANY; 
    _ws->address.sin_port = htons( SOCKET_PORT ); 
	
    // Forcefully attaching socket to the SOCKET_PORT
    if( bind( _ws->server_fd, (struct sockaddr *)&(_ws->address), sizeof(_ws->address)) < 0 ) 
    { 
	_socket_exit( "socket bind failed\n" ); 
    } 
    if( listen( _ws->server_fd, 3 ) < 0 ) 
    { 
	_socket_exit( "socket listen failed\n" ); 
    } 
    if (( _ws->new_socket = accept( _ws->server_fd, (struct sockaddr*)&(_ws->address), (socklen_t*)&(_ws->addrlen) )) < 0 ) 
    { 
	_socket_exit( "accept failed\n" ); 
    } 

    _ws->valread = read( _ws->new_socket, _ws->buffer, sizeof( _ws->buffer )); 
    printf( "%s\n", _ws->buffer ); 

    send( _ws->new_socket, _ws->hello, strlen( _ws->hello ), 0 ); 
    printf( "Wand is online.\n" ); 

    return 1;
}

int socket_wand_close( void )
{
    return 1;
}

int socket_wand_in_cradle( void )
{
    return _socket_send( 'E' );
}

int socket_wand_in_cradle_wrong( void )
{
    return _socket_send( 'W' );
}

int socket_wand_in_preview( void )
{
    return _socket_send( 'P' );
}

int socket_wand_in_capture( void )
{
    return _socket_send( 'C' );
} 

int socket_wand_done_capturing( void )
{
    return _socket_send( 'D' );
}

int socket_wand_aborted_capture( void )
{
    return _socket_send( 'A' );
}

int socket_wand_notification_confirmed( void )
{
    return _socket_send( 'N' );
}

int socket_wand_notification_accepted( void )
{
    return _socket_send( 'B' );
}

