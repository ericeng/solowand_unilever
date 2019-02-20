// Server side C/C++ program to demonstrate Socket programming 
#include <unistd.h> 
#include <stdio.h> 
#include <sys/socket.h> 
#include <stdlib.h> 
#include <netinet/in.h> 
#include <string.h> 

#define PORT 787 

typedef struct _WAND_SOCKET_
{
    int server_fd;
    int new_socket;
    int valread; 
    int opt; 
    struct sockaddr_in address; 
    int addrlen;
    char buffer[1024]; 
    char* hello; 
}
wand_socket;

static void _socket_exit( char* msg )
{
    perror( msg ); 
    exit( EXIT_FAILURE ); 
}

static wand_socket* _wand_constructor( void )
{
    wand_socket* ws;

printf("ws 0 %d\n", sizeof( wand_socket ));
    ws = (wand_socket*)calloc( sizeof( wand_socket ), 1 );

printf("ws 1\n");
    ws->opt = 1;
printf("ws 2\n");
    ws->addrlen = sizeof( ws->address );
printf("ws 3\n");
    memset( ws->buffer, 0, sizeof( ws->buffer ));
printf("ws 4\n");
    ws->hello = "Wand is online.\n";
printf("ws 5\n");

    return ws;
}

int hoolahoop(void);

int main(int argc, char const *argv[]) 
{ 
    wand_socket* ws = _wand_constructor();

    // Creating socket file descriptor 
    if(( ws->server_fd = socket( AF_INET, SOCK_STREAM, 0 )) == 0 ) 
    { 
	_socket_exit( "socket failed" ); 
    } 
	
    // Forcefully attaching socket to PORT 
    if( setsockopt( ws->server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &(ws->opt), sizeof( ws->opt ))) 
    { 
	_socket_exit( "socket setsockopt failed\n" ); 
    } 

    ws->address.sin_family = AF_INET; 
    ws->address.sin_addr.s_addr = INADDR_ANY; 
    ws->address.sin_port = htons( PORT ); 
	
    // Forcefully attaching socket to the PORT
    if( bind( ws->server_fd, (struct sockaddr *)&(ws->address), sizeof(ws->address)) < 0 ) 
    { 
	_socket_exit( "socket bind failed\n" ); 
    } 
    if( listen( ws->server_fd, 3 ) < 0 ) 
    { 
	_socket_exit( "socket listen failed\n" ); 
    } 
    if (( ws->new_socket = accept( ws->server_fd, (struct sockaddr*)&(ws->address), (socklen_t*)&(ws->addrlen) )) < 0 ) 
    { 
	_socket_exit( "accept failed\n" ); 
    } 

    ws->valread = read( ws->new_socket , ws->buffer, sizeof( ws->buffer )); 
    printf( "%s\n", ws->buffer ); 

    send( ws->new_socket, ws->hello, strlen( ws->hello ), 0 ); 
    printf( "Wand is online.\n" ); 

    while( 1 )
    {
	unsigned int c;

	c = getchar();

	if( c == 'q' )
	{
	    break;
	}

	ws->buffer[0] = (char) c;
        ws->buffer[1] = 0;

	switch( c )
	{
	case 'E':
	case 'W':
	case 'P':
	case 'C':
	case 'D':
	case 'N':
	case 'A':
	case 'B':

	    send( ws->new_socket,  ws->buffer, strlen( ws->buffer ), 0 );
	    printf( "\tCommand: '%c'\n", c );
	    ws->valread = read( ws->new_socket, ws->buffer, 1024 ); 
	    printf( "Response: %s\n", ws->buffer ); 

	    switch( (char)ws->buffer[0] )
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

	    default:
		printf("\nResponse not understood.\n\n");
		break;
	    }

	    break;

	case 10:
	    break;

	default:
	    printf("\nCommand not understood.\n\n");
	    break;
	}
    }

    return 0; 
} 


