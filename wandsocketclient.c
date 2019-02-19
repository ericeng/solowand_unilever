
#include <stdio.h> 
#include <unistd.h>
#include <sys/socket.h> 
#include <stdlib.h> 
#include <netinet/in.h> 
#include <string.h> 
#include <arpa/inet.h>

#define PORT 787 

int main(int argc, char const *argv[]) 
{ 
	struct sockaddr_in address; 
	int sock = 0, valread; 
	struct sockaddr_in serv_addr; 
	char *hello = "Frame is looking for the Wand"; 
	char buffer[1024] = {0}; 

	if ( argc != 2 )
        {
		printf("\n Server IP address must be specified.\n");
		return -1;
        }

	if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) 
	{ 
		printf("\n Socket creation error.\n"); 
		return -1; 
	} 

	memset(&serv_addr, '0', sizeof(serv_addr)); 

	serv_addr.sin_family = AF_INET; 
	serv_addr.sin_port = htons(PORT); 
	
	// Convert IPv4 and IPv6 addresses from text to binary form 
	if(inet_pton(AF_INET, argv[1], &serv_addr.sin_addr)<=0) 
	{ 
		printf("\nInvalid address/Address not supported.\n"); 
		return -1; 
	} 

	if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) 
	{ 
		printf("\nConnection Failed \n"); 
		return -1; 
	} 

	send(sock , hello , strlen(hello) , 0 ); 
	printf("%s\n", hello); 
	valread = read( sock , buffer, 1024); 
	printf("%s\n",buffer ); 

	while( 1 )
	{
		valread = read( sock , buffer, 1024); 

		send(sock, buffer, strlen( buffer ), 0);
		printf("\tCommand: '%c'\n", buffer[0] );

		switch( (char)buffer[0] )
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
	
		case 'N':
			printf("Wand is showing a (N)otification indicator (The Blue button light is on)\n\n");
			break;

		case 'B':
			printf("(B)lue lit button in notification mode has been pressed\n\n");
			break;

		default:
			printf("\nStatus or Command not understood.\n\n");
		}
	}

	return 0; 
} 

