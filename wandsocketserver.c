// Server side C/C++ program to demonstrate Socket programming 
#include <unistd.h> 
#include <stdio.h> 
#include <sys/socket.h> 
#include <stdlib.h> 
#include <netinet/in.h> 
#include <string.h> 
#define PORT 787 
int main(int argc, char const *argv[]) 
{ 
	int server_fd, new_socket, valread; 
	struct sockaddr_in address; 
	int opt = 1; 
	int addrlen = sizeof(address); 
	char buffer[1024] = {0}; 
	char* hello = "Wand is online.\n"; 
	
	// Creating socket file descriptor 
	if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) 
	{ 
		perror("socket failed"); 
		exit(EXIT_FAILURE); 
	} 
	
	// Forcefully attaching socket to the port 8080 
	if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, 
												&opt, sizeof(opt))) 
	{ 
		perror("setsockopt"); 
		exit(EXIT_FAILURE); 
	} 
	address.sin_family = AF_INET; 
	address.sin_addr.s_addr = INADDR_ANY; 
	address.sin_port = htons( PORT ); 
	
	// Forcefully attaching socket to the port 8080 
	if (bind(server_fd, (struct sockaddr *)&address, 
								sizeof(address))<0) 
	{ 
		perror("bind failed"); 
		exit(EXIT_FAILURE); 
	} 
	if (listen(server_fd, 3) < 0) 
	{ 
		perror("listen"); 
		exit(EXIT_FAILURE); 
	} 
	if ((new_socket = accept(server_fd, (struct sockaddr *)&address, 
					(socklen_t*)&addrlen))<0) 
	{ 
		perror("accept"); 
		exit(EXIT_FAILURE); 
	} 
	valread = read( new_socket , buffer, 1024); 
	printf("%s\n",buffer ); 
	send(new_socket , hello , strlen(hello) , 0 ); 
	printf("Wand is online.\n"); 

	while( 1 )
	{
		unsigned int c;

		c = getchar();

		buffer[0] = (char) c;
	        buffer[1] = 0;
		if( c == 'q' )
		{
			break;
		}

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

			send(new_socket, buffer, strlen( buffer ), 0);
			printf("\tCommand: '%c'\n", c );
			valread = read( new_socket , buffer, 1024); 
			printf("Response: %s\n",buffer ); 

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


