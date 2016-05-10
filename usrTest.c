#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>
 
int main(int argc, char **argv)
{
    	int	   fd,count;
	char 	   resv[32];
    	fd = 	   open("/dev/nRF24L01", O_RDWR);
if (fd < 0) 
	{
        	printf("open device spi error\n");
        	exit(-1);
    	}
else
{
	while(1)
	{
				ioctl(fd,0,0);		//resv mode
				read(fd,&resv,32);
				for (count=0;count<12;count++)
				printf("%c",resv[count]);
				
				printf("\n");
				resv[6]='2';
				ioctl(fd,0,1);		//send mode
				write(fd,&resv,32);
				for(count=0;count<65535;count++)
				{ count++;count--;}
	}
}
    	return 0;
}
