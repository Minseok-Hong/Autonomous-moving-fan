#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <ctype.h>
#include <sys/ipc.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#define DRIVER_NAME		"/dev/cnbuzzer"
#define MAX_BUZZER_NUMBER		36

void doHelp(void)
{
	printf("Usage:\n");
	printf("buzzertest <buzzerNo> \n")
		;
	printf("buzzerNo: \n");
	printf("do(1,13,25,37) ,do#(2,14,26)");
	printf("re(3,15,27), re#(4,16,28)\n");
	printf("mi(5,17,29)\n");
	printf("fa(6,18,30),fa#(7,19,31)\n");
	printf("sol(8,20,32) ,sol#(9,21,33)\n");
	printf("ra(10,22,34), ra#(11,23,35)\n");
	printf("si(12,24,36)\n");
}

int main(int argc, char **argv)
{
	int buzzerNumber;
	int fd;

	int Do = 25, Re=27, Mi=29 ,Fa=30 ,Sol=32 ,Ra=34 ,Raup=35 ,Si=36, Do2 = 37;
	int End = 0;

	if (argc < 2)
	{
		perror(" Args number is less than 3\n");
		doHelp();
		return 1;
	}//신경 ㄴㄴ

	if (strlen(argv[1]) > 2)
	{
		perror("buzzerNo length > 2 \n");
		doHelp();
		return 1;
	}//신경 ㄴㄴ
	
	buzzerNumber = atoi(argv[1]);

	printf("buzzer number :%d \n", buzzerNumber);

	if (buzzerNumber > MAX_BUZZER_NUMBER)
	{
		perror(" <buzzerNo> over range \n");
		doHelp();
		return 1;
	}//범위초과

	// open  driver 
	fd = open(DRIVER_NAME, O_RDWR);
	
	if (fd < 0)
	{
		perror("driver (//dev//cnbuzzer) open error.\n");
		return 1;
	}
	// control led 
	if (buzzerNumber == 1)
	{
		
		write(fd, &Mi, 4);
		usleep(500000);
		write(fd, &Re, 4);
		usleep(500000);
		write(fd, &Do, 4);
		usleep(500000);
		write(fd, &Re, 4);
		usleep(500000);
		write(fd, &Mi, 4);
		usleep(500000);
		write(fd, &End, 4);
		write(fd, &Mi, 4);
		usleep(500000);
		write(fd, &End, 4);
		write(fd, &Mi, 4);
		usleep(500000);
		usleep(500000);
		usleep(500000);
		write(fd, &End, 4);
/*
		write(fd, &Re, 4);
		write(fd, &End, 4);
		usleep(500000);
		write(fd, &Re, 4);
		write(fd, &End, 4);
		usleep(500000);
		write(fd, &Re, 4);
		usleep(500000);
		write(fd, &Mi, 4);
		usleep(500000);
		write(fd, &End, 4);
		write(fd, &Mi, 4);
		usleep(500000);
		write(fd, &End, 4);
		write(fd, &Mi, 4);
		usleep(500000);
		usleep(500000);
		usleep(500000);
		write(fd, &Mi, 4);
		usleep(500000);
		write(fd, &Re, 4);
		usleep(500000);
		write(fd, &Do, 4);
		usleep(500000);
		write(fd, &Re, 4);
		usleep(500000);
		write(fd, &Mi, 4);
		usleep(500000);
		write(fd, &Mi, 4);
		usleep(500000);
		write(fd, &Mi, 4);
		usleep(500000);
		usleep(500000);
		usleep(500000);
		write(fd, &Re, 4);
		usleep(500000);
		write(fd, &Re, 4);
		usleep(500000);
		write(fd, &Mi, 4);
		usleep(500000);
		write(fd, &Re, 4);
		usleep(500000);
		write(fd, &Do, 4);
		usleep(500000);
		usleep(500000);
		usleep(500000);
		*/
		write(fd, &End, 4);
		usleep(500000);
		write(fd, 0 , 4);
		usleep(500000);
		close(fd);
	}

	else if (buzzerNumber == 3)
	{
		write(fd, &Fa, 4);
		usleep(500000);
		write(fd, &Sol, 4);
		usleep(500000);
		write(fd, &Ra, 4);
		usleep(500000);
		write(fd, &Do2, 4);
		sleep(1);
		usleep(500000);

		write(fd, &Ra, 4);
		usleep(500000);
		write(fd, &Sol, 4);
		usleep(500000);
		write(fd, &Do2, 4);
		sleep(1);
		usleep(500000);

/*
		write(fd, &Sol, 4);
		usleep(500000);
		write(fd, &Fa, 4);
		usleep(500000);
		write(fd, &Re, 4);
		usleep(500000);
		write(fd, &Ra, 4);
		sleep(1);

		write(fd, &Fa, 4);
		usleep(500000);
		write(fd, &Mi, 4);
		sleep(1);
		usleep(500000);

		write(fd, &Mi, 4);
		usleep(500000);
		write(fd, &Re, 4);
		usleep(500000);
		write(fd, &Mi, 4);
		usleep(500000);
		write(fd, &Fa, 4);
		usleep(500000);
		write(fd, &Sol, 4);
		usleep(500000);
		write(fd, &Do2, 4);
		usleep(500000);
		write(fd, &Fa, 4);
		usleep(500000);
		write(fd, &Sol, 4);
		usleep(500000);
		write(fd, &Ra, 4);
		usleep(500000);
		write(fd, &Raup, 4);
		usleep(500000);
		write(fd, &Raup, 4);
		usleep(500000);
		write(fd, &Ra, 4);
		usleep(500000);
		write(fd, &Sol, 4);
		sleep(500000);
		write(fd, &Fa, 4);
		usleep(500000);
		write(fd, &Sol, 4);
		usleep(500000);
		usleep(500000);

		sleep(2);

*/

		write(fd, 0 , 4);
		usleep(500000);
		write(fd, 0 , 4);
		usleep(500000);
		write(fd, &End, 4);
		usleep(500000);

		close(fd);
	}
	else\
	{

	close(fd);
	}



	return 0;
}
