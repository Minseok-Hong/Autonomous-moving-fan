#include <stdio.h>
#include <stdlib.h>     
#include <string.h>
#include <unistd.h>    
#include <fcntl.h>      
#include <termios.h>
#include <sys/ioctl.h>  
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <pthread.h>
#include <linux/input.h>
#include <linux/fb.h>
#include <time.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <errno.h>
#include "videodev2.h"
#include "SecBuffer.h" 
#include "camera.h"

/*socket*/
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#define DATE_SIZE 20
#define ROOM_SIZE 20
#define BUF_SIZE 100
#define PORT 80

/*socket function*/
void Parsing();
void* server(void* i);

/*socket var*/
char msg[BUF_SIZE];
char year[DATE_SIZE];
char month[DATE_SIZE];
char day[DATE_SIZE];
char width[ROOM_SIZE];
char height[ROOM_SIZE];


/* touchFlag
 *
 */
unsigned int 	obstacle_color = 0xCC0000;

#define MAX_TOUCH_X	0x740
#define MAX_TOUCH_Y	0x540
#define FBDEV_FILE "/dev/fb0"
#define ROOM_MARGIN	20	
#define maxBlock 21 // max room Width size
#define overLengthWidth 480 // width = 1280 / height = 800

#define KEYPAD_DRIVER_NAME	"/dev/cnkey"
#define	 FBDEV_FILE "/dev/fb0"
#define  INPUT_DEVICE_LIST	"/proc/bus/input/devices"
#define  EVENT_STR	"/dev/input/event"
#define  MAX_BUFF	200


int		screen_width;
int		screen_height;
int		bits_per_pixel;
int		line_length;


#define BAUDRATE B115200
//#define BAUDRATE B9600
#define KEYPAD "/dev/cnkey"
#define MODEMDEVICE "/dev/ttyACM0"
#define FALSE 0
#define TRUE 1

#define WAITTING  0 
#define MOVING    1 
#define STOPING   2
#define FINISHING 3

volatile int flag = FALSE;


 int temper;
int humid;
int state;
int remainDis;
int totalDis;
int error;
int rowlength;
int collength;
int touchFlag;
int limitTime;



pthread_t tid[10];

void* buzzer(void* i);
void* textlcd(void* i);
void* text_lcd(void* i); 
void* bus_led(void* i);
void* seven_segment(void* i);
void* oled(void* i);
void* color_led(void* i);
void* dip_sw(void* i);
void* touch(void* i);
void* camera(void* arg);
void* dot(void* i);
//void* key_pad(void* i);
int key_pad();


int main()
{
   system("./setting");

   int res;
   int fd;
   int touchCount = 0 ;
   unsigned char *temp = NULL;
   unsigned char buf[400] = { 0, };
   struct termios oldtio, newtio;
   int temper, humid ,distance, remainDis, error;
   char data[400] = "M 1 68 10 70 32 33 34 44 54 57 58 64 67\n";
   char cmd_buffer[50];
   //char buffer[];

   int i = 0;

  

   fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY);//fd == arduiono

   if (fd<0)
   {
      perror("driver  open error.\n");
   }

   tcgetattr(fd, &oldtio);//taget is arduino and save
   bzero(&newtio, sizeof(newtio)); //clear new port settings 

   newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 |CLOCAL |CREAD;
   newtio.c_iflag = IGNPAR | ICRNL; // input mode flag 
   newtio.c_lflag = ICANON;        //local mode flags
   tcflush(fd, TCIFLUSH);
   tcsetattr(fd, TCSANOW, &newtio);

   newtio.c_cc[VTIME] = 0;
   newtio.c_cc[VMIN] = 8;


	pthread_create(&tid[0], NULL, text_lcd, 0);
	pthread_create(&tid[1], NULL, bus_led, 0);
	pthread_create(&tid[2], NULL, seven_segment, 0);
	pthread_create(&tid[3], NULL, buzzer, 0);
	pthread_create(&tid[4],NULL,touch,0);
	pthread_create(&tid[5], NULL, color_led, 0);
	pthread_create(&tid[6], NULL, oled, 0);
	pthread_create(&tid[7], NULL, dot, 0);
  	pthread_create(&tid[8], NULL, dip_sw, 0);
  	pthread_create(&tid[9], NULL, server, 0);


   while(1)
   {
      if(state == MOVING)
      {
         break;
      }
   }
   
   for(i = 0; i < 5 ; i++)
   {
      write(fd, data, sizeof(data)-1);
      sleep(1);
   }
   printf("########################################################");
   tcflush(fd, TCIFLUSH);
   tcsetattr(fd, TCSANOW, &newtio);
   memset(buf, 0, sizeof(char)*255);

   printf("complete output\n");
   sleep(20);
   printf("sleep end\n");
   
   printf("2\n");
   while(flag == FALSE)
   {
      res = read(fd, buf, 400);

      while(res < 1)
      {
         res = read(fd, buf, 400);
      }

      printf("meaage   =   %s\n\n",buf);

      temp = strtok(buf,","); 
      printf("%s\n", temp);

      temper = atoi(temp);

      int order = 1;

      while( temp != NULL && order < 6 )
      {
         temp = strtok(NULL, ",");

         if(temp[0] == '\n')
         {
            flag = TRUE;
         }

         switch(order)
         {
            case 0:
               break;
            case 1:
               humid = atoi(temp);
               break;
            case 2:
               state = atoi(temp);
               break;
            case 3:
               remainDis = atoi(temp);
               break;
            case 4 :
               totalDis = atoi(temp);
               break;
            case 5 :
               error = atoi(temp);
               break;
            default:
            break;
         }
         order++;

      }
      printf("temperature = %d\n", temper);
      printf("humid        = %d\n", humid);
      printf("state        = %d\n", state);
      printf("remainDis   = %d\n", remainDis);
      printf("totalDis     = %d\n", totalDis);
      //printf("error        =  %d\n",error);
   }   
   close(fd);
   
}



/*text lcd 동작 함수*/
void* text_lcd(void* i){
   char cmd_buffer[32];
   system("./tlcdtest c 1 1 1 1");
   system("./tlcdtest c 1 1 2 1");

   int textFlag = 0;

    while(1){
      printf("in textlcd : 1\n");
      /* 온도와 습도에 따라 상태를 출력해준다 */
      if(state!=WAITTING && textFlag==0){
      	temper = 25;
      	humid = 10;
         sprintf(cmd_buffer, "./tlcdtest w 1 tem:%d,hum:%d", temper, humid);
         system(cmd_buffer);
         textFlag++;
      }

      usleep(100000);
   }
			if(state == FINISHING)

   return NULL;
}

/*bus led device 동작 함수*/
void* bus_led(void* i){


   system("./ledtest 0 0");
   sleep(10);

   while (1){ 
      int remainPerTotal = 1;//(remainDis/totalDis)*10;
      if (state == MOVING){      
         
         system("./ledtest 1 1");
         sleep(8);
         system("./ledtest 2 1");
         sleep(8);
         system("./ledtest 3 1");
         sleep(8);
         system("./ledtest 4 1");
         sleep(8);
         system("./ledtest 5 1");
         sleep(8);
         system("./ledtest 6 1");
         sleep(8);
         system("./ledtest 7 1");
         sleep(8);
         system("./ledtest 8 1");
         sleep(8);


         
      }
	  else {
		  switch (remainPerTotal){
		  case 8: system("./ledtest 8 1");
		  case 7: system("./ledtest 7 1");
		  case 6: system("./ledtest 6 1");
		  case 5: system("./ledtest 5 1");
		  case 4: system("./ledtest 4 1");
		  case 3: system("./ledtest 3 1");
		  case 2: system("./ledtest 2 1");
		  case 1: system("./ledtest 1 1");
		  default: break;
		  }
		  sleep(4);
	  }
   }
}

/*seven segment 동작 함수*/


void* seven_segment(void* i)
{
	sleep(3);
   while(1)
   {

   char cmd_buffer[32];
   int t_date;  // ‘시, 분, 초’로 계산된 시간
   char *yTemp;

   t_date = atoi(year) * 10000 + atoi(month) * 100 + atoi(day); // year, month, date -> number
   sprintf(cmd_buffer, "./fndtest s 1 %d", t_date);

   while(1)
   {
      system(cmd_buffer);
      
   }
   return NULL;

   
}





void* textlcd(void* i)
{

   while(1)
   {   
		system("./tlcdtest w 1 an automatic car");
   }
   return NULL;
}
/*dot 동작 함수*/

void* dot(void* i){

   int dotFlag =0;

   printf("in dot : 0\n");
   while(1)
   {
      if(state == MOVING && dotFlag== 0)
      {      printf("in dot : start\n");
         printf("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
         system ("./mledtest c 60 99");
         usleep(10000);
         dotFlag++;

      }
   }
}

void* buzzer(void* i)
{

   int dotBuzzer =0;

   while(1)
   {
      if(state == MOVING && dotBuzzer== 0)
      {  
         usleep(10000);
         system ("./buzzertest 3");
         sleep(1);
         dotBuzzer++;

      }
      else if (state == FINISHING && dotBuzzer== 1)
      {
         system ("./buzzertest 1");
         sleep(1);
         dotBuzzer++;
     }
   }
}
/* color led 동작 함수*/


void* color_led(void* i)
{
   char cmd_buffer[32];

   while(1)
   {   
      printf("in color_led : 1\n");

      if(state == WAITTING )
      {    // state : sleep -> color led: red
         system ("./cledtest 0 255 0 0");
	  }
    else if(state == MOVING)
      { //state : break or absent -> color led: off
         system ("./cledtest 0 0 0 0");
      }
      else if ((state == FINISHING))
      {
         
         system("./cledtest 0 0 100 200");   //매우 밝음 : 밝은 파랑
      }
	  else      { // After exit, attention print.
         system("./cledtest 0 0 0 0");
                     
      }

      sleep(1);
   }
}

/*dip_switch function*/

void* dip_sw(void* i){
   while(1){
      int revalue =system("./dipswtest");

      if(revalue == 256)
      {
         state = FINISHING;

      }

   }
}



/*touch 동작 함수*/

void* touch(void* i){

	int rowlength;
	int collength;

	char cmd_buffer[32];
	int flag =0; 

	int x = 500;
	int y = 500;
	printf("in touch : 0\n");

	touchFlag =0;

	state = WAITTING ;

	printf("in touch : 1\n");



	touchFlag = system ("./touchapp 2 1");//4livingFAN 

	while(1)
	{
		if(touchFlag == 256)
		{   
			touchFlag = 0;
			break;

		}
	}
	while(1)
	{

		system("./touchapp 2 2");
		
		x = atoi(width);
   		y = atoi(height);
		

		rowlength = x / 25;
		collength = y / 25;


		if(rowlength <= 20 && collength <= 20 )
		{


		}

		usleep(10000);
		system("./bitmap image3.bmp");
		usleep(10000);
		limitTime = key_pad();

         	system(cmd_buffer);
         	touchFlag = system ("./touchapp 2 3");
		while(1)
		{
			

			if(touchFlag == 256)
			{
				break;
			}
			else if (touchFlag == 512)
			{
			}
		}

		touchFlag = 0;
		touchFlag = system ("./touchapp 2 4");
		

		while(1)
		{
			if(touchFlag == 512 )
			{
				state = MOVING;
				touchFlag=0;
				break;
			}
		}
		touchFlag = system ("./bitmap image5.bmp");
		sleep(72);
		state = FINISHING;

		while(1)
		{
			if(state == FINISHING)
			{
            int screenshot();
				break;
			}
		}
		if(state == FINISHING)
		{
			system("./bitmap image6.bmp");
			break;
		}

	}

}

/*oled 동작 함수*/
// 이전의 user state와 비교하여 변화가 있을 시
// state와 맞는 img 파일 oled에 출력





void* oled(void* i){
   int pre_state = 0;
   
   system("./oledtest t");
   usleep(10000);

   system("./oledtest i");
   usleep(10000);
      
   system("./oledtest d waiting.img");   
   usleep(10000);

   while(1){

      if(pre_state != state){

         
            
         switch(state){
            case WAITTING:
               system("./oledtest d waiting.img");
               pre_state = WAITTING;
               break;
            case MOVING:
               system("./oledtest d moving.img");
               pre_state = MOVING;
               break;
            case STOPING:   
               system("./oledtest d stoping.img");
               pre_state = STOPING;
               break;
            case FINISHING:
               system("./oledtest d finishing.img");
               pre_state = FINISHING;
               break;
            default: 
               break;
         
         }
      }
      sleep(1);
   }
   return NULL;
}

int key_pad()
{ 
   int rdata;
   int fd1;
   int flag=0;
   int length[4];
   int cnt=0;
   int result;

   fd1 = open(KEYPAD,O_RDWR);

   while(1)
   {
      read(fd1,&rdata,4);      

      if(rdata == 16)
      {
         printf("terminate");
         break;
      }
      else if(rdata<11 && rdata>0)
      {
         printf("button NO:%d\n",rdata);
         if(rdata == 10)
         { //key pad 10번 입력 시 0을 입력받는 것으로 변환 
            rdata = 0;
         }
         length[cnt++]=rdata;
         usleep(500000);   
      }
      else if(rdata>=11) 
      { // key pad에서 11 이상의 번호를 누르면 입력 무시
              continue;
         }
      if(cnt==4)
      {
            //flag=1;
         cnt=0;
         printf("terminate");
         break;
      }
      
   }
   printf("%d %d %d %d",length[0],length[1],length[2],length[3]);
   close(fd1);

   result = length[0]*1000 + length[1]*100 + length[2]*10 + length[3];

   printf("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
   printf("keypadResult = %d",result);

   return result;
}


/*socket function define*/

void* server(void* i)
{
    int listenfd = 0, connfd = 0;
    struct sockaddr_in serv_addr; 
    int len, msg_size;


    time_t ticks; 

    listenfd = socket(AF_INET, SOCK_STREAM, 0);
    memset(&serv_addr, '0', sizeof(serv_addr));
    memset(msg, '0', sizeof(msg)); 

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(5000); 

    bind(listenfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)); 

    listen(listenfd, 10); 

   
    while(1){
       printf("server start!!\n");
        connfd = accept(listenfd, (struct sockaddr*)NULL, NULL); 

        read(connfd,msg,1024);
        printf("server : %s\n",msg);
      Parsing();

        memset(msg,'0',30);
        close(connfd);
        sleep(1);
   } 
}

void Parsing()
{
   if (msg[0] == '_')
      sscanf(msg, "%20[^-]-%20[^-]-%s", year, month, day);
   else if (msg[0] == '_')
      sscanf(msg, "%20[^=]=%20[^=]=", width, height);

   return;

}
