#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>


void main()
{
	system("ifconfig eth0 192.168.0.2");

	//driver
	system("insmod leddrv.ko");
	system("insmod cdc-acm.ko");
	system("insmod tlcddrv.ko");
	system("insmod dipswdrv.ko");
	system("insmod cleddrv.ko");
	system("insmod keydrv.ko");
	system("insmod fnddrv.ko");
	system("insmod mleddrv.ko");
	system("insmod oleddrv.ko");	
	system("insmod bcmdhd.ko");
	system("insmod buzzerdrv.ko");	

	system("lsmod");

	system("chmod 777 touchapp");
	system("chmod 777 colorbar");
	system("chmod 777 camera");
	system("chmod 777 ledtest");
	system("chmod 777 cledtest");
	system("chmod 777 keytest");
	system("chmod 777 oledtest");
	system("chmod 777 bitmap");
	system("chmod 777 fndtest");
	system("chmod 777 dipswtest");
	system("chmod 777 tlcdtest");
	system("chmod 777 mledtest");
	system("chmod 777 buzzertest");

return;
}
