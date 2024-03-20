#include"StandardPointPositioning.h"
#include"sockets.h"

int SerialWrite(CSerial& gps,int COM)
{
	char str1[30] = "unlogall";				//串口log命令
	char str2[30] = "mask gps";	//关掉gps系统
	char str3[30] = "mask bds"; //关掉bds系统		
	char str4[30] = "mask glo";		//关掉glo系统
	char str5[30] = "mask gal";	//关掉gal系统
	char str6[30] = "mask qzss";	//关掉qzss系统
	char str7[30] = "unmask L1";	//打开GPS L1系统
	char str8[30] = "log gpsephemb onchanged";	//星历数据
	char str9[30] = "log rangeb ontime 1";		//观测数据
	char str10[30] = "log psrposb ontime 1";		//定位结果
	char str11[30] = "gpsionb onchanged";	//电离层数据

	unsigned int len_str;	//串口log命令长度（加上回车换行）

							//打开串口
	//11为端口号，大家根据电脑实际端口号修改
	if (gps.Open(COM, 115200) == FALSE)
	{
		printf("Cannot open gps Cserial.\n");
		return 0;
	}
	//写入命令

	//清空log
	len_str = strlen(str1);
	//（/CR）:回车换行
	str1[len_str++] = 0x0D;
	str1[len_str++] = 0x0A;
	if (gps.SendData(str1, len_str) != strlen(str1))
	{
		printf("Cannot send data");
	}
	//星历数据输出命令
	len_str = strlen(str2);
	//（/CR）:回车换行
	str2[len_str++] = 0x0D;
	str2[len_str++] = 0x0A;
	if (gps.SendData(str2, len_str) != strlen(str2))
	{
		printf("Cannot send data");
	}

	//观测数据输出命令
	len_str = strlen(str3);
	//（/CR）:回车换行
	str3[len_str++] = 0x0D;
	str3[len_str++] = 0x0A;
	if (gps.SendData(str3, len_str) != strlen(str3))
	{
		printf("Cannot send data");
	}

	//接收机内部定位结果输出命令
	len_str = strlen(str4);
	//（/CR）:回车换行
	str4[len_str++] = 0x0D;
	str4[len_str++] = 0x0A;
	if (gps.SendData(str4, len_str) != strlen(str4))
	{
		printf("Cannot send data");
	}
	//电离层数据
	len_str = strlen(str5);
	//（/CR）:回车换行
	str5[len_str++] = 0x0D;
	str5[len_str++] = 0x0A;
	if (gps.SendData(str5, len_str) != strlen(str5))
	{
		printf("Cannot send data");
	}

	len_str = strlen(str6);
	//（/CR）:回车换行
	str6[len_str++] = 0x0D;
	str6[len_str++] = 0x0A;
	if (gps.SendData(str6, len_str) != strlen(str6))
	{
		printf("Cannot send data");
	}

	len_str = strlen(str7);
	//（/CR）:回车换行
	str7[len_str++] = 0x0D;
	str7[len_str++] = 0x0A;
	if (gps.SendData(str7, len_str) != strlen(str7))
	{
		printf("Cannot send data");
	}
	len_str = strlen(str8);
	//（/CR）:回车换行
	str8[len_str++] = 0x0D;
	str8[len_str++] = 0x0A;
	if (gps.SendData(str8, len_str) != strlen(str8))
	{
		printf("Cannot send data");
	}
	len_str = strlen(str9);
	//（/CR）:回车换行
	str9[len_str++] = 0x0D;
	str9[len_str++] = 0x0A;
	if (gps.SendData(str9, len_str) != strlen(str9))
	{
		printf("Cannot send data");
	}

	len_str = strlen(str10);
	//（/CR）:回车换行
	str10[len_str++] = 0x0D;
	str10[len_str++] = 0x0A;
	if (gps.SendData(str10, len_str) != strlen(str10))
	{
		printf("Cannot send data");
	}

	len_str = strlen(str11);
	//（/CR）:回车换行
	str11[len_str++] = 0x0D;
	str11[len_str++] = 0x0A;
	if (gps.SendData(str11, len_str) != strlen(str11))
	{
		printf("Cannot send data");
	}

	return 1;
}
