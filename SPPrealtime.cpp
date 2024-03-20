#include"StandardPointPositioning.h"
#include"satpos.h"

typedef struct {
	unsigned char buff[8192] = {};
	int bufflen = 0;
}USARTbuff;

int main()
{
	printf("请输入串口序号以启动程序：");

	int COMnum; scanf("%d", &COMnum);
	CSerial gps;			//串口通信

	unsigned char buffer[MAXBUFLEN] = {};	//串口数据缓冲区

	//串口数据读取
	unsigned int len_str;	//串口log命令长度（加上回车换行）
	int len_buf = 0;		//buffer总长度
	int i = 0;						//当前buff索引值
	int left_num = 0;				//buffer中剩余元素数
	int readdata;
	int time = 0;
	/*
	FILE* FData;
	if ((fopen_s(&FData, "Serial.txt", "wb")) != 0)
	{
		printf("Cannot open the Serial file. \n");
		return 0;
	}
	*/
	//打开串口写入命令
	SerialWrite(gps, COMnum);
	printf("等待串口写入\n");
	Sleep(4000);
	USARTbuff left = {};
	USARTbuff buff = {};//缓冲区
	unsigned char message[2 * 8192] = {};//数据区(上次缓冲区剩余＋本次缓冲区)
	//定义接收机启动后的全局变量
	eph_t eph[36];//星历
	double ion[8] = {};//电离层延迟改正参数结构体
	int slen = 0;
	GPSOBS R = {};//观测值结构体
	//串口读取，解算
	while (true)
	{
		//读取数据到缓冲区：最大为8192,slen表示读取到读取到的长度
		slen = gps.ReadData(buff.buff, MAXBUFLEN);
		buff.bufflen = slen;
		if (slen == 0)
			continue;
		/*
		//写入文件FDATA
		fwrite(buffer , 1, len_buf, FData);
		time++;
		Sleep(200);
		if (time == 100)
		{
			fclose(FData);
			break;

		}
		*/
		//构建有效消息序列数据区(上次缓冲区剩余＋本次缓冲区)
		int messagex = 0;
		{
			memcpy(message, left.buff, left.bufflen);
			memcpy(message + left.bufflen, buff.buff, buff.bufflen);
			messagex = buff.bufflen + left.bufflen;
		}


		//从有效数据区中获取消息报告
		breport epoch[400] = {};
		int epochnum = 0;
		epochnum = getbinaryreport(epoch, message, messagex);
		memset(left.buff, 0x00, sizeof(left.buff)); left.bufflen = 0;//上一次剩余数据区清零
		left.bufflen = messagex - epoch[epochnum].start;//重新构建剩余数据区结构(定长)
		memcpy(left.buff, message + epoch[epochnum].start, left.bufflen);//重新构建剩余数据区

		if (epochnum == 0)    continue;

		//定位主循环
		for (int j = 0; j < epochnum; j++) {


			//星历
			if (epoch[j].ID == 7) {
				int prn;
				prn = getsat(eph, message, epoch[j]);
				eph[prn].statu = EPHYES;
			}
			//电离层，因为老师提供的文件有两种板卡协议，0927数据一定要加上"OEM7"参数调用重载，Com_16不需要
			if (epoch[j].ID == 8) {
				getion(message, ion, epoch[j]);
			}
			//如果读到观测值，则进行定位尝试
			if (epoch[j].ID == 631 || epoch[j].ID == 43) {
				SppResult Result;
				gtime_t lastt = R.rt;
				getobs(R, message, epoch[j]);
				//if (R.rt.time == lastt.time && R.rt.second == lastt.second)
					//continue;
				SPPpos(R, eph, ion, Result);
				printResult(Result, "BLH");

				//fprintResult(fr, Result);
			}
			//输出接收机原始结果
			if (epoch[j].ID == 47) {
				int start = epoch[j].start;
				double lat = bit2double(message + start + 28 + 8);
				double lon = bit2double(message + start + 28 + 16);
				double he = bit2double(message + start + 28 + 24) + bit2float(message + start + 28 + 32);
				//注意接收机直接输出的是海拔高，需要与其内置的水准面模型相加才能得到直接解算的椭球高he
				printf("\n接收机解算结果 WGS-84 %lf %lf %lf\n\n", lat, lon, he);
				//fprintf(fd, "%lld %lf %lf %lf ", gpst2time(breport2GPStime(epoch[j], message).Week, breport2GPStime(epoch[j], message).Second).time, lat, lon, he);
				double xyz[3] = {};
				//blhtoxyz(lat, lon, he, xyz);
				//fprintf(fd, "%lf %lf %lf\n", xyz[0], xyz[1], xyz[2]);
				//printcommtime(time2epoch(gpst2time(breport2GPStime(epoch[j], message).Week, breport2GPStime(epoch[j], message).Second)));
			}
		}

	}
	return 1;
}
