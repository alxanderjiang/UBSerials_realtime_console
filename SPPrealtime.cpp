#include"StandardPointPositioning.h"
#include"satpos.h"

typedef struct {
	unsigned char buff[8192] = {};
	int bufflen = 0;
}USARTbuff;

int main()
{
	printf("�����봮���������������");

	int COMnum; scanf("%d", &COMnum);
	CSerial gps;			//����ͨ��

	unsigned char buffer[MAXBUFLEN] = {};	//�������ݻ�����

	//�������ݶ�ȡ
	unsigned int len_str;	//����log����ȣ����ϻس����У�
	int len_buf = 0;		//buffer�ܳ���
	int i = 0;						//��ǰbuff����ֵ
	int left_num = 0;				//buffer��ʣ��Ԫ����
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
	//�򿪴���д������
	SerialWrite(gps, COMnum);
	printf("�ȴ�����д��\n");
	Sleep(4000);
	USARTbuff left = {};
	USARTbuff buff = {};//������
	unsigned char message[2 * 8192] = {};//������(�ϴλ�����ʣ�࣫���λ�����)
	//������ջ��������ȫ�ֱ���
	eph_t eph[36];//����
	double ion[8] = {};//������ӳٸ��������ṹ��
	int slen = 0;
	GPSOBS R = {};//�۲�ֵ�ṹ��
	//���ڶ�ȡ������
	while (true)
	{
		//��ȡ���ݵ������������Ϊ8192,slen��ʾ��ȡ����ȡ���ĳ���
		slen = gps.ReadData(buff.buff, MAXBUFLEN);
		buff.bufflen = slen;
		if (slen == 0)
			continue;
		/*
		//д���ļ�FDATA
		fwrite(buffer , 1, len_buf, FData);
		time++;
		Sleep(200);
		if (time == 100)
		{
			fclose(FData);
			break;

		}
		*/
		//������Ч��Ϣ����������(�ϴλ�����ʣ�࣫���λ�����)
		int messagex = 0;
		{
			memcpy(message, left.buff, left.bufflen);
			memcpy(message + left.bufflen, buff.buff, buff.bufflen);
			messagex = buff.bufflen + left.bufflen;
		}


		//����Ч�������л�ȡ��Ϣ����
		breport epoch[400] = {};
		int epochnum = 0;
		epochnum = getbinaryreport(epoch, message, messagex);
		memset(left.buff, 0x00, sizeof(left.buff)); left.bufflen = 0;//��һ��ʣ������������
		left.bufflen = messagex - epoch[epochnum].start;//���¹���ʣ���������ṹ(����)
		memcpy(left.buff, message + epoch[epochnum].start, left.bufflen);//���¹���ʣ��������

		if (epochnum == 0)    continue;

		//��λ��ѭ��
		for (int j = 0; j < epochnum; j++) {


			//����
			if (epoch[j].ID == 7) {
				int prn;
				prn = getsat(eph, message, epoch[j]);
				eph[prn].statu = EPHYES;
			}
			//����㣬��Ϊ��ʦ�ṩ���ļ������ְ忨Э�飬0927����һ��Ҫ����"OEM7"�����������أ�Com_16����Ҫ
			if (epoch[j].ID == 8) {
				getion(message, ion, epoch[j]);
			}
			//��������۲�ֵ������ж�λ����
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
			//������ջ�ԭʼ���
			if (epoch[j].ID == 47) {
				int start = epoch[j].start;
				double lat = bit2double(message + start + 28 + 8);
				double lon = bit2double(message + start + 28 + 16);
				double he = bit2double(message + start + 28 + 24) + bit2float(message + start + 28 + 32);
				//ע����ջ�ֱ��������Ǻ��θߣ���Ҫ�������õ�ˮ׼��ģ����Ӳ��ܵõ�ֱ�ӽ���������he
				printf("\n���ջ������� WGS-84 %lf %lf %lf\n\n", lat, lon, he);
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
