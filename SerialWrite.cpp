#include"StandardPointPositioning.h"
#include"sockets.h"

int SerialWrite(CSerial& gps,int COM)
{
	char str1[30] = "unlogall";				//����log����
	char str2[30] = "mask gps";	//�ص�gpsϵͳ
	char str3[30] = "mask bds"; //�ص�bdsϵͳ		
	char str4[30] = "mask glo";		//�ص�gloϵͳ
	char str5[30] = "mask gal";	//�ص�galϵͳ
	char str6[30] = "mask qzss";	//�ص�qzssϵͳ
	char str7[30] = "unmask L1";	//��GPS L1ϵͳ
	char str8[30] = "log gpsephemb onchanged";	//��������
	char str9[30] = "log rangeb ontime 1";		//�۲�����
	char str10[30] = "log psrposb ontime 1";		//��λ���
	char str11[30] = "gpsionb onchanged";	//���������

	unsigned int len_str;	//����log����ȣ����ϻس����У�

							//�򿪴���
	//11Ϊ�˿ںţ���Ҹ��ݵ���ʵ�ʶ˿ں��޸�
	if (gps.Open(COM, 115200) == FALSE)
	{
		printf("Cannot open gps Cserial.\n");
		return 0;
	}
	//д������

	//���log
	len_str = strlen(str1);
	//��/CR��:�س�����
	str1[len_str++] = 0x0D;
	str1[len_str++] = 0x0A;
	if (gps.SendData(str1, len_str) != strlen(str1))
	{
		printf("Cannot send data");
	}
	//���������������
	len_str = strlen(str2);
	//��/CR��:�س�����
	str2[len_str++] = 0x0D;
	str2[len_str++] = 0x0A;
	if (gps.SendData(str2, len_str) != strlen(str2))
	{
		printf("Cannot send data");
	}

	//�۲������������
	len_str = strlen(str3);
	//��/CR��:�س�����
	str3[len_str++] = 0x0D;
	str3[len_str++] = 0x0A;
	if (gps.SendData(str3, len_str) != strlen(str3))
	{
		printf("Cannot send data");
	}

	//���ջ��ڲ���λ����������
	len_str = strlen(str4);
	//��/CR��:�س�����
	str4[len_str++] = 0x0D;
	str4[len_str++] = 0x0A;
	if (gps.SendData(str4, len_str) != strlen(str4))
	{
		printf("Cannot send data");
	}
	//���������
	len_str = strlen(str5);
	//��/CR��:�س�����
	str5[len_str++] = 0x0D;
	str5[len_str++] = 0x0A;
	if (gps.SendData(str5, len_str) != strlen(str5))
	{
		printf("Cannot send data");
	}

	len_str = strlen(str6);
	//��/CR��:�س�����
	str6[len_str++] = 0x0D;
	str6[len_str++] = 0x0A;
	if (gps.SendData(str6, len_str) != strlen(str6))
	{
		printf("Cannot send data");
	}

	len_str = strlen(str7);
	//��/CR��:�س�����
	str7[len_str++] = 0x0D;
	str7[len_str++] = 0x0A;
	if (gps.SendData(str7, len_str) != strlen(str7))
	{
		printf("Cannot send data");
	}
	len_str = strlen(str8);
	//��/CR��:�س�����
	str8[len_str++] = 0x0D;
	str8[len_str++] = 0x0A;
	if (gps.SendData(str8, len_str) != strlen(str8))
	{
		printf("Cannot send data");
	}
	len_str = strlen(str9);
	//��/CR��:�س�����
	str9[len_str++] = 0x0D;
	str9[len_str++] = 0x0A;
	if (gps.SendData(str9, len_str) != strlen(str9))
	{
		printf("Cannot send data");
	}

	len_str = strlen(str10);
	//��/CR��:�س�����
	str10[len_str++] = 0x0D;
	str10[len_str++] = 0x0A;
	if (gps.SendData(str10, len_str) != strlen(str10))
	{
		printf("Cannot send data");
	}

	len_str = strlen(str11);
	//��/CR��:�س�����
	str11[len_str++] = 0x0D;
	str11[len_str++] = 0x0A;
	if (gps.SendData(str11, len_str) != strlen(str11))
	{
		printf("Cannot send data");
	}

	return 1;
}
