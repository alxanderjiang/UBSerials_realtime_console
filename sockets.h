#include<stdio.h>
#include<windows.h>
#pragma comment(lib,"WS2_32.lib")
#pragma warning(disable:4996)
//#define DGPS_PROT 5437              //GNSS���ĵĲ����ַ
//#define DGPS_IP "119.253.45.108"

#define DGPS_PROT  11410                //����α���ָ�����
#define DGPS_IP "219.140.192.34"

bool OpenDGPSSocket(SOCKET& sock);


