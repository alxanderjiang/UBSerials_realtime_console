#include<stdio.h>
#include<windows.h>
#pragma comment(lib,"WS2_32.lib")
#pragma warning(disable:4996)
//#define DGPS_PROT 5437              //GNSS中心的差分网址
//#define DGPS_IP "119.253.45.108"

#define DGPS_PROT  11410                //发布伪距差分改正数
#define DGPS_IP "219.140.192.34"

bool OpenDGPSSocket(SOCKET& sock);


