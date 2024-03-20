#pragma once

#include<stdio.h>
#include<math.h>
#include<stdlib.h>
#include <float.h>
#include<corecrt_memcpy_s.h>
#include"Serial.h"
#include"sockets.h"

#define MAXCHANNEL 16						// 接收机通道（最大）
#define MaxGpsNum  32						//GPS最大卫星数
#define MAXRAWLEN 600*sizeof(char)			// buff缓冲区中最大字节数（信息最长不超过600字节）
#define MAXBUFLEN 8192						// 串口读取时buff缓冲区中最大字节数






//向串口发送log命令
int SerialWrite(CSerial& gps,int COM);











