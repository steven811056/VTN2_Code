#ifndef __UARTDATA_H__
#define __UARTDATA_H__

#include <Arduino.h>

class UartData
{
public:
	String UartRx;
	bool f_r = false,f_n = false;
	bool f_StringEND = false;

};

#endif