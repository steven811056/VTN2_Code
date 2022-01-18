#ifndef __UARTHANDLER_H__
#define __UARTHANDLER_H__

#include <string.h>
#include <Arduino.h>
#include "Model/UartData.h"
// #include "HardwareSerial.h"
// #include "Driver/ArduinoJson-v6.15.2.h"

class UartHandler : public UartData
{
	HardwareSerial* SerialPort;
	HardwareSerial* DebugSerialPort;

public:
	UartHandler(HardwareSerial*,HardwareSerial*);
	int Read_Buffer(void);
	int Write_Buffer(void);

};

#endif