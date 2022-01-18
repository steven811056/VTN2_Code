#include "UartHandler.h"

UartHandler::UartHandler(HardwareSerial* SerialPort , HardwareSerial* DebugPort)
{
	this->SerialPort = SerialPort;
	this->DebugSerialPort = DebugPort;
}

int UartHandler::Read_Buffer(void)
{
	char CMD ;
	f_StringEND = false;
	// long test;
	#if _TEST_MPU_Message_
	f_String_END = true;
	MPU_CMD = _FAKE_MPU_Message;
	#endif

	if( SerialPort->available() )
	{
		delay(10);
		while( SerialPort->available() )
		{
			CMD = (char)SerialPort->read();
			if(CMD == '\r')
			{
				f_r = true;
				// DebugSerialPort->print("f_r");
			}
			else if(CMD == '\n')
			{
				f_n = true;
				// DebugSerialPort->print("f_n");
			}
			else
			{
				UartRx += CMD ;
				// DebugSerialPort->print(UartRx);
			}
		}
	}
	if(f_r==true)
	{
		if(f_n==true)
		{
			DebugSerialPort->print("Read Buffer function MPU CMD = ");
			DebugSerialPort->println(UartRx);
			f_StringEND = true;
			f_n = false;
			f_r = false;
			return 1;
		}
	}
	return 0;
}