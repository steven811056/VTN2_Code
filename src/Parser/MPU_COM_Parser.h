#ifndef __MPU_COM_PARSER_H__
#define __MPU_COM_PARSER_H__

#include <Arduino.h>
#include "Control/UartHandler.h"
#include "Driver/ArduinoJson-v6.15.2.h"
#include "Model/Define.h"
#include "Model/MachineData.h"


class MPU_COM_Parser
{

public:
	void Catch_String(MachineData* MPUData , UartData* UartString)
	{
		MPUData->MPU_String = UartString->UartRx;
		UartString->UartRx = "";
	}
	int Parse_MPU_Command(MachineData*);
	int Read_MPU_ASK(void);
	int Read_MPU_END(void);
	int Write_To_MPU(void);
	int MPU_CMD_Parsing(void);

};

#endif