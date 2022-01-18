#include <Arduino.h>


#define __IO volatile
#define FlagStatus bool

#include "main.h"


MachineHandler VT_N2;
MachineData VT_N2_MachineData;
ResultData VT_N2_ResultData;
MCU_IOHandler VT_N2_MCU_IOHandler;
MPU_COM_Parser VT_N2_MPU_Parser;
UartHandler VT_N2_UartHandler(&_MPU_ , &DebugPort);

//////////////////////////////////////////////////////////////////
void setup()
{
	VT_N2.INIT();
}

//////////////////////////////////////////////////////////////////
void loop()
{
    // Over_time_standard_MPU = millis();
	VT_N2.Process_Work();

}

//////////////////////////////////////////////////////////////////
