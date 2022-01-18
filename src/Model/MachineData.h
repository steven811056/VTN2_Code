#ifndef __MACHINEDATA_H__
#define __MACHINEDATA_H__

#include <stdint.h>
#include <string.h>
#include <Arduino.h>
#include "Define.h"
#include "ResultData.h"

enum _Failitem_
{
	Hardware_Error    = 1 ,
	Nozzle_jam_D6     ,
	major_leakage_D4  ,
	middle_leakage_D5 ,
	minor_leakage_D2  ,
	micro_leakage_D3  ,
	D1                ,
	D2                ,
	D3
};

class MachineData : public ResultData
{
public:
	volatile bool EM_Value = false;
	volatile bool f_Work = false;
	volatile bool f_Ask = false;
	volatile bool f_ERROR = false;
	volatile bool f_Break_Value = false;
	bool f_Clean = false;

	int8_t MCU_Cycle = 0;
	String MPU_String = "";
	int Command_Number = 0;
	char Command[50];
	char VT_version[20];
	uint32_t O2extendtimesvalues;	/* Second */
	uint32_t O2judgmentoftimes;	/* Second */
	uint8_t O2threshol;	/* O2 % */
	uint32_t O2times;	/* Second */
	uint32_t O2pressvalues;	/* mbar */

	uint16_t VTparam1;	/* frequency */
	uint32_t VTparam2;	/* ms */
	uint32_t VTparam3;	/* ms */
	uint16_t VTparam4 = 0;	/* 0.1mbar */
	uint32_t VTparam5 = 0;	/* ms */
	uint32_t VTparam6 = 0;	/* ms */
	uint32_t VTparam7 = 0;	/* ms */ // Nozzle jam detect
	uint16_t VTparam8 = 0;	/* 0.1mbar */ // MAX 0.1 mbar
	uint16_t VTparam9 = 0;	/* 0.ambar */ // MIN 0.1 mbar

	float sensorValue;
	float sensorVoltage;
	float Value_O2;
	float O2_Ampere = 0.0;

	float mbar = NAN;
	float mbar_standard = 0.0;
	float MAX_mbar = 0.0;	//	Negative pressure
	float MIN_mbar = 0.0;	//	Negative pressure
	float mbar_Range = 0.0;
	// uint16_t D3_standard = 0;
    float D3_standard = 0;
    uint8_t micro_leak_buffer = 5;
    uint8_t count_micro_leak_buffer = 0;


};

#endif