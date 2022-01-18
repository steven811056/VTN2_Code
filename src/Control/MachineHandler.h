#ifndef __MACHINEHANDLER_H__
#define __MACHINEHANDLER_H__

#include "main.h"
#include "Model/Define.h"
#include "Driver/sSense-BMx280I2C.h"
#include "MCU_IOHandler.h"
#if (O2_sensor_Choose == _ME2_O2_)
	#include "Driver/ME2_O2.h"
#elif (O2_sensor_Choose == _TW_500B_)
	#include "Driver/TW_500B.h"
#endif

class MachineHandler
{
private:
	// /* data */
	// volatile bool EM_Value = false;
	// volatile bool f_Work_Status = false;
	// volatile bool f_Ask = false;
	// volatile bool f_ERROR = false;
	// volatile bool f_Break_Value = false;
	// bool f_Clean = false;


	// String MPU_CMD = "";
	// char Command[50];
	// char VT_version[20];
	// uint32_t O2extendtimesvalues;	/* Second */
	// uint32_t O2judgmentoftimes;	/* Second */
	// uint8_t O2threshol;	/* O2 % */
	// uint32_t O2times;	/* Second */
	// uint32_t O2pressvalues;	/* mbar */

	// uint16_t VTparam1;	/* frequency */
	// uint32_t VTparam2;	/* ms */
	// uint32_t VTparam3;	/* ms */
	// uint16_t VTparam4 = 0;	/* 0.1mbar */
	// uint32_t VTparam5 = 0;	/* ms */
	// uint32_t VTparam6 = 0;	/* ms */
	// uint32_t VTparam7 = 0;	/* ms */ // Nozzle jam detect
	// uint16_t VTparam8 = 0;	/* 0.1mbar */ // MAX 0.1 mbar
	// uint16_t VTparam9 = 0;	/* 0.ambar */ // MIN 0.1 mbar

	// float sensorValue;
	// float sensorVoltage;
	// float Value_O2;
	// float O2_Ampere = 0.0;

	// float mbar = NAN;
	// float mbar_standard = 0.0;
	// float MAX_mbar = 0.0;	//	Negative pressure
	// float MIN_mbar = 0.0;	//	Negative pressure
	// float mbar_Range = 0.0;
	// // uint16_t D3_standard = 0;
    // float D3_standard = 0;
    // uint8_t micro_leak_buffer = 5;
    // uint8_t count_micro_leak_buffer = 0;

	// char N2Value[10];
	// char VTValue[10];
	// char N2Result[20];
	// char VTResult[20];
	// char Result[25];
	// char ErrorCode[25];	/* String */
	// char RX_Command[25];	/* String */
	// char Testitem1[10];
	// char Testitem2[10];
	// char Testitem3[10];
	// char Testitem4[10];
	// char Testitem5[10];
	// char Testitem6[10];
	// char Testitem7[10];
	// char Testitem8[10];
	// char Testitem9[10];
	// char Failitem[10];

public:
	// MachineHandler(/* args */);
	// ~MachineHandler();

	void Time_INIT(void);
	void INIT(void);
	// void O2_INIT(void);

	//	/*	Process	*/
	inline void Process_VT_N2(void);
	inline void Process_VT_2(void);
	void Process_Work(void);
	inline int Process_Wait_for_buttom(void);
	void Process_Wait_Pumming_D6(uint32_t target, uint32_t time_standard);
	void Process_Wait_Pumming_D4(uint32_t target, uint32_t time_standard);
	void Process_Keep_Pumming_D1(uint32_t target, uint32_t time_standard);
	void Process_Wait_Stabilize_pressure_D5(uint32_t target, uint32_t time_standard);
	void Process_Wait_Stabe_D2(uint32_t target, uint32_t time_standard);
	void Process_Pressure_Comparison_D3(void);
	//	/* Process N2 */
	void Process_N2_Pumming(uint32_t target, uint32_t time_standard);
	void Process_N2_Work(uint32_t target, uint32_t time_standard);
	void Process_O2_Wait(uint32_t target, uint32_t time_standard);
	void Process_O2_measuring(uint32_t target, uint32_t time_standard);

	float O2_readO2Vout(void);
	float O2_readConcentration(void);
	void O2_HOME(void);

	//	/*	BMX	*/
	bool BMP_Check_Connect_(void);
	void BMx_print280Data( Stream* client );
	void BMx_Read280Data(void);

	int Read_MPU(void);
	int Read_MPU_ASK(void);
	int Read_MPU_END(void);
	void Send_To_MPU(void);
	int MPU_COM_Parsing(void);
	int MPU_Version_Parsing(void);

    void Debug_Send_To_MPU_Message(void);

	void Reset_Array(void);
	void Reset_Reply(void);
	inline void Reset_RXCOM(void);
	void Reset_Confined_space(void);
	void Reset_Clean_Buffer(HardwareSerial * Client);

	#if _TEST_FUNC_
	//	/* test */
	void test_O2_Read(void);

	#endif
};



extern volatile unsigned long timer0_overflow_count;
extern MCU_IOHandler VT_N2_MCU_IOHandler;

#endif