#ifndef __MCU_IO_DEFINE_H__
#define __MCU_IO_DEFINE_H__

#define _Logic_ 1
#if _Logic_
	#define _OUTPUT_ HIGH
	#define _STOP_ LOW
	#define _51_Close_	HIGH
	#define _51_Open_ LOW
#else
	#define _OUTPUT_ LOW
	#define _STOP_ HIGH
	#define _51_Close_	HIGH
	#define _51_Open_ LOW
#endif
// /* Pin defien */
//#define _Start_Buttom_	48
//#define _Start_Buttom_LED_	12
//#define _Proportional_Valve_Power_	22
//#define _Proportional_Valve_	7
//#define _Vacuum_Value_	24
//#define _N2_Value_	26
//#define _Atmosphere_Value_ 28
//#define _STOP_Button_ 50
//#define _STOP_Button_LED_ 50
// real
#define _Start_Buttom_	46	// Left
#define _Start_Buttom_Right_	48
#define _Start_Buttom_LED_	25
#define _STOP_Button_ 52
#define _STOP_Button_LED_ 29
#define _O2_Reset_Button_ 50
#define _O2_Reset_LED_	27

#define _Proportional_Valve_Power_	31
#define _Proportional_Valve_	7
#define _Vacuum_Value_	22	// V1
#define _N2_Value_	32	// V2
#define _Atmosphere_Value_ 26	// V3
#define _Product_Value_ 24	// V4
#define _Pumping_	30	// V6
#define _Solenoid_Value_1	22
#define _HSpeed_Solenoid_Value_		6	// not use now

#define _O2_Sensor_ A0
// #define _O2_Sensor_ A1
#define _O2_HOME_Buttom_ 52

#endif