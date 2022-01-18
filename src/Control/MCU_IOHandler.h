#ifndef __MCU_IO_HANDER_H__
#define __MCU_IO_HANDER_H__

#include "../Model/MCU_IO_Define.h"

class MCU_IOHandler
{
private:
	/* data */
public:
	MCU_IOHandler(/* args */);
	~MCU_IOHandler();

	void GPIO_INIT(void);

	int IO_Start_Status(void);
	void IO_Start_LED(bool);
	void IO_Proportional_Power(bool);
	void IO_Proportional_Valve_Control(int); // 0-255
	void IO_Vacuum_Value_Control(bool);
	void IO_N2_Value_Control(bool _val);
	void IO_Atmosphere_Value_Control(bool _val);
	void IO_Break_Status(bool *flag);
	void IO_Break_LED(bool _val);
	void IO_Product_Value(bool);
	void IO_Pumping_Control(bool);

	void Proportional_Open(void);
	void Proportional_Open_Clean(void);
	void Proportional_Close(void);

	void Solenoid_Value(int number, bool value);

};

#endif