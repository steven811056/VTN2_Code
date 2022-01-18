#include "MCU_IOHandler.h"
#include "Arduino.h"

MCU_IOHandler::MCU_IOHandler(/* args */)
{
}

MCU_IOHandler::~MCU_IOHandler()
{
}

void MCU_IOHandler::GPIO_INIT(void)
{

	pinMode(_Start_Buttom_LED_, OUTPUT);
	pinMode(_Start_Buttom_, INPUT);
	pinMode(_Start_Buttom_Right_, INPUT);
	pinMode(_STOP_Button_LED_, OUTPUT);
	pinMode(_STOP_Button_, INPUT);
	pinMode(_O2_Reset_Button_, INPUT);
	pinMode(_O2_Reset_LED_, OUTPUT);

	pinMode(_Proportional_Valve_Power_, OUTPUT);
	pinMode(_Proportional_Valve_, OUTPUT);
	pinMode(_Vacuum_Value_, OUTPUT);
	pinMode(_N2_Value_, OUTPUT);
	pinMode(_Atmosphere_Value_, OUTPUT);
	pinMode(_Product_Value_, OUTPUT);
    pinMode(34, OUTPUT);

	pinMode(_Pumping_, OUTPUT);
	pinMode(_Solenoid_Value_1, OUTPUT);
	pinMode(_HSpeed_Solenoid_Value_, OUTPUT);
	pinMode(_Proportional_Valve_, OUTPUT);

	pinMode(_O2_Sensor_, INPUT);
	pinMode(_O2_HOME_Buttom_, INPUT);

	digitalWrite(_Start_Buttom_LED_, _STOP_);
	digitalWrite(_STOP_Button_LED_, _STOP_);
	digitalWrite(_O2_Reset_LED_, _STOP_);

	digitalWrite(_Proportional_Valve_Power_, _STOP_);
	digitalWrite(_Vacuum_Value_, _STOP_);
	digitalWrite(_N2_Value_, _STOP_);
	digitalWrite(_Atmosphere_Value_, _STOP_);
	digitalWrite(_Product_Value_, _STOP_);
	digitalWrite(_HSpeed_Solenoid_Value_, _STOP_);
    digitalWrite(34, _STOP_);

	pinMode(12, OUTPUT);
	digitalWrite(12, _STOP_);
}

//	/*	----------- IO -----------	*/
int MCU_IOHandler::IO_Start_Status(void)
{
	if( (digitalRead(_Start_Buttom_) == _51_Open_) && (digitalRead(_Start_Buttom_Right_) == _51_Open_))
	{
		return _51_Open_;
	}
	else
	{
		return _51_Close_;
	}

}

void MCU_IOHandler::IO_Start_LED(bool _val)
{
	digitalWrite( _Start_Buttom_LED_ , _val );
}

void MCU_IOHandler::IO_Proportional_Power(bool _val)
{
	digitalWrite(_Proportional_Valve_Power_, _val);
}

void MCU_IOHandler::IO_Proportional_Valve_Control(int _var)
{
	analogWrite(_Proportional_Valve_, _var);
	if(_var == 0)
	{
		digitalWrite(_Proportional_Valve_, _STOP_);
	}
}

void MCU_IOHandler::IO_Vacuum_Value_Control(bool _val)
{
	delay(100);
	digitalWrite(_Vacuum_Value_, _val);
	delay(100);
}

void MCU_IOHandler::IO_N2_Value_Control(bool _val)
{
	digitalWrite(_N2_Value_, _val);
}

void MCU_IOHandler::IO_Atmosphere_Value_Control(bool _val)
{
	digitalWrite(_Atmosphere_Value_, _val);
}

void MCU_IOHandler::IO_Break_Status(bool *flag)
{
	if(digitalRead(_STOP_Button_) == _51_Open_)
	{
		/*DebugPort.print(digitalRead(_STOP_Button_));
		DebugPort.print("------ 123 -----\r\n");*/
		for(int i= 0; i<20; i++)
		{
			if(digitalRead(_STOP_Button_) == _51_Open_)
			{
				//DebugPort.print("------ 456 -----\r\n");
				*flag = true;
			}
			else
				*flag = false;
		}
	}
}

void MCU_IOHandler::IO_Break_LED(bool _val)
{
	digitalWrite(_STOP_Button_LED_, _val);
}

void MCU_IOHandler::IO_Product_Value(bool value)
{
	if(value)
	{
		digitalWrite(_Product_Value_,_OUTPUT_);
	}
	else
	{
		digitalWrite(_Product_Value_,_STOP_);
	}
}

void MCU_IOHandler::IO_Pumping_Control(bool value)
{
	if(value)
	{
		digitalWrite(_Pumping_,_OUTPUT_);
	}
	else
	{
		digitalWrite(_Pumping_,_STOP_);
	}
}

void MCU_IOHandler::Proportional_Open(void)
{
	IO_Proportional_Power(_OUTPUT_);
	delay(300);
	// IO_Proportional_Valve_Control(33);
    IO_Proportional_Valve_Control(38);
	IO_Vacuum_Value_Control(_OUTPUT_);
}

void MCU_IOHandler::Proportional_Open_Clean(void)
{
	IO_Proportional_Power(_OUTPUT_);
	delay(300);
	IO_Proportional_Valve_Control(250);
	IO_Vacuum_Value_Control(_OUTPUT_);
}

void MCU_IOHandler::Proportional_Close(void)
{
	IO_Vacuum_Value_Control(_STOP_);
	IO_Proportional_Valve_Control(0);
	delay(300);
	IO_Proportional_Power(_STOP_);
}

void MCU_IOHandler::Solenoid_Value(int number, bool value)
{
	digitalWrite(number, value);
}
