#include "MachineHandler.h"
#include <Arduino.h>
extern MachineData VT_N2_MachineData;
extern ResultData VT_N2_ResultData;
extern MPU_COM_Parser VT_N2_MPU_Parser;
extern UartHandler VT_N2_UartHandler;
// StaticJsonDocument<500> cJSON_String;

uint32_t Process_time_standard = 0;
uint32_t Process_time_compare = 0;
uint32_t Over_time_standard = 0;
uint32_t Over_time_compare = 0;
uint32_t Over_time_standard_MPU = 0;
uint32_t Over_time_compare_MPU = 0;

BMx280I2C::Settings settings
(
	BME280::OSR_X1,
	BME280::OSR_X1,
	BME280::OSR_X1,
	BME280::Mode_Forced,
	BME280::StandbyTime_1000ms,
	BME280::Filter_Off,
	BME280::SpiEnable_False,
	0x76 // I2C address. I2C specific.
);
BMx280I2C ssenseBMx280(settings);

#if (O2_sensor_Choose == _ME2_O2_)
	// #define O2_readO2Vout() me2_O2.O2_readO2Vout()
	// #define O2_readConcentration() me2_O2.O2_readConcentration(float test)
#elif (O2_sensor_Choose == _TW_500B_)
	// extern _class_TW_500B_ TW_500B;
	#define O2_readO2Vout() tw_500b.O2_readO2Vout()
	#define O2_readConcentration() tw_500b.O2_readConcentration()
#endif

void MachineHandler::Time_INIT(void)
{
	Process_time_standard = micros();
}

void MachineHandler::INIT(void)
{
	VT_N2_MCU_IOHandler.GPIO_INIT();

	DebugPort.begin(SERIAL_SPEED);
	_MPU_.begin(SERIAL_SPEED);
	Wire.begin();

	//while(!DebugPort) {} // Wait
	DebugPort.println("V1_2");
	DebugPort.println("s-Sense BME/BMP280 I2C sensor.");

	if(!ssenseBMx280.begin())
	{
		DebugPort.println("Could not find BME/BMP280 sensor!");
	}

	switch(ssenseBMx280.chipModel())
	{
		case BME280::ChipModel_BME280:
		    DebugPort.println("Found BME280 sensor! Humidity available.");
		break;
		case BME280::ChipModel_BMP280:
		    DebugPort.println("Found BMP280 sensor! No Humidity available.");
		break;
		default:
		    DebugPort.println("Found UNKNOWN sensor! Error!");
		break;
	}

    #if _TEST_FUNC_
        test_O2_Read();
        BMx_print280Data(&DebugPort);
    #endif

	// Change some settings before using.
	settings.tempOSR = BME280::OSR_X4;

	ssenseBMx280.setSettings(settings);
	memcpy(VT_N2_ResultData.RX_Command, _MCU_Work_, strlen(_MCU_Work_) );
	memcpy(VT_N2_ResultData.N2Value, "0", strlen("0") );
	memcpy(VT_N2_ResultData.VTValue, "0", strlen("0") );
	memcpy(VT_N2_ResultData.N2Result, "init", strlen("init") );
	memcpy(VT_N2_ResultData.VTResult, "init", strlen("init") );
	memcpy(VT_N2_ResultData.Result, "init", strlen("init") );
	memcpy(VT_N2_ResultData.ErrorCode, "default", strlen("default") );
	memcpy(VT_N2_ResultData.Testitem1, "0", strlen("0") );
	memcpy(VT_N2_ResultData.Testitem2, "0", strlen("0") );
	memcpy(VT_N2_ResultData.Testitem3, "0", strlen("0") );
	memcpy(VT_N2_ResultData.Testitem4, "0", strlen("0") );
	memcpy(VT_N2_ResultData.Testitem5, "0", strlen("0") );
	memcpy(VT_N2_ResultData.Testitem6, "0", strlen("0") );
	memcpy(VT_N2_ResultData.Testitem7, "0", strlen("0") );
	memcpy(VT_N2_ResultData.Testitem8, "0", strlen("0") );
	memcpy(VT_N2_ResultData.Testitem9, "0", strlen("0") );
	memcpy(VT_N2_ResultData.Failitem, "0", strlen("0") );

	Process_time_standard = micros();

	//	/* Clean Sensor Buffer */
	VT_N2_MachineData.O2_Ampere = _O2_Ampere_;
#if !(_Clean_Confined_Space_Jump)
    Reset_Confined_space();
#endif
	digitalWrite(_O2_Reset_LED_, _OUTPUT_);
	DebugPort.print("Prepare OK\r\n");
}


//	/*	---------- Process ------------	*/
void MachineHandler::Process_Work(void)
{
	digitalWrite(_O2_Reset_LED_, _OUTPUT_);
	if( VT_N2_UartHandler.Read_Buffer() )
	{
		VT_N2_MPU_Parser.Catch_String(&VT_N2_MachineData,&VT_N2_UartHandler);
		Reset_Array();
		if( VT_N2_MPU_Parser.Parse_MPU_Command(&VT_N2_MachineData) )
		{
			VT_N2_MachineData.Command_Number = MPU_COM_Parsing();
			DebugPort.print("Command_Number = ");
			DebugPort.println(VT_N2_MachineData.Command_Number);
		}
	}
	if(VT_N2_MachineData.Command_Number != 0)
	{
		switch ( MPU_Version_Parsing() )
		{
			case 1:
				Process_VT_N2();
				break;

			case 2:
				Process_VT_2();
				break;

			case 0:

			default:

			break;
		}
	}
}

inline void MachineHandler::Process_VT_N2(void)
{
	uint32_t time_standard = 0;
	uint32_t time_compare = 0;
	bool f_Start_buttom = false;
	bool f_END_Status = false;
    float Vout = 0.0;
	float temp(NAN), hum(NAN), pres(NAN);
	BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
	BME280::PresUnit presUnit(BME280::PresUnit_Pa);

	if( BMP_Check_Connect_() )
	{
		BMx_Read280Data();
		//mbar_standard = mbar;
		VT_N2_MachineData.mbar_standard = 1013.65;
	}
	else
	{
		memset(VT_N2_ResultData.ErrorCode, '\0', sizeof(VT_N2_ResultData.ErrorCode));
		memcpy(VT_N2_ResultData.ErrorCode, _BMP_Error_Connect_ , strlen( _BMP_Error_Connect_ ) );
	}

	if(VT_N2_MachineData.f_Work == false)
	{
		Over_time_standard = millis();
		switch ( VT_N2_MachineData.Command_Number )
		{
			case 4:
				#if _Process_VT_N2_
				DebugPort.print("------ test clean -----\r\n");
				#endif
				VT_N2_MachineData.f_Clean = true;
				Reset_Confined_space();
				VT_N2_MachineData.Command_Number = 0;
				break;

			case 5:
				#if _Process_VT_N2_
					DebugPort.print("------ test BMP -----\r\n");
				#endif
				ssenseBMx280.read(pres, temp, hum, tempUnit, presUnit);
				VT_N2_MachineData.mbar = pres/100;

				DebugPort.print("Temp: ");
				DebugPort.print(temp);
				DebugPort.print("\t\tPressure: ");
				DebugPort.print(pres);
				DebugPort.println(" Pa");
				DebugPort.print(VT_N2_MachineData.mbar);
				DebugPort.println(" mbar");
				VT_N2_MachineData.Command_Number = 0;
				break;

			case 6:
				#if _Process_VT_N2_
					DebugPort.print("------ test O2 -----\r\n");
				#endif
				DebugPort.print("Vout =");
				Vout = O2_readO2Vout();
				DebugPort.print(Vout);
				DebugPort.print(" V, Concentration of O2 is ");
				DebugPort.println(O2_readConcentration());
				VT_N2_MachineData.Command_Number = 0;
				break;

			default:
				break;
		}
		// if( VT_N2_MachineData.Command_Number == 4)
		// {

		// }
        // else if( VT_N2_MachineData.Command_Number == 5 )
        // {

        // }
        // else if( VT_N2_MachineData.Command_Number == 6 )
        // {

        // }
	}
	else
	{
		if(Read_MPU_ASK() != 0)
		{
			MPU_COM_Parsing();
			Reset_RXCOM();
		}
	}

	if(VT_N2_MachineData.f_Work)
	{
    #if _TEST_FUNC_
        test_O2_Read();
    #endif

		VT_N2_MCU_IOHandler.IO_Start_LED(_OUTPUT_);

		memset( VT_N2_ResultData.RX_Command, '\0', sizeof(VT_N2_ResultData.RX_Command));
		memcpy( VT_N2_ResultData.RX_Command, _MCU_Work_, strlen(_MCU_Work_) );
		memset( VT_N2_ResultData.VTResult, '\0', sizeof(VT_N2_ResultData.VTResult));
		memcpy( VT_N2_ResultData.VTResult, _MCU_Processing_, strlen(_MCU_Processing_) );

		Reset_Clean_Buffer(&DebugPort);
		while(VT_N2_MachineData.f_Work != false && f_Start_buttom != true)
		{
            #if _Button_Jump_ == 0
                #if _Process_VT_N2_
                DebugPort.print("Wait for button\r\n");
                #endif
                if(Process_Wait_for_buttom() == _51_Open_)
                {
                    #if _Process_VT_N2_
                        DebugPort.print(Process_Wait_for_buttom());
                        DebugPort.print("------ 123 -----\r\n");
                    #endif
                    for(int i= 0; i<20; i++)
                    {
                        if(Process_Wait_for_buttom() == _51_Open_)
                        {
                            #if _Process_VT_N2_
                                DebugPort.print("------ 456 -----\r\n");
                            #endif
                            f_Start_buttom = true;
                        }
                        else
                            f_Start_buttom = false;
                    }
                }
            #else
			    f_Start_buttom = true;
            #endif

			Read_MPU_ASK();
			if(VT_N2_MachineData.f_Ask)
			{
				Send_To_MPU();
			}

			// /* if not push buttom */
			Over_time_compare = millis();
			#if _Time_OUT_Jump != 1
                if( ( Over_time_compare - Over_time_standard) > Time_OUT )
                {
                    timer0_overflow_count = 0;	// timer reset
                    Reset_Array();
                    Reset_Reply();
                    VT_N2_MachineData.f_Work = false;
                    VT_N2_MachineData.f_Break_Value = false;
                    VT_N2_MachineData.f_ERROR = false;
                }
			#elif _Time_OUT_Jump == 1
                memcpy(RX_Command, _MCU_Work_, strlen(_MCU_Work_) );
                memcpy(N2Value, "0", strlen("0") );
                memcpy(VTValue, "0", strlen("0") );
                memcpy(N2Result, _Fail_, strlen(_Fail_) );
                memcpy(VTResult, _Fail_, strlen(_Fail_) );
                memcpy(Result, _Fail_, strlen(_Fail_) );
                memcpy(ErrorCode, _Over_Time_, strlen(_Over_Time_) );
                f_Work_Status = false;
			#endif
		}
	}

	if ( f_Start_buttom == true )
	{
        #if (!_VT_Jump_)
            timer0_overflow_count = 0;	// timer reset
            #if _Process_VT_N2_
                DebugPort.print(" V, Concentration of O2 is ");
                DebugPort.println(O2_readConcentration());
            #endif
            VT_N2_MachineData.f_Break_Value = false;
            digitalWrite(_O2_Reset_LED_, _STOP_);
            Reset_Reply();

            VT_N2_MCU_IOHandler.IO_Start_LED(_STOP_);
            VT_N2_MCU_IOHandler.IO_Atmosphere_Value_Control(_STOP_);
            VT_N2_MCU_IOHandler.IO_Pumping_Control(_OUTPUT_);
            VT_N2_MCU_IOHandler.Proportional_Open();
            VT_N2_MCU_IOHandler.IO_Product_Value(_OUTPUT_);

            time_standard = micros();	// update time
            Process_Wait_Pumming_D6((uint32_t)VT_N2_MachineData.VTparam7, time_standard);
            #if _Read_Procress_BMP
                BMx_print280Data(&DebugPort);
            #endif

            Process_Wait_Pumming_D4((uint32_t)VT_N2_MachineData.VTparam5, time_standard);
            #if _Read_Procress_BMP
                BMx_print280Data(&DebugPort);
            #endif

            time_standard = micros();
            Process_Keep_Pumming_D1((uint32_t)VT_N2_MachineData.VTparam2, time_standard);
            // /* Close Vacuum */
            VT_N2_MCU_IOHandler.IO_Vacuum_Value_Control(_STOP_);
            VT_N2_MCU_IOHandler.IO_Pumping_Control(_STOP_);
            #if _Read_Procress_BMP
                BMx_print280Data(&DebugPort);
            #endif

            Process_Wait_Stabilize_pressure_D5((uint32_t)VT_N2_MachineData.VTparam6, time_standard);
            VT_N2_MCU_IOHandler.IO_Vacuum_Value_Control(_STOP_);
            VT_N2_MCU_IOHandler.IO_Pumping_Control(_STOP_);
            #if _Read_Procress_BMP
                BMx_print280Data(&DebugPort);
            #endif

            BMx_Read280Data();
            VT_N2_MachineData.D3_standard = VT_N2_MachineData.mbar;
            time_standard = micros();
            Process_Wait_Stabe_D2((uint32_t)VT_N2_MachineData.VTparam3, time_standard);

            //	/* Close Proportional */
            VT_N2_MCU_IOHandler.IO_Pumping_Control(_STOP_);
            VT_N2_MCU_IOHandler.Proportional_Close();

            //	/* VT Result */
            if(VT_N2_MachineData.f_ERROR == true)
            {
                memset( VT_N2_ResultData.VTResult, '\0', sizeof(VT_N2_ResultData.VTResult));
                memcpy( VT_N2_ResultData.VTResult, _Fail_, strlen(_Fail_) );
            }
            else
            {
                memset( VT_N2_ResultData.VTResult, '\0', sizeof(VT_N2_ResultData.VTResult));
                memcpy( VT_N2_ResultData.VTResult, _PASS_, strlen(_PASS_) );
            }
            #if _VT_Value_Jump
                memcpy(VT_N2_ResultData.Result, _PASS_, strlen(_PASS_) );
            #endif
        #else
            memset( VT_N2_ResultData.VTResult, '\0', sizeof(VT_N2_ResultData.VTResult));
			memcpy( VT_N2_ResultData.VTResult, _PASS_, strlen(_PASS_) );
        #endif


        #if(!_N2_Jump_)
                    // /* ---------- N2 -------- */
            time_standard = micros();
            Process_N2_Work((uint32_t)VT_N2_Machine.O2times, time_standard );

            time_standard = micros();
            Process_O2_Wait((uint32_t)5000000, time_standard);

            VT_N2_Machine.MAX_mbar = 200.0;
            time_standard = micros();
            if(VT_N2_Machine.f_ERROR != true)
            {
                VT_N2_MCU_IOHandler.IO_Start_LED(_STOP_);
                VT_N2_MCU_IOHandler.IO_Atmosphere_Value_Control(_STOP_);
                VT_N2_MCU_IOHandler.IO_Pumping_Control(_OUTPUT_);
                VT_N2_MCU_IOHandler.Proportional_Open_Clean();
                VT_N2_MCU_IOHandler.IO_Product_Value(_OUTPUT_);
                // Process_Wait_Pumming_D4((uint32_t)VTparam5, time_standard);
				Process_N2_Pumming((uint32_t)VT_N2_Machine.VTparam5, time_standard);
                VT_N2_MCU_IOHandler.IO_Vacuum_Value_Control(_STOP_);
                VT_N2_MCU_IOHandler.IO_Pumping_Control(_STOP_);
                VT_N2_MCU_IOHandler.Proportional_Close();
                memset( VT_N2_ResultData.ErrorCode, '\0', sizeof(VT_N2_ResultData.ErrorCode));
                VT_N2_Machine.f_ERROR = false;
            }

            time_standard = micros();
            Process_N2_Work((uint32_t)VT_N2_Machine.O2times, time_standard );

            time_standard = micros();
            Process_O2_Wait((uint32_t)5000000, time_standard);

            time_standard = micros();
            if(VT_N2_Machine.f_ERROR != true)
            {
                VT_N2_MCU_IOHandler.IO_Start_LED(_STOP_);
                VT_N2_MCU_IOHandler.IO_Atmosphere_Value_Control(_STOP_);
                VT_N2_MCU_IOHandler.IO_Pumping_Control(_OUTPUT_);
                VT_N2_MCU_IOHandler.Proportional_Open_Clean();
                VT_N2_MCU_IOHandler.IO_Product_Value(_OUTPUT_);
                // Process_Wait_Pumming_D4((uint32_t)VTparam5, time_standard);
				Process_N2_Pumming((uint32_t)VT_N2_Machine.VTparam5, time_standard);
                VT_N2_MCU_IOHandler.IO_Vacuum_Value_Control(_STOP_);
                VT_N2_MCU_IOHandler.IO_Pumping_Control(_STOP_);
                VT_N2_MCU_IOHandler.Proportional_Close();
            }

            time_standard = micros();
            Process_N2_Work((uint32_t)VT_N2_Machine.O2times, time_standard );

            VT_N2_MCU_IOHandler.IO_Atmosphere_Value_Control(_STOP_);
            //	/* Pumping */
            VT_N2_MCU_IOHandler.IO_Pumping_Control(_OUTPUT_);
            VT_N2_MCU_IOHandler.Proportional_Open_Clean();
            BMx_Read280Data();
            Over_time_standard = micros();
            while( ( Over_time_compare - Over_time_standard) < 10000000 ) // major large
            {
                Over_time_compare = micros();
                BMx_Read280Data();
                #if _Process_Debug
                DebugPort.print("3 mbar = ");
                DebugPort.println(VT_N2_Machine.mbar);
                #endif

                Read_MPU_ASK();
                if(VT_N2_Machine.f_Ask)
                {
                    memset( VT_N2_ResultData.RX_Command, '\0', sizeof(VT_N2_Machine.Command));
                    memcpy( VT_N2_ResultData.RX_Command, _MCU_Pressure_Reset_, strlen(_MCU_Pressure_Reset_) );
                    Send_To_MPU();
                }
                if(VT_N2_Machine.mbar < 1000)
                {
                    break;
                }
            }
            VT_N2_MCU_IOHandler.Proportional_Close();
	        VT_N2_MCU_IOHandler.IO_Pumping_Control(_STOP_);

            time_standard = micros();
            Process_O2_Wait((uint32_t)5000000, time_standard);

            time_standard = micros();
            Process_O2_measuring((uint32_t)VT_N2_Machine.O2judgmentoftimes, time_standard);

            VT_N2_MCU_IOHandler.IO_Product_Value(_STOP_);
            VT_N2_MCU_IOHandler.IO_Pumping_Control(_STOP_);
            VT_N2_MCU_IOHandler.IO_Product_Value(_STOP_);

            if(VT_N2_Machine.f_ERROR == true)
            {
                memset( VT_N2_ResultData.N2Result, '\0', sizeof(VT_N2_ResultData.N2Result));
                memcpy( VT_N2_ResultData.N2Result, _Fail_, strlen(_Fail_) );
            }
            else
            {
                memset( VT_N2_ResultData.N2Result, '\0', sizeof(VT_N2_ResultData.N2Result));
                memcpy( VT_N2_ResultData.N2Result, _PASS_, strlen(_PASS_) );
            }

            #if _Process_Debug
                DebugPort.print("------ N2 -----\r\n");
                DebugPort.print("\r\n");
                DebugPort.print("f_ERROR = ");
                DebugPort.print(VT_N2_Machine.f_ERROR);
                DebugPort.print("\r\n");
                DebugPort.print("f_Break_Value = ");
                DebugPort.print(VT_N2_Machine.f_Break_Value);
                DebugPort.print("\r\n");
		    #endif
        #else
            memset( VT_N2_ResultData.N2Result, '\0', sizeof(VT_N2_ResultData.N2Result));
            memcpy( VT_N2_ResultData.N2Result, _PASS_, strlen(_PASS_) );
        #endif


		if(VT_N2_MachineData.f_ERROR != true)
		{
            #if !(_Status_Jump_)
                while(1)
                {
                    if(Read_MPU_ASK() != 0)
                    {
                        //MPU_CMD = MPU_COM_Parsing();
                        if(VT_N2_MachineData.f_Ask)
                        {
                            Send_To_MPU();
                            Reset_RXCOM();
                            break;
                        }
                        else
                        {
                            Reset_RXCOM();
                        }
                    }
                }
            #endif
		}

		//	/* Clean Sensor Buffer */
        #if !(_Clean_Confined_Space_Jump)
		    Reset_Confined_space();
        #endif

		//	/* N2 & All Result */
		if(VT_N2_MachineData.f_ERROR == true)
		{
			memset( VT_N2_ResultData.RX_Command, '\0', sizeof(VT_N2_ResultData.RX_Command));
			memcpy( VT_N2_ResultData.RX_Command, _MCU_Work_, strlen(_MCU_Work_) );
			memset( VT_N2_ResultData.N2Result, '\0', sizeof(VT_N2_ResultData.N2Result));

			memcpy( VT_N2_ResultData.N2Result, _Fail_, strlen(_Fail_) );
			memset( VT_N2_ResultData.Result, '\0', sizeof(VT_N2_ResultData.Result));
			memcpy( VT_N2_ResultData.Result, _Fail_, strlen(_Fail_) );
		}
		else
		{
			memset( VT_N2_ResultData.RX_Command, '\0', sizeof(VT_N2_ResultData.RX_Command));
			memcpy( VT_N2_ResultData.RX_Command, _MCU_Work_, strlen(_MCU_Work_) );
			memset( VT_N2_ResultData.N2Result, '\0', sizeof(VT_N2_ResultData.N2Result));
			memcpy( VT_N2_ResultData.N2Result, _PASS_, strlen(_PASS_) );
			memset( VT_N2_ResultData.Result, '\0', sizeof(VT_N2_ResultData.Result));
			memcpy( VT_N2_ResultData.Result, _PASS_, strlen(_PASS_) );
		}

        Reset_Clean_Buffer(&_MPU_);

		//	/* Wait for END CMD */
        #if !(_END_Status_Jump_)
		while(1)
		{
			if(Read_MPU_ASK() != 0)
			{
				VT_N2_MachineData.Command_Number = MPU_COM_Parsing();
				if(f_END_Status == false)
				{
					if(VT_N2_MachineData.f_Ask)
					{
						Send_To_MPU();
						Reset_RXCOM();
						f_END_Status = true;
					}
				}
				else
				{
					if(VT_N2_MachineData.f_Ask)
					{
						Send_To_MPU();
						Reset_RXCOM();
					}

					if(VT_N2_MachineData.Command_Number == 3)
					{
						break;
					}
					else
					{
						Reset_RXCOM();
					}
				}
			}
		}
        #endif

        #if _Debug_End_Mes_
            Debug_Send_To_MPU_Message();
        #endif

		//	/* Reset All */
		timer0_overflow_count = 0;	// timer reset
		Reset_Array();
		Reset_Reply();
		VT_N2_MachineData.f_Work = false;
		VT_N2_MachineData.f_Break_Value = false;
		VT_N2_MachineData.f_ERROR = false;
	}
}

inline void MachineHandler::Process_VT_2(void)
{
	uint32_t time_standard = 0;
	uint32_t time_compare = 0;
	bool f_Start_buttom = false;
	bool f_END_Status = false;
	uint8_t MPU_CMD = 0;

	if( BMP_Check_Connect_() )
	{
		BMx_Read280Data();
		//mbar_standard = mbar;
		VT_N2_MachineData.mbar_standard = 1013.65;
	}
	else
	{
		memset( VT_N2_ResultData.ErrorCode, '\0', sizeof(VT_N2_ResultData.ErrorCode));
		memcpy( VT_N2_ResultData.ErrorCode, _BMP_Error_Connect_ , strlen( _BMP_Error_Connect_ ) );
	}

	if(VT_N2_MachineData.f_Work == false)
	{
		Over_time_standard = millis();
		MPU_CMD = MPU_COM_Parsing() ;
		if( MPU_CMD == 1 )
		{
			#if _Process_VT_N2_
			DebugPort.print("------ test Start -----\r\n");
			#endif
			Time_INIT();
		}
		else if( MPU_CMD == 4)
		{
			#if _Process_VT_N2_
			DebugPort.print("------ test clean -----\r\n");
			#endif
			VT_N2_MachineData.f_Clean = true;
			Reset_Confined_space();
		}
	}
	else
	{
		if(Read_MPU_ASK() != 0)
		{
			MPU_COM_Parsing();
			Reset_RXCOM();
		}
	}

	if(VT_N2_MachineData.f_Work)
	{
		#if _TEST_FUNC_
		test_O2_Read();
		#endif

		VT_N2_MCU_IOHandler.IO_Start_LED(_OUTPUT_);

		memset( VT_N2_ResultData.RX_Command, '\0', sizeof(VT_N2_ResultData.RX_Command));
		memcpy( VT_N2_ResultData.RX_Command, _MCU_Work_, strlen(_MCU_Work_) );
		memset( VT_N2_ResultData.VTResult, '\0', sizeof(VT_N2_ResultData.VTResult));
		memcpy( VT_N2_ResultData.VTResult, _MCU_Processing_, strlen(_MCU_Processing_) );

		Reset_Clean_Buffer(&DebugPort);
		while(VT_N2_MachineData.f_Work != false && f_Start_buttom != true)
		{
			#if _Button_Jump_ == 0
			#if _Process_VT_N2_
			DebugPort.print("Wait for button\r\n");
			#endif
			if(Process_Wait_for_buttom() == _51_Open_)
			{
				#if _Process_VT_N2_
				DebugPort.print(Process_Wait_for_buttom());
				DebugPort.print("------ 123 -----\r\n");
				#endif
				for(int i= 0; i<20; i++)
				{
					if(Process_Wait_for_buttom() == _51_Open_)
					{
						#if _Process_VT_N2_
						DebugPort.print("------ 456 -----\r\n");
						#endif
						f_Start_buttom = true;
	                }
					else
						f_Start_buttom = false;
				}
			}

			#else
			f_Start_buttom = true;
			#endif

			Read_MPU_ASK();
			if(VT_N2_MachineData.f_Ask)
			{
				Send_To_MPU();
			}

			// /* if not push buttom */
			Over_time_compare = millis();
			#if _Time_OUT_Jump != 1
			if( ( Over_time_compare - Over_time_standard) > 23000 )
			{
				timer0_overflow_count = 0;	// timer reset
				Reset_Array();
				Reset_Reply();
				VT_N2_MachineData.f_Work = false;
				VT_N2_MachineData.f_Break_Value = false;
				VT_N2_MachineData.f_ERROR = false;
			}
			#elif _Time_OUT_Jump == 1
			memcpy(VT_N2_ResultData.RX_Command, _MCU_Work_, strlen(_MCU_Work_) );
			memcpy(VT_N2_ResultData.N2Value, "0", strlen("0") );
			memcpy(VT_N2_ResultData.VTValue, "0", strlen("0") );
			memcpy(VT_N2_ResultData.N2Result, _Fail_, strlen(_Fail_) );
			memcpy(VT_N2_ResultData.VTResult, _Fail_, strlen(_Fail_) );
			memcpy(VT_N2_ResultData.Result, _Fail_, strlen(_Fail_) );
			memcpy(VT_N2_ResultData.ErrorCode, _Over_Time_, strlen(_Over_Time_) );
			VT_N2_Machine.f_Work_Status = false;
			#endif
        }
    }


	if ( f_Start_buttom == true )
	{
		timer0_overflow_count = 0;	// timer reset
		#if _Process_VT_N2_
		DebugPort.print(" V, Concentration of O2 is ");
		DebugPort.println(O2_readConcentration());
		#endif
		VT_N2_MachineData.f_Break_Value = false;
		digitalWrite(_O2_Reset_LED_, _STOP_);

		Reset_Reply();

		VT_N2_MCU_IOHandler.IO_Start_LED(_STOP_);
		VT_N2_MCU_IOHandler.IO_Atmosphere_Value_Control(_STOP_);
		VT_N2_MCU_IOHandler.IO_Pumping_Control(_OUTPUT_);
		VT_N2_MCU_IOHandler.Proportional_Open();
		VT_N2_MCU_IOHandler.IO_Product_Value(_OUTPUT_);

		time_standard = micros();	// update time
		Process_Wait_Pumming_D6((uint32_t)VT_N2_MachineData.VTparam7, time_standard);
		#if _Read_Procress_BMP
		BMx_print280Data(&DebugPort);
		#endif

		Process_Wait_Pumming_D4((uint32_t)VT_N2_MachineData.VTparam5, time_standard);
		#if _Read_Procress_BMP
		BMx_print280Data(&DebugPort);
		#endif

		time_standard = micros();
		Process_Keep_Pumming_D1((uint32_t)VT_N2_MachineData.VTparam2, time_standard);
		// /* Close Vacuum */
		VT_N2_MCU_IOHandler.IO_Vacuum_Value_Control(_STOP_);
		VT_N2_MCU_IOHandler.IO_Pumping_Control(_STOP_);
		#if _Read_Procress_BMP
		BMx_print280Data(&DebugPort);
		#endif

		Process_Wait_Stabilize_pressure_D5((uint32_t)VT_N2_MachineData.VTparam6, time_standard);
		VT_N2_MCU_IOHandler.IO_Vacuum_Value_Control(_STOP_);
		VT_N2_MCU_IOHandler.IO_Pumping_Control(_STOP_);
		#if _Read_Procress_BMP
		BMx_print280Data(&DebugPort);
		#endif

		BMx_Read280Data();
		VT_N2_MachineData.D3_standard = VT_N2_MachineData.mbar;
		time_standard = micros();
		Process_Wait_Stabe_D2((uint32_t)VT_N2_MachineData.VTparam3, time_standard);

		//	/* Close Proportional */
		VT_N2_MCU_IOHandler.IO_Pumping_Control(_STOP_);
		VT_N2_MCU_IOHandler.Proportional_Close();

		//	/* VT Result */
		if(VT_N2_MachineData.f_ERROR == true)
		{
			memset( VT_N2_ResultData.VTResult, '\0', sizeof(VT_N2_ResultData.VTResult));
			memcpy( VT_N2_ResultData.VTResult, _Fail_, strlen(_Fail_) );
		}
		else
		{
			memset( VT_N2_ResultData.VTResult, '\0', sizeof(VT_N2_ResultData.VTResult));
			memcpy( VT_N2_ResultData.VTResult, _PASS_, strlen(_PASS_) );
		}

		if(VT_N2_MachineData.f_ERROR != true)
		{
			while(1)
			{
				if(Read_MPU_ASK() != 0)
				{
					//MPU_CMD = MPU_COM_Parsing();
					if(VT_N2_MachineData.f_Ask)
					{
						Send_To_MPU();
						Reset_RXCOM();
						break;
					}
					else
					{
						Reset_RXCOM();
					}
				}
			}
		}

		//	/* All Result */
		if(VT_N2_MachineData.f_ERROR == true)
		{
			memset( VT_N2_ResultData.RX_Command, '\0', sizeof(VT_N2_ResultData.RX_Command));
			memcpy( VT_N2_ResultData.RX_Command, _MCU_Work_, strlen(_MCU_Work_) );
			memset( VT_N2_ResultData.Result, '\0', sizeof(VT_N2_ResultData.Result));
			memcpy( VT_N2_ResultData.Result, _Fail_, strlen(_Fail_) );
		}
		else
		{
			memset( VT_N2_ResultData.RX_Command, '\0', sizeof(VT_N2_ResultData.RX_Command));
			memcpy(VT_N2_ResultData. RX_Command, _MCU_Work_, strlen(_MCU_Work_) );
			memset( VT_N2_ResultData.Result, '\0', sizeof(VT_N2_ResultData.Result));
			memcpy( VT_N2_ResultData.Result, _PASS_, strlen(_PASS_) );
		}

		Reset_Clean_Buffer(&_MPU_);

		//	/* Wait for END CMD */
		while(1)
		{
			if(Read_MPU_ASK() != 0)
			{
				MPU_CMD = MPU_COM_Parsing();

				if(MPU_CMD != 0)
				{
					if(f_END_Status == false)
					{
						// MPU_CMD = Read_MPU_ASK();
						if( MPU_CMD == 2 )
						{
							Send_To_MPU();
							Reset_RXCOM();
							f_END_Status = true;
						}
					}
					else
					{
						if( MPU_CMD == 2 )
						{
							Send_To_MPU();
							Reset_RXCOM();
						}
						if( MPU_CMD == 3)
						{
							break;
						}
						else
						{
							Reset_RXCOM();
						}
					}
				}
			}
		}

		//	/* Reset All */
		timer0_overflow_count = 0;	// timer reset
		Reset_Array();
		Reset_Reply();
		VT_N2_MachineData.f_Work = false;
		VT_N2_MachineData.f_Break_Value = false;
		VT_N2_MachineData.f_ERROR = false;
	}
}


bool MachineHandler::BMP_Check_Connect_(void)
{
	if(!ssenseBMx280.begin())
	{
		return false;
	}
	return true;
}

inline int MachineHandler::Process_Wait_for_buttom(void)
{
	return VT_N2_MCU_IOHandler.IO_Start_Status();
}

void MachineHandler::Process_Wait_Pumming_D6(uint32_t target, uint32_t time_standard)
{
	uint32_t time_compare = 0;
	time_compare = micros();
	int16_t _mbar = 0;

	#if _Process_Debug
        if(target != 0)
        {
            DebugPort.print("------ D6 -----\r\n");
            DebugPort.print("time_compare = ");
            DebugPort.print(time_compare);
            DebugPort.print("\r\n");
        }
        DebugPort.print("f_ERROR = ");
        DebugPort.print(VT_N2_MachineData.f_ERROR);
        DebugPort.print("\r\n");
        DebugPort.print("f_Break_Value = ");
        DebugPort.print(VT_N2_MachineData.f_Break_Value);
        DebugPort.print("\r\n");
	#endif

	if(VT_N2_MachineData.f_ERROR != true)
	{
		if(target != 0 && VT_N2_MachineData.f_Break_Value!= true)
		{
			while( (time_compare - time_standard) <= target) // major large
			{
				time_compare = micros();
				BMx_Read280Data();
				_mbar = (int)floor(VT_N2_MachineData.mbar_standard) - (int)floor(VT_N2_MachineData.mbar);

				#if _Procress_D6_
                    DebugPort.print("mbar_standard = ");
                    DebugPort.println(mbar_standard);
                    DebugPort.print("_mbar = ");
                    DebugPort.println(_mbar);
                    DebugPort.print("mbar = ");
                    DebugPort.println(mbar);
                    DebugPort.print("MAX_mbar = ");
                    DebugPort.println(MAX_mbar);
                    DebugPort.print("MIN_mbar = ");
                    DebugPort.println(MIN_mbar);
				#endif
				if(_mbar < floor(VT_N2_MachineData.MAX_mbar) )
				{
                    time_compare = micros();
                    if((time_compare - time_standard) >= (uint32_t)floor(target / 3))
                    {
                        memset( VT_N2_ResultData.ErrorCode, '\0', sizeof(VT_N2_ResultData.ErrorCode));
                        memcpy( VT_N2_ResultData.ErrorCode, _D6_Error_, strlen(_D6_Error_) );
                        VT_N2_MachineData.f_ERROR = true;
                    }
				}
				else
				{
					memset( VT_N2_ResultData.ErrorCode, '\0', sizeof(VT_N2_ResultData.ErrorCode));
				}

				Read_MPU_ASK();
				if(VT_N2_MachineData.f_Ask)
				{
					Send_To_MPU();
				}

				VT_N2_MCU_IOHandler.IO_Break_Status((bool*)&VT_N2_MachineData.f_Break_Value);
				if(VT_N2_MachineData.f_Break_Value == true)
				{
					#if _Process_Debug
					DebugPort.print("Pause \r\n");
					#endif
					memset( VT_N2_ResultData.ErrorCode, '\0', sizeof(VT_N2_ResultData.ErrorCode));
					memcpy( VT_N2_ResultData.ErrorCode, _Pause_Error_, strlen(_Pause_Error_) );
                    memset( VT_N2_ResultData.Failitem, '\0', sizeof(VT_N2_ResultData.Failitem));
			        memcpy( VT_N2_ResultData.Failitem, "2", strlen("2") );
					VT_N2_MachineData.f_ERROR = true;
					break;
				}

			}
			itoa( floor(_mbar) , VT_N2_MachineData.VTValue , 10);
			memset( VT_N2_ResultData.Testitem2, '\0', sizeof(VT_N2_ResultData.Testitem2));
			memcpy( VT_N2_ResultData.Testitem2, VT_N2_MachineData.VTValue, strlen(VT_N2_MachineData.VTValue) );

		}
	}


	#if _Process_Debug
	DebugPort.print("time_compare = ");
	DebugPort.print(time_compare);
	DebugPort.print("\r\n");
	#endif
}

void MachineHandler::Process_Wait_Pumming_D4(uint32_t target, uint32_t time_standard)
{
	uint32_t time_compare = 0;
	int16_t _mbar = 0;
	time_compare = micros();
	float Vout =0;

	#if _Process_Debug
	DebugPort.print("------ D4 -----\r\n");
	DebugPort.print("time_standard = ");
	DebugPort.print(time_standard);
	DebugPort.print("\r\n");
	DebugPort.print("time_compare = ");
	DebugPort.print(time_compare);
	DebugPort.print("\r\n");
	DebugPort.print("f_ERROR = ");
	DebugPort.print(VT_N2_MachineData.f_ERROR);
	DebugPort.print("\r\n");
	DebugPort.print("f_Break_Value = ");
	DebugPort.print(VT_N2_MachineData.f_Break_Value);
	DebugPort.print("\r\n");
	#endif
	if(VT_N2_MachineData.f_ERROR == false)
	{
		#if _Process_Debug
		DebugPort.print("target = ");
		DebugPort.print(target);
		DebugPort.print("\r\n");
		#endif
		if(target != 0 && VT_N2_MachineData.f_Break_Value == false )
		{
			while( (time_compare - time_standard) <= target) // pummping too fast
			{
				time_compare = micros();
				BMx_Read280Data();
				_mbar = (int)floor(VT_N2_MachineData.mbar_standard) - (int)floor(VT_N2_MachineData.mbar);
				itoa( _mbar , VT_N2_MachineData.VTValue , 10);
				#if _Procress_D4_
				DebugPort.print("mbar_standard = ");
				DebugPort.println(VT_N2_MachineData.mbar_standard);
				DebugPort.print("_mbar = ");
				DebugPort.println(_mbar);
				DebugPort.print("mbar = ");
				DebugPort.println(VT_N2_MachineData.mbar);
				DebugPort.print("VTValue = ");
				DebugPort.println(VT_N2_MachineData.VTValue);
				DebugPort.print("MAX_mbar = ");
				DebugPort.println(VT_N2_MachineData.MAX_mbar);
				DebugPort.print("MIN_mbar = ");
				DebugPort.println(VT_N2_MachineData.MIN_mbar);
				#endif
				if(_mbar > (VT_N2_MachineData.MAX_mbar) )
				{
					memset( VT_N2_ResultData.ErrorCode, '\0', sizeof(VT_N2_ResultData.ErrorCode));
					break;
				}
				Read_MPU_ASK();
				if(VT_N2_MachineData.f_Ask)
				{
					Send_To_MPU();
				}

				VT_N2_MCU_IOHandler.IO_Break_Status((bool*) &VT_N2_MachineData.f_Break_Value);
				if(VT_N2_MachineData.f_Break_Value == true)
				{
					#if _Process_Debug
					DebugPort.print("Pause \r\n");
					#endif
					memset( VT_N2_ResultData.ErrorCode, '\0', sizeof(VT_N2_ResultData.ErrorCode));
					memcpy( VT_N2_ResultData.ErrorCode, _Pause_Error_, strlen(_Pause_Error_) );
					VT_N2_MachineData.f_ERROR = true;
					break;
				}
			}
			if(_mbar < (VT_N2_MachineData.MAX_mbar))
			{
				memset( VT_N2_ResultData.ErrorCode, '\0', sizeof(VT_N2_MachineData.ErrorCode));
				memcpy( VT_N2_ResultData.ErrorCode, _D4_Error_, strlen(_D4_Error_) );
                memset( VT_N2_ResultData.Failitem, '\0', sizeof(VT_N2_ResultData.Failitem));
                memcpy( VT_N2_ResultData.Failitem, "3", strlen("3") );
				VT_N2_MachineData.f_ERROR = true;
			}
			itoa( floor(_mbar) , VT_N2_MachineData.VTValue , 10);
			memset( VT_N2_ResultData.Testitem3, '\0', sizeof(VT_N2_ResultData.Testitem3));
			memcpy( VT_N2_ResultData.Testitem3, VT_N2_MachineData.VTValue, strlen(VT_N2_MachineData.VTValue) );
		}
	}

	#if _Process_Debug
	DebugPort.print("time_compare = ");
	DebugPort.print(time_compare);
	DebugPort.print("\r\n");
	#endif
}

void MachineHandler::Process_Keep_Pumming_D1(uint32_t target, uint32_t time_standard)
{
	uint32_t time_compare = 0;
	time_compare = micros();

    DebugPort.print("------ D1 -----\r\n");
	#if _Procress_D1_
	DebugPort.print("time_compare = ");
	DebugPort.print(time_compare);
	DebugPort.print("\r\n");
	DebugPort.print("f_ERROR = ");
	DebugPort.print(VT_N2_MachineData.f_ERROR);
	DebugPort.print("\r\n");
	DebugPort.print("f_Break_Value = ");
	DebugPort.print(VT_N2_MachineData.f_Break_Value);
	#endif
    DebugPort.print("\r\n");

	if(VT_N2_MachineData.f_ERROR != true)
	{
		if(target != 0 && VT_N2_MachineData.f_Break_Value!= true )
		{
			while( (time_compare - time_standard) <= target) // major large
			{
				time_compare = micros();
				Read_MPU_ASK();
				if(VT_N2_MachineData.f_Ask)
				{
					Send_To_MPU();
				}

				VT_N2_MCU_IOHandler.IO_Break_Status((bool*) &VT_N2_MachineData.f_Break_Value);
				if(VT_N2_MachineData.f_Break_Value == true)
				{
					#if _Procress_D1_
					DebugPort.print("Pause \r\n");
					#endif
					memset( VT_N2_ResultData.ErrorCode, '\0', sizeof(VT_N2_ResultData.ErrorCode));
					memcpy( VT_N2_ResultData.ErrorCode, _Pause_Error_, strlen(_Pause_Error_) );
					VT_N2_MachineData.f_ERROR = true;
					break;
				}
			}
		}
	}
	#if _Procress_D1_
	DebugPort.print("time_compare = ");
	DebugPort.print(time_compare);
	DebugPort.print("\r\n");
	#endif
}

void MachineHandler::Process_Wait_Stabilize_pressure_D5(uint32_t target, uint32_t time_standard)
{
	uint32_t time_compare = 0;
	uint8_t frequency = 0;
	int16_t _mbar = 0;
	time_compare = micros();

    DebugPort.print("------ D5 -----\r\n");
	#if _Procress_D5_
	DebugPort.print("time_compare = ");
	DebugPort.print(time_compare);
	DebugPort.print("\r\n");
	DebugPort.print("f_ERROR = ");
	DebugPort.print(VT_N2_MachineData.f_ERROR);
	DebugPort.print("\r\n");
	DebugPort.print("f_Break_Value = ");
	DebugPort.print(VT_N2_MachineData.f_Break_Value);
	#endif
    DebugPort.print("\r\n");

	if(VT_N2_MachineData.f_ERROR != true)
	{
		if(target != 0 &&  VT_N2_MachineData.f_Break_Value!= true )
		{
			while( (time_compare - time_standard) <= target)
			{
				time_compare = micros();
				BMx_Read280Data();
				_mbar = (int)floor(VT_N2_MachineData.mbar_standard) - (int)floor(VT_N2_MachineData.mbar);
				#if _Procress_D5_
				DebugPort.print("mbar_standard = ");
				DebugPort.println(VT_N2_MachineData.mbar_standard);
				DebugPort.print("_mbar = ");
				DebugPort.println(_mbar);
				DebugPort.print("mbar = ");
				DebugPort.println(VT_N2_MachineData.mbar);
				DebugPort.print("MAX_mbar = ");
				DebugPort.println(VT_N2_MachineData.MAX_mbar);
				DebugPort.print("MIN_mbar = ");
				DebugPort.println(VT_N2_MachineData.MIN_mbar);
				#endif
				if(_mbar < (VT_N2_MachineData.MIN_mbar))
				{
					frequency++;
					while(_mbar < (VT_N2_MachineData.MIN_mbar))
					{
						#if _Procress_D5_
						DebugPort.print(frequency);
						DebugPort.print("    +1 \r\n");
						#endif
						VT_N2_MCU_IOHandler.IO_Vacuum_Value_Control(_OUTPUT_);
						VT_N2_MCU_IOHandler.IO_Pumping_Control(_OUTPUT_);

						BMx_Read280Data();
						_mbar = (int16_t)floor(VT_N2_MachineData.mbar_standard) - (int16_t)floor(VT_N2_MachineData.mbar);
						VT_N2_MCU_IOHandler.IO_Break_Status((bool*) &VT_N2_MachineData.f_Break_Value);
						if(VT_N2_MachineData.f_Break_Value == true )
						{
							memset( VT_N2_ResultData.ErrorCode, '\0', sizeof(VT_N2_ResultData.ErrorCode));
							memcpy( VT_N2_ResultData.ErrorCode, _Pause_Error_, strlen(_Pause_Error_) );
							VT_N2_MachineData.f_ERROR = true;
							break;
						}
					}
					VT_N2_MCU_IOHandler.IO_Vacuum_Value_Control(_STOP_);
					VT_N2_MCU_IOHandler.IO_Pumping_Control(_STOP_);
				}

				if(frequency > VT_N2_MachineData.VTparam1)
				{
					memset( VT_N2_ResultData.ErrorCode, '\0', sizeof(VT_N2_ResultData.ErrorCode));
					memcpy( VT_N2_ResultData.ErrorCode, _D5_Error_, strlen(_D5_Error_) );
                    memset( VT_N2_ResultData.Failitem, '\0', sizeof(VT_N2_ResultData.Failitem));
			        memcpy( VT_N2_ResultData.Failitem, "4", strlen("4") );
					VT_N2_MachineData.f_ERROR = true;
				}

				Read_MPU_ASK();
				if(VT_N2_MachineData.f_Ask)
				{
					Send_To_MPU();
				}

				VT_N2_MCU_IOHandler.IO_Break_Status((bool*) &VT_N2_MachineData.f_Break_Value);
				if(VT_N2_MachineData.f_Break_Value == true)
				{
					#if _Procress_D5_
					DebugPort.print("Pause \r\n");
					#endif
					memset( VT_N2_ResultData.ErrorCode, '\0', sizeof(VT_N2_ResultData.ErrorCode));
					memcpy( VT_N2_ResultData.ErrorCode, _Pause_Error_, strlen(_Pause_Error_) );
					VT_N2_MachineData.f_ERROR = true;
					break;
				}
			}
			itoa( floor(_mbar) , VT_N2_MachineData.VTValue , 10);
			memset( VT_N2_ResultData.Testitem4, '\0', sizeof(VT_N2_ResultData.Testitem4));
			memcpy( VT_N2_ResultData.Testitem4, VT_N2_MachineData.VTValue, strlen(VT_N2_MachineData.VTValue) );
		}
	}

	#if _Procress_D5_
	DebugPort.print("time_compare = ");
	DebugPort.print(time_compare);
	DebugPort.print("\r\n");
	#endif
}

void MachineHandler::Process_Wait_Stabe_D2(uint32_t target, uint32_t time_standard)
{
	uint32_t time_compare = 0;
	int16_t _mbar = 0;
	time_compare = micros();
    VT_N2_MachineData.count_micro_leak_buffer = 0;

    DebugPort.print("------ D2 -----\r\n");
	#if _Procress_D2_
	DebugPort.print("time_compare = ");
	DebugPort.print(time_compare);
	DebugPort.print("\r\n");
	DebugPort.print("f_ERROR = ");
	DebugPort.print(VT_N2_MachineData.f_ERROR);
	DebugPort.print("\r\n");
	DebugPort.print("f_Break_Value = ");
	DebugPort.print(VT_N2_MachineData.f_Break_Value);
	#endif
    DebugPort.print("\r\n");

	if(VT_N2_MachineData.f_ERROR != true)
	{
		if(target != 0 &&  VT_N2_MachineData.f_Break_Value!= true )
		{
			while( (time_compare - time_standard) <= target)
			{
				time_compare = micros();
				BMx_Read280Data();
				_mbar = (int16_t)floor(VT_N2_MachineData.mbar_standard) - (int16_t)floor(VT_N2_MachineData.mbar);
                if((time_compare - time_standard) >= (target/(uint32_t)3))
                {
                    if(_mbar < VT_N2_MachineData.MIN_mbar)
                    {
                        memset( VT_N2_ResultData.ErrorCode, '\0', sizeof(VT_N2_ResultData.ErrorCode));
                        memcpy( VT_N2_ResultData.ErrorCode, _D2_Error_, strlen(_D2_Error_) );
                        VT_N2_MachineData.f_ERROR = true;
                    }
                }


				Read_MPU_ASK();
				if(VT_N2_MachineData.f_Ask)
				{
					Send_To_MPU();
				}

				VT_N2_MCU_IOHandler.IO_Break_Status((bool*) &VT_N2_MachineData.f_Break_Value);
				if(VT_N2_MachineData.f_Break_Value == true)
				{
					#if _Procress_D2_
					    DebugPort.print("Pause \r\n");
					#endif
					memset( VT_N2_ResultData.ErrorCode, '\0', sizeof(VT_N2_ResultData.ErrorCode));
					memcpy( VT_N2_ResultData.ErrorCode, _Pause_Error_, strlen(_Pause_Error_) );
					VT_N2_MachineData.f_ERROR = true;
					break;
				}
				Process_Pressure_Comparison_D3();

				if(VT_N2_MachineData.f_ERROR == true)
				{
					break;
				}
			}
			itoa( floor(_mbar) , VT_N2_MachineData.VTValue , 10);
			memset( VT_N2_ResultData.Testitem5, '\0', sizeof(VT_N2_ResultData.Testitem5));
			memcpy( VT_N2_ResultData.Testitem5, VT_N2_MachineData.VTValue, strlen(VT_N2_MachineData.VTValue) );
		}
	}

	#if _Procress_D2_
	DebugPort.print("time_compare = ");
	DebugPort.print(time_compare);
	DebugPort.print("\r\n");
	#endif
}

void MachineHandler::Process_Pressure_Comparison_D3(void)
{
	int16_t _mbar = 0;
	// int16_t _mbarD3 = 0;
    float _mbarD3 = 0;

	#if _Procress_D3_
	DebugPort.print("------ D3 -----\r\n");
	#endif
	BMx_Read280Data();
	//if(VT_N2_Machine.f_ERROR != true)
	//{
		if(VT_N2_MachineData.f_Break_Value!= true )
		{
			// _mbarD3 = (int)floor(mbar) - (int)floor(D3_standard);
            _mbarD3 = VT_N2_MachineData.mbar - VT_N2_MachineData.D3_standard;
			_mbar = (int)floor(VT_N2_MachineData.mbar_standard) - (int)floor(VT_N2_MachineData.mbar);
			// #if _Procress_D3_
			// DebugPort.print("mbar_standard = ");
			// DebugPort.println(mbar_standard);
			// DebugPort.print("_mbar = ");
			// DebugPort.println(_mbar);
			// DebugPort.print("_mbarD3 = ");
			// DebugPort.println(_mbarD3);
			// DebugPort.print("mbar = ");
			// DebugPort.println(mbar);
			// DebugPort.print("D3_standard = ");
			// DebugPort.println(D3_standard);
            // DebugPort.print("mbar_Range = ");
			// DebugPort.println(mbar_Range);
			// #endif

			// if( (unsigned int)(_mbarD3) > (int)floor(mbar_Range))
			// {
			// 	memset( ErrorCode, '\0', sizeof(ErrorCode));
			// 	memcpy(ErrorCode, _D3_Error_, strlen(_D3_Error_) );
            //     memset( Failitem, '\0', sizeof(Failitem));
            //     memcpy( Failitem, "6", strlen("6") );
			// 	VT_N2_Machine.f_ERROR = true;
			// }
            if( _mbarD3 > VT_N2_MachineData.mbar_Range)
			{
                #if _Procress_D3_
                DebugPort.print("mbar_standard = ");
                DebugPort.println(VT_N2_MachineData.mbar_standard);
                DebugPort.print("_mbar = ");
                DebugPort.println(_mbar);
                DebugPort.print("_mbarD3 = ");
                DebugPort.println(_mbarD3);
                DebugPort.print("mbar = ");
                DebugPort.println(VT_N2_MachineData.mbar);
                DebugPort.print("D3_standard = ");
                DebugPort.println(VT_N2_MachineData.D3_standard);
                DebugPort.print("mbar_Range = ");
                DebugPort.println(VT_N2_MachineData.mbar_Range);
                #endif
                VT_N2_MachineData.count_micro_leak_buffer ++ ;
                if(VT_N2_MachineData.count_micro_leak_buffer >= VT_N2_MachineData.micro_leak_buffer)
                {
                    memset( VT_N2_ResultData.ErrorCode, '\0', sizeof(VT_N2_ResultData.ErrorCode));
                    memcpy( VT_N2_ResultData.ErrorCode, _D3_Error_, strlen(_D3_Error_) );
                    memset( VT_N2_ResultData.Failitem, '\0', sizeof(VT_N2_ResultData.Failitem));
                    memcpy( VT_N2_ResultData.Failitem, "6", strlen("6") );
                    VT_N2_MachineData.f_ERROR = true;
                }
			}
			else
			{
				memset( VT_N2_ResultData.ErrorCode, '\0', sizeof(VT_N2_MachineData.ErrorCode));
				VT_N2_MachineData.f_ERROR = false;
			}
		}
		itoa( floor(_mbar) , VT_N2_MachineData.VTValue , 10);
		memset( VT_N2_ResultData.Testitem6, '\0', sizeof(VT_N2_ResultData.Testitem6));
		memcpy( VT_N2_ResultData.Testitem6, VT_N2_MachineData.VTValue, strlen(VT_N2_MachineData.VTValue) );

		Read_MPU_ASK();
		if(VT_N2_MachineData.f_Ask)
		{
			Send_To_MPU();
		}
	//}
}

void MachineHandler::Process_N2_Pumming(uint32_t target, uint32_t time_standard)
{
	uint32_t time_compare = 0;
	int16_t _mbar = 0;
	time_compare = micros();
	float Vout =0;

	// #if _Process_Debug
	// DebugPort.print("------ D4 -----\r\n");
	// DebugPort.print("time_standard = ");
	// DebugPort.print(time_standard);
	// DebugPort.print("\r\n");
	// DebugPort.print("time_compare = ");
	// DebugPort.print(time_compare);
	// DebugPort.print("\r\n");
	// DebugPort.print("f_ERROR = ");
	// DebugPort.print(VT_N2_Machine.f_ERROR);
	// DebugPort.print("\r\n");
	// DebugPort.print("f_Break_Value = ");
	// DebugPort.print(VT_N2_Machine.f_Break_Value);
	// DebugPort.print("\r\n");
	// #endif
	if(VT_N2_MachineData.f_ERROR == false)
	{
		#if _Process_Debug
		DebugPort.print("target = ");
		DebugPort.print(target);
		DebugPort.print("\r\n");
		#endif
		if(target != 0 && VT_N2_MachineData.f_Break_Value == false )
		{
			while( (time_compare - time_standard) <= target) // pummping too fast
			{
				time_compare = micros();
				BMx_Read280Data();
				_mbar = (int)floor(VT_N2_MachineData.mbar_standard) - (int)floor(VT_N2_MachineData.mbar);
				// itoa( _mbar , VTValue , 10);
				// #if _Procress_D4_
				// DebugPort.print("mbar_standard = ");
				// DebugPort.println(mbar_standard);
				// DebugPort.print("_mbar = ");
				// DebugPort.println(_mbar);
				// DebugPort.print("mbar = ");
				// DebugPort.println(mbar);
				// DebugPort.print("VTValue = ");
				// DebugPort.println(VTValue);
				// DebugPort.print("MAX_mbar = ");
				// DebugPort.println(MAX_mbar);
				// DebugPort.print("MIN_mbar = ");
				// DebugPort.println(MIN_mbar);
				// #endif
				if(_mbar > (VT_N2_MachineData.MAX_mbar) )
				{
					// memset( ErrorCode, '\0', sizeof(ErrorCode));
					break;
				}
				Read_MPU_ASK();
				if(VT_N2_MachineData.f_Ask)
				{
					Send_To_MPU();
				}

				VT_N2_MCU_IOHandler.IO_Break_Status((bool*) &VT_N2_MachineData.f_Break_Value);
				if(VT_N2_MachineData.f_Break_Value == true)
				{
					#if _Process_Debug
					DebugPort.print("Pause \r\n");
					#endif
					memset( VT_N2_ResultData.ErrorCode, '\0', sizeof(VT_N2_ResultData.ErrorCode));
					memcpy( VT_N2_ResultData.ErrorCode, _Pause_Error_, strlen(_Pause_Error_) );
					VT_N2_MachineData.f_ERROR = true;
					break;
				}
			}
			// if(_mbar < (MAX_mbar))
			// {
			// 	memset( ErrorCode, '\0', sizeof(ErrorCode));
			// 	memcpy(ErrorCode, _D4_Error_, strlen(_D4_Error_) );
            //     memset( Failitem, '\0', sizeof(Failitem));
            //     memcpy( Failitem, "3", strlen("3") );
			// 	VT_N2_Machine.f_ERROR = true;
			// }
			// itoa( floor(_mbar) , VTValue , 10);
			// memset( Testitem3, '\0', sizeof(Testitem3));
			// memcpy( Testitem3, VTValue, strlen(VTValue) );
		}
	}

	#if _Process_Debug
	DebugPort.print("time_compare = ");
	DebugPort.print(time_compare);
	DebugPort.print("\r\n");
	#endif
}

void MachineHandler::Process_N2_Work(uint32_t target, uint32_t time_standard)
{
	uint32_t time_compare = 0;
	int16_t _mbar = 0;
	float Vout =0;
	uint8_t Pause_Val = 0;
	time_compare = micros();

	#if _Process_Debug
	DebugPort.print("------ N2 -----\r\n");
	DebugPort.print("time_compare = ");
	DebugPort.print(time_compare);
	DebugPort.print("\r\n");
	DebugPort.print("f_ERROR = ");
	DebugPort.print(VT_N2_MachineData.f_ERROR);
	DebugPort.print("\r\n");
	DebugPort.print("f_Break_Value = ");
	DebugPort.print(VT_N2_MachineData.f_Break_Value);
	DebugPort.print("\r\n");
	#endif

	VT_N2_MachineData.Value_O2 = O2_readConcentration();

	if(VT_N2_MachineData.f_ERROR != true)
	{
		if(target != 0 && VT_N2_MachineData.f_Break_Value!= true )
		{
			VT_N2_MCU_IOHandler.IO_N2_Value_Control(_OUTPUT_);

			BMx_Read280Data();
			#if _Process_Debug
			DebugPort.print("mbar_standard = ");
			DebugPort.println(VT_N2_MachineData.mbar_standard);
			DebugPort.print("_mbar = ");
			DebugPort.println(_mbar);
			DebugPort.print("mbar = ");
			DebugPort.println(VT_N2_MachineData.mbar);
			#endif

			while( (time_compare - time_standard) <= target)
			{
				time_compare = micros();
				BMx_Read280Data();

				#if _Process_Debug
				DebugPort.print("time_compare = ");
				DebugPort.println(time_compare);
				DebugPort.print("mbar_standard = ");
				DebugPort.println(VT_N2_MachineData.mbar_standard);
				DebugPort.print("_mbar = ");
				DebugPort.println(_mbar);
				DebugPort.print("mbar = ");
				DebugPort.println(VT_N2_MachineData.mbar);
				#endif

				if(VT_N2_MachineData.mbar >= (float)VT_N2_MachineData.O2pressvalues )
				{
					VT_N2_MCU_IOHandler.IO_N2_Value_Control(_STOP_);
					break;
				}
				else
				{	// Error

				}

				Read_MPU_ASK();
				if(VT_N2_MachineData.f_Ask)
				{
					Send_To_MPU();
				}

				VT_N2_MCU_IOHandler.IO_Break_Status((bool*)&VT_N2_MachineData.f_Break_Value);
				if(VT_N2_MachineData.f_Break_Value == true)
				{
					#if _Process_Debug
					DebugPort.print("Pause \r\n");
					#endif
					memset( VT_N2_ResultData.ErrorCode, '\0', sizeof(VT_N2_ResultData.ErrorCode));
					memcpy( VT_N2_ResultData.ErrorCode, _Pause_Error_, strlen(_Pause_Error_) );
					VT_N2_MachineData.f_ERROR = true;
					break;
				}

			}

			BMx_Read280Data();
			if(VT_N2_MachineData.mbar < (VT_N2_MachineData.mbar_standard) )
			{		// Error
				VT_N2_MCU_IOHandler.IO_N2_Value_Control(_STOP_);
				memset( VT_N2_ResultData.ErrorCode, '\0', sizeof(VT_N2_ResultData.ErrorCode) );
				memcpy( VT_N2_ResultData.ErrorCode, _O2_Error_D1_, strlen(_O2_Error_D1_) );
				VT_N2_MachineData.f_ERROR = true;
			}
		}
	}

	VT_N2_MCU_IOHandler.IO_N2_Value_Control(_STOP_);
}

void MachineHandler::Process_O2_Wait(uint32_t target, uint32_t time_standard)
{
	uint32_t time_compare = 0;
	float Vout =0;
	time_compare = micros();

	#if _Process_Debug
	DebugPort.print("------ O2 Wait -----\r\n");
	DebugPort.print("time_compare = ");
	DebugPort.print(time_compare);
	DebugPort.print("\r\n");
	DebugPort.print("f_ERROR = ");
	DebugPort.print(VT_N2_MachineData.f_ERROR);
	DebugPort.print("\r\n");
	DebugPort.print("f_Break_Value = ");
	DebugPort.print(VT_N2_MachineData.f_Break_Value);
	DebugPort.print("\r\n");
	Vout = O2_readO2Vout();
	DebugPort.print(Vout);
	DebugPort.print(" V, Concentration of O2 is ");
	DebugPort.println(O2_readConcentration());
	#endif

	if(VT_N2_MachineData.f_ERROR != true && VT_N2_MachineData.f_Break_Value!= true)
	{
		while( (time_compare - time_standard) <= target)
		{
			time_compare = micros();
			VT_N2_MachineData.Value_O2 = O2_readConcentration();
			memset( VT_N2_ResultData.N2Value, '\0', sizeof(VT_N2_ResultData.N2Value));
			itoa((int)floor(VT_N2_MachineData.Value_O2), VT_N2_MachineData.N2Value, 10);

			VT_N2_MCU_IOHandler.IO_Break_Status((bool*) &VT_N2_MachineData.f_Break_Value);
			if(VT_N2_MachineData.f_Break_Value == true)
			{
				#if _Process_Debug
				DebugPort.print("Pause \r\n");
				#endif
				memset( VT_N2_ResultData.ErrorCode, '\0', sizeof(VT_N2_ResultData.ErrorCode));
				memcpy( VT_N2_ResultData.ErrorCode, _Pause_Error_, strlen(_Pause_Error_) );
				VT_N2_MachineData.f_ERROR = true;
				break;
			}

			Read_MPU_ASK();
			if(VT_N2_MachineData.f_Ask)
			{
				Send_To_MPU();
			}

			VT_N2_MCU_IOHandler.IO_Break_Status((bool*)&VT_N2_MachineData.f_Break_Value);
			if(VT_N2_MachineData.f_Break_Value == true)
			{
				#if _Process_Debug
				DebugPort.print("Pause \r\n");
				#endif
				memset( VT_N2_ResultData.ErrorCode, '\0', sizeof(VT_N2_ResultData.ErrorCode));
				memcpy( VT_N2_ResultData.ErrorCode, _Pause_Error_, strlen(_Pause_Error_) );
				VT_N2_MachineData.f_ERROR = true;
				break;
			}
		}
	}
}

void MachineHandler::Process_O2_measuring(uint32_t target, uint32_t time_standard)
{
	uint32_t time_compare = 0;
	int16_t _mbar = 0;
	float Vout =0;
	time_compare = micros();

	#if _Process_Debug
	DebugPort.print("------ O2 -----\r\n");
	DebugPort.print("time_compare = ");
	DebugPort.print(time_compare);
	DebugPort.print("\r\n");
	DebugPort.print("f_ERROR = ");
	DebugPort.print(VT_N2_MachineData.f_ERROR);
	DebugPort.print("\r\n");
	DebugPort.print("f_Break_Value = ");
	DebugPort.print(VT_N2_MachineData.f_Break_Value);
	DebugPort.print("\r\n");
	DebugPort.print("Vout =");
	Vout = O2_readO2Vout();
	DebugPort.print(Vout);
	DebugPort.print(" V, Concentration of O2 is ");
	DebugPort.println(O2_readConcentration());
	#endif

	VT_N2_MachineData.Value_O2 = O2_readConcentration();

	if(VT_N2_MachineData.f_ERROR != true)
	{
		if(target != 0 && VT_N2_MachineData.f_Break_Value!= true )
		{
			while( (time_compare - time_standard) <= target)
			{
				time_compare = micros();
				BMx_Read280Data();
				VT_N2_MachineData.Value_O2 = O2_readConcentration();
				memset( VT_N2_ResultData.N2Value, '\0', sizeof(VT_N2_ResultData.N2Value));
				itoa((int)floor(VT_N2_MachineData.Value_O2), VT_N2_MachineData.N2Value, 10);
				#if _Process_Debug
				DebugPort.print("time_compare = ");
				DebugPort.println(time_compare);
				DebugPort.print("mbar_standard = ");
				DebugPort.println(VT_N2_MachineData.mbar_standard);
				DebugPort.print("_mbar = ");
				DebugPort.println(_mbar);
				DebugPort.print("mbar = ");
				DebugPort.println(VT_N2_MachineData.mbar);
				DebugPort.print("Vout =");
				Vout = O2_readO2Vout();
				DebugPort.print(Vout);
				DebugPort.print(" V, Concentration of O2 is ");
				DebugPort.println(O2_readConcentration());
				#endif


				if(VT_N2_MachineData.Value_O2 <= (float)VT_N2_MachineData.O2threshol )
				{
					DebugPort.println("----- O2 break -----");
					memset( VT_N2_ResultData.ErrorCode, '\0', sizeof(VT_N2_ResultData.ErrorCode));
					VT_N2_MachineData.f_ERROR = false;
					break;
				}
				else
				{	// Error
					memset( VT_N2_ResultData.ErrorCode, '\0', sizeof(VT_N2_ResultData.ErrorCode));
					memcpy( VT_N2_ResultData.ErrorCode, _O2_Error_D3_, strlen(_O2_Error_D3_) );
					VT_N2_MachineData.f_ERROR = true;
				}

				Read_MPU_ASK();
				if(VT_N2_MachineData.f_Ask)
				{
					Send_To_MPU();
				}

				VT_N2_MCU_IOHandler.IO_Break_Status((bool*)&VT_N2_MachineData.f_Break_Value);
				if(VT_N2_MachineData.f_Break_Value == true)
				{
					#if _Process_Debug
					DebugPort.print("Pause \r\n");
					#endif
					memset( VT_N2_ResultData.ErrorCode, '\0', sizeof(VT_N2_ResultData.ErrorCode));
					memcpy( VT_N2_ResultData.ErrorCode, _Pause_Error_, strlen(_Pause_Error_) );
					VT_N2_MachineData.f_ERROR = true;
					break;
				}

			}

			if(VT_N2_MachineData.mbar < (VT_N2_MachineData.mbar_standard) )
			{		// Error


			}
		}
	}
}

float MachineHandler::O2_readO2Vout(void)
{
	long sum = 0;
	for(int i=0; i<32; i++)
	{
		sum += analogRead(_O2_Sensor_);
	}
	sum >>= 5;
	float MeasuredVout = sum * ((double)_O2_VRefer / 1023.0);
	return MeasuredVout;
}

float MachineHandler::O2_readConcentration(void)
{
	float MeasuredVout = O2_readO2Vout();

	//float Concentration = FmultiMap(MeasuredVout, VoutArray,O2ConArray, 6);
	//when its output voltage is 2.0V,
	//float Concentration = MeasuredVout * 0.21 / 2.0;
	//float Concentration = MeasuredVout * 0.21 / _O2_Ampere_;
	float Concentration = MeasuredVout * 0.21 / VT_N2_MachineData.O2_Ampere;
	float Concentration_Percentage = Concentration*100;
	return Concentration_Percentage;
}

void MachineHandler::O2_HOME(void)
{

}


//	/*	MPU Function	*/

int MachineHandler::Read_MPU_ASK(void)
{
	StaticJsonDocument<500> cJSON_String;
	char CMD ;
	const char *json;
	bool f_String_END = false;
	#if _TEST_MPU_Message_
	f_String_END = true;
	MPU_CMD = _FAKE_MPU_Message;
	#endif
	if(_MPU_.available())
	{
		while(_MPU_.available())
		{
			CMD = (char)_MPU_.read();
			if(CMD == '\r')
			{
				CMD = (char)_MPU_.read();
				if(CMD == '\n')
				{
					while(_MPU_.available())
						CMD = (char)_MPU_.read();
					f_String_END = true;
					break;
				}
			}
			VT_N2_MachineData.MPU_String = VT_N2_MachineData.MPU_String + CMD ;
		}
		#if _Read_MPU_Message
		DebugPort.print("MPU CMD = ");
		DebugPort.println(VT_N2_MachineData.MPU_String);
		#endif
	}

	if(f_String_END)
	{
		#if _Read_MPU_Message
		DebugPort.println();
		DebugPort.println("------- JSON Message ASK FUNC -------");
		#endif
		DeserializationError error = deserializeJson(cJSON_String, VT_N2_MachineData.MPU_String);
		if (!error)
		{
			json = cJSON_String["Command"];
			memcpy( VT_N2_MachineData.Command , (const char *)json , strlen(json) );
			MPU_COM_Parsing();
			#if _Read_MPU_Message
                DebugPort.print("Command = ");
                DebugPort.println(VT_N2_MachineData.Command);
			#endif
			VT_N2_MachineData.MPU_String = "";
			return 1;
		}
		VT_N2_MachineData.MPU_String = "";
	}

	_MPU_.flush();
	return 0;
}

int MachineHandler::Read_MPU_END(void)
{
	StaticJsonDocument<500> cJSON_String;
	char CMD ;
	const char *json;
	bool f_String_END = false;
	#if _TEST_MPU_Message_
        f_String_END = true;
        MPU_CMD = _FAKE_MPU_Message;
	#endif
	if(_MPU_.available())
	{
		while(_MPU_.available())
		{
			CMD = (char)_MPU_.read();
			if(CMD == '\r')
			{
				CMD = (char)_MPU_.read();
				if(CMD == '\n')
				{
					while(_MPU_.available())
						CMD = (char)_MPU_.read();
					f_String_END = true;
					break;
				}
			}
			VT_N2_MachineData.MPU_String = VT_N2_MachineData.MPU_String + CMD ;
		}
		#if _Read_MPU_Message
		DebugPort.print("MPU CMD = ");
		DebugPort.println(VT_N2_MachineData.MPU_String);
		#endif
	}

	if(f_String_END)
	{
		#if _Read_MPU_Message
		DebugPort.println();
		DebugPort.println("------- JSON Message -------");
		#endif
		DeserializationError error = deserializeJson(cJSON_String, VT_N2_MachineData.MPU_String);
		if (!error)
		{
			json = cJSON_String["Command"];
			memcpy( VT_N2_MachineData.Command , (const char *)json , strlen(json) );
			MPU_COM_Parsing();
			#if _Read_MPU_Message
			DebugPort.print("Command = ");
			DebugPort.println(VT_N2_MachineData.Command);
			#endif
			VT_N2_MachineData.MPU_String = "";
			return 1;
		}
	}
	return 0;
}

void MachineHandler::Send_To_MPU(void)
{
	_MPU_.print("{");
	_MPU_.print("\"Command\":\"");
	_MPU_.print(VT_N2_ResultData.RX_Command);
	_MPU_.print("\",");
	_MPU_.print("\"N2Value\":\"");
	_MPU_.print(VT_N2_ResultData.N2Value);
	_MPU_.print("\",");
	_MPU_.print("\"VTValue\":\"");
	_MPU_.print(VT_N2_ResultData.VTValue);
	_MPU_.print("\",");
	_MPU_.print("\"N2Result\":\"");
	_MPU_.print(VT_N2_ResultData.N2Result);
	_MPU_.print("\",");
	_MPU_.print("\"VTResult\":\"");
	_MPU_.print(VT_N2_ResultData.VTResult);
	_MPU_.print("\",");
	_MPU_.print("\"Result\":\"");
	_MPU_.print(VT_N2_ResultData.Result);
	_MPU_.print("\",");
	_MPU_.print("\"Errorcode\":\"");
	_MPU_.print(VT_N2_ResultData.ErrorCode);
	_MPU_.print("\",");
	_MPU_.print("\"Testitem1\":\"");
	_MPU_.print(VT_N2_ResultData.Testitem1);
	_MPU_.print("\",");
	_MPU_.print("\"Testitem2\":\"");
	_MPU_.print(VT_N2_ResultData.Testitem2);
	_MPU_.print("\",");
	_MPU_.print("\"Testitem3\":\"");
	_MPU_.print(VT_N2_ResultData.Testitem3);
	_MPU_.print("\",");
	_MPU_.print("\"Testitem4\":\"");
	_MPU_.print(VT_N2_ResultData.Testitem4);
	_MPU_.print("\",");
	_MPU_.print("\"Testitem5\":\"");
	_MPU_.print(VT_N2_ResultData.Testitem5);
	_MPU_.print("\",");
	_MPU_.print("\"Testitem6\":\"");
	_MPU_.print(VT_N2_ResultData.Testitem6);
	_MPU_.print("\",");
	_MPU_.print("\"Testitem7\":\"");
	_MPU_.print(VT_N2_ResultData.Testitem7);
	_MPU_.print("\",");
	_MPU_.print("\"Testitem8\":\"");
	_MPU_.print(VT_N2_ResultData.Testitem8);
	_MPU_.print("\",");
	_MPU_.print("\"Testitem9\":\"");
	_MPU_.print(VT_N2_ResultData.Testitem9);
	_MPU_.print("\",");
	_MPU_.print("\"Failitem\":\"");
	_MPU_.print(VT_N2_ResultData.Failitem);
	_MPU_.print("\"}");
	_MPU_.print("\r\n");

	#if _Send_MPU_Message
	DebugPort.print("{");
	DebugPort.print("\"Command\":\"");
	DebugPort.print(VT_N2_ResultData.RX_Command);
	DebugPort.print("\",");
	DebugPort.print("\"N2Value\":\"");
	DebugPort.print(VT_N2_ResultData.N2Value);
	DebugPort.print("\",");
	DebugPort.print("\"VTValue\":\"");
	DebugPort.print(VT_N2_ResultData.VTValue);
	DebugPort.print("\",");
	DebugPort.print("\"N2Result\":\"");
	DebugPort.print(VT_N2_ResultData.N2Result);
	DebugPort.print("\",");
	DebugPort.print("\"VTResult\":\"");
	DebugPort.print(VT_N2_ResultData.VTResult);
	DebugPort.print("\",");
	DebugPort.print("\"Result\":\"");
	DebugPort.print(VT_N2_ResultData.Result);
	DebugPort.print("\",");
	DebugPort.print("\"Errorcode\":\"");
	DebugPort.print(VT_N2_ResultData.ErrorCode);
	DebugPort.print("\",");
	DebugPort.print("\"Testitem1\":\"");
	DebugPort.print(VT_N2_ResultData.Testitem1);
	DebugPort.print("\",");
	DebugPort.print("\"Testitem2\":\"");
	DebugPort.print(VT_N2_ResultData.Testitem2);
	DebugPort.print("\",");
	DebugPort.print("\"Testitem3\":\"");
	DebugPort.print(VT_N2_ResultData.Testitem3);
	DebugPort.print("\",");
	DebugPort.print("\"Testitem4\":\"");
	DebugPort.print(VT_N2_ResultData.Testitem4);
	DebugPort.print("\"Testitem5\":\"");
	DebugPort.print(VT_N2_ResultData.Testitem5);
	DebugPort.print("\"Testitem6\":\"");
	DebugPort.print(VT_N2_ResultData.Testitem6);
	DebugPort.print("\"Testitem7\":\"");
	DebugPort.print(VT_N2_ResultData.Testitem7);
	DebugPort.print("\"Testitem8\":\"");
	DebugPort.print(VT_N2_ResultData.Testitem8);
	DebugPort.print("\",");
	DebugPort.print("\"Testitem9\":\"");
	DebugPort.print(VT_N2_ResultData.Testitem9);
	DebugPort.print("\",");
	DebugPort.print("\"Failitem\":\"");
	DebugPort.print(VT_N2_ResultData.Failitem);
	DebugPort.print("\"}");
	DebugPort.print("\r\n");
	#endif
	VT_N2_MachineData.f_Ask = false;
}

void MachineHandler::Debug_Send_To_MPU_Message(void)
{
    DebugPort.print("{");
	DebugPort.print("\"Command\":\"");
	DebugPort.print(VT_N2_ResultData.RX_Command);
	DebugPort.print("\",");
	DebugPort.print("\"N2Value\":\"");
	DebugPort.print(VT_N2_ResultData.N2Value);
	DebugPort.print("\",");
	DebugPort.print("\"VTValue\":\"");
	DebugPort.print(VT_N2_ResultData.VTValue);
	DebugPort.print("\",");
	DebugPort.print("\"N2Result\":\"");
	DebugPort.print(VT_N2_ResultData.N2Result);
	DebugPort.print("\",");
	DebugPort.print("\"VTResult\":\"");
	DebugPort.print(VT_N2_ResultData.VTResult);
	DebugPort.print("\",");
	DebugPort.print("\"Result\":\"");
	DebugPort.print(VT_N2_ResultData.Result);
	DebugPort.print("\",");
	DebugPort.print("\"Errorcode\":\"");
	DebugPort.print(VT_N2_ResultData.ErrorCode);
	DebugPort.print("\",");
	DebugPort.print("\"Testitem1\":\"");
	DebugPort.print(VT_N2_ResultData.Testitem1);
	DebugPort.print("\",");
	DebugPort.print("\"Testitem2\":\"");
	DebugPort.print(VT_N2_ResultData.Testitem2);
	DebugPort.print("\",");
	DebugPort.print("\"Testitem3\":\"");
	DebugPort.print(VT_N2_ResultData.Testitem3);
	DebugPort.print("\",");
	DebugPort.print("\"Testitem4\":\"");
	DebugPort.print(VT_N2_ResultData.Testitem4);
	DebugPort.print("\"Testitem5\":\"");
	DebugPort.print(VT_N2_ResultData.Testitem5);
	DebugPort.print("\"Testitem6\":\"");
	DebugPort.print(VT_N2_ResultData.Testitem6);
	DebugPort.print("\"Testitem7\":\"");
	DebugPort.print(VT_N2_ResultData.Testitem7);
	DebugPort.print("\"Testitem8\":\"");
	DebugPort.print(VT_N2_ResultData.Testitem8);
	DebugPort.print("\",");
	DebugPort.print("\"Testitem9\":\"");
	DebugPort.print(VT_N2_ResultData.Testitem9);
	DebugPort.print("\",");
	DebugPort.print("\"Failitem\":\"");
	DebugPort.print(VT_N2_ResultData.Failitem);
	DebugPort.print("\"}");
	DebugPort.print("\r\n");
}

int MachineHandler::MPU_COM_Parsing(void)
{
	if( memcmp("Start", VT_N2_MachineData.Command, sizeof("Start") ) == 0 )
	{
		VT_N2_MachineData.f_Work = true;
		memset( VT_N2_ResultData.RX_Command, '\0', sizeof(VT_N2_ResultData.RX_Command));
		memcpy( VT_N2_ResultData.RX_Command , "OK" , strlen("OK") );
		Send_To_MPU();
		Reset_RXCOM();
		return 1;
	}
	else if( memcmp("Status", VT_N2_MachineData.Command, sizeof("Status") ) == 0 )
	{
		VT_N2_MachineData.f_Ask = true;
		memset( VT_N2_ResultData.RX_Command, '\0', sizeof(VT_N2_ResultData.RX_Command));
		memcpy( VT_N2_ResultData.RX_Command, _MCU_Work_, strlen(_MCU_Work_) );
		return 2;
	}
	else if( memcmp("END", VT_N2_MachineData.Command, sizeof("END") ) == 0 )
	{
		memset( VT_N2_ResultData.RX_Command, '\0', sizeof(VT_N2_ResultData.RX_Command));
		memcpy( VT_N2_ResultData.RX_Command, _MCU_Work_, strlen(_MCU_Work_) );
		return 3;
	}
	else if( memcmp("Clean", VT_N2_MachineData.Command, sizeof("Clean") ) == 0 )
	{
		memset( VT_N2_ResultData.RX_Command, '\0', sizeof(VT_N2_ResultData.RX_Command));
		memcpy( VT_N2_ResultData.RX_Command , "OK" , strlen("OK") );
		Send_To_MPU();
		Reset_RXCOM();
		return 4;
	}
    else if( memcmp("TestBMP", VT_N2_MachineData.Command, sizeof("TestBMP") ) == 0 )
	{
		memset( VT_N2_ResultData.RX_Command, '\0', sizeof(VT_N2_ResultData.RX_Command));
		memcpy( VT_N2_ResultData.RX_Command , "OK" , strlen("OK") );
		Send_To_MPU();
		Reset_RXCOM();
		return 5;
	}
    else if( memcmp("TestO2", VT_N2_MachineData.Command, sizeof("TestO2") ) == 0 )
	{
		memset( VT_N2_ResultData.RX_Command, '\0', sizeof(VT_N2_ResultData.RX_Command));
		memcpy( VT_N2_ResultData.RX_Command , "OK" , strlen("OK") );
		Send_To_MPU();
		Reset_RXCOM();
		return 6;
	}
	else
	{
		return 0;
	}
}

int MachineHandler::MPU_Version_Parsing(void)
{
	if( memcmp(_VT_N2_, VT_N2_MachineData.VT_version, sizeof(_VT_N2_) ) == 0 )
	{
		DebugPort.print("------ 1 -----\r\n");
		return 1;
	}
	else if( memcmp(_VT_2_, VT_N2_MachineData.VT_version, sizeof(_VT_2_) ) == 0 )
	{
		DebugPort.print("------ 2 -----\r\n");
		return 2;
	}
	else
	{
		// DebugPort.print("------ 0 -----\r\n");
		return 0;
	}
}


//	/*	BMP 280 Function	*/
void MachineHandler::BMx_print280Data( Stream* client )
{
	float temp(NAN), hum(NAN), pres(NAN);
	//float temp(NAN),hum(NAN);

	BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
	BME280::PresUnit presUnit(BME280::PresUnit_Pa);

	ssenseBMx280.read(pres, temp, hum, tempUnit, presUnit);
	VT_N2_MachineData.mbar = pres/100;

	#if _Read_BMP_Message
	client->print("Temp: ");
	client->print(temp);
	//client->print("�X"+ String(tempUnit == BME280::TempUnit_Celsius ? 'C' :'F'));
	client->print(" "+ String(tempUnit == BME280::TempUnit_Celsius ? 'C' :'F'));
	client->print("\t\tHumidity: ");
	client->print(hum);
	client->print("% RH");
	client->print("\t\tPressure: ");
	client->print(pres);
	client->println(" Pa");
	client->print(mbar);
	client->println(" mbar");
	#endif

}

void MachineHandler::BMx_Read280Data(void)
{
	float temp(NAN), hum(NAN), pres(NAN);
    double _mber_ = 0.0;
    uint8_t amount_count = 0;
    uint8_t amount_standard = 2;
	//float temp(NAN),hum(NAN);
	Stream* client = &DebugPort;

	BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
	BME280::PresUnit presUnit(BME280::PresUnit_Pa);

    // for( amount_count = 0;amount_count<amount_standard ; amount_count++)
    // {
    //     ssenseBMx280.read(pres, temp, hum, tempUnit, presUnit);
    //     _mber_ =_mber_ + ( (double)pres/ 100.0 );
    // }
    // mbar = (float)_mber_/(float)amount_standard;

    ssenseBMx280.read(pres, temp, hum, tempUnit, presUnit);
	VT_N2_MachineData.mbar = 	pres/(float)100.0;

	#if _Read_BMP_Message
	client->print("Temp: ");
	client->print(temp);
	client->print(" "+ String(tempUnit == BME280::TempUnit_Celsius ? 'C' :'F'));
	client->print("\t\tHumidity: ");
	client->print(hum);
	client->print("% RH");
	client->print("\t\tPressure: ");
	client->print(pres);
	client->println(" Pa");
    client->print(_mber_);
	client->println(" _mber_");
	client->print(mbar);
	client->println(" mbar");
	#endif

}



//	/*	Reset Function	*/
void MachineHandler::Reset_Array(void)
{
	memset( VT_N2_MachineData.Command, '\0', sizeof(VT_N2_MachineData.Command));
	memset( VT_N2_MachineData.VT_version, '\0', sizeof(VT_N2_MachineData.VT_version));
	memset( VT_N2_ResultData.ErrorCode, '\0', sizeof(VT_N2_ResultData.ErrorCode));
	memcpy( VT_N2_ResultData.ErrorCode, "default", strlen("default") );
	VT_N2_MachineData.O2judgmentoftimes = 0;
	VT_N2_MachineData.O2threshol = 0;
	VT_N2_MachineData.O2times = 0;
	VT_N2_MachineData.VTparam1 = 0;
	VT_N2_MachineData.VTparam2 = 0;
	VT_N2_MachineData.VTparam3 = 0;
	VT_N2_MachineData.VTparam4 = 0;
	VT_N2_MachineData.VTparam5 = 0;
	VT_N2_MachineData.VTparam6 = 0;
	VT_N2_MachineData.VTparam7 = 0;
	VT_N2_MachineData.VTparam8 = 0;
	VT_N2_MachineData.VTparam9 = 0;


	VT_N2_MCU_IOHandler.IO_Start_LED(_STOP_);
}

void MachineHandler::Reset_Reply(void)
{
	memset(VT_N2_ResultData.RX_Command, '\0', sizeof(VT_N2_ResultData.RX_Command));
	memset(VT_N2_ResultData.N2Value, '\0', sizeof(VT_N2_ResultData.N2Value));
	memset(VT_N2_ResultData.VTValue, '\0', sizeof(VT_N2_ResultData.VTValue));
	memset(VT_N2_ResultData.N2Result, '\0', sizeof(VT_N2_ResultData.N2Result));
	memset(VT_N2_ResultData.VTResult, '\0', sizeof(VT_N2_ResultData.VTResult));
	memset(VT_N2_ResultData.Result, '\0', sizeof(VT_N2_ResultData.Result));
	memset(VT_N2_ResultData.ErrorCode, '\0', sizeof(VT_N2_ResultData.ErrorCode));
    memset(VT_N2_ResultData.Failitem, '\0', sizeof(VT_N2_ResultData.Failitem));

	memcpy(VT_N2_ResultData.RX_Command, _MCU_Work_, strlen(_MCU_Work_) );
	memcpy(VT_N2_ResultData.N2Value, "0", strlen("0") );
	memcpy(VT_N2_ResultData.VTValue, "0", strlen("0") );
	memcpy(VT_N2_ResultData.N2Result, "init", strlen("init") );
	memcpy(VT_N2_ResultData.VTResult, "init", strlen("init") );
	memcpy(VT_N2_ResultData.Result, "init", strlen("init") );
	memcpy(VT_N2_ResultData.ErrorCode, "default", strlen("default") );
    memcpy(VT_N2_ResultData.Failitem, "0", strlen("0") );
}

inline void MachineHandler::Reset_RXCOM(void)
{
	memset(VT_N2_MachineData.Command, '\0', sizeof(VT_N2_MachineData.Command));
}

void MachineHandler::Reset_Confined_space(void)
{
	bool f_END_Status = false;
	bool f_break = false;
	float Vout =0;

	if(VT_N2_MachineData.f_Clean == true)
	{
		Reset_Reply();
		Reset_Array();
	}

	memset(VT_N2_ResultData.RX_Command, '\0', sizeof(VT_N2_MachineData.Command));
	memcpy(VT_N2_ResultData.RX_Command, _MCU_Pressure_Reset_, strlen(_MCU_Pressure_Reset_) );

	VT_N2_MCU_IOHandler.IO_Product_Value(_STOP_);
	for(int i = 0;i<2;i++)
	{
		VT_N2_MCU_IOHandler.IO_Atmosphere_Value_Control(_STOP_);
		//	/* Pumping */
		VT_N2_MCU_IOHandler.IO_Pumping_Control(_OUTPUT_);
		VT_N2_MCU_IOHandler.Proportional_Open_Clean();

		BMx_Read280Data();
		Over_time_standard = micros();
		Reset_Clean_Buffer(&_MPU_);
		while( f_break == false )
		{
			Over_time_compare = micros();
			BMx_Read280Data();
			memset( VT_N2_ResultData.VTValue, '\0', sizeof(VT_N2_ResultData.VTValue));
			itoa( floor(VT_N2_MachineData.mbar) , VT_N2_MachineData.VTValue , 10);
			#if _Process_Debug
			DebugPort.print("1 mbar = ");
			DebugPort.println(VT_N2_MachineData.mbar);
			#endif

			#if _Process_Debug
			VT_N2_MachineData.Value_O2 = O2_readConcentration();
			memset( VT_N2_ResultData.N2Value, '\0', sizeof(VT_N2_ResultData.N2Value));
			itoa((int)floor(VT_N2_MachineData.Value_O2), VT_N2_MachineData.N2Value, 10);
			DebugPort.print("Vout =");
			Vout = O2_readO2Vout();
			DebugPort.print(Vout);
			DebugPort.print(" V, Concentration of O2 is ");
			DebugPort.println(O2_readConcentration());
			#endif

			Read_MPU_ASK();
			if(VT_N2_MachineData.f_Ask)
			{
				memset( VT_N2_ResultData.RX_Command, '\0', sizeof(VT_N2_MachineData.Command));
				memcpy( VT_N2_ResultData.RX_Command, _MCU_Pressure_Reset_, strlen(_MCU_Pressure_Reset_) );
				Send_To_MPU();
			}

			//Reset_Clean_Buffer(&_MPU_);
			if(VT_N2_MachineData.mbar < _Clean_Value_)
			{
				if(VT_N2_MachineData.f_Clean == false)
				{
					break;
				}
				else
				{
					while(f_break == false)
					{
						Read_MPU_ASK();
						if(VT_N2_MachineData.f_Ask)
						{
							memset(VT_N2_ResultData.RX_Command, '\0', sizeof(VT_N2_MachineData.Command));
							memcpy(VT_N2_ResultData.RX_Command, _MCU_Pressure_Reset_, strlen(_MCU_Pressure_Reset_) );
							Send_To_MPU();
							f_break = true;
							break;
						}
					}
				}
			}
			if( (Over_time_compare - Over_time_standard) > 10000000 )
			{
				f_break = true;
				break;
			}
		}
		f_break = false;
		VT_N2_MCU_IOHandler.Proportional_Close();
		VT_N2_MCU_IOHandler.IO_Pumping_Control(_STOP_);

		//	/* blow up */
		VT_N2_MCU_IOHandler.IO_Atmosphere_Value_Control(_OUTPUT_);
		delay(1000);

		BMx_Read280Data();
		Over_time_standard = micros();
		Reset_Clean_Buffer(&_MPU_);
		while( f_break == false )
		{
			Over_time_compare = micros();
			BMx_Read280Data();
			#if _Clean_Space_Data_
			DebugPort.print("2 mbar = ");
			DebugPort.println(VT_N2_MachineData.mbar);
			#endif

			#if _Process_Debug
			VT_N2_MachineData.Value_O2 = O2_readConcentration();
			memset(VT_N2_ResultData.N2Value, '\0', sizeof(VT_N2_ResultData.N2Value));
			itoa((int)floor(VT_N2_MachineData.Value_O2), VT_N2_ResultData.N2Value, 10);
			DebugPort.print("Vout =");
			Vout = O2_readO2Vout();
			DebugPort.print(Vout);
			DebugPort.print(" V, Concentration of O2 is ");
			DebugPort.println(O2_readConcentration());
			#endif

			Read_MPU_ASK();
			if(VT_N2_MachineData.f_Ask)
			{
				memset(VT_N2_ResultData.RX_Command, '\0', sizeof(VT_N2_MachineData.Command));
				memcpy(VT_N2_ResultData.RX_Command, _MCU_Pressure_Reset_, strlen(_MCU_Pressure_Reset_) );
				Send_To_MPU();
			}
			if(VT_N2_MachineData.mbar > 1000.0)
			{
				f_break = true;
				break;
			}
			if( (Over_time_compare - Over_time_standard) > 10000000 )
			{
				f_break = true;
				break;
			}
		}
		f_break = false;
		VT_N2_MCU_IOHandler.IO_Atmosphere_Value_Control(_STOP_);
		delay(800);
	}

	//	/* Pumping */
	BMx_Read280Data();
	#if _Process_Debug
	DebugPort.print("3 mbar = ");
	DebugPort.println(VT_N2_MachineData.mbar);
	#endif
	VT_N2_MCU_IOHandler.IO_Pumping_Control(_OUTPUT_);
	VT_N2_MCU_IOHandler.Proportional_Open_Clean();

	Over_time_standard = micros();
	Reset_Clean_Buffer(&_MPU_);
	while( f_break == false ) // major large
	{
		Over_time_compare = micros();
		BMx_Read280Data();
		#if _Process_Debug
		DebugPort.print("3 mbar = ");
		DebugPort.println(VT_N2_MachineData.mbar);
		#endif

		Read_MPU_ASK();
		if(VT_N2_MachineData.f_Ask)
		{
			memset(VT_N2_ResultData.RX_Command, '\0', sizeof(VT_N2_MachineData.Command));
			memcpy(VT_N2_ResultData.RX_Command, _MCU_Pressure_Reset_, strlen(_MCU_Pressure_Reset_) );
			Send_To_MPU();
		}
		if(VT_N2_MachineData.mbar < 1020)
		{
			f_break = true;
			break;
		}
		if( (Over_time_compare - Over_time_standard) > 10000000 )
		{
			f_break = true;
			break;
		}
	}
	VT_N2_MCU_IOHandler.Proportional_Close();
	VT_N2_MCU_IOHandler.IO_Pumping_Control(_STOP_);

	Reset_Clean_Buffer(&_MPU_);

	#if _Process_Debug
	VT_N2_MachineData.Value_O2 = O2_readConcentration();
	memset(VT_N2_ResultData.N2Value, '\0', sizeof(VT_N2_ResultData.N2Value));
	itoa((int)floor(VT_N2_MachineData.Value_O2), VT_N2_ResultData.N2Value, 10);
	DebugPort.print("Vout =");
	Vout = O2_readO2Vout();
	DebugPort.print(Vout);
	DebugPort.print(" V, Concentration of O2 is ");
	DebugPort.println(O2_readConcentration());
	#endif

	if(VT_N2_MachineData.f_Clean == true)
	{
		memset(VT_N2_ResultData.RX_Command, '\0', sizeof(VT_N2_MachineData.Command));
		memcpy(VT_N2_ResultData.RX_Command, _MCU_Pressure_Reset_, strlen(_MCU_Pressure_Reset_) );
		memset(VT_N2_ResultData.VTResult, '\0', sizeof(VT_N2_ResultData.VTResult));
		memcpy(VT_N2_ResultData.VTResult, _OK_, strlen(_OK_) );
		memset(VT_N2_ResultData.Result, '\0', sizeof(VT_N2_ResultData.Result));
		memcpy(VT_N2_ResultData.Result, _MCU_Processing_, strlen(_MCU_Processing_) );
		while(1)
		{
			if(Read_MPU_ASK() != 0)
			{
				if(f_END_Status == false)
				{
					if( MPU_COM_Parsing() == 2 )
					{
						memset(VT_N2_ResultData.RX_Command, '\0', sizeof(VT_N2_ResultData.RX_Command));
						memcpy(VT_N2_ResultData.RX_Command, _MCU_Pressure_Reset_, strlen(_MCU_Pressure_Reset_) );
						Send_To_MPU();
						DebugPort.print("Send Result\r\n");
						Reset_RXCOM();
						f_END_Status = true;
					}
				}
				else
				{
					if( MPU_COM_Parsing() == 3)
					{
						break;
					}
					else
					{
						Reset_RXCOM();
					}
				}

			}
		}
		Reset_Reply();
	}

	if(VT_N2_MachineData.O2_Ampere != O2_readO2Vout())
	{
		VT_N2_MachineData.O2_Ampere = O2_readO2Vout();
		DebugPort.print(" O2_Ampere = ");
		DebugPort.println(VT_N2_MachineData.O2_Ampere);
	}
	VT_N2_MachineData.f_Clean = false;
}


void MachineHandler::Reset_Clean_Buffer(HardwareSerial* Client)
{
	char read ;
	if(Client->available())
	{
		while(Client->available())
		{
			read = Client->read();
		}
	}
}

#if _TEST_FUNC_
void MachineHandler::test_O2_Read(void)
{
	float Vout = 0.0;
	DebugPort.print("Vout =");
	Vout = O2_readO2Vout();
	DebugPort.print(Vout);
	DebugPort.print(" V, Concentration of O2 is ");
	DebugPort.println(O2_readConcentration());
}

#endif