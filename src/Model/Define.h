#ifndef __DEFINE_H__
#define __DEFINE_H__

#include "MCU_IO_Define.h"

#define DebugPort Serial
// #define SERIAL_SPEED  9600
#define SERIAL_SPEED  115200
#define MPU_Serial Serial3
#define _MPU_ MPU_Serial
#define STM32 0
#define _Clean_Value_ 800.0

#define _ME2_O2_ 0
#define _TW_500B_ 1
#define O2_sensor_Choose _ME2_O2_
#if (O2_sensor_Choose == _ME2_O2_)
    #define _O2_VRefer 3.3
#elif (O2_sensor_Choose == _TW_500B_)
    #define _O2_VRefer 5
#endif
// #define _O2_VRefer 3.3

#define _O2_D1_standard_ 22.0	// /* float */
#define _O2_D2_standard_ 22.0	// /* float */
#define _O2_Ampere_ 0.51
//#define _O2_Ampere_ 2.1
#define _O2_Wait_time_ 20000000

// /* machine define */
#define _Time_OUT_ 30   // second
#define Time_OUT (_Time_OUT_ * 1000)    // mini second
#define _Time_OUT_MPU_RX_ 0.8
#define Time_OUT_MPU_RX (uint32_t)((float)_Time_OUT_MPU_RX_ * (float)1000)

// /* Version define */
#define _VT_N2_ "VT5"
#define _VT_2_	"VT2"

// /* pdf write 0.1 mbar */

// /* CMD define */
#define _MCU_Work_           "MCUWorking"
#define _MCU_Pressure_Reset_ "ResetWorking"
#define _PASS_               "Pass"
#define _Fail_               "Fail"
#define _OK_                 "OK"
#define _MCU_Processing_     "Processing"

// /* Error define */
#define _Pause_Error_      	"Pause"
#define _BMP_Error_Connect_ "BMP_disconnect"
#define _Over_Time_         "Over_time"
#define _D6_Error_          "Nozzle_jam"
#define _D4_Error_          "major_leakage"
#define _D1_Error_
#define _D5_Error_          "middle_leakage"
#define _D2_Error_          "minor_leakage"
#define _D3_Error_          "micro_leakag"
#define _O2_Error_D1_       "D1"
#define _O2_Error_D2_       "D2"
#define _O2_Error_D3_       "D3"


// /* Debug define */
#define _TEST_FUNC_ 0

#define _TEST_MPU_Message_         0

// /* Software Jump */
//  official version should be All 0
#define _Button_Jump_              1
#define _VT_Jump_                  0
#define _VT_Value_Jump             0
#define _N2_Jump_                  1
#define _Time_OUT_Jump             0
#define _Status_Jump_              0 // by pass MPU send Status
#define _END_Status_Jump_          0 // by pass MPU send END
#define _Clean_Confined_Space_Jump 1


#define _Process_Debug     1
#define _Read_MPU_Message  1
#define _Read_BMP_Message  0
#define _Send_MPU_Message  0
#define _Read_Procress_BMP 0
#define _Process_VT_N2_    1
#define _Procress_D6_      0
#define _Procress_D4_      1
#define _Procress_D5_      1
#define _Procress_D1_      1
#define _Procress_D2_      1
#define _Procress_D3_      1
#define _Debug_End_Mes_    1
#define _Clean_Space_Data_ 1

#define _FAKE_MPU_Message "{\"Command\": \"Start\", \"ESN\": \"1234567890\",\"stationid\":\"VT3\", \
\"O2extendtimesvalues\": \"1\", \"O2judgmentoftimes\": \"5\", \"O2threshol\": \"5\", \"O2times\": \"5\", \
\"VTparam1\": \"5\", \"VTparam2\": \"1\", \"VTparam3\": \"1\", \"VTparam4\": \"15\", \"VTparam5\": \"2\", \"VTparam6\": \"2\", \"VTparam7\": \"1\", \"VTparam8\": \"1120\", \"VTparam9\": \"1070\"}"



#endif