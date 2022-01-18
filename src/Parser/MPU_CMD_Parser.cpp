#include "MPU_COM_Parser.h"


int MPU_COM_Parser::Parse_MPU_Command(MachineData* _MachineData)
{
	StaticJsonDocument<500> cJSON_String;
	DeserializationError error = deserializeJson(cJSON_String, _MachineData->MPU_String);
	const char *json;
	char CtoI[20];
	if (!error)
	{
		json = cJSON_String["Command"];
		memcpy( _MachineData->Command , (const char *)json , strlen(json) );

		json = cJSON_String["stationid"];
		memcpy( _MachineData->VT_version , (const char *)json , strlen(json) );

		json = cJSON_String["O2extendtimesvalues"];
		memcpy( CtoI , (const char *)json , strlen(json) );
		_MachineData->O2extendtimesvalues = atol(CtoI);
		_MachineData->O2extendtimesvalues = _MachineData->O2extendtimesvalues * pow(10,6);	// second to micro seconds
		memset( CtoI, '\0', sizeof(CtoI));

		json = cJSON_String["O2judgmentoftimes"];
		memcpy( CtoI , (const char *)json , strlen(json) );
		_MachineData->O2judgmentoftimes = atol(CtoI);
		_MachineData->O2judgmentoftimes = _MachineData->O2judgmentoftimes * pow(10,6);	// second to micro seconds
		memset(CtoI, '\0', sizeof(CtoI));

		json = cJSON_String["O2threshol"];
		memcpy( CtoI , (const char *)json , strlen(json) );
		_MachineData->O2threshol = atol(CtoI);
		memset( CtoI, '\0', sizeof(CtoI));

		json = cJSON_String["O2times"];
		memcpy( CtoI , (const char *)json , strlen(json) );
		_MachineData->O2times = atol(CtoI);
		_MachineData->O2times = _MachineData->O2times * pow(10,6);	// second to micro seconds
		memset( CtoI, '\0', sizeof(CtoI));

		json = cJSON_String["O2pressvalues"];
		memcpy( CtoI , (const char *)json , strlen(json) );
		_MachineData->O2pressvalues = atol(CtoI);
		memset( CtoI, '\0', sizeof(CtoI));


		json = cJSON_String["VTparam1"];
		memcpy( CtoI , (const char *)json , strlen(json) );
		_MachineData->VTparam1 = atol(CtoI);
		memset( CtoI, '\0', sizeof(CtoI));

		json = cJSON_String["VTparam2"];
		memcpy( CtoI , (const char *)json , strlen(json) );
		_MachineData->VTparam2 = atol(CtoI);
		_MachineData->VTparam2 = _MachineData->VTparam2 * pow(10,6);	// second to micro seconds
		memset( CtoI, '\0', sizeof(CtoI));

		json = cJSON_String["VTparam3"];
		memcpy( CtoI , (const char *)json , strlen(json) );
		_MachineData->VTparam3 = atol(CtoI);
		_MachineData->VTparam3 = _MachineData->VTparam3 * pow(10,6);	// second to micro seconds
		memset( CtoI, '\0', sizeof(CtoI));

		json = cJSON_String["VTparam4"];
		memcpy( CtoI , (const char *)json , strlen(json) );
		_MachineData->VTparam4 = atol(CtoI);
		_MachineData->mbar_Range = (float)_MachineData->VTparam4/(float)10.0;
		_MachineData->mbar_Range = _MachineData->mbar_Range - (float)1.0;
		memset( CtoI, '\0', sizeof(CtoI));

		json = cJSON_String["VTparam5"];
		memcpy( CtoI , (const char *)json , strlen(json) );
		_MachineData->VTparam5 = atol(CtoI);
		_MachineData->VTparam5 = _MachineData->VTparam5 * pow(10,6);	// second to micro seconds
		memset( CtoI, '\0', sizeof(CtoI));

		json = cJSON_String["VTparam6"];
		memcpy( CtoI , (const char *)json , strlen(json) );
		_MachineData->VTparam6 = atol(CtoI);
		_MachineData->VTparam6 = _MachineData->VTparam6 * pow(10,6);	// second to micro seconds
		memset( CtoI, '\0', sizeof(CtoI));

		json = cJSON_String["VTparam7"];
		memcpy( CtoI , (const char *)json , strlen(json) );
		_MachineData->VTparam7 = atol(CtoI);
		_MachineData->VTparam7 = _MachineData->VTparam7 * pow(10,6);	// second to micro seconds
		memset( CtoI, '\0', sizeof(CtoI));

		json = cJSON_String["VTparam8"];
		memcpy( CtoI , (const char *)json , strlen(json) );
		_MachineData->VTparam8 = atol(CtoI);
		_MachineData->MAX_mbar = (float)_MachineData->VTparam8/10.0;
		memset( CtoI, '\0', sizeof(CtoI));

		json = cJSON_String["VTparam9"];
		memcpy( CtoI , (const char *)json , strlen(json) );
		_MachineData->VTparam9 = atol(CtoI);
		_MachineData->MIN_mbar = (float)_MachineData->VTparam9/10.0;
		memset( CtoI, '\0', sizeof(CtoI));

		DebugPort.print("Command = ");
		_MachineData->O2pressvalues = (uint32_t)1013;

		#if _Read_MPU_Message
		DebugPort.println(_MachineData->Command);
		DebugPort.print("stationid = ");
		DebugPort.println(_MachineData->VT_version);
		DebugPort.print("O2extendtimesvalues = ");
		DebugPort.println(_MachineData->O2extendtimesvalues);
		DebugPort.print("O2judgmentoftimes = ");
		DebugPort.println(_MachineData->O2judgmentoftimes);
		DebugPort.print("O2threshol = ");
		DebugPort.println(_MachineData->O2threshol);
		DebugPort.print("O2times = ");
		DebugPort.println(_MachineData->O2times);
		DebugPort.print("O2pressvalues = ");
		DebugPort.println(_MachineData->O2pressvalues);
		DebugPort.print("VTparam1 = ");
		DebugPort.println(_MachineData->VTparam1);
		DebugPort.print("VTparam2 = ");
		DebugPort.println(_MachineData->VTparam2);
		DebugPort.print("VTparam3 = ");
		DebugPort.println(_MachineData->VTparam3);
		DebugPort.print("VTparam4 = ");
		DebugPort.println(_MachineData->VTparam4);
		DebugPort.print("VTparam5 = ");
		DebugPort.println(_MachineData->VTparam5);
		DebugPort.print("VTparam6 = ");
		DebugPort.println(_MachineData->VTparam6);
		DebugPort.print("VTparam7 = ");
		DebugPort.println(_MachineData->VTparam7);
		DebugPort.print("VTparam8 = ");
		DebugPort.println(_MachineData->VTparam8);
		DebugPort.print("VTparam9 = ");
		DebugPort.println(_MachineData->VTparam9);
		#endif
		_MachineData->MPU_String = "";
		return 1;
	}
	_MachineData->MPU_String = "";
	return 0;
}

