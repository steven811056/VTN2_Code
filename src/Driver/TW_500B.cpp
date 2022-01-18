#include "TW_500B.h"
TW_500B tw_500b;

TW_500B::TW_500B()
{

}

float TW_500B::O2_readO2Vout()
{
	long sum = 0;
	for(int i=0; i<32; i++)
	{
		sum += analogRead(_O2_Sensor_);
	}
	sum >>= 5;
	float MeasuredVout = sum * (O2_VRefer / 1023.0);
	return MeasuredVout;
}

float TW_500B::O2_readConcentration(float O2_Ampere)
{
	float MeasuredVout = O2_readO2Vout();
	// float Concentration = MeasuredVout/ 5;
	// float Concentration_Percentage = Concentration*24;
	float Concentration_Percentage = 0.9*pow((MeasuredVout+0.4),2);
	return Concentration_Percentage;
}

void TW_500B::O2_Set_VRefer(int VRefer)
{
	O2_VRefer = VRefer;
}