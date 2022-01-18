#include "ME2_O2.h"
ME2_O2 me2_O2;

ME2_O2::ME2_O2()
{

}

float ME2_O2::O2_readO2Vout()
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

float ME2_O2::O2_readConcentration(float O2_Ampere)
{
	float MeasuredVout = O2_readO2Vout();

	//float Concentration = FmultiMap(MeasuredVout, VoutArray,O2ConArray, 6);
	//when its output voltage is 2.0V,
	//float Concentration = MeasuredVout * 0.21 / 2.0;
	// float Concentration = MeasuredVout * 0.21 / _O2_Ampere_;
	float Concentration = MeasuredVout * 0.21 / O2_Ampere;
	float Concentration_Percentage = Concentration*100;
	return Concentration_Percentage;
}

void ME2_O2::O2_Set_VRefer(int VRefer)
{
	O2_VRefer = VRefer;
}