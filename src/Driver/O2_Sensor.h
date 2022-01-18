#ifndef __O2_Sensor__
#define __O2_Sensor__

#include "Model/Define.h"
#if (O2_sensor_Choose == _ME2_O2_)
	#include "ME2_O2.h"
	class O2_Sensor : public ME2_O2
	{
	private:


	public:

		// virtual float O2_readO2Vout();
		// virtual float O2_readConcentration(float O2_Ampere);
		// virtual void O2_Set_VRefer(int VRefer);

	};
#elif (O2_sensor_Choose == _TW_500B_)
	#include "TW_500B.h"
	class O2_Sensor : public TW_500B
	{
	private:


	public:

		// virtual float O2_readO2Vout();
		// virtual float O2_readConcentration(float O2_Ampere);
		// virtual void O2_Set_VRefer(int VRefer);

	};
#endif




#endif

