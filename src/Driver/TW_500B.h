#ifndef __TW_500B__
#define __TW_500B__

// #include "O2_Sensor.h"
#include "Arduino.h"
#include "Control/MCU_IOHandler.h"
#include "Model/Define.h"

class TW_500B
{
private:
	double O2_VRefer = _O2_VRefer;

protected:

public:

// --------------------

public:
	TW_500B();
	virtual float O2_readO2Vout() ;
	virtual float O2_readConcentration(float O2_Ampere) ;
	virtual void O2_Set_VRefer(int VRefer) ;

};
extern TW_500B tw_500b;
#endif

