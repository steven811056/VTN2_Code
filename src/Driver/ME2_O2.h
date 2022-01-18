#ifndef __ME2_O2__
#define __ME2_O2__

#include "Arduino.h"
#include "Control/MCU_IOHandler.h"
#include "Model/Define.h"

class ME2_O2
{
private:
	double O2_VRefer = _O2_VRefer;

protected:

public:

// --------------------

public:
	ME2_O2();
	virtual float O2_readO2Vout() ;
	virtual float O2_readConcentration(float O2_Ampere) ;
	virtual void O2_Set_VRefer(int VRefer) ;

};
extern ME2_O2 me2_O2;
#endif

