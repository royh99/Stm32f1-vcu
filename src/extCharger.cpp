#include <extCharger.h>

static bool chargeAllow=false;

//this mode accepts a request for HV via a 12v inputfrom a charger controller e.g. Tesla Gen2/3 M3 PCS etc.



bool extCharger::ControlCharge(bool RunCh ,bool ACReq)
{
    bool dummy=RunCh;
    dummy=0;
    dummy=dummy;

    if(ACReq)
        {
            chargeAllow = IOMatrix::GetPin(IOMatrix::HVREQ)->Get();
            //enable charger digital line.
            IOMatrix::GetPin(IOMatrix::OBCENABLE)->Set();
            return chargeAllow;
        }
    else
        {
            //disable charger digital line when requested by timer or webui.
            IOMatrix::GetPin(IOMatrix::OBCENABLE)->Clear();
            return false;
        }
return false;
}


