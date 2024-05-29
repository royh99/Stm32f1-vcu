

#ifndef NOVEHICLE_H_INCLUDED
#define NOVEHICLE_H_INCLUDED

#include <vehicle.h>


class NoVehicle : public Vehicle
{
public:
   void SetRevCounter(int speed) {speed = speed;}
   void SetTemperatureGauge(float temp){temp = temp;}
   bool Ready() { return DigIo::t15_digi.Get();}
   bool Start() { return Param::GetBool(Param::din_start); }
protected:

};

#endif // VEHICLE_H_INCLUDED


