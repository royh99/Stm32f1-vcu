/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2012-2020 Johannes Huebner <dev@johanneshuebner.com>
 *               2021-2022 Damien Maguire <info@evbmw.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "throttle.h"
#include "my_math.h"

#define POT_SLACK 200

int Throttle::potmin[2];
int Throttle::potmax[2];
float Throttle::regenRpm;
float Throttle::regenendRpm;
float Throttle::brknompedal;
float Throttle::regenmax;
float Throttle::regenBrake;
float Throttle::brkcruise;
int Throttle::idleSpeed;
int Throttle::cruiseSpeed;
float Throttle::speedkp;
int Throttle::speedflt;
int Throttle::speedFiltered; // used for cruise control
float Throttle::idleThrotLim;
float Throttle::potnomFiltered;
float Throttle::throtmax;
float Throttle::throtmaxRev;
float Throttle::throtmin;
float Throttle::throtdead;
float Throttle::regenRamp;
float Throttle::throttleRamp;
int Throttle::bmslimhigh;
int Throttle::bmslimlow;
float Throttle::udcmin;
float Throttle::udcmax;
float Throttle::idcmin;
float Throttle::idcmax;
int Throttle::speedLimit;
float Throttle::throtRpmFilt;
// internal variable, reused every time the function is called
static float throttleRamped = 0.0;
static float speedFilt = 0.0; // used in throttle ramping

static float regenlim = 0;

#define PEDALPOSARRLEN 50
static float pedalPos;
static float lastPedalPos;
static float pedalChange = 0;
static float pedalPosTot = 0;
static float pedalPosArr[PEDALPOSARRLEN];
static uint8_t pedalPosIdx = 0;
static int8_t pedalReq = 0; //positive is accel negative is decell

/**
 * @brief Check the throttle input for sanity and limit the range to min/max values
 *
 * @param potval Pointer to the throttle input array, range should be [potMin, potMax].
 * @param potIdx Index of the throttle input array, range is [0, 1].
 * @return true if the throttle input was within bounds (accounting for POT_SLACK).
 * @return false the throttle was too far out of bounds. Setting potval to the minimum.
 */
bool Throttle::CheckAndLimitRange(int* potval, int potIdx)
{
    // The range check accounts for inverted throttle pedals, where the minimum
    // value is higher than the maximum. To accomodate for that, the potMin and potMax
    // variables are set for internal use.
    int potMin = potmax[potIdx] > potmin[potIdx] ? potmin[potIdx] : potmax[potIdx];
    int potMax = potmax[potIdx] > potmin[potIdx] ? potmax[potIdx] : potmin[potIdx];

    if (((*potval + POT_SLACK) < potMin) || (*potval > (potMax + POT_SLACK)))
    {
        *potval = potMin;
        return false;
    }
    else if (*potval < potMin)
    {
        *potval = potMin;
    }
    else if (*potval > potMax)
    {
        *potval = potMax;
    }

    return true;
}

/**
 * @brief Normalize the throttle input value to the min-max scale.
 *
 * Returns 0.0% for illegal indices and if potmin and potmax are equal.
 *
 * @param potval Throttle input value, range is [potmin[potIdx], potmax[potIdx]], not checked!
 * @param potIdx Index of the throttle input, should be [0, 1].
 * @return Normalized throttle value output with range [0.0, 100.0] with correct input.
 */
float Throttle::NormalizeThrottle(int potval, int potIdx)
{
    if(potIdx < 0 || potIdx > 1)
        return 0.0f;

    if(potmin[potIdx] == potmax[potIdx])
        return 0.0f;


    return 100.0f * ((float)(potval - potmin[potIdx]) / (float)(potmax[potIdx] - potmin[potIdx]));
}

/**
 * @brief Calculate a throttle percentage from the potval input.
 *
 * After the previous range checks, the throttle input potval lies within the
 * range of [potmin[0], potmax[0]]. From this range, the input is converted to
 * a percent range of [-100.0, +100.0] +ve for forward/ reverse, -ve for regen
 *
 *
 * @param potval
 * @param idx Index of the throttle input that should be used for calculation.
 * @param brkpedal Brake pedal input (true for brake pedal pressed, false otherwise).
 * @return float
 */
float Throttle::CalcThrottle(int potval, int potIdx, bool brkpedal)
{
    int speed = Param::GetInt(Param::speed);
    int dir = Param::GetInt(Param::dir);
    float potnom = 0.0f;  // normalize potval against the potmin and potmax values

    if(speed< 0)//make sure speed is not negative
    {
        speed *= -1;
    }

    //limiting speed change rate
    if(ABS(speed-speedFilt)>throtRpmFilt)
    {
        if(speed > speedFilt)
        {
            speedFilt +=  throtRpmFilt;
        }
        else
        {
            speedFilt -=  throtRpmFilt;
        }
    }
    else
    {
        speedFilt = speed;
    }

    speed = speedFilt;

    ///////////////////////

    if(dir == 0)//neutral no torque command
    {
        return 0;
    }

    if (brkpedal)
    {
        if(speed < 100)
        {
            return 0;
        }
        else if (speed < regenRpm)
        {
            //potnom = utils::change(speed, regenendRpm, regenRpm, 0, regenBrake);//taper regen according to speed
			potnom = ((speed - regenendRpm) * regenBrake) / (regenRpm - regenendRpm);//taper regen according to speed
            return potnom;
        }
        else
        {
            potnom =  regenBrake;
            return potnom;
        }
    }

    // substract offset, bring potval to the potmin-potmax scale and make a percentage
    potnom = NormalizeThrottle(potval, potIdx);

    // Apply the deadzone parameter. To avoid that we lose the range between
    // 0 and throtdead, the scale of potnom is mapped from the [0.0, 100.0] scale
    // to the [throtdead, 100.0] scale.
    if(potnom < throtdead)
    {
        potnom = 0.0f;
    }
    else
    {
        potnom = ((potnom - throtdead) * 100.0f) / (100.0f - throtdead);
    }

//!! pedal command intent coding

    pedalPos = potnom; //save comparison next time to check if pedal had moved

    float tempAvgPos = AveragePos(pedalPos); //get an rolling average pedal position over the last 50 measurements for smoothing

    pedalChange = pedalPos - tempAvgPos; //current pedal position compared to average

    if(pedalChange < -1.0 )//Check pedal is release compared to last time
    {
        pedalReq = -1; //pedal is released enough - Commanding regen or slowing
    }
    else if(pedalChange > 1.0 )//Check pedal is increased compared to last time
    {
        pedalReq = 1; //pedal pressed - Commanding accelerating - thus always more power
    }
    else//pedal not changed
    {
        potnom = tempAvgPos; //use the averaged pedal
    }

    //Do clever bits for regen and such.
    if(speed < 50)//No regen under 50 rpm
    {
        regenlim = 0;
    }
    else if(speed < regenRpm)
    {
        //regenlim = utils::change(speed, regenendRpm, regenRpm, 0, regenmax);//taper regen according to speed
	    regenlim = (speed - regenendRpm ) * regenmax / (regenRpm - regenendRpm);
    }
    else
    {
        regenlim = regenmax;
    }

    //!!!potnom is throttle position up to this point//

    if(dir == 1)//Forward
    {
	    potnom = regenlim + (potnom * (throtmax - regenlim) / 100); // remap potnom to regenlim..throtmax																 				  
     //   potnom = utils::change(potnom,0,100,regenlim*10,throtmax*10);
     //   potnom *= 0.1;
    }
    else //Reverse, as neutral already exited function
    {
	    potnom = regenlim + (potnom * (throtmaxRev - regenlim) / 100); // remap potnom to regenlim..throtmaxrev
     //   potnom = utils::change(potnom,0,100,regenlim*10,throtmaxRev*10);
     //   potnom *= 0.1;
    }

    lastPedalPos = pedalPos; //Save current pedal position for next loop.
    return potnom;
}

/**
 * @brief Apply the throttle ramping parameters for ramping up and down.
 *
 * @param potnom Normalized throttle command in percent, range [-100.0, 100.0].
 * @return float Ramped throttle command in percent, range [-100.0, 100.0].
 */
float Throttle::RampThrottle(float potnom)
{
    // make sure potnom is within the boundaries of [throtmin, throtmax]
    potnom = MIN(potnom, throtmax);
    potnom = MAX(potnom, throtmin);

    if (potnom >= throttleRamped) // higher throttle command than currently applied
    {
        if(potnom > 0)
        {
            throttleRamped = RAMPUP(throttleRamped, potnom, throttleRamp);
            potnom = throttleRamped;
        }
        else
        {
            throttleRamped = RAMPUP(throttleRamped, potnom, regenRamp);
            potnom = throttleRamped;
        }
    }
    else //(potnom < throttleRamped) // lower throttle command than currently applied
    {
        if(potnom >= 0)
        {
            throttleRamped = potnom; //No ramping from high throttle to low throttle
        }
        else
        {
            if(throttleRamped > 0)
            {
                throttleRamped = 0;
            }
            throttleRamped = RAMPDOWN(throttleRamped, potnom, regenRamp);
            potnom = throttleRamped;
        }
    }

    return potnom;
}

float Throttle::CalcIdleSpeed(int speed)
{
    int speederr = idleSpeed - speed;
    return MIN(idleThrotLim, speedkp * speederr);
}

float Throttle::CalcCruiseSpeed(int speed)
{
    speedFiltered = IIRFILTER(speedFiltered, speed, speedflt);
    int speederr = cruiseSpeed - speedFiltered;

    float potnom = speedkp * speederr;
    potnom = MIN(100, potnom);
    potnom = MAX(brkcruise, potnom);

    return potnom;
}

bool Throttle::TemperatureDerate(float temp, float tempMax, float& finalSpnt)
{
    float limit = 0;

    if (temp <= tempMax)
        limit = 100.0f;
    else if (temp < (tempMax + 2.0f))
        limit = 50.0f;

    if (finalSpnt >= 0)
        finalSpnt = MIN(finalSpnt, limit);
    else
        finalSpnt = MAX(finalSpnt, -limit);

    return limit < 100.0f;
}

void Throttle::UdcLimitCommand(float& finalSpnt, float udc)
{
    if(udcmin>0)    //ignore if set to zero. Useful for bench testing without isa shunt
    {
        if (finalSpnt >= 0)
        {
            float udcErr = udc - udcmin;
            float res = udcErr * 5;
            res = MAX(0, res);
            finalSpnt = MIN(finalSpnt, res);
        }
        else
        {
            float udcErr = udc - udcmax; // udcmax = udclim
            float res = udcErr * 3.5;
            res = MIN(0, res);
            finalSpnt = MAX(finalSpnt, res);
        }
    }
    else
    {
        finalSpnt = finalSpnt;
    }
}

void Throttle::IdcLimitCommand(float& finalSpnt, float idc)
{
    static float idcFiltered = 0;

    idcFiltered = IIRFILTERF(idcFiltered, idc, 4);

    if (finalSpnt >= 0)
    {
        float idcerr = idcmax - idcFiltered;
        float res = idcerr * 10;

        res = MAX(0, res);
        finalSpnt = MIN(res, finalSpnt);
    }
    else
    {
        float idcerr = idcmin - idcFiltered;
        float res = idcerr * 10;

        res = MIN(0, res);
        finalSpnt = MAX(res, finalSpnt);
    }
}

void Throttle::SpeedLimitCommand(float& finalSpnt, int speed)
{
    static int speedFiltered = 0;

    speedFiltered = IIRFILTER(speedFiltered, speed, 4);

    if (finalSpnt > 0)
    {
        int speederr = speedLimit - speedFiltered;
        int res = speederr / 4;

        res = MAX(0, res);
        finalSpnt = MIN(res, finalSpnt);
    }
}

float Throttle::AveragePos(float pos)
{
    pedalPosIdx++; //next average arrray positon
    if(pedalPosIdx >= PEDALPOSARRLEN )
    {
        pedalPosIdx = 0;
    }
    pedalPosTot -= pedalPosArr[pedalPosIdx];
    pedalPosTot += pos;
    pedalPosArr[pedalPosIdx] = pos;

    return pedalPosTot/PEDALPOSARRLEN ;
}

bool Throttle::PotTest()
{
return (Param::GetInt(Param::pot) > Param::GetInt(Param::potmin)); // 1 (true) = pot pressed	
// this needs to check against potnom as that takes into account inverted throttle pedal and throtdead
// but potnum only valid in run mode. Need to calc value here for test.
}