/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2010 Johannes Huebner <contact@johanneshuebner.com>
 * Copyright (C) 2010 Edward Cheeseman <cheesemanedward@gmail.com>
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2019-2022 Damien Maguire <info@evbmw.com>
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
#include "stm32_vcu.h"

extern "C" void __cxa_pure_virtual()
{
    while (1);
}

static Stm32Scheduler* scheduler;
static bool chargeMode = false;
static bool chargeModeDC = false;
static bool chgLck = false;
static CanHardware* canInterface[2];
static CanMap* canMap;
static ChargeModes targetCharger;
static ChargeInterfaces targetChgint;
static uint8_t chgSet;
static bool runChg;
static uint8_t chgHrs_tmp;
static uint8_t chgMins_tmp;
static uint16_t chgDur_tmp;
static uint32_t chgTicks=0,chgTicks_1Min=0;
static bool startSig=false;
static bool ACrequest=false;
static bool initbyStart=false;
static bool initbyCharge=false;


static volatile unsigned
days=0,
hours=0, minutes=0, seconds=0,
alarm=0; // != 0 when alarm is pending

static uint8_t rlyDly=25;

// Instantiate Classes
static Can_LR lrVehicle;
static NoVehicle novehicle;
static notused UnUsed;
static noCharger nochg;
static outlanderCharger outChg;
static FCChademo chademoFC;
static OutlanderInverter outlanderInv;
static noHeater Heaternone;
static NoVehicle VehicleNone;
static Inverter* selectedInverter = &outlanderInv;
static Vehicle* selectedVehicle = &lrVehicle;
static Heater* selectedHeater = &Heaternone;
static Chargerhw* selectedCharger = &outChg;
static Chargerint* selectedChargeInt = &UnUsed;
static BMS BMSnone;
static SimpBMS BMSsimp;
static DCDC DCDCnone;
static outlanderDCDC DCDCoutlander;
static BMS* selectedBMS = &BMSnone;
static DCDC* selectedDCDC = &DCDCnone;
static Can_OBD2 canOBD2;
static LinBus* lin;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void Ms200Task(void)
{
    int opmode = Param::GetInt(Param::opmode);
    DigIo::led_rd.Toggle();
    selectedVehicle->Task200Ms();
    if(opmode==MOD_CHARGE) selectedCharger->Task200Ms();

    Param::SetInt(Param::Day,days);
    Param::SetInt(Param::Hour,hours);
    Param::SetInt(Param::Min,minutes);
    Param::SetInt(Param::Sec,seconds);
    Param::SetInt(Param::ChgTime,chgDur_tmp);
    if(chgSet==2 && !chgLck)  //if in timer mode and not locked out from a previous full charge.
    {
        if(opmode!=MOD_CHARGE)
        {
            if((chgHrs_tmp==hours)&&(chgMins_tmp==minutes)&&(chgDur_tmp!=0))runChg=true;//if we arrive at set charge time and duration is non zero then initiate charge
            else runChg=false;
        }

        if(opmode==MOD_CHARGE)
        {
            if(chgTicks!=0)
            {
                chgTicks--; //decrement charge timer ticks
                chgTicks_1Min++;
            }

            if(chgTicks==0)
            {
                runChg=false; //end charge if still charging once timer expires.
                chgTicks = (GetInt(Param::Chg_Dur)*300);//recharge the tick timer
            }

            if (chgTicks_1Min==300)
            {
                chgTicks_1Min=0;
                chgDur_tmp--; //countdown minutes of charge time remaining.
            }
        }

    }
    if(chgSet==0 && !chgLck) runChg=true;//enable from webui if we are not locked out from an auto termination
    if(chgSet==1) runChg=false;//disable from webui

    //Handle PP on the Charging port
    if(Param::GetInt(Param::GPA1Func) == IOMatrix::PILOT_PROX)
    {
        int ppThresh = Param::GetInt(Param::ppthresh);

        int ppValue = IOMatrix::GetAnaloguePin(IOMatrix::PILOT_PROX)->Get();
        Param::SetInt(Param::PPVal, ppValue);


        //if PP is less than threshold and currently disabled and not already finished
        if (ppValue < ppThresh && chgSet==1 && !chgLck)
        {
            runChg=true;
        }
        else if (ppValue > ppThresh)
        {
            //even if timer was enabled, change to disabled, we've unplugged
            runChg=false;
        }
    }

    if(selectedCharger->ControlCharge(runChg, ACrequest) && (opmode != MOD_RUN))
    {
        chargeMode = true;   //AC charge mode
        Param::SetInt(Param::chgtyp,AC);
    }
    else if(!chargeModeDC)
    {
        Param::SetInt(Param::chgtyp,OFF);
        chargeMode = false;  //no charge mode
    }

    //in chademo , we do not want to run the 200ms task unless in dc charge mode
    if(targetChgint == ChargeInterfaces::Chademo && chargeModeDC) selectedChargeInt->Task200Ms();
    //and just to be thorough ...
    if(targetChgint == ChargeInterfaces::Unused) selectedChargeInt->Task200Ms();



    ///////////////////////////////////////
    //Charge term logic for AC charge
    ///////////////////////////////////////
    /*
    if we are in charge mode and battV >= setpoint and power is <= termination setpoint
        Then we end charge.
    */
    if(opmode==MOD_CHARGE && !chargeModeDC)
    {
        if(Param::GetFloat(Param::udc)>=Param::GetFloat(Param::Voltspnt) && Param::GetFloat(Param::idc)<=Param::GetFloat(Param::IdcTerm))
        {
            runChg=false;//end charge
            chgLck=true;//set charge lockout flag
        }
        
        if(selectedBMS->MaxChargeCurrent()==0)//BMS can command an AC charge shutdown if its current limit is 0
        {
            runChg=false;//end charge
            chgLck=true;//set charge lockout flag
        }
        
        
    }
    if(opmode==MOD_RUN)
    {
        chgLck=false;//reset charge lockout flag when we drive off
        
        //Brake Vac Sensor
        if(Param::GetInt(Param::GPA1Func) == IOMatrix::VAC_SENSOR)
       {
            int brkVacThresh = Param::GetInt(Param::BrkVacThresh);
            int BrkVacHyst = Param::GetInt(Param::BrkVacHyst);

            int brkVacVal = IOMatrix::GetAnaloguePin(IOMatrix::VAC_SENSOR)->Get();
            Param::SetInt(Param::BrkVacVal, brkVacVal);

            // if brkVacThresh > BrkVacHyst then sensor reads higher with more vacuum else other way round
            if (brkVacThresh > BrkVacHyst)
            {
                //enable pump
                if (brkVacVal > brkVacThresh)
                {
                    IOMatrix::GetPin(IOMatrix::BRAKEVACPUMP)->Clear();
                }
                else if (brkVacVal < BrkVacHyst)
                {
                    IOMatrix::GetPin(IOMatrix::BRAKEVACPUMP)->Set();
                }
            }
            else
            {
                //enable pump
                if (brkVacVal < brkVacThresh)
                {
                    IOMatrix::GetPin(IOMatrix::BRAKEVACPUMP)->Clear();
                }
                else if (brkVacVal > BrkVacHyst)
                {
                    IOMatrix::GetPin(IOMatrix::BRAKEVACPUMP)->Set();
                }
            }
        }
    }
    else
    {
        IOMatrix::GetPin(IOMatrix::BRAKEVACPUMP)->Clear();  
    }
    
    //dac_load_data_buffer_single(Param::GetInt(Param::DAC1), RIGHT12, CHANNEL_1);
    //dac_software_trigger(CHANNEL_1);
    //dac_load_data_buffer_single(Param::GetInt(Param::DAC2), RIGHT12, CHANNEL_2);
    //dac_software_trigger(CHANNEL_2);
    
}

uint8_t i = 0;

static void Ms100Task(void)
{
    /*    
        if (i < 115) 
        {
        dac_load_data_buffer_single(400+(i*32), RIGHT12, CHANNEL_1);
        dac_software_trigger(CHANNEL_1);  
        dac_load_data_buffer_single(200+(i*16), RIGHT12, CHANNEL_2);
        dac_software_trigger(CHANNEL_2);
        i++;
        }
        if (i >= 115) 
        {
        dac_load_data_buffer_single(400+((228-i)*32), RIGHT12, CHANNEL_1);
        dac_software_trigger(CHANNEL_1);  
        dac_load_data_buffer_single(200+((228-i)*16), RIGHT12, CHANNEL_2);
        dac_software_trigger(CHANNEL_2);
        i++;
        }
        if (i==229) i=0;
    */    
    DigIo::led_out.Toggle();
    iwdg_reset();
    float cpuLoad = scheduler->GetCpuLoad() / 10.0f;
    Param::SetFloat(Param::cpuload, cpuLoad);
    Param::SetInt(Param::lasterr, ErrorMessage::GetLastError());
    int opmode = Param::GetInt(Param::opmode);
    utils::SelectDirection(selectedVehicle);
    utils::CalcSOC();
       
    selectedInverter->Task100Ms();
    selectedVehicle->Task100Ms();
    selectedCharger->Task100Ms();
    selectedBMS->Task100Ms();
    selectedDCDC->Task100Ms();
    //selectedShifter->Task100Ms();
    canMap->SendAll();

    if (Param::GetInt(Param::dir) < 0)
    {
        IOMatrix::GetPin(IOMatrix::REVERSELIGHT)->Set();
    }
    else
    {
        IOMatrix::GetPin(IOMatrix::REVERSELIGHT)->Clear();
    }

    if(opmode==MOD_RUN)
    {
        IOMatrix::GetPin(IOMatrix::RUNINDICATION)->Set();
    }
    else
    {
        IOMatrix::GetPin(IOMatrix::RUNINDICATION)->Clear();
    }
       
    Param::SetFloat(Param::tmphs, selectedInverter->GetInverterTemperature()); //send inverter temp to web interface
    Param::SetFloat(Param::tempmotor, selectedInverter->GetMotorTemperature()); //send motor temp to web interface
    Param::SetFloat(Param::InvStat, selectedInverter->GetInverterState()); //update inverter status on web interface
    Param::SetFloat(Param::INVudc, selectedInverter->GetInverterVoltage()); //display inverter derived dc link voltage on web interface
       
    Param::SetInt(Param::T15Stat, selectedVehicle->Ready());
       
    int32_t IsaTemp=ISA::Temperature;
    Param::SetInt(Param::tmpaux,IsaTemp);
       
    if(targetChgint == chargeModeDC) selectedChargeInt->Task100Ms();// send the 100ms task request for the lim all the time and for others if in DC charge mode
       
    if(selectedChargeInt->DCFCRequest(runChg))//Request to run dc fast charge
    {
        //Here we receive a valid DCFC startup request.
        if(opmode != MOD_RUN) chargeMode = true;// set charge mode to true to bring up hv
        chargeModeDC = true;   //DC charge mode on
    }
    else if(chargeModeDC)
    {
        Param::SetInt(Param::chgtyp,OFF);
        chargeMode = false;  //no charge mode
        chargeModeDC = false;   //DC charge mode off
    }

    if(!chargeModeDC)//Request to run ac charge from the interface (e.g. LIM) if we are NOT in DC charge mode.
    {
        ACrequest=selectedChargeInt->ACRequest(runChg);
			  
    }
    Param::SetInt(Param::HeatReq,IOMatrix::GetPin(IOMatrix::HEATREQ)->Get());
}

static void ControlCabHeater(int opmode)
{
    //Only run heater in run mode
    //What about charge mode and timer mode?
    if (opmode == MOD_RUN && Param::GetInt(Param::Control) == 1)
    {
        IOMatrix::GetPin(IOMatrix::HEATERENABLE)->Set();//Heater enable and coolant pump on
        selectedHeater->SetTargetTemperature(50); //TODO: Currently does nothing
        selectedHeater->SetPower(Param::GetInt(Param::HeatPwr),Param::GetBool(Param::HeatReq));
    }
    else
    {
        IOMatrix::GetPin(IOMatrix::HEATERENABLE)->Clear(); //Disable heater and coolant pump
        selectedHeater->SetPower(0,0);
    }
}

static void Ms10Task(void)
{
    static uint32_t vehicleStartTime = 0;

    int16_t previousSpeed=Param::GetInt(Param::speed);
    int16_t speed = 0;
    float torquePercent;
    int opmode = Param::GetInt(Param::opmode);
    int stt = STAT_NONE;
    int requestedDirection = Param::GetInt(Param::dir);
    int rollingDirection = 0;

    dac_load_data_buffer_single(Param::GetInt(Param::DAC1), RIGHT12, CHANNEL_1);
    dac_software_trigger(CHANNEL_1);
    dac_load_data_buffer_single(Param::GetInt(Param::DAC2), RIGHT12, CHANNEL_2);
    dac_software_trigger(CHANNEL_2);
    
    ErrorMessage::SetTime(rtc_get_counter_val());
    
    selectedChargeInt->Task10Ms();
    
    if (Param::GetInt(Param::opmode) == MOD_RUN)
    {
        torquePercent = utils::ProcessThrottle(ABS(previousSpeed)); //run the throttle reading and checks and then generate Potnom/torquePercent

        //When requesting regen we need to be careful. If the car is not rolling
        //in the same direction as the selected gear, we will actually accelerate!
        //Exclude openinverter here because that has its own regen logic
      
	  //if (torquePercent < 0 && Param::GetInt(Param::Inverter) != InvModes::OpenI)
        if (torquePercent < 0)	    
        {
/*          if(Param::GetInt(Param::reversemotor) == 0)
            {
                rollingDirection = previousSpeed >= 0 ? 1 : -1;
            }
            else
            {
                rollingDirection = previousSpeed >= 0 ? -1 : 1;
            } */
			
            rollingDirection = Param::GetInt(Param::reversemotor) == 0 ? (previousSpeed >= 0 ? 1 : -1): (previousSpeed >= 0 ? -1 : 1);

            //When rolling backward while in forward gear, apply POSITIVE ( or negative if reversemotor = true) torque to slow down backward motion
            //Vice versa when in reverse gear and rolling forward.
            if (rollingDirection != requestedDirection)
            {
                torquePercent = -torquePercent;
            }
        }

        torquePercent *= requestedDirection; //torque requests invert when reverse direction is selected
        selectedInverter->Task10Ms();
    }
    else
    {
        torquePercent = 0;
        utils::displayThrottle();//just displays pot and pot2 when not in run mode to allow throttle cal
    }


    selectedInverter->SetTorque(torquePercent);

    //Brake light based on regen being below the set threshold
    if(torquePercent < Param::GetFloat(Param::RegenBrakeLight))
    {
        //enable Brake Light Ouput
        IOMatrix::GetPin(IOMatrix::BRAKELIGHT)->Set();
    }
    else
    {
        IOMatrix::GetPin(IOMatrix::BRAKELIGHT)->Clear();
    }

    //speed = ABS(selectedInverter->GetMotorSpeed());//set motor rpm on interface NO ABS allowed on speed as we need to know direction
    speed = selectedInverter->GetMotorSpeed();//set motor rpm on interface

    Param::SetInt(Param::speed, speed);
    utils::GetDigInputs(canInterface[Param::GetInt(Param::InverterCan)]);

    selectedVehicle->SetRevCounter(ABS(speed)); //ABS allowed here to keep number from rolling over.
    selectedVehicle->SetTemperatureGauge(Param::GetFloat(Param::tmphs));
    selectedVehicle->Task10Ms();
    //selectedDCDC->Task10Ms();
    //selectedShifter->Task10Ms();
    if(opmode==MOD_CHARGE) selectedCharger->Task10Ms();
    if(opmode==MOD_RUN) Param::SetInt(Param::canctr, (Param::GetInt(Param::canctr) + 1) & 0xF);//Update the OI can counter in RUN mode only

    //////////////////////////////////////////////////
    //            MODE CONTROL SECTION              //
    //////////////////////////////////////////////////
    float udc = utils::ProcessUdc(speed);
    //stt |= Param::GetInt(Param::pot) <= Param::GetInt(Param::potmin) ? STAT_NONE : STAT_POTPRESSED;
    stt |= Throttle::PotTest() == 0 ? STAT_NONE : STAT_POTPRESSED;
    stt |= udc >= Param::GetFloat(Param::udcsw) ? STAT_NONE : STAT_UDCBELOWUDCSW;
    stt |= udc < Param::GetFloat(Param::udclim) ? STAT_NONE : STAT_UDCLIM;
    Param::SetInt(Param::status, stt);
    
    switch (opmode)
    {
        case MOD_OFF:
        initbyStart=false;
        initbyCharge=false;
        DigIo::inv_out.Clear();//inverter power off
        DigIo::dcsw_out.Clear();
        IOMatrix::GetPin(IOMatrix::NEGCONTACTOR)->Clear();//Negative contactors off if used
        IOMatrix::GetPin(IOMatrix::COOLANTPUMP)->Clear();//Coolant pump off if used
        DigIo::prec_out.Clear();
        Param::SetInt(Param::dir, 0); // shift to park/neutral on shutdown regardless of shifter pos
        selectedVehicle->DashOff();
        startSig=false;//reset for next time
        if( !Throttle::PotTest() ) //check if throttle not pressed
        //if(Param::GetInt(Param::pot) < Param::GetInt(Param::potmin))
        {
            if ((selectedVehicle->Start() && selectedVehicle->Ready()))
            {
                startSig=true;
                opmode = MOD_PRECHARGE;//proceed to precharge if 1)throttle not pressed , 2)ign on , 3)start signal rx
                vehicleStartTime = rtc_get_counter_val();
                initbyStart=true;
            }
        }
        if(chargeMode)
        {
            opmode = MOD_PRECHARGE;//proceed to precharge if charge requested.
            vehicleStartTime = rtc_get_counter_val();
            initbyCharge=true;
        }
        Param::SetInt(Param::opmode, opmode);
        rlyDly=25;//Recharge sequence timer
        break;

        case MOD_PRECHARGE:
        if (!chargeMode)
        {
            DigIo::inv_out.Set();//inverter power on but not if we are in charge mode!
        }
        IOMatrix::GetPin(IOMatrix::NEGCONTACTOR)->Set();
        IOMatrix::GetPin(IOMatrix::COOLANTPUMP)->Set();
        if(rlyDly!=0) rlyDly--;//here we are going to pause before energising precharge to prevent too many contactors pulling amps at the same time
        if(rlyDly==0) DigIo::prec_out.Set();//commence precharge
        if ((stt & (STAT_POTPRESSED | STAT_UDCBELOWUDCSW | STAT_UDCLIM)) == STAT_NONE)
        {
            if(startSig)
            {
                opmode = MOD_RUN;
                startSig=false;//reset for next time
                rlyDly=25;//Recharge sequence timer
            }
            else if(chargeMode)
            {
                opmode = MOD_CHARGE;
                rlyDly=25;//Recharge sequence timer
            }
        }
        if(initbyCharge && !chargeMode) opmode = MOD_OFF;// These two statements catch a precharge hang from either start mode or run mode.
        if(initbyStart && !selectedVehicle->Ready()) opmode = MOD_OFF;
        if (udc < (Param::GetFloat(Param::udcsw)) && rtc_get_counter_val() > (vehicleStartTime + PRECHARGE_TIMEOUT))
        {
            DigIo::prec_out.Clear();
            ErrorMessage::Post(ERR_PRECHARGE);
            opmode = MOD_PRECHGFAIL;
        }
        Param::SetInt(Param::opmode, opmode);
        break;
        
        case MOD_PRECHGFAIL:
        startSig=false;
        DigIo::prec_out.Clear();
        if(initbyCharge && !chargeMode) opmode = MOD_OFF;//only go to off if the signal from charge or vehicle start is removed
        if(initbyStart && !selectedVehicle->Ready()) opmode = MOD_OFF;//this avoids oscillation in the event of a precharge system failure
        Param::SetInt(Param::opmode, opmode);
        break;
        
        case MOD_CHARGE:
        if(rlyDly!=0) rlyDly--;//pause before energising precharge to prevent too many contactors pulling amps at the same time
        if(rlyDly==0) DigIo::dcsw_out.Set();
        ErrorMessage::UnpostAll();
        if(!chargeMode) opmode = MOD_OFF;
        Param::SetInt(Param::opmode, opmode);
        break;
        
        case MOD_RUN:
        DigIo::led_gr.Set();
		DigIo::led_yl.Clear();
        if(rlyDly!=0) rlyDly--;//pause before energising precharge to prevent too many contactors pulling amps at the same time
        if(rlyDly==0) DigIo::dcsw_out.Set();
        Param::SetInt(Param::opmode, MOD_RUN);
        ErrorMessage::UnpostAll();
        if(!selectedVehicle->Ready())
        {
            opmode = MOD_OFF;
            DigIo::led_gr.Clear();
        }
        Param::SetInt(Param::opmode, opmode);
        break;
        
    }
    
    ControlCabHeater(opmode);
    if (Param::GetInt(Param::Type) == 1)  SBOX::ControlContactors(opmode,canInterface[Param::GetInt(Param::ShuntCan)]);//BMW contactor box
    //if (Param::GetInt(Param::Type) == 2)  VWBOX::ControlContactors(opmode,canInterface[Param::GetInt(Param::ShuntCan)]);//VW contactor box
    
}

static void Ms1Task(void)
{
    selectedInverter->Task1Ms();
    selectedVehicle->Task1Ms();
    selectedCharger->Task1Ms();
    selectedChargeInt->Task1Ms();
    //selectedDCDC->Task1Ms();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void UpdateInv()
{
    selectedInverter->DeInit();
    switch (Param::GetInt(Param::Inverter))
    {
        case InvModes::Outlander:
        selectedInverter = &outlanderInv;
        break;
    }
    //This will call SetCanFilters() via the Clear Callback
    canInterface[0]->ClearUserMessages();
    canInterface[1]->ClearUserMessages();
}

static void UpdateVehicle()
{
    switch (Param::GetInt(Param::Vehicle))
    {
        case None:
        selectedVehicle = &novehicle;
        break;
              case LR:
        selectedVehicle = &lrVehicle;
        break;
    }
    //This will call SetCanFilters() via the Clear Callback
    canInterface[0]->ClearUserMessages();
    canInterface[1]->ClearUserMessages();
    
}

static void UpdateCharger()
{
    selectedCharger->DeInit();
    switch (Param::GetInt(Param::chargemodes))
    {
        case ChargeModes::Off:
        chargeMode = false;
        selectedCharger = &nochg;
        break;
        case ChargeModes::Out_lander:
        selectedCharger = &outChg;
        break;
        
    }
    //This will call SetCanFilters() via the Clear Callback
    canInterface[0]->ClearUserMessages();
    canInterface[1]->ClearUserMessages();
}

static void UpdateChargeInt()
{
    selectedChargeInt->DeInit();
    switch (Param::GetInt(Param::interface))
    {
        case ChargeInterfaces::Unused:
        selectedChargeInt = &UnUsed;
        break;
        case ChargeInterfaces::Chademo:
        selectedChargeInt = &chademoFC;
        break;
    }
    //This will call SetCanFilters() via the Clear Callback
    canInterface[0]->ClearUserMessages();
    canInterface[1]->ClearUserMessages();
}

static void UpdateHeater()
{
    selectedHeater->DeInit();
    switch (Param::GetInt(Param::Heater))
    {
        case HeatType::Noheater:
        selectedHeater = &Heaternone;
        break;
    }
    //This will call SetCanFilters() via the Clear Callback
    canInterface[0]->ClearUserMessages();
    canInterface[1]->ClearUserMessages();
}

static void UpdateBMS()
{
    selectedBMS->DeInit();
    switch (Param::GetInt(Param::BMS_Mode))
    {
        case BMSModes::BMSModeSimpBMS:
        selectedBMS = &BMSsimp;
        break;
        default:
        // Default to no BMS
        selectedBMS = &BMSnone;
        break;
    }
    //This will call SetCanFilters() via the Clear Callback
    canInterface[0]->ClearUserMessages();
    canInterface[1]->ClearUserMessages();
}

static void UpdateDCDC()
{
    selectedBMS->DeInit();
    switch (Param::GetInt(Param::DCdc_Type))
    {
        case DCDCModes::NoDCDC:
        selectedDCDC = &DCDCnone;
        break;
        case DCDCModes::OutlanderDCDC:
        selectedDCDC = &DCDCoutlander;
        break;
        
        default:
        // Default to no DCDC
        selectedDCDC = &DCDCnone;
        break;
    }
    //This will call SetCanFilters() via the Clear Callback
    canInterface[0]->ClearUserMessages();
    canInterface[1]->ClearUserMessages();
}


//Whenever the user clears mapped can messages or changes the
//CAN interface of a device, this will be called by the CanHardware module
static void SetCanFilters()
{
    CanHardware* inverter_can = canInterface[Param::GetInt(Param::InverterCan)];
    CanHardware* vehicle_can = canInterface[Param::GetInt(Param::VehicleCan)];
    CanHardware* shunt_can = canInterface[Param::GetInt(Param::ShuntCan)];
    CanHardware* charger_can = canInterface[Param::GetInt(Param::ChargerCan)];
    CanHardware* bms_can = canInterface[Param::GetInt(Param::BMSCan)];
    CanHardware* obd2_can = canInterface[Param::GetInt(Param::OBD2Can)];
    CanHardware* dcdc_can = canInterface[Param::GetInt(Param::DCDCCan)];
    
    selectedInverter->SetCanInterface(inverter_can);
    selectedVehicle->SetCanInterface(vehicle_can);
    selectedCharger->SetCanInterface(charger_can);
    selectedBMS->SetCanInterface(bms_can);
    selectedDCDC->SetCanInterface(dcdc_can);
    canOBD2.SetCanInterface(obd2_can);
    
    if (Param::GetInt(Param::Type) == 0)  ISA::RegisterCanMessages(shunt_can);//select isa shunt
    if (Param::GetInt(Param::Type) == 1)  SBOX::RegisterCanMessages(shunt_can);//select bmw sbox
    
    canInterface[1]->RegisterUserMessage(0x601); //CanSDO
    canInterface[0]->RegisterUserMessage(0x601); //CanSDO
}

void Param::Change(Param::PARAM_NUM paramNum)
{
    // This function is called when the user changes a parameter
    switch (paramNum)
    {
        case Param::Inverter:
        UpdateInv();
        break;
        case Param::Vehicle:
        UpdateVehicle();
        break;
        case Param::chargemodes:
        UpdateCharger();
        break;
        case Param::interface:
        UpdateChargeInt();
        break;
        case Param::Heater:
        UpdateHeater();
        break;
        case Param::BMS_Mode:
        UpdateBMS();
        break;
        case Param::DCdc_Type:
        UpdateDCDC();
        break;
        case Param::InverterCan:
        case Param::VehicleCan:
        case Param::ShuntCan:
        case Param::ChargerCan:
        canInterface[0]->ClearUserMessages();
        canInterface[1]->ClearUserMessages();
        break;
        default:
              break;
    }
           
    if(Param::GetInt(Param::reversemotor) != 0 && Param::GetInt(Param::Inverter) !=InvModes::Outlander)
    {
            Param::SetInt(Param::reversemotor, 0);  // only allow reversemotor for outlander
    }
       
    Throttle::potmin[0] = Param::GetInt(Param::potmin);
    Throttle::potmax[0] = Param::GetInt(Param::potmax);
    Throttle::potmin[1] = Param::GetInt(Param::pot2min);
    Throttle::potmax[1] = Param::GetInt(Param::pot2max);
    Throttle::regenRpm = Param::GetFloat(Param::regenrpm);
    Throttle::regenendRpm = Param::GetFloat(Param::regenendrpm);
    Throttle::throtRpmFilt = Param::GetFloat(Param::throtrpmfilt);
    if (Throttle::regenRpm < Throttle::regenendRpm)
    {
        Throttle::regenRpm = 1500;
        Throttle::regenendRpm = 100;
        Param::SetFloat(Param::regenrpm, 1500);
        Param::SetFloat(Param::regenendrpm, 100);
    }
    Throttle::regenmax = Param::GetFloat(Param::regenmax);
    Throttle::throtmax = Param::GetFloat(Param::throtmax);
    Throttle::throtmin = Param::GetFloat(Param::throtmin);
    Throttle::throtdead = Param::GetFloat(Param::throtdead);
    Throttle::idcmin = Param::GetFloat(Param::idcmin);
    Throttle::idcmax = Param::GetFloat(Param::idcmax);
    Throttle::udcmin = Param::GetFloat(Param::udcmin);
    Throttle::udcmax = Param::GetFloat(Param::udclim);
    Throttle::speedLimit = Param::GetInt(Param::revlim);
    Throttle::regenRamp = Param::GetFloat(Param::regenramp);
    Throttle::throttleRamp = Param::GetFloat(Param::throtramp);
    Throttle::throtmaxRev = Param::GetFloat(throtmaxRev);
    Throttle::regenBrake = Param::GetFloat(Param::regenBrake);
    targetCharger=static_cast<ChargeModes>(Param::GetInt(Param::chargemodes));//get charger setting from menu
    targetChgint=static_cast<ChargeInterfaces>(Param::GetInt(Param::interface));//get interface setting from menu
    if(chgSet==1)
    {
        seconds=Param::GetInt(Param::Set_Sec);//only update these params if charge command is set to disable
        minutes=Param::GetInt(Param::Set_Min);
        hours=Param::GetInt(Param::Set_Hour);
        days=Param::GetInt(Param::Set_Day);
        chgHrs_tmp=GetInt(Param::Chg_Hrs);
        chgMins_tmp=GetInt(Param::Chg_Min);
        chgDur_tmp=GetInt(Param::Chg_Dur);
    }
    chgSet = Param::GetInt(Param::Chgctrl);//0=enable,1=disable,2=timer.
    chgTicks = (GetInt(Param::Chg_Dur)*300);//number of 200ms ticks that equates to charge timer in minutes
    IOMatrix::AssignFromParams();
    IOMatrix::AssignFromParamsAnalogue();
}


static bool CanCallback(uint32_t id, uint32_t data[2], uint8_t dlc) //This is where we go when a defined CAN message is received.
{
    dlc = dlc;
    switch (id)
    {
    case 0x7DF:
        canOBD2.DecodeCAN(id,data);
        break;

    default:
        if (Param::GetInt(Param::Type) == 0)  ISA::DecodeCAN(id, data);
        if (Param::GetInt(Param::Type) == 1)  SBOX::DecodeCAN(id, data);
        //if (Param::GetInt(Param::Type) == 2)  VWBOX::DecodeCAN(id, data);
        selectedInverter->DecodeCAN(id, data);
        selectedVehicle->DecodeCAN(id, data);
        selectedCharger->DecodeCAN(id, data);
        selectedChargeInt->DecodeCAN(id, data);
        selectedBMS->DecodeCAN(id, (uint8_t*)data);
        selectedDCDC->DecodeCAN(id, (uint8_t*)data);
        //selectedShifter->DecodeCAN(id,data);
        break;
    }
    return false;
}


static void ConfigureVariantIO()
{
    ANA_IN_CONFIGURE(ANA_IN_LIST);
    DIG_IO_CONFIGURE(DIG_IO_LIST);

    AnaIn::Start();
    dac_setup();
}


extern "C" void tim4_isr(void)
{
    scheduler->Run();
}

extern "C" void rtc_isr(void)
{
    /* The interrupt flag isn't cleared by hardware, we have to do it. */
    rtc_clear_flag(RTC_SEC);
    
    if ( ++seconds >= 60 )
    {
        ++minutes;
        seconds -= 60;
    }
    if ( minutes >= 60 )
    {
        ++hours;
        minutes -= 60;
    }
    if ( hours >= 24 )
    {
        ++days;
        hours -= 24;
    }
}

extern "C" int main(void)
{    
    extern const TERM_CMD TermCmds[];
    
    clock_setup();
    rtc_setup();
    ConfigureVariantIO();
    //gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, AFIO_MAPR_CAN2_REMAP | AFIO_MAPR_TIM1_REMAP_FULL_REMAP);//32f107
    gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, AFIO_MAPR_TIM3_REMAP_FULL_REMAP);//disable JTAG, enable SWD, Remap tim3 to PC6, 7 , 8 ,9
       gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, 0);//disable JTAG, enable SWD,
    nvic_setup();
    parm_load();
    tim3_setup(); //For general purpose PWM output
    Param::Change(Param::PARAM_LAST);
    DigIo::inv_out.Clear();//inverter power off during bootup
    
    Terminal t(USART3, TermCmds);
    Stm32Can c(CAN1, CanHardware::Baud500, false);
    Stm32Can c2(CAN2, CanHardware::Baud500, false);
    FunctionPointerCallback cb(CanCallback, SetCanFilters);
    Stm32Can *CanMapDev = &c;
    if (Param::GetInt(Param::CanMapCan) == 0)
    {
        CanMapDev = &c;
    }
    else
    {
        CanMapDev = &c2;
    }
    CanMap cm(CanMapDev);
    //CanMap cm(&c);
    CanSdo sdo(&c, &cm);
    sdo.SetNodeId(3);//id 3 for vcu?
    // Set up CAN 1 callback and messages to listen for
    canInterface[0] = &c;
    canInterface[1] = &c2;
    c.AddCallback(&cb);
    c2.AddCallback(&cb);
    TerminalCommands::SetCanMap(&cm);
    canMap = &cm;
    //canSdo = &sdo;
    CanHardware* shunt_can = canInterface[Param::GetInt(Param::ShuntCan)];
    
    canOBD2.SetCanInterface(canInterface[Param::GetInt(Param::OBD2Can)]);
    
    LinBus l(USART1, 19200);
    lin = &l;
    UpdateInv();
    UpdateVehicle();
    UpdateCharger();
    UpdateChargeInt();
    UpdateBMS();
    UpdateHeater();
    UpdateDCDC();
    
    Stm32Scheduler s(TIM4); //We never exit main so it's ok to put it on stack
    scheduler = &s;
    
    s.AddTask(Ms1Task, 1);
    s.AddTask(Ms10Task, 10);
    s.AddTask(Ms100Task, 100);
    s.AddTask(Ms200Task, 200);
    
    //if(Param::GetInt(Param::IsaInit)==1) ISA::initialize(shunt_can);//only call this once if a new sensor is fitted.
    
    Param::SetInt(Param::version, 4); //backward compatibility
    Param::SetInt(Param::opmode, MOD_OFF);//always off at startup

    while(1)
    {
        char c = 0;
        t.Run();
        if (sdo.GetPrintRequest() == PRINT_JSON)
        {
            TerminalCommands::PrintParamsJson(&sdo, &c);
        }
    }

    return 0;
}
