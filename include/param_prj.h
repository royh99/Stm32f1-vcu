/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2011-2019 Johannes Huebner <dev@johanneshuebner.com>
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
#define VER 1.15.A


/* Entries must be ordered as follows:
   1. Saveable parameters (id != 0)
   2. Temporary parameters (id = 0)
   3. Display values
 */
//Next param id (increase when adding new parameter!): 132
/*              category     name         unit       min     max     default id */
#define PARAM_LIST \
    PARAM_ENTRY(CAT_SETUP,     Inverter,     INVMODES, 0,      1,      1,      5  ) \
    PARAM_ENTRY(CAT_SETUP,     Vehicle,      VEHMODES, 0,      2,      2,      6  ) \
    PARAM_ENTRY(CAT_SETUP,     Transmission, TRNMODES, 0,      1,      0,      78 ) \
	PARAM_ENTRY(CAT_SETUP,     interface,    CHGINT,    0,     3,      0,      39 ) \
    PARAM_ENTRY(CAT_SETUP,     chargemodes,  CHGMODS,   0,     6,      0,	   37 ) \
	PARAM_ENTRY(CAT_SETUP,     InverterCan,  CAN_DEV,  0,      1,      0,      70 ) \
    PARAM_ENTRY(CAT_SETUP,     VehicleCan,   CAN_DEV,  0,      1,      1,      71 ) \
    PARAM_ENTRY(CAT_SETUP,     ShuntCan,     CAN_DEV,  0,      1,      0,      72 ) \
    PARAM_ENTRY(CAT_SETUP,     LimCan,       CAN_DEV,  0,      1,      0,      73 ) \
    PARAM_ENTRY(CAT_SETUP,     ChargerCan,   CAN_DEV,  0,      1,      0,      74 ) \
    PARAM_ENTRY(CAT_SETUP,     BMSCan,       CAN_DEV,  0,      1,      1,      89 ) \
    PARAM_ENTRY(CAT_SETUP,     OBD2Can,      CAN_DEV,  0,      1,      0,      96 ) \
    PARAM_ENTRY(CAT_SETUP,     CanMapCan,    CAN_DEV,  0,      1,      0,      97 ) \
    PARAM_ENTRY(CAT_SETUP,     DCDCCan,      CAN_DEV,  0,      1,      0,     107 ) \
	PARAM_ENTRY(CAT_THROTTLE,  potmin,      "digval",  0,      4095,     0,    7  ) \
    PARAM_ENTRY(CAT_THROTTLE,  potmax,      "digval",  0,      4095,   4095,   8  ) \
    PARAM_ENTRY(CAT_THROTTLE,  pot2min,     "digval",  0,      4095,   4095,   9  ) \
    PARAM_ENTRY(CAT_THROTTLE,  pot2max,     "digval",  0,      4095,   4095,   10 ) \
    PARAM_ENTRY(CAT_THROTTLE,  regenrpm,    "rpm",     100,    10000,  1500,   60 ) \
	PARAM_ENTRY(CAT_THROTTLE,  regenendrpm, "rpm",     100,    10000,  100,   126 ) \
    PARAM_ENTRY(CAT_THROTTLE,  regenmax,    "%",       -30,    0,     -10,     61 ) \
	PARAM_ENTRY(CAT_THROTTLE,  regenBrake,    "%",     -30,    0,     -10,    122 ) \
    PARAM_ENTRY(CAT_THROTTLE,  regenramp,   "%/10ms",  0.1,    10,     0.1,    68 ) \
    PARAM_ENTRY(CAT_THROTTLE,  potmode,     POTMODES,  0,      1,      0,      11 ) \
    PARAM_ENTRY(CAT_THROTTLE,  dirmode,     DIRMODES,  0,      4,      1,      12 ) \
    PARAM_ENTRY(CAT_THROTTLE,  reversemotor,   ONOFF,  0,      1,      0,     127 ) \
	PARAM_ENTRY(CAT_THROTTLE,  throtramp,   "%/10ms",  0.1,    20,     5,      13 ) \
    PARAM_ENTRY(CAT_THROTTLE,  throtramprpm,"rpm",     0,      10000,  10000,  14 ) \
    PARAM_ENTRY(CAT_THROTTLE,  revlim,      "rpm",     0,      10000,  6000,   15 ) \
	PARAM_ENTRY(CAT_THROTTLE,  bmslimhigh,  "%",       0,      100,    50,     17 ) \
    PARAM_ENTRY(CAT_THROTTLE,  bmslimlow,   "%",      -100,    0,     -1,      18 ) \
    PARAM_ENTRY(CAT_THROTTLE,  udcmin,      "V",       0,      1000,   450,    19 ) \
    PARAM_ENTRY(CAT_THROTTLE,  udclim,      "V",       0,      1000,   520,    20 ) \
    PARAM_ENTRY(CAT_THROTTLE,  idcmax,      "A",       0,      5000,   5000,   21 ) \
    PARAM_ENTRY(CAT_THROTTLE,  idcmin,      "A",      -5000,   0,     -5000,   22 ) \
    PARAM_ENTRY(CAT_THROTTLE,  temphsmax,    "°C",     50,     127,    85,     23 ) \
    PARAM_ENTRY(CAT_THROTTLE,  tempmmax,     "°C",     70,     300,    300,    24 ) \
    PARAM_ENTRY(CAT_THROTTLE,  throtmax,    "%",       0,      100,    100,    25 ) \
    PARAM_ENTRY(CAT_THROTTLE,  throtmin,    "%",      -100,    0,     -100,    26 ) \
    PARAM_ENTRY(CAT_THROTTLE,  throtmaxRev, "%",       0,      100,    30,    123 ) \
    PARAM_ENTRY(CAT_THROTTLE,  throtdead,   "%",       0,      50,     5,      76 ) \
	PARAM_ENTRY(CAT_THROTTLE,  RegenBrakeLight, "%",  -100,    0,     -15,    128 ) \
	PARAM_ENTRY(CAT_THROTTLE,  throtrpmfilt,"rpm/10ms",0.1,    200,    15,    131 ) \
    PARAM_ENTRY(CAT_CRUISE,    regenlevel,  "",        0,      3,      2,      31 ) \
    PARAM_ENTRY(CAT_CONTACT,   udcsw,       "V",       0,      1000,   330,    32 ) \
    PARAM_ENTRY(CAT_CONTACT,   errlights,   ERRLIGHTS, 0,      255,    0,      34 ) \
    PARAM_ENTRY(CAT_CHARGER,   BattCap,     "kWh",     0.1,    250,    22,     38 ) \
    PARAM_ENTRY(CAT_CHARGER,   Voltspnt,    "V",       0,      1000,   395,    40 ) \
    PARAM_ENTRY(CAT_CHARGER,   Powerspnt,   "W",       0,      12000,  1500,   41 ) \
    PARAM_ENTRY(CAT_CHARGER,   IdcTerm,     "A",       0,      150,    0,      56 ) \
    PARAM_ENTRY(CAT_CHARGER,   CCS_ICmd,    "A",       0,      150,    0,      42 ) \
    PARAM_ENTRY(CAT_CHARGER,   CCS_ILim,    "A",       0,      350,    100,    43 ) \
    PARAM_ENTRY(CAT_CHARGER,   CCS_SOCLim,  "%",       0,      100,    80,     44 ) \
    PARAM_ENTRY(CAT_CHARGER,   SOCFC,       "%",       0,      100,    50,     79 ) \
    PARAM_ENTRY(CAT_CHARGER,   Chgctrl,     CHGCTRL,   0,      2,      0,      45 ) \
    PARAM_ENTRY(CAT_CHARGER,   ChgAcVolt,   "Vac",     0,      250,  240,     120 ) \
    PARAM_ENTRY(CAT_CHARGER,   ChgEff,      "%",       0,      100,   90,      121) \
	PARAM_ENTRY(CAT_DCDC,      DCdc_Type,   DCDCTYPES, 0,      2,      0,     105 ) \
    PARAM_ENTRY(CAT_DCDC,      DCSetPnt,    "V",       9,      15,     14,    106 ) \
    PARAM_ENTRY(CAT_BMS,       BMS_Mode,    BMSMODES,  0,      1,      0,      90 ) \
    PARAM_ENTRY(CAT_BMS,       BMS_Timeout,  "sec",    1,      120,    10,     91 ) \
    PARAM_ENTRY(CAT_BMS,       BMS_VminLimit, "V",     0,      10,     3.0,    92 ) \
    PARAM_ENTRY(CAT_BMS,       BMS_VmaxLimit, "V",     0,      10,     4.2,    93 ) \
    PARAM_ENTRY(CAT_BMS,       BMS_TminLimit, "°C",    -100,   100,    5,      94 ) \
    PARAM_ENTRY(CAT_BMS,       BMS_TmaxLimit, "°C",    -100,   100,    50,     95 ) \
    PARAM_ENTRY(CAT_HEATER,    Heater,      HTTYPE,    0,      3,      0,      57 ) \
    PARAM_ENTRY(CAT_HEATER,    Control,     HTCTRL,    0,      2,      0,      58 ) \
    PARAM_ENTRY(CAT_HEATER,    HeatPwr,     "W",       0,      6500,   0,      59 ) \
    PARAM_ENTRY(CAT_HEATER,    HeatPercnt,  "%",       0,      100,    0,     124 )	\
    PARAM_ENTRY(CAT_CLOCK,     Set_Day,     DOW,       0,      6,      0,      46 ) \
    PARAM_ENTRY(CAT_CLOCK,     Set_Hour,    "Hours",   0,      23,     0,      47 ) \
    PARAM_ENTRY(CAT_CLOCK,     Set_Min,     "Mins",    0,      59,     0,      48 ) \
    PARAM_ENTRY(CAT_CLOCK,     Set_Sec,     "Secs",    0,      59,     0,      49 ) \
    PARAM_ENTRY(CAT_CLOCK,     Chg_Hrs,     "Hours",   0,      23,     0,      50 ) \
    PARAM_ENTRY(CAT_CLOCK,     Chg_Min,     "Mins",    0,      59,     0,      51 ) \
    PARAM_ENTRY(CAT_CLOCK,     Chg_Dur,     "Mins",    0,      600,    0,      52 ) \
    PARAM_ENTRY(CAT_CLOCK,     Pre_Hrs,     "Hours",   0,      59,     0,      53 ) \
    PARAM_ENTRY(CAT_CLOCK,     Pre_Min,     "Mins",    0,      59,     0,      54 ) \
    PARAM_ENTRY(CAT_CLOCK,     Pre_Dur,     "Mins",    0,      60,     0,      55 ) \
    PARAM_ENTRY(CAT_IOPINS,    Out1Func,    PINFUNCS,  0,      14,     14,     80 ) \
    PARAM_ENTRY(CAT_IOPINS,    Out2Func,    PINFUNCS,  0,      14,     7,      81 ) \
    PARAM_ENTRY(CAT_IOPINS,    Out3Func,    PINFUNCS,  0,      14,     3,      82 ) \
    PARAM_ENTRY(CAT_IOPINS,    SL1Func,     PINFUNCS,  0,      13,     0,      83 ) \
    PARAM_ENTRY(CAT_IOPINS,    SL2Func,     PINFUNCS,  0,      13,     0,      84 ) \
    PARAM_ENTRY(CAT_IOPINS,    PWM1Func,    PINFUNCS,  0,      14,     0,      85 ) \
    PARAM_ENTRY(CAT_IOPINS,    PWM2Func,    PINFUNCS,  0,      14,     4,      86 ) \
    PARAM_ENTRY(CAT_IOPINS,    PWM3Func,    PINFUNCS,  0,      15,     2,      87 ) \
    PARAM_ENTRY(CAT_IOPINS,    GP12VInFunc, PINFUNCS,  0,      13,     12,     98 ) \
    PARAM_ENTRY(CAT_IOPINS,    HVReqFunc,   PINFUNCS,  0,      13,     11,     99 ) \
    PARAM_ENTRY(CAT_IOPINS,    GPA1Func,    APINFUNCS, 0,      2,      0,     110 ) \
	PARAM_ENTRY(CAT_IOPINS,    ppthresh,    "dig",     0,      4095,   1500,  114 ) \
    PARAM_ENTRY(CAT_IOPINS,    BrkVacThresh,"dig",     0,      4095,   2500,  115 ) \
    PARAM_ENTRY(CAT_IOPINS,    BrkVacHyst,  "dig",     0,      4095,   2500,  116 ) \
    PARAM_ENTRY(CAT_IOPINS,    DAC1,        "dig",     0,      4095,   0,     117 ) \
    PARAM_ENTRY(CAT_IOPINS,    DAC2,        "dig",     0,      4095,   0,     118 ) \
    PARAM_ENTRY(CAT_SHUNT,     IsaInit,     ONOFF,     0,      1,      0,      75 ) \
    PARAM_ENTRY(CAT_SHUNT,     Type,        SHNTYPE,   0,      1,      1,      88 ) \
    PARAM_ENTRY(CAT_PWM,       Tim3_Presc,  "",        1,      72000,  719,   100 ) \
    PARAM_ENTRY(CAT_PWM,       Tim3_Period, "",        1,      100000, 2000,  101 ) \
    PARAM_ENTRY(CAT_PWM,       Tim3_1_OC,   "",        1,      100000,  300,  104 ) \
	VALUE_ENTRY(version,       VERSTR,              2000 ) \
    VALUE_ENTRY(opmode,        OPMODES,             2002 ) \
    VALUE_ENTRY(chgtyp,        CHGTYPS,             2003 ) \
    VALUE_ENTRY(lasterr,       errorListString,     2004 ) \
	VALUE_ENTRY(currenterror,  errorListString,     3999 ) \
    VALUE_ENTRY(status,        STATUS,              2005 ) \
    VALUE_ENTRY(udc,           "V",                 2006 ) \
    VALUE_ENTRY(udc2,          "V",                 2007 ) \
    VALUE_ENTRY(udc3,          "V",                 2008 ) \
    VALUE_ENTRY(deltaV,        "V",                 2009 ) \
    VALUE_ENTRY(INVudc,        "V",                 2010 ) \
    VALUE_ENTRY(power,         "kW",                2011 ) \
    VALUE_ENTRY(idc,           "A",                 2012 ) \
    VALUE_ENTRY(KWh,           "kWh",               2013 ) \
    VALUE_ENTRY(AMPh,          "Ah",                2014 ) \
    VALUE_ENTRY(SOC,           "%",                 2015 ) \
    VALUE_ENTRY(BMS_Vmin,      "V",                 2084 ) \
    VALUE_ENTRY(BMS_Vmax,      "V",                 2085 ) \
    VALUE_ENTRY(BMS_Tmin,      "°C",                2086 ) \
    VALUE_ENTRY(BMS_Tmax,      "°C",                2087 ) \
    VALUE_ENTRY(BMS_ChargeLim, "A",                 2088 ) \
    VALUE_ENTRY(speed,         "rpm",               2016 ) \
    VALUE_ENTRY(Veh_Speed,     "kph",               2017 ) \
    VALUE_ENTRY(torque,        "dig",               2018 ) \
    VALUE_ENTRY(pot,           "dig",               2019 ) \
    VALUE_ENTRY(pot2,          "dig",               2020 ) \
    VALUE_ENTRY(potbrake,      "dig",               2021 ) \
    VALUE_ENTRY(brakepressure, "dig",               2022 ) \
    VALUE_ENTRY(potnom,        "%",                 2023 ) \
    VALUE_ENTRY(torquepercent,  "%",                2103 ) \
    VALUE_ENTRY(dir,           DIRS,                2024 ) \
    VALUE_ENTRY(tmphs,         "°C",                2028 ) \
    VALUE_ENTRY(tempmotor,     "°C",                2029 ) \
    VALUE_ENTRY(tmpaux,        "°C",                2030 ) \
    VALUE_ENTRY(uaux,          "V",                 2031 ) \
    VALUE_ENTRY(canio,         CANIOS,              2032 ) \
    VALUE_ENTRY(din_start,     ONOFF,               2036 ) \
    VALUE_ENTRY(din_brake,     ONOFF,               2037 ) \
    VALUE_ENTRY(din_forward,   ONOFF,               2038 ) \
    VALUE_ENTRY(din_reverse,   ONOFF,               2039 ) \
    VALUE_ENTRY(din_bms,       ONOFF,               2040 ) \
    VALUE_ENTRY(din_12Vgp,     ONOFF,               2071 ) \
    VALUE_ENTRY(handbrk,       ONOFF,               2041 ) \
    VALUE_ENTRY(T15Stat,       ONOFF,               2045 ) \
    VALUE_ENTRY(InvStat,       ONOFF,               2046 ) \
    VALUE_ENTRY(CableLim,      "A",                 2048 ) \
    VALUE_ENTRY(PilotLim,      "A",                 2049 ) \
    VALUE_ENTRY(PlugDet,       ONOFF,               2050 ) \
    VALUE_ENTRY(PilotTyp,      PLTMODES,            2051 ) \
    VALUE_ENTRY(CCS_I_Avail,   "A",                 2052 ) \
    VALUE_ENTRY(CCS_V_Avail,   "V",                 2053 ) \
    VALUE_ENTRY(CCS_I,         "A",                 2054 ) \
    VALUE_ENTRY(CCS_Ireq,      "A",                 2068 ) \
    VALUE_ENTRY(CCS_V,         "V",                 2055 ) \
    VALUE_ENTRY(CCS_V_Min,     "V",                 2056 ) \
    VALUE_ENTRY(CCS_V_Con,     "V",                 2057 ) \
    VALUE_ENTRY(hvChg,         ONOFF,               2058 ) \
    VALUE_ENTRY(CCS_COND,      CCS_STATUS,          2059 ) \
    VALUE_ENTRY(CCS_State,     "s",                 2060 ) \
    VALUE_ENTRY(CP_DOOR,       DMODES,              2061 ) \
    VALUE_ENTRY(CCS_Contactor, ONOFF,               2062 ) \
    VALUE_ENTRY(Day,           DOW,                 2064 ) \
    VALUE_ENTRY(Hour,          "H",                 2065 ) \
    VALUE_ENTRY(Min,           "M",                 2066 ) \
    VALUE_ENTRY(Sec,           "S",                 2067 ) \
    VALUE_ENTRY(ChgTime,       "M",                 2090 ) \
    VALUE_ENTRY(HeatReq,       ONOFF,               2069 ) \
    VALUE_ENTRY(U12V,          "V",                 2070 ) \
    VALUE_ENTRY(I12V,          "A",                 2083 ) \
    VALUE_ENTRY(ChgTemp,       "°C",                2078 ) \
    VALUE_ENTRY(AC_Volts,      "V",                 2079 ) \
    VALUE_ENTRY(AC_Amps,       "A",                 2089 ) \
    VALUE_ENTRY(cpuload,       "%",                 2063 ) \
    VALUE_ENTRY(PPVal,         "dig",               2094 ) \
    VALUE_ENTRY(BrkVacVal,     "dig",               2095 ) \
	VALUE_ENTRY(tmpheater,     "°C",                2096 ) \
    VALUE_ENTRY(udcheater,     "V",                 2097 ) \
    VALUE_ENTRY(powerheater,   "W",                 2098 ) \
    VALUE_ENTRY(limpmode,      ONOFF,               4000 ) \
	
//Next value Id: 2099
#define VERSTR STRINGIFY(4=VER)
#define PINFUNCS     "0=None, 1=ChaDeMoAlw, 2=OBCEnable, 3=HeaterEnable, 4=RunIndication, 5=WarnIndication," \
                     "6=CoolantPump, 7=NegContactor, 8=BrakeLight, 9=ReverseLight, 10=HeatReq, 11=HVRequest," \
                     "12=DCFCRequest, 13=BrakeVacPump, 14=DCDCEnable"
#define APINFUNCS    "0=None, 1=ProxPilot, 2=BrakeVacSensor"
#define SHNTYPE      "0=ISA, 1=SBOX"
#define DMODES       "0=CLOSED, 1=OPEN, 2=ERROR, 3=INVALID"
#define POTMODES     "0=SingleChannel, 1=DualChannel"
#define BTNSWITCH    "0=Button, 1=Switch, 2=CAN"
#define DIRMODES     "0=Button, 1=Switch, 2=ButtonReversed, 3=SwitchReversed, 4=DefaultForward"
#define INVMODES     "0=None, 1=Outlander"
#define PLTMODES     "0=Absent, 1=ACStd, 2=ACchg, 3=Error, 4=CCS_Not_Rdy, 5=CCS_Rdy, 6=Static"
#define VEHMODES     "0=None, 1=classic, 2=LR"
#define BMSMODES     "0=Off, 1=SimpBMS"
#define OPMODES      "0=Off, 1=Run, 2=Precharge, 3=PchFail, 4=Charge"
#define DOW          "0=Sun, 1=Mon, 2=Tue, 3=Wed, 4=Thu, 5=Fri, 6=Sat"
#define CHGTYPS      "0=Off, 1=AC, 2=DCFC"
#define DCDCTYPES    "0=NoDCDC, 1=Outlander"
#define STATUS       "0=None, 1=UdcLow, 2=UdcHigh, 4=UdcBelowUdcSw, 8=UdcLim, 16=EmcyStop, 32=MProt, 64=PotPressed, 128=TmpHs, 256=WaitStart"
#define CCS_STATUS   "0=NotRdy, 1=ready, 2=SWoff, 3=interruption, 4=Prech, 5=insulmon, 6=estop, 7=malfunction, 15=invalid"
#define DIRS         "-1=Reverse, 0=Neutral, 1=Forward"
#define ONOFF        "0=Off, 1=On"
#define LOWHIGH      "0=LOW, 1=HIGH, 2=AUTO"
#define OKERR        "0=Error, 1=Ok, 2=na"
#define CANSPEEDS    "0=125k, 1=250k, 2=500k, 3=800k, 4=1M"
#define CANIOS       "1=Cruise, 2=Start, 4=Brake, 8=Fwd, 16=Rev, 32=Bms"
#define CANPERIODS   "0=100ms, 1=10ms"
#define ERRLIGHTS    "0=Off, 4=EPC, 8=engine"
#define CRUISESTATES "0=None, 1=On, 2=Disable, 4=Set, 8=Resume"
#define CDMSTAT      "1=Charging, 2=Malfunction, 4=ConnLock, 8=BatIncomp, 16=SystemMalfunction, 32=Stop"
#define HTTYPE       "0=None, 1=Ampera, 2=VW, 3=Outlander"
#define HTCTRL       "0=Disable, 1=Enable, 2=Timer"
#define CHGMODS      "0=Off, 1=Out_lander"
#define CHGCTRL      "0=Enable, 1=Disable, 2=Timer"
#define CHGINT       "0=Unused, 1=Chademo"
#define CAN3Spd      "0=k33.3, 1=k500"
#define TRNMODES     "0=Manual, 1=Auto"
#define CAN_DEV      "0=CAN1, 1=CAN2"
#define CAT_THROTTLE "Throttle"
#define CAT_POWER    "Power Limit"
#define CAT_CONTACT  "Contactor Control"
#define CAT_TEST     "Testing"
#define CAT_COMM     "Communication"
#define CAT_SETUP    "General Setup"
#define CAT_CLOCK    "RTC Module"
#define CAT_HEATER   "Heater Module"
#define CAT_BMS      "Battery Management"
#define CAT_CRUISE   "Cruise Control"
#define CAT_LEXUS    "Gearbox Control"
#define CAT_CHARGER  "Charger Control"
#define CAT_DCDC     "DC-DC Converter"
#define CAT_SHUNT    "ISA Shunt Control"
#define CAT_IOPINS   "General Purpose I/O"
#define CAT_PWM      "PWM Control"

#define CAN_PERIOD_100MS    0
#define CAN_PERIOD_10MS     1

#define FIRST_IO_PARAM Param::Out1Func
#define FIRST_AI_PARAM Param::GPA1Func
enum modes
{
    MOD_OFF = 0,
    MOD_RUN,
    MOD_PRECHARGE,
    MOD_PRECHGFAIL,
    MOD_CHARGE,
    MOD_LAST
};

enum ctyps
{
    OFF = 0,
    AC,
    DCFC
};

enum _tripmodes
{
    TRIP_ALLOFF = 0,
    TRIP_DCSWON,
    TRIP_PRECHARGEON
};

enum _dirmodes
{
    DIR_BUTTON = 0,
    DIR_SWITCH = 1,
    DIR_REVERSED = 2,
    DIR_DEFAULTFORWARD = 4
};

enum InvModes
{
	NoInv = 0,
    Outlander = 1
};

enum ChargeModes
{
    Off = 0,
    Out_lander = 1
};

enum ChargeInterfaces
{
    Unused = 0,
    Chademo = 1
};

enum HeatType
{
    Noheater = 0,
    AmpHeater = 1,
    VW = 2
};

enum BMSModes
{
    BMSModeNoBMS = 0,
    BMSModeSimpBMS = 1
};

enum DCDCModes
{
    NoDCDC = 0,
	OutlanderDCDC = 1
};

enum ChargeControl
{
    Enable = 0,
    Disable = 1,
    Timer = 2
};

enum Gear
{
    LOW = 0,
    HIGH = 1,
    AUTO = 2
};

enum vehicles
{
    None = 0,
    Classic = 1, //used as a flag
	LR = 2
};

enum _potmodes
{
    POTMODE_SINGLECHANNEL = 0,
    POTMODE_DUALCHANNEL,
};

enum _canio
{
    CAN_IO_CRUISE = 1,
    CAN_IO_START = 2,
    CAN_IO_BRAKE = 4,
    CAN_IO_FWD = 8,
    CAN_IO_REV = 16,
    CAN_IO_BMS = 32
};

enum status
{
    STAT_NONE = 0,
    STAT_UDCLOW = 1,
    STAT_UDCHIGH = 2,
    STAT_UDCBELOWUDCSW = 4,
    STAT_UDCLIM = 8,
    STAT_EMCYSTOP = 16,
    STAT_MPROT = 32,
    STAT_POTPRESSED = 64,
    STAT_TMPHS = 128,
    STAT_WAITSTART = 256
};

enum ccs_status
{
    CCS_NOTREADY = 0,
    CCS_READY = 1,
    CCS_SWITCHOFF = 2,
    CCS_INTERRUPTION = 3,
    CCS_PRECHARGE = 4,
    CCS_INSULATION = 5,
    CCS_ESTOP = 6,
    CCS_MALFUNCTION = 7,
    CCS_INVAID = 15
};

enum can_devices
{
    CAN_DEV1 = 0,
    CAN_DEV2 = 1
};

enum direction
{
    DIR_REVERSE = -1,
    DIR_NEUTRAL = 0,
	DIR_FORWARD = 1
};


extern const char* errorListString;

