/*
	* This file is part of the ZombieVerter project.
	*
	* Copyright (C) 2021-2023  Johannes Huebner <dev@johanneshuebner.com>
	* 	                        Damien Maguire <info@evbmw.com>
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
	*
	*Control of the Mitsubishi Outlander PHEV on board charger (OBC) and DCDC Converter.
	*
*/


#include <outlanderCharger.h>

uint8_t outlanderCharger::chgStatus;
uint8_t outlanderCharger::evseDuty = 0;
float   outlanderCharger::dcBusV;
float   outlanderCharger::temp_1;
float   outlanderCharger::temp_2;
float   outlanderCharger::ACVolts;
float   outlanderCharger::ACAmps;
float   outlanderCharger::DCAmps;
float   outlanderCharger::LV_Volts;
float   outlanderCharger::LV_Amps;
bool    flag;


bool outlanderCharger::ControlCharge(bool RunCh, bool ACReq)
{
	//if(RunCh && ACReq && (Param::GetInt(Param::PlugDet) || evseDuty > 5))//we have a startup request to AC charge from a charge interface
	if(RunCh && ACReq)//we have a startup request to AC charge from a charge interface
	{		
		IOMatrix::GetPin(IOMatrix::OBCENABLE)->Set();
		if (evseDuty > 5)
		{
			clearToStart=true;	
			Param::SetInt(Param::CtrlPilot_AC, evseDuty);
			return true;
		}
		else
		{
			clearToStart=false;
			IOMatrix::GetPin(IOMatrix::OBCENABLE)->Clear();
			return false;
		}
	}
	else
	{
		clearToStart=false;
		IOMatrix::GetPin(IOMatrix::OBCENABLE)->Clear();
		return false;
	}
}


void outlanderCharger::SetCanInterface(CanHardware* c)
{
	can = c;
	can->RegisterUserMessage(0x377);//dc_dc status
	can->RegisterUserMessage(0x389);//charger status
	can->RegisterUserMessage(0x38A);//charger status 2
}

void outlanderCharger::DecodeCAN(int id, uint32_t data[2])
{
	switch (id)
	{
		case 0x377:
		outlanderCharger::handle377(data);
		break;
		case 0x389:
		outlanderCharger::handle389(data);
		break;
		case 0x38A:
		outlanderCharger::handle38A(data);
		break;
	}
	}
	
	void outlanderCharger::Task100Ms()
	{
		if(clearToStart)
		{
			flag = true;
			uint8_t bytes[8];
			bytes[0] = 0x00;
			bytes[1] = 0x00;
			bytes[2] = 0x00;
			bytes[3] = 0x00;
			bytes[4] = 0x00;
			bytes[5] = 0x00;
			bytes[6] = 0x00;
			bytes[7] = 0x00;
			
			bytes[2] = 0xB6;// 0xb6 in byte 2 enables charger
			can->Send(0x285, (uint32_t*)bytes, 8);
			
			setVolts=Param::GetInt(Param::Voltspnt)*10;
			actVolts=Param::GetInt(Param::udc);
			uint8_t currentMax = (10 * Param::GetInt(Param::Powerspnt))/actVolts;
			currentMax = MIN(currentMax, 120 ); // limit current to Pwrspnt, LIM current or 12A ( charger limits itself based on CP PWM duty cycle )
			//if (Param::GetInt(Param::interface) == i3LIM) // check if LIM is being used
			//currentMax = MIN(currentMax, 10 * GetInt(Param::PilotLim) * ACVolts/GetInt(Param::udc)); //LIM pilot current scaled from AC to DC
			
			bytes[0] = setVolts >> 8;
			bytes[1] = setVolts & 0xff;//B0+B1   = voltage setpoint    (0x0E74=370.0V, 0,1V/bit)
			bytes[2] = currentRamp;//B2  = current setpoint DC-side  (0x78=120=12A -> 0,1A/bit)
			bytes[3] = 0x00;
			bytes[4] = 0x00;
			bytes[5] = 0x00;
			bytes[6] = 0x00;
			bytes[7] = 0x00;
			can->Send(0x286, (uint32_t*)bytes, 8);
			//if(clearToStart)
			//{
			if(actVolts<Param::GetInt(Param::Voltspnt)) currentRamp++;
			if(actVolts>=Param::GetInt(Param::Voltspnt)) currentRamp--;
			if(currentRamp>=currentMax) currentRamp=currentMax;//clamp to max of setpwr, cp or 12A

		}
		else if ( flag == true) // set voltage to 0 at end of charge to avoid voltage spike when contactors open
		{
			currentRamp=0;
			flag = false;
			// set voltage to zero at end of charge session
			uint8_t bytes[8];
			bytes[0] = 0;
			bytes[1] = 0;//B1+B2   = voltage setpoint = 0
			bytes[2] = currentRamp;// = 0
			bytes[3] = 0x00;
			bytes[4] = 0x00;
			bytes[5] = 0x00;
			bytes[6] = 0x00;
			bytes[7] = 0x00;
			can->Send(0x286, (uint32_t*)bytes, 8);		
		}
		else currentRamp=0;
	}
	
	void outlanderCharger::handle377(uint32_t data[2])
	
	{
		uint8_t* bytes = (uint8_t*)data;// this converts the two 32bit array into bytes
		LV_Volts = ((bytes[0]<<8) | (bytes[1]))*0.01;
		LV_Amps = ((bytes[2]<<8) | (bytes[3]))*0.1;
		Param::SetFloat(Param::U12V , LV_Volts);
		Param::SetFloat(Param::I12V , LV_Amps);
		
	}
	
	void outlanderCharger::handle389(uint32_t data[2])
	
	{
		uint8_t* bytes = (uint8_t*)data;// this converts the two 32bit array into bytes
		ACVolts = bytes[1]; //AC voltage measured at charger. Scale 1 to 1.
		ACAmps = bytes[6] * 0.1; //Current in Amps from mains. scale 0.1.
		DCAmps = bytes[2] * 0.1; //Current in Amps from charger to battery. scale 0.1.
		Param::SetFloat(Param::AC_Volts , ACVolts);
		Param::SetFloat(Param::AC_Amps , ACAmps);
	}
	
	void outlanderCharger::handle38A(uint32_t data[2])
	
	{
		uint8_t* bytes = (uint8_t*)data;// this converts the two 32bit array into bytes
		chgStatus = bytes[4];
		evseDuty = bytes[3];
		dcBusV = bytes[2]*2;// Volts scale 2
		temp_1 = bytes[0]-40;// degC bias -40
		temp_2 = bytes[1]-40;// degC bias -40
		Param::SetFloat(Param::ChgTemp , MAX(temp_1 , temp_2));
	}
	
	
		