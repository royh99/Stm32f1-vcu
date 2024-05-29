/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2023 Damien Maguire
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
#include <outlanderDCDC.h>


 void outlanderDCDC::SetCanInterface(CanHardware* c)
{
   can = c;
   can->RegisterUserMessage(0x377);
}

// Process voltage , current and temperature message from DCDC converter.
void outlanderDCDC::DecodeCAN(int id, uint32_t *data)
{
   if (id == 0x377)
   {
   uint8_t* bytes = (uint8_t*)data;// convert the two 32bit array into bytes
   Param::SetFloat(Param::U12V , ((bytes[0]<<8) | (bytes[1]))*0.01);
   Param::SetFloat(Param::I12V , ((bytes[2]<<8) | (bytes[3]))*0.1);
   }
}

void outlanderDCDC::Task100Ms() {

int opmode = Param::GetInt(Param::opmode);

   if(opmode==MOD_RUN || opmode==MOD_CHARGE)
   {
      IOMatrix::GetPin(IOMatrix::OBCENABLE)->Set();
      if (Param::GetFloat(Param::U12V) > 12.5 && Param::GetFloat(Param::I12V) < 2.0) // Disable DC-DC converter if U12V is above 12.5V and current bellow 2A
      {
         IOMatrix::GetPin(IOMatrix::DCDCENABLE)->Clear();
      }
      else
      {
         IOMatrix::GetPin(IOMatrix::DCDCENABLE)->Set(); // DC-DC on if U12V <= 12.5V
      }
   }
   else
   {
      IOMatrix::GetPin(IOMatrix::DCDCENABLE)->Clear(); // DC-DC off unless in run or charge mode.
	  IOMatrix::GetPin(IOMatrix::OBCENABLE)->Clear();
   }
}

