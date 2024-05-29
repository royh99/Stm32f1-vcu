/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2021-2022  Johannes Huebner <dev@johanneshuebner.com>
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
 */
#include "outlanderinverter.h"
#include "my_math.h"
#include "params.h"

OutlanderInverter::OutlanderInverter()
{
    //ctor
}

void OutlanderInverter::SetCanInterface(CanHardware* c)
{
    can = c;

    can->RegisterUserMessage(0x289);//Outlander Inv Msg
    can->RegisterUserMessage(0x299);//Outlander Inv Msg
    can->RegisterUserMessage(0x733);//Outlander Inv Msg
}

void OutlanderInverter::DecodeCAN(int id, uint32_t data[2])
{
    uint8_t* bytes = (uint8_t*)data; // convert the two 32bit array into bytes
    switch (id)
    {
    case 0x289:
        speed = (bytes[2]<<8 | bytes[3]) - 20000;
        voltage = (bytes[4]<<8) + bytes[5];
		if (voltage > 920) voltage = 0; // make voltage 0 when no V applied. ( reading 921.6 - 1023.9 )
        break;
    case 0x299:
        //motor_temp = bytes[0] -40;
        inv_temp = ((bytes[1]-40) + (bytes[4] - 40)) / 2;
        break;
    case 0x733:
        motor_temp = bytes[0] -40;
        break;
    }
}

void OutlanderInverter::SetTorque(float torquePercent)
{
    if(Param::GetInt(Param::reversemotor) == 0)
    {
        final_torque_request = 10000 + (torquePercent * 20);
    }
    else
    {
        final_torque_request = 10000 - (torquePercent * 20);
    }

    Param::SetInt(Param::torque,final_torque_request);//post processed final torque value sent to inv to web interface
}

void OutlanderInverter::Task10Ms()
{
    run10ms++;

    //Run every 50 ms
    if (run10ms == 5)
    {
        uint32_t data[2];
        run10ms = 0;

        data[0] = (final_torque_request & 0xff)<<24 | (final_torque_request & 0xff00)<<8; // swap high and low bytes and shift 16 bits left
        // enable inverter. Byte 6 0x0 to disable inverter, 0x3 for drive ( torque > 0 )
        data[1] = (final_torque_request == 10000 ? 0x00 : 0x03)<<16;

        can->Send(0x287, data, 8);
    }
}

void OutlanderInverter::Task100Ms()
{
    int opmode = Param::GetInt(Param::opmode);
    if(opmode==MOD_RUN)
    {
        uint32_t data[2];
        data[0] = 0x30;
        data[1] = 0x00;
        can->Send(0x371, data, 8);

        data[0] = 0x39140000;
        data[1] = 0x100CFE8F;
        can->Send(0x285, data, 8);

        data[0] = 0x3D000000;
        data[1] = 0x00210000;
        can->Send(0x286, data, 8);
    }
}
