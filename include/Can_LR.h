
/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
 *                      Damien Maguire <info@evbmw.com>
 * Yes I'm really writing software now........run.....run away.......
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
#ifndef Can_LR_h
#define Can_LR_h

#include <stdint.h>
#include "stm32_can.h"
#include "vehicle.h"


class Can_LR: public Vehicle
{
public:
   void Task10Ms();
   void Task100Ms();
   void SetRevCounter(int s) { rpm = s; }
   void SetTemperatureGauge(float) { } //TODO
   bool Ready();// { return true; }
   bool Start();

private:
   uint16_t rpm;
};

#endif // Can_LR_h
