/*
 * This file is part of the stm32-vcu project.
 *
 * Copyright (C) 2023 Roy Henderson
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
#include "Can_LR.h"
#include "stm32_can.h"
#include "utils.h"
#include "digio.h"


void Can_LR::Task100Ms()
{

}

void Can_LR::Task10Ms()
{

}

bool Can_LR::Start()
{
   return Param::GetBool(Param::din_start);
}

bool Can_LR::Ready()
{
   return DigIo::t15_digi.Get();
}