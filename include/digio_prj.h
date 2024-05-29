
/*
 * This file is part of the ZombieVerter project.
 *
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

#ifndef PinMode_PRJ_H_INCLUDED
#define PinMode_PRJ_H_INCLUDED

#include "hwdefs.h"

#define DIG_IO_LIST \
    DIG_IO_ENTRY(HV_req,    GPIOB, GPIO8,  PinMode::INPUT_PD)    \
    DIG_IO_ENTRY(start_in,  GPIOC, GPIO12, PinMode::INPUT_PD)    \
    DIG_IO_ENTRY(brake_in,  GPIOB, GPIO7,  PinMode::INPUT_PD)    \
    DIG_IO_ENTRY(fwd_in,    GPIOB, GPIO6,  PinMode::INPUT_PD)    \
    DIG_IO_ENTRY(rev_in,    GPIOB, GPIO5,  PinMode::INPUT_PD)    \
    DIG_IO_ENTRY(dcsw_out,  GPIOC, GPIO7,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(led_out,	GPIOB, GPIO9,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(led_rd,	GPIOC, GPIO13, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(led_yl,	GPIOA, GPIO1,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(led_gr,	GPIOA, GPIO3,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(led_bl,	GPIOA, GPIO6,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(gp_out1,   GPIOC, GPIO8,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(gp_out2,   GPIOC, GPIO9,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(gp_out3,   GPIOB, GPIO15, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(lin_nslp,  GPIOA, GPIO8,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(prec_out,  GPIOC, GPIO6,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(inv_out,   GPIOB, GPIO14, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(t15_digi,  GPIOD, GPIO2,  PinMode::INPUT_PD)    \
    DIG_IO_ENTRY(gp_12Vin,  GPIOB, GPIO0,  PinMode::INPUT_PD)   \
	DIG_IO_ENTRY(gp_12Vin2, GPIOB, GPIO1,  PinMode::INPUT_PD)   \
    DIG_IO_ENTRY(dummypin,  GPIOE, GPIO7,  PinMode::INPUT_PD)   \

//dummypin is used by IOMatrix class for unused functions. Must be set to a pin that has no effect

#endif // PinMode_PRJ_H_INCLUDED
