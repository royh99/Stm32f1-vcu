/*
 * This file is part of the stm32-vcu project.
 *
 * Copyright (C) 2021 Damien Maguire
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
#include <stdint.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/dac.h>
#include "stm32_can.h"
#include "terminal.h"
#include "params.h"
#include "hwdefs.h"
#include "digio.h"
#include "hwinit.h"
#include "anain.h"
#include "temp_meas.h"
#include "param_save.h"
#include "my_math.h"
#include "errormessage.h"
#include "printf.h"
#include "stm32scheduler.h"
#include "cansdo.h"
#include "isa_shunt.h"
#include "Can_LR.h"
#include "outlanderinverter.h"
#include "throttle.h"
#include "utils.h"
#include "i3LIM.h"
#include "CANSPI.h"
#include "chademo.h"
#include "heater.h"
#include "inverter.h"
#include "vehicle.h"
#include "chargerhw.h"
#include "canmap.h"
#include "terminalcommands.h"
#include "iomatrix.h"
#include "bmw_sbox.h"
#include "chargerint.h"
#include "notused.h"
#include "nocharger.h"
#include "extCharger.h"
#include "noHeater.h"
#include "bms.h"
#include "simpbms.h"
#include "outlanderCharger.h"
#include "Can_OBD2.h"
#include "dcdc.h"
#include "outlanderDCDC.h"
#include "linbus.h"
#include "NoVehicle.h"


#define PRECHARGE_TIMEOUT 2  //2s

#define PRINT_JSON 0

typedef union {
    struct {
        uint32_t data[2];
        uint8_t datab[8];

    } frame;
} CAN_Msg;
