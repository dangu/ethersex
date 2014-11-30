/*
 * Copyright(C) 2012 Daniel Gullberg <daniel_gullberg@hotmail.com>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * For more information on the GPL, please go to:
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <stdint.h>
#include "hardware/onewire/onewire.h"

#ifndef HAVE_HEATINGCTRL_H
#define HAVE_HEATINGCTRL_H

/* This struct contains the parameters as well as the state of the
 * PID controller
 */
typedef struct
{
  // Parameters
  int16_t Pgain;
  int16_t Igain;
  int16_t uMin;
  int16_t uMax;

  // States
  int16_t I;
  int16_t u;

} pid_data_t;

/* Eeprom parameters */
typedef struct
{
  uint16_t t_target_room;
  pid_data_t pid_room;
  pid_data_t pid_rad;

} heating_ctrl_params_t;

/* Temperature sensor data
 * This struct contains the rom ID of the 1w sensor as well as the
 * signal
 */

typedef struct
{
  ow_rom_code_t rom;
  int16_t signal;

} sensor_data_t;

// Sensors
#define N_SENSORS 3

#define SENSOR_T_ROOM 0
#define SENSOR_T_RAD 1
#define SENSOR_T_OUT 2

// Parameters
#define T_RES(x)  (x*100)        // Temperature resolution

#define MAX_RADTEMP T_RES(60)   // Do not try to set radiator temp higher than this
#define MAX_RADTEMPDIFF T_RES(5)        // Do not try to increase radiator temp higher than this


int16_t heating_ctrl_onrequest(char *cmd, char *output, uint16_t len);

void heating_ctrl_init(void);

void heating_ctrl_periodic(void);

int16_t heating_ctrl_info(uint8_t index);

int16_t heating_ctrl_controller(void);

int16_t pid_controller(pid_data_t * pPtr, int16_t tTarget,
                       sensor_data_t * sensorPtr);


#include "config.h"
#ifdef DEBUG_HEATING_CTRL
# include "core/debug.h"
# define HEATINGCTRLDEBUG(a...)  debug_printf("heating ctrl: " a)
//#define HEATINGCTRLDEBUG(s, args...) printf_P(PSTR("D: " s), ## args)

#define ctrl_printf(s, args...) printf_P(PSTR(s), ## args)

#else
# define HEATINGCTRLDEBUG(a...)
#define ctrl_printf(s, args...)
#endif

#endif /* HAVE_HEATINGCTRL_H */
