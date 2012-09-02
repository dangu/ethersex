 /*  Heating controller for home automation

 Copyright(C) 2012 Daniel Gullberg <daniel_gullberg@hotmail.com>

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software Foundation,
 Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
*/

#include <avr/pgmspace.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "heating_ctrl.h"

#include "hardware/onewire/onewire.h"

#include "protocols/ecmd/ecmd-base.h"

/*
  If enabled in menuconfig, this function is called during boot up of ethersex
*/
int16_t
heating_ctrl_init(void)
{
	HEATINGCTRLDEBUG ("init\n");
  // enter your code here

  return ECMD_FINAL_OK;
}

/*
  If enabled in menuconfig, this function is periodically called
  change "timer(1000,heating_ctrl_periodic)" if needed

  Sensors:
  "a":["105602a501080011",
		"Radiatorer, tillopp"],
"tIn":["28dbfa7102000051",
		  "Inomhus, nere"],
"d":["10d136a5010800e5",
	"Ventilation, uteluft"],
*/
int16_t
heating_ctrl_periodic(void)
{
	int16_t	tIndoor, tOutdoor, tRad; // Measured temperatures
	int16_t tTargetIndoor, tTargetRad; // Target temperatures
	int16_t uShunt;

	int8_t ret;
	//const ow_rom_code_t romSensorRad = {0x105602a501080011};
	ow_rom_code_t romSensorRad;
	HEATINGCTRLDEBUG("periodic\n");
	romSensorRad.raw = 0x105602a501080011;
	tTargetRad = 40;
	ret=ow_temp_start_convert_wait(&romSensorRad)
	HEATINGCTRLDEBUG("conv %d\n",ret);

	ow_temp_scratchpad_t sp;
	ret = ow_temp_read_scratchpad(&romSensorRad, &sp);

	if (ret != 1)
	{
	HEATINGCTRLDEBUG("scratchpad read failed: %d\n", ret);
	}
	else{	HEATINGCTRLDEBUG("successfully read scratchpad\n");

	}

    //self.s.send("1w convert" + "\n")
    //self.s.send("1w get " + sensorID + "\n")
    //response = float(self.s.recv(1024).rstrip("\n").lstrip())

	//tRad =
	uShunt = pid_controller(tTargetRad, tRad);
	HEATINGCTRLDEBUG("uShunt: %d\n", uShunt);
	//setPWM(uShunt);




  return ECMD_FINAL_OK;
}

/*
  This function will be called on request by menuconfig, if wanted...
  You need to enable ECMD_SUPPORT for this.
  Otherwise you can use this function for anything you like
*/
int16_t
heating_ctrl_onrequest(char *cmd, char *output, uint16_t len){
	HEATINGCTRLDEBUG ("main\n");
  // enter your code here

  return ECMD_FINAL_OK;
}

/*
 * This is an implementation of a PID (proportional, integral, derivative) controller
 *
 */
int16_t
pid_controller(int16_t tTarget, int16_t tMeasured){
	return (tTarget-tMeasured) + 127;
}


/*
  -- Ethersex META --
  header(services/heating_controller/heating_ctrl.h)
  ifdef(`conf_HEATING_CTRL_INIT_AUTOSTART',`init(heating_ctrl_init)')
  ifdef(`conf_HEATING_CTRL_PERIODIC_AUTOSTART',`timer(1000,heating_ctrl_periodic())')
*/
