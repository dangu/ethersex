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
#include "core/bit-macros.h"

#include "hardware/onewire/onewire.h"
#include "hardware/pwm/pwm.h"

#include "protocols/ecmd/ecmd-base.h"

static int16_t periodicCounter = 0;

static pidData_t pidDataRoom, pidDataRad;
static sensor_data_t sensors[N_SENSORS];

/*
  If enabled in menuconfig, this function is called during boot up of ethersex
 */
void
heating_ctrl_init(void)
{
  HEATINGCTRLDEBUG("init\n");

  // Init room temperature controller parameter
  pidDataRoom.I = 0;
  pidDataRoom.Igain = 1;
  pidDataRoom.Pgain = 1;
  pidDataRoom.u = 20;
  pidDataRoom.uMax = 70;
  pidDataRoom.uMin = 15;

  // Init radiator temperature controller parameter
  pidDataRad.I = 0;
  pidDataRad.Igain = 1;
  pidDataRad.Pgain = 1;
  pidDataRad.u = 0;
  pidDataRad.uMax = 255;
  pidDataRad.uMin = 0;

  /* Sensor rom init
   * "a":["105602a501080011",
   * "Radiatorer, tillopp"],
   * "tIn":["28dbfa7102000051",
   * "Inomhus, nere"],
   * "d":["10d136a5010800e5",
   * "Ventilation, uteluft"],
   */
  sensors[SENSOR_T_IN].rom.raw = 0x5100000271fadb28;
  sensors[SENSOR_T_RAD].rom.raw = 0x11000801a5025610;
  sensors[SENSOR_T_OUT].rom.raw = 0xe5000801a536d110;

}

/*
  If enabled in menuconfig, this function is periodically called
  change "timer(1000,heating_ctrl_periodic)" if needed
*/

void
heating_ctrl_periodic(void)
{


  HEATINGCTRLDEBUG("Counter: %d\n", periodicCounter++);
  heating_ctrl_controller();


}

int16_t
get_sensor(sensor_data_t * sensor)
{

  int8_t ret;
  ow_rom_code_t *romPtr;

  romPtr = &sensor->rom;

  //HEATINGCTRLDEBUG("*romPtr 0x%x%x%x\n", romPtr->bytewise[0],
  //                 romPtr->bytewise[1], romPtr->bytewise[2]);

  ret = ow_temp_start_convert_wait(romPtr);
  //HEATINGCTRLDEBUG("conv %d\n", ret);

  ow_temp_scratchpad_t sp;
  ret = ow_temp_read_scratchpad(romPtr, &sp);

  if (ret != 1)
  {
    HEATINGCTRLDEBUG("scratchpad read failed: %d\n", ret);
  }
  else
  {
    //HEATINGCTRLDEBUG("successfully read scratchpad\n");


      sensor->signal = ow_temp_normalize(romPtr, &sp)>>4;

    //HEATINGCTRLDEBUG("temperature: %d.%d\n", HI8(temp),
    //                 HI8(((temp & 0x00ff) * 10) + 0x80));


  }


  return 0;
}

int16_t
heating_ctrl_controller(void)
{
  static int16_t tTargetIndoor=20<<4, tTargetRad=40<<4;    // [degC*16] Target temperatures
  int16_t uShunt;

  uint16_t i;

  int16_t ret;

  // Read all sensors
  for (i = 0; i < N_SENSORS; i++)
      ret = get_sensor(&sensors[i]);


  uShunt = pid_controller(&pidDataRad, tTargetRad, &sensors[SENSOR_T_RAD]);

  HEATINGCTRLDEBUG("uShunt: %d\n", uShunt);

  setpwm('b',(uint8_t) uShunt);




  return ECMD_FINAL_OK;
}

/*
  This function will be called on request by menuconfig, if wanted...
  You need to enable ECMD_SUPPORT for this.
  Otherwise you can use this function for anything you like
 */
int16_t
heating_ctrl_onrequest(char *cmd, char *output, uint16_t len)
{
  HEATINGCTRLDEBUG("main\n");
  heating_ctrl_periodic();
  // enter your code here

  return ECMD_FINAL_OK;
}

/*
 * This is an implementation of a PID (proportional, integral, derivative) controller
 *
 */
int16_t
pid_controller(pidData_t *pPtr, int16_t tTarget, sensor_data_t *sensorPtr)
{

//         self.I = self.I + self.I_GAIN*err*dt
  int16_t e,P, u, uLim;

  e = tTarget-sensorPtr->signal;
  HEATINGCTRLDEBUG("PID: e=%d-%d=%d sens>>4=%d ", tTarget,sensorPtr->signal,e,sensorPtr->signal>>4);

  P = e*pPtr->Pgain;
  ctrl_printf("P=%d ", P);

  pPtr->I = pPtr->I + e*pPtr->Igain;
  ctrl_printf("I=%d ", pPtr->I);

  u = (P + pPtr->I)>>4;
  ctrl_printf("u=%d ", u);

  if(u > pPtr->uMax)
    {
      uLim = pPtr->uMax;
    }
  else if(u < pPtr->uMin)
    {
      uLim =pPtr->uMin;
    }
  else
    {
      uLim = u;
    }

  pPtr->I = pPtr->I - ((u - uLim)<<4);
  ctrl_printf("Ilim=%d\n", pPtr->I);





  return uLim;
}

/*
  -- Ethersex META --
  header(services/heating_controller/heating_ctrl.h)
  init(heating_ctrl_init)
  timer(500,heating_ctrl_periodic())
 */
