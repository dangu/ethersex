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
#include "core/eeprom.h"
#include "heating_ctrl.h"
#include "core/bit-macros.h"

#include "hardware/onewire/onewire.h"
#include "hardware/pwm/pwm.h"

#include "protocols/ecmd/ecmd-base.h"

static int16_t periodicCounter = 0;
static uint8_t aFilter=20; /* Filter constant */

sensor_data_t sensors[N_SENSORS];

heating_ctrl_params_t heating_ctrl_params_ram;

/*
  If enabled in menuconfig, this function is called during boot up of ethersex
 */
void
heating_ctrl_init(void)
{
  int i;
  int8_t ret;
  HEATINGCTRLDEBUG("init\n");

  /* restore parameters */
  eeprom_restore(heating_ctrl_params, &heating_ctrl_params_ram,
      sizeof(heating_ctrl_params_t));


  /* Sensor rom init
   * "a":["105602a501080011",
   * "Radiatorer, tillopp"],
   * "tIn":["28dbfa7102000051",
   * "Inomhus, nere"],
   * "d":["10d136a5010800e5",
   * "Ventilation, uteluft"],
   */
  sensors[SENSOR_T_ROOM].rom.raw = 0x5100000271fadb28;
  sensors[SENSOR_T_RAD].rom.raw = 0x11000801a5025610;
  sensors[SENSOR_T_OUT].rom.raw = 0xe5000801a536d110;

  // Must init the filter of the sensors
  ret = ow_temp_start_convert_wait(NULL);

  for (i = 0; i < N_SENSORS; i++)
    {
      ret = get_sensor(&sensors[i]);
      sensors[i].signalFilt = sensors[i].signal;
    }
}

/*
 * If enabled in menuconfig, this function is periodically called
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
  int16_t raw_temp; /* The raw temperature value, including 85 degC samples */
  ow_rom_code_t *romPtr;

  romPtr = &sensor->rom;

  ow_temp_scratchpad_t sp;
  ret = ow_temp_read_scratchpad(romPtr, &sp);



  if (ret != 1)
    {
      /* TODO: We need some kind of error handling here */
      HEATINGCTRLDEBUG("scratchpad read failed: %d\n", ret);
    }
  else
    {
      /* TODO: The format of ow_temp_normalize has changed. Now it
       * returns a struct with "twodigits". Don't know what that means.
       */
      ow_temp_t temp = ow_temp_normalize(romPtr, &sp);
      /* If the result has two digits (21.89 deg represented as 2189)
       * Use temp*100 as resolution
       *
       * Strange. The representation of temp.val seems to be 15 bit (as the
       * 16th bit is the twodigits). Must copy the 15th bit to the 16th to
       * get a signed value
       * */
      if(temp.twodigits)
        {
          raw_temp = temp.val|((temp.val<<1)&0x8000);
        }
      else
        {
          raw_temp = 10*(temp.val|((temp.val<<1)&0x8000));
        }

      /* If this sample was 85.0 degC, discard it. It means there was an error
       * reading from the sensor */
      if(raw_temp == 8500)
        {
          HEATINGCTRLDEBUG("Sensor read error! (85 degC)");
        }
      else
        {
          sensor->signal = raw_temp;
          sensor->signalFilt = filter_ewma(sensor->signalFilt, sensor->signal, aFilter);
          HEATINGCTRLDEBUG("Sensor: %d %d %d\n", sensor->signal, sensor->signalFilt, aFilter);
        }
    }
  return 0;
}

/*
 * Return selected internal information about the heating controller
 *
 * Returns degC*100
 */
int16_t
heating_ctrl_info(uint8_t index)
{
  int16_t val;
  switch (index)
  {
  case 0:
    val = heating_ctrl_params_ram.t_target_room;
    break;
  case 1:
    val = sensors[SENSOR_T_ROOM].signalFilt;
    break;
  case 2:
    val = heating_ctrl_params_ram.pid_room.u;
    break;
  case 3:
    val = sensors[SENSOR_T_RAD].signalFilt;
    break;
  case 4:
    val = sensors[SENSOR_T_OUT].signalFilt;
    break;
  default:
    val = 0;
    break;
  }
  return val;
}

int16_t
heating_ctrl_controller(void)
{
  static int16_t tTargetRad;    // Target temperatures
  int16_t uShunt, tRadMaxDynamic;

  uint16_t i;

  int16_t ret;

  // Read all sensors
  ret = ow_temp_start_convert_wait(NULL);

  for (i = 0; i < N_SENSORS; i++)
    ret = get_sensor(&sensors[i]);

//  /*
//   * Do not try to set a radiator temp higher than the maximum
//   * available (especially if the accumulator tanks are empty). This should
//   * prevent windup and 70 deg temperature after the next fire
//   self.pidRoom.uMax = min(self.maxRadtemp, tRadMeasured + self.maxRadtempDiff)
//   */
//  tRadMaxDynamic = sensors[SENSOR_T_RAD].signalFilt + MAX_RADTEMPDIFF;
//  if (MAX_RADTEMP > tRadMaxDynamic)
//    {
//      heating_ctrl_params_ram.pid_room.uMax = tRadMaxDynamic;
//    }
//  else
//    {
//      heating_ctrl_params_ram.pid_room.uMax = MAX_RADTEMP;
//    }

  // PID room temp
  tTargetRad =
      pid_controller(&heating_ctrl_params_ram.pid_room,
          heating_ctrl_params_ram.t_target_room,
          &sensors[SENSOR_T_ROOM]);

  // PID radiator temp
  uShunt =
      pid_controller(&heating_ctrl_params_ram.pid_rad, tTargetRad,
          &sensors[SENSOR_T_RAD]);
  uShunt = uShunt/100;

  HEATINGCTRLDEBUG("uShunt: %d\n", uShunt);

  setpwm('b', (uint8_t) uShunt);

  return ECMD_FINAL_OK;
}



/*
 * This function will be called on request by menuconfig, if wanted...
 * You need to enable ECMD_SUPPORT for this.
 * Otherwise you can use this function for anything you like
 */
int16_t
heating_ctrl_onrequest(char *cmd, char *output, uint16_t len)
{
  int16_t ret = 0;
  uint8_t tTarget;
  uint8_t val;
  HEATINGCTRLDEBUG("onrequest\n");

  //  ret = sscanf_P(cmd, PSTR("%hhu"), &val);
  //  if (ret == 1)
  //    {
  //      // Found a number
  //      if ((0 < val) && (val <= 255))
  //        {
  //          // Sane value
  //          aFilter = val;
  //        }
  //      else
  //        {
  //          return ECMD_ERR_PARSE_ERROR;
  //        }
  //      ret = snprintf_P(output, len, PSTR("Set new filter param to %d"), aFilter);
  //
  //
  //    }
  //  else
  //    {
  //      ret = snprintf_P(output, len, PSTR("%d"), aFilter);
  //    }
  //  return ECMD_FINAL(ret);

  ret = sscanf_P(cmd, PSTR("%hhu"), &tTarget);
  if (ret == 1)
    {
      // Found a number
      if ((100 < tTarget) && (tTarget < 250))
        {
          // Sane value
          heating_ctrl_params_ram.t_target_room = (int16_t) T_RES(tTarget)/10;
          eeprom_save(heating_ctrl_params, &heating_ctrl_params_ram,
              sizeof(heating_ctrl_params_t));
          eeprom_update_chksum();

        }
      else
        {
          return ECMD_ERR_PARSE_ERROR;
        }
      ret = snprintf_P(output, len, PSTR("Set new target to %d degC*10 (10th of degrees)"), tTarget);
    }
  else
    {
      ret = snprintf_P(output, len, PSTR("%d %d %d %d I%d I%d uR%d uS%d"),
          //heating_ctrl_params_ram.t_target_room,
          sensors[SENSOR_T_ROOM].signalFilt,
          sensors[SENSOR_T_OUT].signalFilt,
          sensors[SENSOR_T_RAD].signal,
          sensors[SENSOR_T_RAD].signalFilt,
          (int16_t)(heating_ctrl_params_ram.pid_room.I>>8),
          (int16_t)(heating_ctrl_params_ram.pid_rad.I>>8),
          heating_ctrl_params_ram.pid_room.u,
          heating_ctrl_params_ram.pid_rad.u);
    }
  return ECMD_FINAL(ret);
}

/*
 * This is an implementation of a PID (proportional, integral, derivative) controller
 * (although the derivative part is not there yet)
 *
 */
int16_t
pid_controller(pid_data_t * pPtr, int16_t tTarget, sensor_data_t * sensorPtr)
{

  //         self.I = self.I + self.I_GAIN*err*dt
  int32_t e, P, u;

  e = tTarget - sensorPtr->signalFilt;
  HEATINGCTRLDEBUG("PID: e=%d-%d=%d sens/100=%d ", tTarget, sensorPtr->signalFilt,
      e, sensorPtr->signalFilt/100);

  P = e * pPtr->Pgain;
  ctrl_printf("P=%d ", P);

  pPtr->I = pPtr->I + (e * pPtr->Igain);
  ctrl_printf("I=%d ", (int16_t)(pPtr->I>>8));

  u = (P + (pPtr->I>>8));
  ctrl_printf("u=%d ", (int16_t)u);
  ctrl_printf("uMax=%d ", pPtr->uMax);


  if (u > pPtr->uMax)
    {
      pPtr->u = pPtr->uMax;
    }
  else if (u < pPtr->uMin)
    {
      pPtr->u = pPtr->uMin;
    }
  else
    {
      pPtr->u = (int16_t)u;
    }

  pPtr->I = pPtr->I - ((int32_t)(u - pPtr->u)<<6); /* TODO: This anti-windup was <<8 and should be checked */
  ctrl_printf("Ilim=%d\n", (int16_t)(pPtr->I>>8));

  return pPtr->u;
}

int16_t filter_ewma(int16_t y_1, int16_t x, uint8_t a)
{
  int32_t y;
  y = ((int32_t)y_1*((1<<8)-a) + (int32_t)x*(int32_t)a)>>8;
  return (int16_t)y;
}


/*
  -- Ethersex META --
  header(services/heating_controller/heating_ctrl.h)
  init(heating_ctrl_init)
  timer(500,heating_ctrl_periodic())
 */
