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

#include "protocols/ecmd/ecmd-base.h"

/*
  If enabled in menuconfig, this function is called during boot up of ethersex
*/
int16_t
heating_ctrl_init(void)
{
  APPSAMPLEDEBUG ("init\n");
  // enter your code here

  return ECMD_FINAL_OK;
}

/*
  If enabled in menuconfig, this function is periodically called
  change "timer(100,app_sample_periodic)" if needed
*/
int16_t
heating_ctrl_periodic(void)
{
  APPSAMPLEDEBUG ("periodic\n");
  // enter your code here

  return ECMD_FINAL_OK;
}

/*
  This function will be called on request by menuconfig, if wanted...
  You need to enable ECMD_SUPPORT for this.
  Otherwise you can use this function for anything you like
*/
int16_t
heating_ctrl_onrequest(char *cmd, char *output, uint16_t len){
  APPSAMPLEDEBUG ("main\n");
  // enter your code here

  return ECMD_FINAL_OK;
}

/*
  -- Ethersex META --
  header(services/heating_ctrl/heating_ctrl.h)
  ifdef(`conf_HEATING_CTRL_INIT_AUTOSTART',`init(heating_ctrl_init)')
  ifdef(`conf_HEATING_CTRL_PERIODIC_AUTOSTART',`timer(100,heating_ctrl_periodic())')
*/
