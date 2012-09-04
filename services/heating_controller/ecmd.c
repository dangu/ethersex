 /* Copyright(C) 2012 Daniel Gullberg <daniel_gullberg@hotmail.com>
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
  * along with this program; if not, write to the Free Software Foundation,
  * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
  */

#include <avr/io.h>
#include <avr/pgmspace.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>

#include "config.h"
#include "heating_ctrl.h"
#include "protocols/ecmd/ecmd-base.h"

int16_t
parse_cmd_heating_ctrl_command(char *cmd, char *output, uint16_t len)
{
  return heating_ctrl_onrequest(cmd, output, len);
}

int16_t
parse_cmd_heating_ctrl_init(char *cmd, char *output, uint16_t len)
{
  return 0;
}

int16_t
parse_cmd_heating_ctrl_periodic(char *cmd, char *output, uint16_t len)
{
  return 0;
}

/*
-- Ethersex META --
block([[Heating_Controller]])
ecmd_feature(heating_ctrl_command, "heating",, Manually call heating_ctrl commands)
ecmd_feature(heating_ctrl_init, "heating_init",, Manually call heating_ctrl init method)
ecmd_feature(heating_ctrl_periodic, "heating_periodic",, Manually call heating_ctrl periodic method)
*/
