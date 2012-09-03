/*
 * Copyright (c) 2009 by Stefan Riepenhausen <rhn@gmx.net>
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

#ifndef HAVE_HEATINGCTRL_H
#define HAVE_HEATINGCTRL_H

int16_t
heating_ctrl_onrequest(char *cmd, char *output, uint16_t len);

int16_t
heating_ctrl_init(void);

void
heating_ctrl_periodic(void);

int16_t
heating_ctrl_controller(void);

int16_t
pid_controller(int16_t tTarget, int16_t tMeasured);

#include "config.h"
#ifdef DEBUG_HEATING_CTRL
# include "core/debug.h"
# define HEATINGCTRLDEBUG(a...)  debug_printf("heating ctrl: " a)
//#define HEATINGCTRLDEBUG(s, args...) printf_P(PSTR("D: " s), ## args)

#else
# define HEATINGCTRLDEBUG(a...)
#endif

#endif  /* HAVE_HEATINGCTRL_H */
