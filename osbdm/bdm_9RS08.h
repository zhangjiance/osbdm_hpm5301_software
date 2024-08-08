/* OSBDM-JM60 Target Interface Software Package
 * Copyright (C) 2009  Freescale
 *
 * This software package is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */


/*****************************************************************************\
*
*	Author:		Axiom Manufacturing
*
*	File:		bdm_9RS08.h
*
*	Purpose: 	header file for bdm_9RS08.c command formatter
*				Supports 9RS08 processors
*
*
*
* 03/08/2011  : Modified by P&E Microcomputer Systems
*               http://www.pemicro.com/osbdm
*
*
\*****************************************************************************/

#ifndef _BDM_9RS08_H_
#define _BDM_9RS08_H_

//----------------------------------------------------------------------------
// Register enumerator for t_*_ad() function calls
//----------------------------------------------------------------------------

enum
{
  BGNDREG_A,
  BGNDREG_CCR,
  BGNDREG_HX,
  BGNDREG_SP,
  BGNDREG_PC
};

//----------------------------------------------------------------------------
// Configuration enumerator for t_config() function calls
//----------------------------------------------------------------------------

typedef enum rs08cfgTag
{
  rs08cfg_enable_vpp_powersupply, // set to 1 to enable the flash programming voltage power supply
                                  // set to 0 to disable the flash programming voltage power supply
  rs08cfg_enable_vpp,             // set to 1 to turn ON the flash programming voltage to the target
                                  // set to 0 to turn OFF the flash programming voltage to the target
  rs08cfg_power_reset_on_connect, // set to 1 to enable Power Reset on connect.
  rs08cfg_count
} rs08cfgType;

//----------------------------------------------------------------------------
// MCU Signal Defines
//----------------------------------------------------------------------------

#define VSW_EN      PTBD_PTBD0
#define VSW_EN_MASK PTBD_PTBD0_MASK
#define VSW_EN_DIR  PTBDD_PTBDD0

#define VTRG_EN      PTBD_PTBD1
#define VTRG_EN_MASK PTBD_PTBD1_MASK
#define VTRG_EN_DIR  PTBDD_PTBDD1

#define tRSTO      PTCD_PTCD4
#define tRSTO_MASK PTCD_PTCD4_MASK
#define tRSTO_DIR  PTCDD_PTCDD4

#define tPWR_LED      PTDD_PTDD0
#define tPWR_LED_MASK PTDD_PTDD0_MASK
#define tPWR_LED_DIR  PTDDD_PTDDD0

#define STATUS_LED      PTDD_PTDD1
#define STATUS_LED_MASK PTDD_PTDD1_MASK
#define STATUS_LED_DIR  PTDDD_PTDDD1

#define tRSTI      PTDD_PTDD2
#define tRSTI_MASK PTDD_PTDD2_MASK
#define tRSTI_DIR  PTDDD_PTDDD2

#define VSW_FAULT      PTGD_PTGD0
#define VSW_FAULT_MASK PTGD_PTGD0_MASK
#define VSW_FAULT_DIR  PTGDD_PTGDD0

#define VTRG_FAULT      PTGD_PTGD1
#define VTRG_FAULT_MASK PTGD_PTGD1_MASK
#define VTRG_FAULT_DIR  PTGDD_PTGDD1


//----------------------------------------------------------------------------
// BDM Driver Defines
//----------------------------------------------------------------------------

#define BDMRXDELAY   (3) // Added delay between TX and RX
#define BDMEXITDELAY (2) // Added delay between RX and exit

#endif // _BDM_9RS08_H_
// --------------------------------- EOF -------------------------------------
