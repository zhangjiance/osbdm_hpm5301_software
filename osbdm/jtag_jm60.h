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


#ifndef _JTAG_JM60_H_
#define _JTAG_JM60_H_

#define JTAG_OK    0
#define JTAG_ERROR 1

#define DATA_REGISTER        0
#define INSTRUCTION_REGISTER 1

//----------------------------------------------------------------------------
// MCU Signal Defines
#define VSW_EN      (*(unsigned char *)0) // PTBD_PTBD0
#define VSW_EN_MASK (*(unsigned char *)0) // PTBD_PTBD0_MASK
#define VSW_EN_DIR  (*(unsigned char *)0) // PTBDD_PTBDD0

#define VTRG_EN      (*(unsigned char *)0) // PTBD_PTBD1
#define VTRG_EN_MASK (*(unsigned char *)0) // PTBD_PTBD1_MASK
#define VTRG_EN_DIR  (*(unsigned char *)0) // PTBDD_PTBDD1

#define TCLK_CTL      (*(unsigned char *)0) // PTBD_PTBD2
#define TCLK_CTL_MASK (*(unsigned char *)0) // PTBD_PTBD2_MASK
#define TCLK_CTL_DIR  (*(unsigned char *)0) // PTBDD_PTBDD2

#define BRK_TMS      (*(unsigned char *)0) // PTBD_PTBD3
#define BRK_TMS_MASK (*(unsigned char *)0) // PTBD_PTBD3_MASK
#define BRK_TMS_DIR  (*(unsigned char *)0) // PTBDD_PTBDD3

#define TA_OUT      (*(unsigned char *)0) // PTBD_PTBD4
#define TA_OUT_MASK (*(unsigned char *)0) // PTBD_PTBD4_MASK
#define TA_OUT_DIR  (*(unsigned char *)0) // PTBDD_PTBDD4

#define VTRG_IN      (*(unsigned char *)0) // PTBD_PTBD5 // ADC input to monitor VTRG_IN
#define VTRG_IN_MASK (*(unsigned char *)0) // PTBD_PTBD5_MASK
#define VTRG_IN_DIR  (*(unsigned char *)0) // PTBDD_PTBDD5

#define P4_IN      (*(unsigned char *)0) // PTCD_PTCD0
#define P4_IN_MASK (*(unsigned char *)0) // PTCD_PTCD0_MASK
#define P4_IN_DIR  (*(unsigned char *)0) // PTCDD_PTCDD0

#define P5_IN      (*(unsigned char *)0) // PTCD_PTCD1
#define P5_IN_MASK (*(unsigned char *)0) // PTCD_PTCD1_MASK
#define P5_IN_DIR  (*(unsigned char *)0) // PTCDD_PTCDD1

#define P6_IN      (*(unsigned char *)0) // PTCD_PTCD2
#define P6_IN_MASK (*(unsigned char *)0) // PTCD_PTCD2_MASK
#define P6_IN_DIR  (*(unsigned char *)0) // PTCDD_PTCDD2

#define P7_DE_IN      (*(unsigned char *)0) // PTCD_PTCD3
#define P7_DE_IN_MASK (*(unsigned char *)0) // PTCD_PTCD3_MASK
#define P7_DE_IN_DIR  (*(unsigned char *)0) // PTCDD_PTCDD3

#define tRSTO      (*(unsigned char *)0) // PTCD_PTCD4
#define tRSTO_MASK (*(unsigned char *)0) // PTCD_PTCD4_MASK
#define tRSTO_DIR  (*(unsigned char *)0) // PTCDD_PTCDD4

#define VPP_ON      (*(unsigned char *)0) // PTCD_PTCD5
#define VPP_ON_MASK (*(unsigned char *)0) // PTCD_PTCD5_MASK
#define VPP_ON_DIR  (*(unsigned char *)0) // PTCDD_PTCDD5

#define tPWR_LED      (*(unsigned char *)0) // PTDD_PTDD0
#define tPWR_LED_MASK (*(unsigned char *)0) // PTDD_PTDD0_MASK
#define tPWR_LED_DIR  (*(unsigned char *)0) // PTDDD_PTDDD0

#define STATUS_LED      (*(unsigned char *)0) // PTDD_PTDD1
#define STATUS_LED_MASK (*(unsigned char *)0) // PTDD_PTDD1_MASK
#define STATUS_LED_DIR  (*(unsigned char *)0) // PTDDD_PTDDD1

#define tRSTI      (*(unsigned char *)0) // PTDD_PTDD2
#define tRSTI_MASK (*(unsigned char *)0) // PTDD_PTDD2_MASK
#define tRSTI_DIR  (*(unsigned char *)0) // PTDDD_PTDDD2

#define TDSCLK_EN      (*(unsigned char *)0) // PTED_PTED2
#define TDSCLK_EN_MASK (*(unsigned char *)0) // PTED_PTED2_MASK
#define TDSCLK_EN_DIR  (*(unsigned char *)0) // PTEDD_PTEDD2

#define TCLK_EN      (*(unsigned char *)0) // PTED_PTED3
#define TCLK_EN_MASK (*(unsigned char *)0) // PTED_PTED3_MASK
#define TCLK_EN_DIR  (*(unsigned char *)0) // PTEDD_PTEDD3

#define DIN      (*(unsigned char *)0) // PTED_PTED4
#define DIN_MASK (*(unsigned char *)0) // PTED_PTED4_MASK
#define DIN_DIR  (*(unsigned char *)0) // PTEDD_PTEDD4

#define DOUT      (*(unsigned char *)0) // PTED_PTED5
#define DOUT_MASK (*(unsigned char *)0) // PTED_PTED5_MASK
#define DOUT_DIR  (*(unsigned char *)0) // PTEDD_PTEDD5

#define SCLK_OUT      (*(unsigned char *)0) // PTED_PTED6
#define SCLK_OUT_MASK (*(unsigned char *)0) // PTED_PTED6_MASK
#define SCLK_OUT_DIR  (*(unsigned char *)0) // PTEDD_PTEDD6

#define OUT_EN      (*(unsigned char *)0) // PTED_PTED7
#define OUT_EN_MASK (*(unsigned char *)0) // PTED_PTED7_MASK
#define OUT_EN_DIR  (*(unsigned char *)0) // PTEDD_PTEDD7

#define VPP_EN      (*(unsigned char *)0) // PTFD_PTFD0
#define VPP_EN_MASK (*(unsigned char *)0) // PTFD_PTFD0_MASK
#define VP_EN_DIR   (*(unsigned char *)0) // PTFDD_PTFDD0

#define VSW_FAULT      (*(unsigned char *)0) // PTGD_PTGD0
#define VSW_FAULT_MASK (*(unsigned char *)0) // PTGD_PTGD0_MASK
#define VSW_FAULT_DIR  (*(unsigned char *)0) // PTGDD_PTGDD0

#define VTRG_FAULT      (*(unsigned char *)0) // PTGD_PTGD1
#define VTRG_FAULT_MASK (*(unsigned char *)0) // PTGD_PTGD1_MASK
#define VTRG_FAULT_DIR  (*(unsigned char *)0) // PTGDD_PTGDD1

#define RTS      (*(unsigned char *)0) // PTGD_PTGD2
#define RTS_MASK (*(unsigned char *)0) // PTGD_PTGD2_MASK
#define RTS_DIR  (*(unsigned char *)0) // PTGDD_PTGDD2

#define CTS      (*(unsigned char *)0) // PTGD_PTGD3
#define CTS_MASK (*(unsigned char *)0) // PTGD_PTGD3_MASK
#define CTS_DIR  (*(unsigned char *)0) // PTGDD_PTGDD3

#define TMS_HIGH 1
#define TMS_LOW  0

// JTAG Signal Macros

#define TRST_SET()   TDSCLK_EN = 1
#define TRST_RESET() TDSCLK_EN = 0

#define TMS_SET()   BRK_TMS = 1
#define TMS_RESET() BRK_TMS = 0

#define TCLK_SET()   TCLK_EN = 1
#define TCLK_RESET() TCLK_EN = 0

#define TDI_OUT_SET()   DOUT = 1
#define TDI_OUT_RESET() DOUT = 0

#define TDO_IN_SET 0 // DIN == 1


typedef enum
{
  TEST_LOGIC_RESET,
  RUN_TEST_IDLE,
  PAUSE_DR,
  PAUSE_IR,
  SHIFT_DR,
  SHIFT_IR,
  UPDATE_DR,
  UPDATE_IR,

  MAX_JTAG_STATE,

} JTAG_STATE_TYPE;


// JTAG functions for Freescale JM60 OSBDM

void          jdly_loop (int i);
char          TCLK_transition (char tms, char tdi);
void          Jtag_ScanIO (char            jtag_register,
                           char            bitcount,
                           unsigned char  *out_data,
                           unsigned char  *in_data,
                           JTAG_STATE_TYPE state);
void          Jtag_ScanIn (char jtag_register, char bitcount, unsigned char *in_data, JTAG_STATE_TYPE state);
void          Jtag_ScanOut (char jtag_register, char bitcount, unsigned char *out_data, JTAG_STATE_TYPE state);
void          Jtag_UlongToScanData (unsigned long ulong_data, unsigned char *scandata);
unsigned long Jtag_ScanDataToUlong (unsigned char *scandata);

#endif
