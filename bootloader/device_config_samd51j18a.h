/* ----------------------------------------------------------------------------
 *         SAM Software Package License
 * ----------------------------------------------------------------------------
 * Copyright (c) 2011-2014, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following condition is met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

#ifndef DEVICE_INCLUDE_H
#define DEVICE_INCLUDE_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "samd51j18a.h"
#include "clks.h"
#include "spi.h"
#include "adc.h"

#if defined ( __ICCARM__ ) /* IAR Ewarm 5.41+ */
    #include <iosamd21.h>
    #define barrier()                   __DMB()
    #define COMPILER_PRAGMA(arg)        _Pragma(#arg)
    #define COMPILER_ALIGNED(a)         COMPILER_PRAGMA(data_alignment = a)
    #define COMPILER_WORD_ALIGNED       COMPILER_PRAGMA(data_alignment = 4)
#elif defined (  __GNUC__  ) /* GCC CS3 2009q3-68 */
    #include "sam.h"
    #define COMPILER_ALIGNED(a)         __attribute__((__aligned__(a)))
    #define COMPILER_WORD_ALIGNED       __attribute__((__aligned__(4)))
#else
    #error Compiler Macro is unknown
#endif

#define CPU_FREQUENCY                       48000000

#define NVM_USB_PAD_TRANSN_POS              45
#define NVM_USB_PAD_TRANSN_SIZE             5
#define NVM_USB_PAD_TRANSP_POS              50
#define NVM_USB_PAD_TRANSP_SIZE             5
#define NVM_USB_PAD_TRIM_POS                55
#define NVM_USB_PAD_TRIM_SIZE               3

#define NVM_SW_CALIB_DFLL48M_COARSE_VAL     58
#define NVM_SW_CALIB_DFLL48M_FINE_VAL       64

static inline void enable_usb_digital_interface_clock(void)
{
  Mclk *pmclk = MCLK;

  pmclk->AHBMASK.bit.USB_ = 1;
  pmclk->APBBMASK.bit.USB_ = 1;
}

#define GCLK_USB 10
static inline void clock_configuration_for_usb(void)
{
    Gclk *pgclk = GCLK;
    Port *pport = PORT;
    Oscctrl *posc = OSCCTRL;

    pport->Group[0].PMUX[12].reg = 0x77; //PA24, PA25, function column H for USB D-, D+
    pport->Group[0].PINCFG[24].bit.PMUXEN = 1;
    pport->Group[0].PINCFG[25].bit.PMUXEN = 1;
    pport->Group[1].PMUX[11].bit.PMUXE = 7; //PB22, function column H for USB SOF_1KHz output
    pport->Group[1].PINCFG[22].bit.PMUXEN = 1;

    posc->DFLLCTRLA.bit.ENABLE=0;
    while(posc->DFLLSYNC.bit.ENABLE) {}
    while(posc->DFLLSYNC.bit.DFLLCTRLB) {}
    posc->DFLLCTRLB.bit.USBCRM = 1;
    while(posc->DFLLSYNC.bit.DFLLCTRLB) {}
    posc->DFLLCTRLB.bit.MODE = 1;
    while(posc->DFLLSYNC.bit.DFLLCTRLB) {}
    posc->DFLLCTRLB.bit.QLDIS = 0;
    while(posc->DFLLSYNC.bit.DFLLCTRLB) {}
    posc->DFLLCTRLB.bit.CCDIS = 1;
    posc->DFLLMUL.bit.MUL = 0xBB80; //4800 x 1KHz
    while(posc->DFLLSYNC.bit.DFLLMUL) {}
    posc->DFLLCTRLA.bit.ENABLE=1;
    while(posc->DFLLSYNC.bit.ENABLE) {}

    pgclk->PCHCTRL[GCLK_USB].bit.GEN = 0;
    pgclk->PCHCTRL[GCLK_USB].bit.CHEN = 1;
}

static inline void configure_usb_port_pins(void)
{
    Port *pport = PORT;

    pport->Group[0].PMUX[12].reg = 0x77; //PA24, PA25, function column H for USB D-, D+
    pport->Group[0].PINCFG[24].bit.PMUXEN = 1;
    pport->Group[0].PINCFG[25].bit.PMUXEN = 1;
    pport->Group[1].PMUX[11].bit.PMUXE = 7; //PB22, function column H for USB SOF_1KHz output
    pport->Group[1].PINCFG[22].bit.PMUXEN = 1;
}

static inline void load_usb_pin_pad_calibration_values(void)
{
    uint32_t pad_transn, pad_transp, pad_trim;

    pad_transn = (USB_FUSES_TRANSN_ADDR >>  USB_FUSES_TRANSN_Pos) & USB_FUSES_TRANSN_Msk;
    if (pad_transn == 0x1F) {
        pad_transn = 5;
    }

    USB->DEVICE.PADCAL.bit.TRANSN = pad_transn;

    pad_transp = (USB_FUSES_TRANSP_ADDR >>  USB_FUSES_TRANSP_Pos) & USB_FUSES_TRANSP_Msk;
    if (pad_transp == 0x1F) {
        pad_transp = 29;
    }

    USB->DEVICE.PADCAL.bit.TRANSP = pad_transp;

    pad_trim = (USB_FUSES_TRIM_ADDR >>  USB_FUSES_TRIM_Pos) & USB_FUSES_TRIM_Msk;
    if (pad_trim == 0x7) {
        pad_trim = 3;
    }

    USB->DEVICE.PADCAL.bit.TRIM = pad_trim;
}

#endif //DEVICE_INCLUDE_H

