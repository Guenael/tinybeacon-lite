/*
 * FreeBSD License
 * Copyright (c) 2016, Guenael
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


/* |                                                                   |
   |  TinyBeacon project                                               |
   |                                                                   |
   |  - 10 MHz oscillator stabilized by GPS (GPSDO)                    |
   |  - 4 multiplexed output and 1 mixed output                        |
   |  - Compact design / Credit card size                              |
   |  - DC-DC Power supply within 10-15V, 0.5A max                     |
   |                                                                   |
   |                                                                   |
   |  IO Mapping uController, rev.A                                    |
   |                                                                   |
   |  - PC0      (pin 23) | AN1                                        |
   |  - PC1      (pin 24) | AN2                                        |
   |  - PD0/RXD  (pin 30) | USART RX                                   |
   |  - PD1/TXD  (pin 31) | USART TX                                   |
   |  - PD2      (pin 32) | SYNC                                       |
   |  - PD7      (pin 11) | INFO LED                                   |
   |  - PD6      (pin 10) | PA EN                                      |
   |  - PB0      (pin 12) | PLL LOCK                                   |
   |  - PB2      (pin 14) | PLL_LE (DAC)                               |
   |  - PB3      (pin 15) | PRG_MOSI (DAC)                             |
   |  - PB5      (pin 17) | PRG_SCK (DAC)                              |
   |                                                                   | */


#include "config.h"

#include "pll.h"
#include "usart.h"

#include "morse.h"
#include "pi4.h"
#include "wspr.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


void pi4sequence() {

}


int main (void) {
    /* CKDIV8 fuse is set -- Frequency is divided by 8 at startup : 2.5MHz */
    cli();
    CLKPR = _BV(CLKPCE);  // Enable change of CLKPS bits
    CLKPR = 0;            // Set prescaler to 0 = Restore system clock to 10 MHz
    sei();

    /* LED : Set pin 11 of PORT-PD7 for output*/
    DDRD |= _BV(DDD7);

    /* For now, used for DEBUG purpose only. Future : CLI for freq settings & modes */
    usartInit();
    _delay_ms(10);

    /* Prepare the message to encode for PI4 message */
    pi4Encode();

    /* Prepare the message to encode for WSPR message */
    wsprEncode();

    /* ADF4355 PLL Init, conf & settings */
    pllInit(0x60);  // 0x60 used only for Si5351 (I2C addr.)

    /* End of init sequence : Turn on the LED (pin 11) */
    PORTD |= _BV(PORTD7);

    /* Start with a unsync TX, boring to wait a full sync... */
    //pi4Send();
    //wsprSend();

    // DEBUG
    _delay_ms(30000);  // TODO : TimeAlign
    while(1) {
        pllSetFreq((uint64_t)MORSE_FREQUENCY * 1000000ULL, 0);
        pllUpdate(0);
        _delay_ms(10);

        pllPA(1);
        pllRfOutput(1);
        _delay_ms(10000);
        pllRfOutput(0);
        pllPA(0);
    }

    /* Loop sequence :
       - PI4 + Morse + Tone (1 minute)
       - PI4 + Morse + Tone (1 minute)
       - WSPR (2 minutes)
    */
    while(1) {
        /* 1st part : Send PI4 message, 25 sec */
        pi4Send();

        /* 2nd part : Send morse message */
        morse2TonesSendMessage();

        /* 3th part : Send a carrier, 10 sec, same frequency */
        pllUpdate(1);
        pllPA(1);
        pllRfOutput(1);
        _delay_ms(10000);
        pllRfOutput(0);
        pllPA(0);

        //timeAlignWSPR();
        wsprSend();
    }

    /* This case never happens :) Useless without powermanagement... */
    return 0;
}

/* === Si5351 DEBUG 60sec (2500 x 2 x 12ms)
pllSetFreq(144430000000000,0);
pllSetFreq(144435000000000,1);
pllUpdate(0);
pllRfOutput(1);
pllPA(1);

for (uint32_t i=0; i<2500; i++) { //while(1) {
  pllUpdate(0);
  //pllRfOutput(0);
  //_delay_ms(1000);

  pllUpdate(1);
  //pllRfOutput(1);
  //_delay_ms(1000);
}

pllRfOutput(0);
pllPA(0);
*/

/* === ADF4355 DEBUG 60sec
pllSetFreq(222295000000000,0); 
pllUpdate(0);

pllRfOutput(1);
pllPA(1);
_delay_ms(667);
pllRfOutput(0);
pllPA(0);

for (uint32_t i=0; i<60000; i++) { //while(1) {
  pllUpdate(0);
}

pllRfOutput(1);
pllPA(1);
_delay_ms(667);
pllRfOutput(0);
pllPA(0);
*/

/* TEST 28.8MHz clock 
pllSetFreq(28800000000000,0);
pllUpdate(0);
pllRfOutput(1);
pllPA(1);
while(1) { }
*/