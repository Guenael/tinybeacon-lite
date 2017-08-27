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
   |  - PC2      (pin 32) | SYNC                                       |
   |  - PD0/RXD  (pin 30) | USART RX                                   |
   |  - PD1/TXD  (pin 31) | USART TX                                   |
   |  - PD7      (pin 11) | INFO LED                                   |
   |  - PD6      (pin 10) | PA EN                                      |
   |  - PB0      (pin 12) | PLL LOCK                                   |
   |  - PB2      (pin 14) | PLL_LE                                     |
   |  - PB3      (pin 15) | PRG_MOSI                                   |
   |  - PB5      (pin 17) | PRG_SCK                                    |
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
}


int main (void) {
    /* CKDIV8 fuse is set -- Frequency is divided by 8 at startup : 2.5MHz */
    cli();
    CLKPR = _BV(CLKPCE);  // Enable change of CLKPS bits
    CLKPR = 0;            // Set prescaler to 0 = Restore system clock to 10 MHz
    sei();

    /* LED : Set pin 11 of PORT-PD7 for output + Sync pin */
    DDRD |= _BV(DDD7);
    
    /* SYNC : Set pin 32 of PORT-PC2 as input */
    DDRC &= ~_BV(DDC2);

    /* For now, used for DEBUG purpose only. Future : CLI for freq settings & modes */
    //usartInit();
    //_delay_ms(10);

    /* Prepare the message to encode for PI4 message */
    pi4Encode();

    /* Prepare the message to encode for WSPR message */
    wsprEncode();

    /* ADF4355 PLL Init, conf & settings */
    pllInit();

    /* End of init sequence : Turn on the LED (pin 11) */
    PORTD |= _BV(PD7);

    /* Wait SYNC low (PC2, pin 32) */
    while ( bit_is_clear(PINC, PC2) ) {}

    /* Loop sequence :
       - WSPR (2 minutes)x
       - PI4 + Morse + Tone (1 minute)
       - PI4 + Morse + Tone (1 minute)
    */
    while(1) {
    	/* Wait for an even minute */
    	while ( bit_is_set(PINC, PC2) ) {}
    	/* Send WSPR message */
        wsprSend();		

    	/* Wait for an even minute */
    	while ( bit_is_set(PINC, PC2) ) {}
    	/* Send PI4 message */
		pi4sequence();

        /* Two time! */
    	/* Wait for an odd minute */
    	while ( bit_is_clear(PINC, PC2) ) {}
    	/* Send PI4 message */    	
		pi4sequence();
    }

    /* This case never happens :) Useless without powermanagement... */
    return 0;
}
