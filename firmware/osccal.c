//  osccal.c
//  PowerMeter - an open source AVR based power meter
//
// Copyright (c) 2009 - 2011 Jon Escombe
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

//
// Based on code by Dean Camera, as posted to avrfreaks.net
// http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=36237
//
// Enhancements;
//   Only start from preset value on first iteration - move value out of function
//   Use incremental adjustments
//   Don't keep going, use threshold value for 'close enough'
//   Can't assume loopcount, need to track min/max values (and maybe iterations)?
//

#include "osccal.h"

// Variables used for debugging output only
extern volatile uint8_t debug_osccal_iterations, debug_prev_osccal_iterations;
extern volatile uint16_t debug_osccal_tcnt;

void OSCCAL_Calibrate(void)
{
    // Maximum range is 128, and we start from the centre for initial calibration,
    // so 64 is the max number of iterations possible. We are starting from the 
    // previous OSCCAL value on subsequent iterations, so should only take a 
    // couple of iterations to calibrate. We check for min/max allowable values,
    // and leaving LoopCount in will ensure we don't end up with an endless loop
    // if we don't match the threshold (which obviously should never happen).
    uint8_t LoopCount = (0x7F / 2);

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        debug_prev_osccal_iterations = debug_osccal_iterations;
        debug_osccal_iterations = 0;
    
        // Make sure all clock division is turned off (8MHz RC clock)
        CLKPR  = (1 << CLKPCE);
        CLKPR  = 0x00;
   
        // Inital OSCCAL of half its maximum
        // OSCCAL = (0x7F / 2);
      
        // Stop the timers
        TCCR1B = 0x00;
        TCCR2B = 0x00;

        // Disable timer interrupts
        TIMSK1 = 0;
        TIMSK2 = 0;
         
        // Set timer 2 to asyncronous mode (32.768KHz crystal)
        ASSR  |= (1 << AS2);
   
        // Ensure timer control registers A are cleared
        TCCR1A = 0;
        TCCR2A = 0;
   
        // Start both counters with no prescaling
        TCCR1B = (1 << CS10);
        TCCR2B = (1 << CS20);

        // Wait until timer 2's external 32.768KHz crystal is stable
        while (ASSR & ((1 << TCN2UB) | (1 << TCR2AUB) | (1 << TCR2BUB) | (1 << OCR2AUB) | (1 << OCR2BUB)));
      
        // Clear the timer values
        TCNT1  = 0;
        TCNT2  = 0;

        while (LoopCount--)
        {
            ++debug_osccal_iterations;

            // Wait until timer 2 overflows
            while (!(TIFR2 & (1 << TOV2)));
      
            // Stop timer 1 so it can be read
            TCCR1B = 0x00;
         
            // Check timer value against ideal constant
            if (TCNT1 > (OSCCAL_TARGETCOUNT + OSCCAL_THRESHOLD))      // Clock is running too fast
            {
                if (--OSCCAL== 0)
                    LoopCount = 0;
            }
            else if (TCNT1 < (OSCCAL_TARGETCOUNT - OSCCAL_THRESHOLD)) // Clock is running too slow
            {
                if (++ OSCCAL == 0x7F)
                    LoopCount = 0;
            }
            else
                LoopCount = 0;

            debug_osccal_tcnt = TCNT1;
         
            // Clear timer 2 overflow flag
            TIFR2 |= (1 << TOV2);
   
            // Restart timer 1
            TCCR1B = (1 << CS10);
   
            // Clear the timer values
            TCNT1  = 0;
            TCNT2  = 0;
        }
   
        // Stop the timers
        TCCR1B = 0x00;
        TCCR2B = 0x00;

        // Turn off timer 2 asynchronous mode
        // ASSR  &= ~(1 << AS2);
    }   
    END_ATOMIC_BLOCK
   
    return;
}
