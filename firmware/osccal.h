//  osccal.h
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

#ifndef OSCCAL_H
#define OSCCAL_H

    // INCLUDES:
    #include <avr/io.h>
    #include <avr/interrupt.h>

    // CODE DEFINES:
    #define ATOMIC_BLOCK(exitmode)   { exitmode cli();
    #define END_ATOMIC_BLOCK         }
   
    #define ATOMIC_RESTORESTATE      inline void __irestore(uint8_t *s) { SREG = *s; }         \
                                     uint8_t __isave __attribute__((__cleanup__(__irestore))) = SREG;
    #define ATOMIC_ASSUMEON          inline void __irestore(uint8_t *s) { sei(); *s = 0; }     \
                                     uint8_t __isave __attribute__((__cleanup__(__irestore))) = 0;

    // CONFIG DEFINES:
    #define OSCCAL_TARGETCOUNT       (uint16_t)(F_CPU / (32768 / 256)) // (Target Freq / Reference Freq)   
    #define OSCCAL_THRESHOLD         OSCCAL_TARGETCOUNT / 100          // Aim for within 1% of target count
   
    // PROTOTYPES:
    void OSCCAL_Calibrate(void);
   
#endif 
