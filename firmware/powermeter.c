//
// 8.0 MHz internal oscillator, clocked to 7.3728MHz by calibration routine
// 32.768 kHz watch crystal - async timer2 & oscillator calibration
// timer2 - 5 sec - periodic wakeup (clocked from 32.768 crystal)
//
// PORT B1 - input  - radio sleep status (CTS)
// PORT B2 - output - radio sleep select (DTR)
// PORT C0 - input  - ADC0 battery voltage (via voltage divider)
// PORT D0 - input  - USART RXD
// PORT D1 - output - USART TXD
// PORT D2 - input  - INT0 - phototransistor pulse counter
// PORT D6 - output - switch LED (debug only)
// PORT D7 - output - timer LED (debug only)

#include <stdio.h>                              // Note: big hit for using sprintf (1.5K)
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "osccal.h"

#define SETBIT(ADDRESS,BIT) (ADDRESS |= (1<<BIT))
#define CLEARBIT(ADDRESS,BIT) (ADDRESS &= ~(1<<BIT))
#define FLIPBIT(ADDRESS,BIT) (ADDRESS ^= (1<<BIT))
#define CHECKBIT(ADDRESS,BIT) (ADDRESS & (1<<BIT)) 
#define nop() asm volatile("nop")

#define TICK_INTERVAL 5                         // timer 2 fires every 5 seconds
#define RADIO_INTERVAL 300                      // send stats every 300 seconds (5 mins)
#define MAX_TICKS (RADIO_INTERVAL / TICK_INTERVAL)
#define USART_BAUDRATE 115200
#define USART_BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)
#define ADC_ITERATIONS 5                        // number of ADC readings to average

#define DEBUG_ENABLED (!(PIND & (1 << 3)))		// Use D3/INT1 for debug enable - jumper wire to ground pin

// The following values will need to be changed for different installations
#define METER_NAME "PWR02"
#define METER_PULSE_PER_KWH 800

volatile uint8_t  timer_tick, compute_and_send;
volatile uint16_t counter;

volatile uint8_t  debug_osccal_iterations, debug_prev_osccal_iterations ;
volatile uint16_t debug_osccal_tcnt;
volatile uint16_t debug_adc_value[ADC_ITERATIONS];

void GetADCValue(uint16_t* battery)
{
    uint16_t batteryADC = 0;
    uint8_t  adc_count;

    ADCSRA |= (1 << ADEN);                      // Enable the ADC

    for (adc_count = 0; adc_count < ADC_ITERATIONS; adc_count++)
    {
        ADCSRA |= (1 << ADSC);                  // Start ADC conversion
        while(ADCSRA & (1<<ADSC));              // Wait for conversion to finish

        debug_adc_value[adc_count] = ADC;
        batteryADC += debug_adc_value[adc_count];
    }
    batteryADC = batteryADC / ADC_ITERATIONS;

    ADCSRA &= ~(1 << ADEN);                     // Disable the ADC

    // 0 = 0v, 1023 = 3.0v ADC reference
    //
    // Expecting battery voltage up to ~5v, so use 680k:470k voltage divider
    //  giving us 0 = 0v, 1023 = 5.10v, therefore v = ADC / 200.4

    *battery = (batteryADC * 100UL) / 200;
}

void ADCSetup(void)
{
    ADMUX |= (1<<REFS0);                        // Use AVcc with external capacitor at AREF pin

	ADMUX  &= ~(1 << MUX0);						// Clear MUX bits for channel 0 - battery voltage
	ADMUX  &= ~(1 << MUX1);						//
	ADMUX  &= ~(1 << MUX2);						//
	ADMUX  &= ~(1 << MUX3);						//

    ADCSRA |= (1<<ADPS2);                       // ADC clock prescaler = F_CPU/16
                                                // 7.3728 MHz/32 = 115kHz
}

void ExternalIntSetup(void)
{
    EICRA |= (1 << ISC01);                      // Interrupt on falling edge
                                                //  - note: this should NOT work for waking from
                                                //    power save sleep - but does ????
    EIMSK |= (1 << INT0);                       // Enable INT0 - Pin D2
}

void USART_Setup(void)
{
   UCSR0B |= (1 << TXEN0) | (1 << RXEN0);       // Turn on the transmit / receive circuitry
   UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01);     // Use 8-bit character sizes (default - skip?)
   UCSR0C |= (1 << USBS0);                      // Use two stop bits to work around XBee series 1 bug

   UBRR0L = USART_BAUD_PRESCALE;                // Load lower 8-bits of the baud rate register
   UBRR0H = (USART_BAUD_PRESCALE >> 8);         // Load upper 8-bits of the baud rate register
}

void USART_SendChar(char ByteToSend)
{
      while ((UCSR0A & (1 << UDRE0)) == 0) {};  // Wait until UDR is ready for more data
      UDR0 = ByteToSend;                        // Write the current byte
}

void USART_SendString(char* StringPtr)
{
   while (*StringPtr)
   {
      USART_SendChar(*StringPtr);
      StringPtr++;
   }

   // wait for transmit to complete before returning.
   //  - else we were powering down too quickly
   while (!(UCSR0A & (1 << TXC0)));

   // reset the transmit completion flag
   UCSR0A |= (1 << TXC0);
} 

void TimerSetup(void)
{
    //
    // Setup timer 2 -- used for periodic wakeup, supports wake from sleep mode
    //

    // Disable timer 2 interrupts
    TIMSK2 = 0;

    // Reset timer 2 counter (perhaps do this after stablised?)
    TCNT2  = 0;

    // Use external 32.768kHz clock instead of a crystal (STK500)
    // - NOW SET IN MAIN (for here and osccal routine)
    // ASSR |= (1 << EXCLK);

    // Set timer 2 to asyncronous mode (32.768KHz crystal)
    //  - should already be in async mode from osccal routine 
    //    but do we want to rely on this here?
    ASSR |= (1 << AS2);

    // Start timer 2 at external clock/1024
    TCCR2B |= (1 << CS20) | (1 << CS21) | (1 << CS22);        

    // Wait for TCN2xUB, OCR2xUB, and TCR2xUB.
    while ((ASSR & ((1 << TCN2UB)  |
                    (1 << OCR2AUB) |
                    (1 << OCR2BUB) |
                    (1 << TCR2AUB) |
                    (1 << TCR2BUB))));
 
    OCR2A   = 0x9F;                             // Set CTC compare value 
                                                // - 5 sec @ 32.768 kHz clock, with prescaler of 1024
    TCCR2A |= (1 << WGM21);                     // Configure timer 2 for CTC mode

    // Clear timer 2 output compare interrupt flag to prevent interrupt 
    // firing immediately when CTC interrupt is enabled following 
    // calibration routine timer overflows...
    TIFR2 |= (1 << OCF2A);

    // Enable CTC interrupt
    TIMSK2 |= (1 << OCIE2A);
}

// Convert a (max) three digit unsigned int to a text string with n.nn formatting
//
void UnsignedToDecimalString3(uint16_t input, char * output_string)
{
    char temp_string[6];

    utoa(input, temp_string, 10);

    switch (strlen(temp_string))
    {
        case 3:
            *output_string++ = temp_string[0];
            *output_string++ = '.';
            *output_string++ = temp_string[1];
            *output_string++ = temp_string[2];
            *output_string++ = 0x0;
            break;

        case 2:
            *output_string++ = '0';
            *output_string++ = '.';
            *output_string++ = temp_string[0];
            *output_string++ = temp_string[1];
            *output_string++ = 0x0;
            break;

        case 1:
            *output_string++ = '0';
            *output_string++ = '.';
            *output_string++ = '0';
            *output_string++ = temp_string[0];
            *output_string++ = 0x0;
            break;

        default:
            *output_string = 0x0;
    }
}

// Convert a (max) five digit unsigned int to a text string with (n)n.nnn formatting
//  - doing it this way to avoid floating point sprintf() overhead
void UnsignedToDecimalString5(uint16_t input, char * output_string)
{
    char temp_string[6];

    utoa(input, temp_string, 10);

    switch (strlen(temp_string))
    {
        case 5:
            *output_string++ = temp_string[0];
            *output_string++ = temp_string[1];
            *output_string++ = '.';
            *output_string++ = temp_string[2];
            *output_string++ = temp_string[3];
            *output_string++ = temp_string[4];
            *output_string++ = 0x0;
            break;

        case 4:
            *output_string++ = temp_string[0];
            *output_string++ = '.';
            *output_string++ = temp_string[1];
            *output_string++ = temp_string[2];
            *output_string++ = temp_string[3];
            *output_string++ = 0x0;
            break;

        case 3:
            *output_string++ = '0';
            *output_string++ = '.';
            *output_string++ = temp_string[0];
            *output_string++ = temp_string[1];
            *output_string++ = temp_string[2];
            *output_string++ = 0x0;
            break;

        case 2:
            *output_string++ = '0';
            *output_string++ = '.';
            *output_string++ = '0';
            *output_string++ = temp_string[0];
            *output_string++ = temp_string[1];
            *output_string++ = 0x0;
            break;

        case 1:
            *output_string++ = '0';
            *output_string++ = '.';
            *output_string++ = '0';
            *output_string++ = '0';
            *output_string++ = temp_string[0];
            *output_string++ = 0x0;
            break;

        default:
            *output_string = 0x0;
    }
}

void EnableRadio(int enable)
{
    if (enable)
    {
        // Power up the Xbee module
        PORTB &= ~(1 << 2);                     // De-assert port B2 to power radio on

        // Wait for module to be ready (CTS)
        while((PINB & (1<<1)));                 // Wait until port B4 (CTS) goes low
    }
    else
    {
        // Power down the Xbee
        PORTB |= (1 << 2);                      // Assert port B2 to power radio down
    }
}

void PinConfig(void)
{
	// Set output pins
    DDRB |= (1 << 2);                           // Set port B2 as output for radio sleep select
    DDRD |= (1 << 6);                           // Set port D6 as output for debug LED
    DDRD |= (1 << 7);                           // Set port D7 as output for debug LED

    // Set initial output pin states
    PORTB |= (1 << 2);                          // Assert port B2 to enable XBee sleep

    // Enable pullup resistors for all unused pins
    PORTB |= (1 << 0);                          // Enable pullup resistor on port B0
    PORTB |= (1 << 1);                          // Enable pullup resistor on port B1
    PORTB |= (1 << 3);                          // Enable pullup resistor on port B3
    PORTB |= (1 << 4);                          // Enable pullup resistor on port B4
    PORTB |= (1 << 5);                          // Enable pullup resistor on port B5

    PORTC |= (1 << 1);                          // Enable pullup resistor on port C1
    PORTC |= (1 << 2);                          // Enable pullup resistor on port C2
    PORTC |= (1 << 3);                          // Enable pullup resistor on port C3
    PORTC |= (1 << 4);                          // Enable pullup resistor on port C4
    PORTC |= (1 << 5);                          // Enable pullup resistor on port C5
    PORTC |= (1 << 6);                          // Enable pullup resistor on port C6
    PORTC |= (1 << 7);                          // Enable pullup resistor on port C7

    PORTD |= (1 << 3);                          // Enable pullup resistor on port D3 ** Pulled down for debug enable
    PORTD |= (1 << 4);                          // Enable pullup resistor on port D4
    PORTD |= (1 << 5);                          // Enable pullup resistor on port D5
}

ISR(INT0_vect)
{
        PORTD ^= (1 << 6);                      // Toggle the debug LED on port D6
        counter++;                              // increment our pulse counter
}

ISR(TIMER2_COMPA_vect)
{
    PORTD ^= (1 << 7);                          // Toggle the debug LED on port D7

    // If we have reached data transmission interval, 
    // or if port D6 is pulled low (for debug output)
    if ((++timer_tick == MAX_TICKS) || DEBUG_ENABLED)
    {
        timer_tick = 0;
        compute_and_send = 1;
    }
} 

int main (void)
{
    char Buffer[64];
    char str_batteryV[5], str_kWh[7];

    uint16_t total_count, hourly_rate, Wh;
    uint16_t batteryV;
    uint8_t  adc_count;

    //
    // STK500 configuration - not required for standalone design
    //
    // PORTD |= (1 << 2);                       // Enable pullup resistor on port D2
    // ASSR |= (1 << EXCLK);                    // Use external 32.768kHz clock instead of a crystal

    PinConfig();
    ExternalIntSetup();
    USART_Setup();
    ADCSetup();

    // Initialise OSCCAL to centre point of it's range before initial calibration
    OSCCAL = (0x7F / 2);

    // Calibrate the internal oscillator using the 32.768 KHz crystal
    OSCCAL_Calibrate();

    // Set up the timer for periodic wakeup
    TimerSetup();

    sei();                                      // Enable global interrupts

    set_sleep_mode(SLEEP_MODE_PWR_SAVE);        // Set POWER SAVE sleep mode, IDLE or PWR_SAVE
                                                //  - Can only use SLEEP_MODE_PWR_SAVE if timer 2 is 
                                                //    clocked asynchronously (using a watch crystal)
                                                //  - also INT0 is only supposed to work as a level triggered
                                                //    interrupt. However, it appears to be running fine as 
                                                //    edge triggered with the 88PA, but NOT the original 88 ???
                                                //  - should probably switch to using a pin change interrupt
                                                //    (PCINT18 is on the same pin), and checking the pin state
                                                //     is low in the ISR, so that we only react to falling event
    for (;;) // Loop forever
    {
        if (compute_and_send)
        {
            // copy globals into local variables & reset the counters
            total_count = counter;
            counter = compute_and_send = 0;

            // Calibrate the internal oscillator for reliable serial comms (resets timer 2)
            OSCCAL_Calibrate();

            // Restart timer2
            //  - Note: We lose a bit of time here with the recalibration, but have
            //    optimised the recalibration routine - so minimal impact
            TimerSetup();

            // Calculate the hourly pulse rate, and convert into watt hours
            // Then format the value as a kWh string, accurate to three decimal places
            hourly_rate = (total_count * (3600 / RADIO_INTERVAL));
            Wh = ((hourly_rate * 1000UL) / METER_PULSE_PER_KWH);
            UnsignedToDecimalString5(Wh, str_kWh);

            // Enable the radio link
            // - moved before ADC routine to get battery observations under load?
            //
            EnableRadio(1);

            // Get the battery voltage (actually voltage after schottky diode)
            // - do we need more of a delay to let reading stabilise under load?
            GetADCValue(&batteryV);

            //
            // Format the strings as three digits including two decimal places..
            UnsignedToDecimalString3(batteryV, str_batteryV);

            // Write the data to the serial port
            //
            if (!(DEBUG_ENABLED))
            {
                //
                // Normal output
                //
                sprintf(Buffer, "%s,%s,%s,%i\n", METER_NAME, str_kWh, str_batteryV, OSCCAL);
                USART_SendString(Buffer);
            }
            else
            {
                //
                // Debug output
                //  - can't use str_kWh, as interval is different
                //
                sprintf(Buffer, "OSCCAL TCNT %u, calibration steps: %i, previously %i\n", debug_osccal_tcnt, debug_osccal_iterations, debug_prev_osccal_iterations);
                USART_SendString(Buffer);

                sprintf(Buffer, "OSCCAL: %i\n", OSCCAL);
                USART_SendString(Buffer);

                sprintf(Buffer, "Pulses: %i\n", total_count);
                USART_SendString(Buffer);

                for (adc_count = 0; adc_count < ADC_ITERATIONS; adc_count++)
                {
                    sprintf(Buffer, "ADC value #%i: %i\n", adc_count, debug_adc_value[adc_count]);
                    USART_SendString(Buffer);
                }

                sprintf(Buffer, "Battery voltage: %sv\n", str_batteryV);
                USART_SendString(Buffer);
            }

            // Disable the radio link
            //
            EnableRadio(0);
        }

        // Sleeping too quickly after waking from aysnc clock causes issues, so re-write
        // the CTC value and wait for OCR2AUB to clear before re-entering sleep mode
        OCR2A = 0x9F;
        while (ASSR & (1 << OCR2AUB));

        // Sleep until next interrupt
        sleep_mode();
    }
    return (0);
}
