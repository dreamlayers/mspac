/* MSPAC - AC phase control via MSP430, by Boris Gjenero */

#include "io430.h"

#define ZC_CCTL (CCIS_1 | SCS | CAP | CCIE)

static unsigned short hperiod;
static unsigned short zerocross;
static unsigned short dim = 0;

#pragma vector = TIMERA1_VECTOR
__interrupt void Timer_A1_ISR(void)
{
    static char zcmode = 0;
    static unsigned short t1, t2;
    static unsigned short peak;

    (void)TAIV; // Clear interrupt

    // Note that optocoupler activation pulls pin low
    switch (__even_in_range(zcmode, 6)) {
    case 0: // Falling edge detected
            t1 = TACCR1;
            // Set up debounce delay
            TACCR1 = t1 + 1000;
            TACCTL1 = CCIE;
            break;
    case 2: // End of rising edge debounce delay
            // Prepare for rising edge
            TACCTL1 = CM_1 | ZC_CCTL;
            break;
    case 4: // Rising edge detected
            t2 = TACCR1;
            // Set up debounce delay
            // Must be long enough to end in the next half cycle
            TACCR1 = t2 + 4000;
            TACCTL1 = CCIE;
            break;
    case 6: // End of falling edge debounce delay
            // Prepare for falling edge
            TACCTL1 = CM_2 | ZC_CCTL;

            // Calculate
            t2 -= t1; // Length of optocoupler activation
            t1 += t2 >> 1; // Time of peak
            hperiod = (t1 - peak) >> 1; // Half-cycle length
            peak = t1;
            t2 = hperiod >> 1; // Quarter-cycle length
            t1 += t2; // Time of previous zero crossing

            // Update zero crossing time if needed
            if (dim > 0) {
                unsigned short delta = zerocross - t1;
                zerocross = t1;
                if (delta > t2) delta = -delta;
                if (delta > t2) {
                    zerocross += hperiod;
                    TACCR0 = zerocross + dim;
                    TACCTL0 = OUTMOD_1 | CCIE;
                }
            }

            // Back to mode 0;
            zcmode = 0xFE;
            break;
    }
    zcmode += 2;
}

#pragma vector = TIMERA0_VECTOR
__interrupt void Timer_A0_ISR(void)
{
    // Set up next
    zerocross += hperiod;
    TACCR0 = zerocross + dim;

    TACCTL0 = OUTMOD_0 | CCIE; // Turn off TRIAC driver
    TACCTL0 = OUTMOD_1 | CCIE; // Enable for next cycle
}

int main( void )
{
    // Stop watchdog timer to prevent time out reset
    WDTCTL = WDTPW + WDTHOLD;

    // Set DCOCLK to 1 MHz.
    DCOCTL = 0x00;          // Errata BCL12
    BCSCTL1 = CALBC1_1MHZ;
    DCOCTL = CALDCO_1MHZ;

    // TA0: SMCLK/1, continuous
    TACTL = TASSEL_2 | MC_2 | TACLR;

    // TA0.1: Monitoring zero crossing
    // Capture on falling edge
    TACCTL1 = CM_2 | ZC_CCTL;

    // TA0.0: Driving TRIAC
    TACCTL0 = 0;

    // P1.5 = TRIAC driver, P1.6 = zero crossing detector
    P1OUT = 0;
    P1DIR = 0x20;
    P1SEL = 0x60;

    __enable_interrupt();

    while(1);

    return 0;
}
