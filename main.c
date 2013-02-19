/* MSPAC - AC phase control via MSP430, by Boris Gjenero */

#include "io430.h"

#define ZC_CCTL (CCIS_1 | SCS | CAP | CCIE)

#define DIMTAB_BITS 4

__cc_version2 unsigned short mult(unsigned short a, unsigned short b);

static unsigned short hperiod;
static unsigned short zerocross;
static unsigned short dim = 0;
static unsigned short dimpower;

#pragma vector = TIMERA1_VECTOR
__interrupt void Timer_A1_ISR(void)
{
    static char zcmode = 0;
    static unsigned short t1, t2;
    static unsigned short peak;
    static unsigned char jiffies = 60;

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

            // Wake main thread
            //if (--jiffies == 0) {
            //    jiffies=6;
            if (dimpower < 0xFFFF) {
                dimpower++;
                __bic_SR_register_on_exit(LPM4_bits);
            }
            //}

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

const unsigned short dimtab[(1 << DIMTAB_BITS) + 1] = {
    5112, /* 0.055556: 14.041953 */
    8166, /* 0.111111: 22.429249 */
    10782, /* 0.166667: 29.615139 */
    13178, /* 0.222222: 36.193965 */
    15445, /* 0.277778: 42.420748 */
    17637, /* 0.333333: 48.441431 */
    19790, /* 0.388889: 54.355904 */
    21933, /* 0.444444: 60.242977 */
    24093, /* 0.500000: 66.173231 */
    26293, /* 0.555556: 72.217730 */
    28565, /* 0.611111: 78.456174 */
    30942, /* 0.666667: 84.987230 */
    33476, /* 0.722222: 91.945192 */
    36238, /* 0.777778: 99.531896 */
    39353, /* 0.833333: 108.089150 */
    43072, /* 0.888889: 118.303607 */
    48080, /* 0.944444: 132.058542 */
};

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

    int i;

    for (i = 0; i < 5; i++) {
        __bis_SR_register(LPM0_bits);
    }

    dimpower = 0;
    while (1) {
        __bis_SR_register(LPM0_bits);
        // Avoid glitches if dimpower is changed by interrupt
        unsigned short curdimpower = dimpower;
        // Find table index
        unsigned short dimidx = curdimpower >> (16 - DIMTAB_BITS);
        // Linearly interpolate and scale based on current period
        dim = hperiod - mult(dimtab[dimidx] +
                             mult(dimtab[dimidx+1]-dimtab[dimidx],
                                  curdimpower << DIMTAB_BITS),
                             hperiod);
    }

    //while(1) __bis_SR_register(LPM0_bits);

    //return 0;
}
