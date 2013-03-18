/* MSPAC - AC phase control via MSP430, by Boris Gjenero */

#include <stdbool.h>
#include "io430.h"

/*** Configuration constants ***/

// Speed for on-off dimming
#define DIMSPEED 200
// Speed for triggered slow brightness increase
#define TRIGDIMSPEED 1
// Target for triggered slow brightness increase
#define TRIGDIMTARGET 0xFFFF
// Button and trigger debounce length, in terms of 1/60s
#define DEBOUNCE_LEN 5

/*** Other defines ***/

// States
#define STATE_OFF 0       // Turned off, ignoring trigger
#define STATE_TRIGWAIT 1  // Awaiting trigger
#define STATE_TRIGGERED 2 // Was triggered
#define STATE_ON 3        // On and controlled by potentiometer
#define STATE_INITIAL 4   // Initial state after reset

// Port pin usage
#define P1_SW_OFF 1
#define P1_LED 2
#define P1_UNUSED 4 // P1.2 = 4 reserved in case serial is used
#define P1_SW_ON 8
#define P1_TRIGGER 0x10
#define P1_TRIAC 0x20
#define P1_ZEROCROSS 0x40
#define P1_POTCH 7 // Channel for ADC10
#define P1_POT (1 << P1_POTCH)

// Register values
// Zero crossing detector:
#define ZC_CCTL (CCIS_1 | SCS | CAP | CCIE)
// P1_POTCH channel, ADC10SC, binary, sample and hold not inverted
// divider zero, SMCLK,
#define ADC10CTL1_VAL ((P1_POTCH << 12) | ADC10SSEL_3)
// VCC to VSS, sample for 4*ADC10CLKs, not set for low sampling rate,
// reference off, single sample, ADC10 on
#define ADC10CTL0_VAL (ADC10ON)

__cc_version2 unsigned short mult(unsigned short a, unsigned short b);

/*** Global variables ***/

static unsigned char state = STATE_INITIAL; // Current operating mode

// Variables for phase control based on timer A cycles
static char zcmode;                   // Zero crossing detector state
static unsigned short hperiod;        // Half of AC period
static unsigned short zerocross;      // TAR value for zero crossing
static unsigned short triacdelay = 0; // Delay after zero crossing
                                      // Zero disables TRIAC driver

// Variables for linear dimming
static unsigned short dimpower = 0;  // Set by fading code in ISR, used by
                                     // main code to calculate triacdelay.
static bool updatedim;               // Set by ISR when dimpower updated,
                                     // used to ensure requested value is set.
static unsigned short dimtarget = 0; // Target for current fade
static unsigned short dimdelta = 0;  // Light output change per AC cycle

// Variables for user input
static unsigned short potavg = 0;           // Averaged potentiometer value
static unsigned char debctr = DEBOUNCE_LEN; // Switch debounce counter
                                            // Set to DEBOUNCE_LEN after bounce
                                            // Decremented once per AC cycle
                                            // until zero when debouncing ends
static unsigned char inputval = 0xFF;       // Previous input, for debouncing

/*** State descriptors ***/

// Map from state to dimming target value
static const unsigned s2dimtarg[] = { 0, 0, TRIGDIMTARGET, 0 };
// Map from state to dimming step
static const unsigned short s2dimstep[] = { 0xFFFF-DIMSPEED+1,
                                            0,
                                            1,
                                            DIMSPEED };
// Map from state to which port 1 interupts are enabled
static const unsigned char s2p1ie[] = { P1_SW_ON | P1_SW_OFF,
                                        P1_SW_ON | P1_SW_OFF | P1_TRIGGER,
                                        P1_SW_ON | P1_SW_OFF,
                                        P1_SW_ON | P1_SW_OFF };
// Map from state to which port 1 interrupts are falling edge
// Assuming switch has pullup and shorts to ground
// Assuming trigger has pullup and shorts to ground
static const unsigned char s2p1ies[] = { P1_SW_ON,
                                         P1_SW_OFF | P1_SW_ON | P1_TRIGGER,
                                         P1_SW_OFF | P1_SW_ON,
                                         P1_SW_OFF };

/*** TACCR1 ISR, for zero crossing detection and other periodic work ***/
#pragma vector = TIMERA1_VECTOR
__interrupt void TACCR1_ISR(void)
{
    static unsigned short t1, t2;
    static unsigned short peak;
    static bool adc10start;

    (void)TAIV; // Clear interrupt

    // Note that optocoupler activation pulls pin low
    switch (__even_in_range(zcmode, 6)) {
    case 0: // Falling edge detected
            t1 = TACCR1;
            // Set up debounce delay
            TACCR1 = t1 + 1000;
            TACCTL1 = CCIE;
            break;
    case 2: // End of falling edge debounce delay
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
    case 6: // End of rising edge debounce delay
            // Prepare for falling edge
            TACCTL1 = CM_2 | ZC_CCTL;

            /* Other periodic work can happen here because time is available.
             * The next falling edge is in the next half cycle. */

            /*** Calculate half-period and zero crossing time ***/
            t2 -= t1; // Length of optocoupler activation
            t1 += t2 >> 1; // Time of peak
            hperiod = (t1 - peak) >> 1; // Half-cycle length
            peak = t1;
            t2 = hperiod >> 1; // Quarter-cycle length
            t1 += t2; // Time of previous zero crossing

            // Update zero crossing time if needed
            if (triacdelay > 0) {
                unsigned short delta;

                /* Turn on indicator LED so it truly indicates TRIAC is active.
                 * This will indicate if any bug is wasting power by
                 * keeping the lamp on at very low levels. */
                P1OUT |= P1_LED;

                delta = zerocross - t1;
                zerocross = t1;
                if (delta > t2) delta = -delta;
                if (delta > t2 || (TACCTL0&CCIE) == 0) {
                    /* This is used in two cases:
                     * - Dimming ISR already set up next interrupt based on
                     *   old zero crossing time. Update it based on new time.
                     * - TRIAC interrupt is disabled. It is enabled. */
                    zerocross += hperiod;
                    TACCR0 = zerocross + triacdelay;
                    TACCTL0 = OUTMOD_1 | CCIE;
                }
            }

            /*** Read ADC ***/
            // Done before input debouncing so first reading after ADC
            // power-up is delayed by one AC cycle
            if (state == STATE_ON) {
                if (ADC10CTL0 & ADC10IFG) {
                    unsigned short adjustedavg;

                    if (adc10start) {
                        // First value not averaged
                        potavg = ADC10MEM << 6;
                        adc10start = false;
                    } else {
                        // Read value, and average with old values
                        // potavg = 7/8*potavg + 1/8*adc;
                        // ADC is 10 bits and this results in a 16 bit value
                        potavg = potavg - (potavg >> 3) + (ADC10MEM << 3);
                    } // else !adcstart

                    // Start new conversion
                    ADC10CTL0 = ADC10CTL0_VAL | (ENC | ADC10SC);

                    // Ensure that 0xFFFF can be reached
                    adjustedavg = potavg + 1500;
                    if (adjustedavg < potavg) adjustedavg = 0xFFFF;

                    if (dimdelta != 0) {
                        // Still fading, so pot becomes target
                        dimtarget = adjustedavg;
                    } else {
                        // Simply following pot
                        updatedim = true;
                        dimpower = adjustedavg;
                    } // else dimdelta == 0
                } else {
                    // Start first conversion here after settling
                    ADC10CTL0 = ADC10CTL0_VAL | (ENC | ADC10SC);
                } // else (ADC10CTL0 & ADC10IFG) == 0
            } // state == STATE_ON

            /*** Debounce input ***/
            if (debctr > 0) {
                register unsigned char newinput = P1IN;
                unsigned char p1inmask = P1_SW_ON | P1_SW_OFF;

                if ((newinput & (P1_SW_ON | P1_SW_OFF)) ==
                    (P1_SW_ON | P1_SW_OFF) && state != STATE_TRIGGERED) {
                      // Only care about trigger when it can have an effect
                      p1inmask |= P1_TRIGGER;
                }
                newinput &= p1inmask;

                if (newinput == inputval) {
                    if (--debctr == 0) {
                        // Debounce counter expired
                        unsigned char nextstate;

                        // Figure out new state based on inputs
                        if ((newinput & P1_SW_OFF) == 0) {
                            nextstate = STATE_OFF;
                        } else if ((newinput & P1_SW_ON) == 0) {
                            nextstate = STATE_ON;
                        } else if (state == STATE_TRIGWAIT &&
                                   (newinput & P1_TRIGGER) == 0) {
                            nextstate = STATE_TRIGGERED;
                        } else {
                            nextstate = STATE_TRIGWAIT;
                        }

                        // Enable interrupts to detect leaving of new state
                        P1IFG = 0;
                        P1IES = s2p1ies[nextstate];
                        P1IE = s2p1ie[nextstate];

                        // Re-verify, because P1IN change before interrupt
                        // enabling would have been missed
                        if ((P1IN & p1inmask) == newinput) {
                            // Debouncing finally done

                            if (state != nextstate) {
                                // Set up ADC10 for new state
                                ADC10CTL0 = 0; // Ensure ENC=0
                                if (nextstate == STATE_ON) {
                                    ADC10CTL0 = ADC10CTL0_VAL;
                                    ADC10CTL1 = ADC10CTL1_VAL;
                                    adc10start = true;
                                } else {
                                    // ADC10 not needed
                                    ADC10CTL0 = 0;
                                    adc10start = false;
                                }

                                // Transition to new state
                                state = nextstate;
                                dimtarget = s2dimtarg[nextstate];
                                dimdelta = s2dimstep[nextstate];
                            }

                            // Main thread will, when appropriate
                            // - return to LPM4
                            // - set triacdelay to enable TRIAC driver
                            __bic_SR_register_on_exit(LPM4_bits);
                        } else { // (P1IN & p1inmask) != newinput
                            // Failure, keep debouncing
                            P1IE = 0;
                            debctr = DEBOUNCE_LEN;
                        }
                    } // if (--debctr == 0)
                } else { // newinput != inputval
                    // Still bouncing
                    debctr = DEBOUNCE_LEN;
                    inputval = newinput;
                }
            }

            /*** Update dimming ***/
            if (dimdelta != 0 && !adc10start) {
                register unsigned short newpower = dimpower + dimdelta;

                if ((dimdelta >= 0x8000) ?
                      // Decreasing
                     (newpower > dimpower || newpower < dimtarget)
                    : // Increasing
                     (newpower < dimpower || newpower > dimtarget)) {
                        // End of fade
                        newpower = dimtarget;
                        dimdelta = 0;
                }

                dimpower = newpower;
                updatedim = true;
            } // if (dimdelta != 0 && !adc10start)

            /*** Wake main thread ***/
            // Use of updatedim ensures that last fade value is actually set
            if (updatedim) {
                __bic_SR_register_on_exit(LPM4_bits);
            }

            /*** Back to mode 0 ***/
            zcmode = 0xFE;
            break;
    } // switch (__even_in_range(zcmode, 6)) {

    zcmode += 2;
} // TACCR1_ISR

/*** TACCR0 ISR, for TRIAC triggering ***/
#pragma vector = TIMERA0_VECTOR
__interrupt void TACCR0_ISR(void)
{
    static bool delayoff = false;

    if (triacdelay > 1000 || delayoff) {
        // Set up next
        zerocross += hperiod;
        TACCR0 = zerocross + triacdelay;

        TACCTL0 = OUTMOD_0 | CCIE; // Turn off TRIAC driver
        TACCTL0 = OUTMOD_1 | CCIE; // Enable for next cycle

        delayoff = false;
    } else {
        /* Attempting to turn on the TRIAC too early might fail, because
         * voltage is too low in that part of the AC cycle. Keep the
         * trigger signal active longer, so the TRIAC turns on as soon as
         * voltage is high enough to trigger it. */
        TACCR0 = TAR + 1000;
        delayoff = true;
    }
} // TACCR0_ISR

/*** Port 1 ISR, for user interface and fade triggering ***/
#pragma vector=PORT1_VECTOR
__interrupt void port1_ISR(void) {
    // Disable and clear port interrupts, and let debouncing code handle this
    P1IE = 0;
    P1IFG = 0;
    debctr = DEBOUNCE_LEN;

    // Zero crossing detector is needed for debouncing and figuring out
    // zero crossings before turning on the lamp
    if ((TACCTL1 & CCIE) == 0) {
        // Enable TA0.1 for monitoring zero crossing
        // Capture on falling edge
        zcmode = 0;
        TACCTL1 = CM_2 | ZC_CCTL;
        // Transition from LPM4 to LPM1
        __bic_SR_register_on_exit(LPM4_bits & ~LPM1_bits);
    }
} // port1_ISR

/*** Dimming table, translating desired output power to trigger angle ***/
/* The 16 bit dimming value uses the high order DIMTAB_BITS to select a
 * slot in this table, and the rest of the bits to linearly interpolate
 * within that slot. The last value would correspond to 0x10000, so it is
 * only approached using interpolation from 0xFFFF. */
#define DIMTAB_BITS 5
static const unsigned short dimtab[(1 << DIMTAB_BITS) + 1] = {
    56707, /* 0.124568: 24.247302 */
    55420, /* 0.151925: 27.782507 */
    54194, /* 0.179282: 31.150470 */
    53013, /* 0.206640: 34.392322 */
    51868, /* 0.233997: 37.537146 */
    50751, /* 0.261354: 40.606771 */
    49654, /* 0.288711: 43.618355 */
    48574, /* 0.316069: 46.585897 */
    47505, /* 0.343426: 49.521292 */
    46444, /* 0.370783: 52.434933 */
    45388, /* 0.398140: 55.336210 */
    44333, /* 0.425498: 58.233863 */
    43276, /* 0.452855: 61.136266 */
    42215, /* 0.480212: 64.051672 */
    41146, /* 0.507569: 66.988419 */
    40065, /* 0.534927: 69.955189 */
    38971, /* 0.562284: 72.961213 */
    37859, /* 0.589641: 76.016556 */
    36724, /* 0.616998: 79.132443 */
    35563, /* 0.644356: 82.321685 */
    34370, /* 0.671713: 85.599198 */
    33138, /* 0.699070: 88.982812 */
    31859, /* 0.726427: 92.494287 */
    30524, /* 0.753785: 96.160909 */
    29120, /* 0.781142: 100.017907 */
    27629, /* 0.808499: 104.112323 */
    26028, /* 0.835856: 108.509643 */
    24282, /* 0.863214: 113.305916 */
    22336, /* 0.890571: 118.652257 */
    20093, /* 0.917928: 124.811447 */
    17360, /* 0.945285: 132.319079 */
    13597, /* 0.972643: 142.654819 */
    183, /* 1.000000: 179.496261 */
};

int main( void )
{
    // Stop watchdog timer to prevent time out reset
    WDTCTL = WDTPW + WDTHOLD;

    // Set up DCO
    DCOCTL = 0x00;          // Errata BCL12
    BCSCTL1 = CALBC1_1MHZ;
    // DCOx is 1 more than 1Mhz calibrated value
    // MODx is 0 to prevent jitter
    DCOCTL = (CALDCO_1MHZ & (DCO0|DCO1|DCO2)) + DCO0;

    /*** Set up ports ***/
    P1OUT = P1_SW_ON | P1_SW_OFF | P1_TRIGGER;
    P1DIR = P1_TRIAC | P1_LED;
    // Trigger uses external pullup
    P1REN = P1_SW_ON | P1_SW_OFF | P1_TRIGGER | P1_UNUSED;
    P1SEL = P1_TRIAC | P1_ZEROCROSS;
    ADC10AE0 = P1_POT;

    // Port 2 as GPIO because XIN and XOUT are unused
    P2SEL = 0;
    P2OUT = 0;
    P2REN = 0xFF;

    // RST/NMI needs 47 kohm pullup with 10 nF (or for 2.2 nF SBW) pulldown
    // TEST can remain open

    /*** Set up timer A ***/
    // TA0: SMCLK/1, continuous
    TACTL = TASSEL_2 | MC_2 | TACLR;
    // Enable zero crossing detector to determine initial state
    TACCTL1 = CM_2 | ZC_CCTL;

    __enable_interrupt();

    /*** Main loop ***/
    while (1) {
        static unsigned short curdimpower = 0;

        if (state > STATE_TRIGWAIT || curdimpower != 0 || debctr != 0) {
            // Lit or figuring out next state
            // Wait here until new dimming value is available
            __bis_SR_register(LPM0_bits);
        } else {
            // Unlit, waiting for trigger or switch

            // LED off here, ensuring it can't remain off while TRIAC is on
            P1OUT &= ~P1_LED;

            // Disable timer interrupts.
            // Only port interrupts can exit this state.
            TACCTL0 = OUTMOD_0; // Also turn off TRIAC driver
            TACCTL1 = 0;

            // TRIAC stays off until new delay is calculated
            triacdelay = 0;

            // Wait here until lamp needs to be lit
            __bis_SR_register(LPM4_bits);

            // Zero crossing detector is turned on by port 1 ISR

            // TRIAC driver is turned on by zero crossing detector ISR
        }

        // Don't inadvertently turn on TRIAC
        if (state > STATE_TRIGWAIT || curdimpower != 0) {
            /*** Convert linearized dimming power to TRIAC delay */
            // Avoid glitches if dimpower is changed by interrupt
            __disable_interrupt();
            curdimpower = dimpower;
            updatedim = false;
            __enable_interrupt();

            // Find table index
            unsigned short dimidx = curdimpower >> (16 - DIMTAB_BITS);
            // Linearly interpolate and scale based on current period
            triacdelay = mult(dimtab[dimidx] -
                              mult(dimtab[dimidx]-dimtab[dimidx+1],
                                   curdimpower << DIMTAB_BITS),
                              hperiod);
        }
    } // while(1)
} // main()
