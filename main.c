/* MSPAC - AC phase control via MSP430, by Boris Gjenero */

#include "io430.h"

#define ZC_CCTL (CCIS_1 | SCS | CAP | CCIE)

#define DIMTAB_BITS ***

__cc_version2 unsigned short mult(unsigned short a, unsigned short b);

static unsigned short hperiod;
static unsigned short zerocross;
static unsigned short dim = 0;

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
            if (--jiffies == 0) {
                jiffies=60;
                __bic_SR_register_on_exit(LPM4_bits);
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

const unsigned short dimtab[] = {
    11112, /* 0.030303: 30.520476 */
    12282, /* 0.040404: 33.733675 */
    13282, /* 0.050505: 36.481652 */
    14167, /* 0.060606: 38.912668 */
    14968, /* 0.070707: 41.111692 */
    15704, /* 0.080808: 43.132507 */
    16388, /* 0.090909: 45.011527 */
    17030, /* 0.101010: 46.774753 */
    17637, /* 0.111111: 48.441431 */
    18214, /* 0.121212: 50.026272 */
    18765, /* 0.131313: 51.540803 */
    19294, /* 0.141414: 52.994249 */
    19804, /* 0.151515: 54.394124 */
    20296, /* 0.161616: 55.746640 */
    20774, /* 0.171717: 57.057003 */
    21237, /* 0.181818: 58.329626 */
    21688, /* 0.191919: 59.568286 */
    22128, /* 0.202020: 60.776251 */
    22557, /* 0.212121: 61.956367 */
    22978, /* 0.222222: 63.111133 */
    23390, /* 0.232323: 64.242764 */
    23794, /* 0.242424: 65.353218 */
    24191, /* 0.252525: 66.444261 */
    24582, /* 0.262626: 67.517481 */
    24967, /* 0.272727: 68.574310 */
    25346, /* 0.282828: 69.616056 */
    25720, /* 0.292929: 70.643909 */
    26090, /* 0.303030: 71.658965 */
    26455, /* 0.313131: 72.662234 */
    26816, /* 0.323232: 73.654650 */
    27174, /* 0.333333: 74.637083 */
    27528, /* 0.343434: 75.610343 */
    27880, /* 0.353535: 76.575192 */
    28228, /* 0.363636: 77.532345 */
    28574, /* 0.373737: 78.482477 */
    28918, /* 0.383838: 79.426230 */
    29259, /* 0.393939: 80.364214 */
    29599, /* 0.404040: 81.297012 */
    29937, /* 0.414141: 82.225188 */
    30273, /* 0.424242: 83.149266 */
    30608, /* 0.434343: 84.069779 */
    30942, /* 0.444444: 84.987230 */
    31276, /* 0.454545: 85.902111 */
    31608, /* 0.464646: 86.814903 */
    31940, /* 0.474747: 87.726079 */
    32271, /* 0.484848: 88.636106 */
    32602, /* 0.494949: 89.545445 */
    32933, /* 0.505051: 90.454555 */
    33264, /* 0.515152: 91.363894 */
    33595, /* 0.525253: 92.273921 */
    33927, /* 0.535354: 93.185097 */
    34259, /* 0.545455: 94.097889 */
    34593, /* 0.555556: 95.012770 */
    34927, /* 0.565657: 95.930221 */
    35262, /* 0.575758: 96.850734 */
    35598, /* 0.585859: 97.774812 */
    35936, /* 0.595960: 98.702988 */
    36276, /* 0.606061: 99.635786 */
    36617, /* 0.616162: 100.573770 */
    36961, /* 0.626263: 101.517523 */
    37307, /* 0.636364: 102.467655 */
    37655, /* 0.646465: 103.424808 */
    38007, /* 0.656566: 104.389657 */
    38361, /* 0.666667: 105.362917 */
    38719, /* 0.676768: 106.345350 */
    39080, /* 0.686869: 107.337766 */
    39445, /* 0.696970: 108.341035 */
    39815, /* 0.707071: 109.356091 */
    40189, /* 0.717172: 110.383944 */
    40568, /* 0.727273: 111.425690 */
    40953, /* 0.737374: 112.482519 */
    41344, /* 0.747475: 113.555739 */
    41741, /* 0.757576: 114.646782 */
    42145, /* 0.767677: 115.757236 */
    42557, /* 0.777778: 116.888867 */
    42978, /* 0.787879: 118.043633 */
    43407, /* 0.797980: 119.223749 */
    43847, /* 0.808081: 120.431714 */
    44298, /* 0.818182: 121.670374 */
    44761, /* 0.828283: 122.942997 */
    45239, /* 0.838384: 124.253360 */
    45731, /* 0.848485: 125.605876 */
    46241, /* 0.858586: 127.005751 */
    46770, /* 0.868687: 128.459197 */
    47321, /* 0.878788: 129.973728 */
    47898, /* 0.888889: 131.558569 */
    48505, /* 0.898990: 133.225247 */
    49147, /* 0.909091: 134.988473 */
    49831, /* 0.919192: 136.867493 */
    50567, /* 0.929293: 138.888308 */
    51368, /* 0.939394: 141.087332 */
    52253, /* 0.949495: 143.518348 */
    53253, /* 0.959596: 146.266325 */
    54423, /* 0.969697: 149.479524 */
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

    for (i = 0; i < sizeof(dimtab)/sizeof(dimtab[i]); i++) {
        __bis_SR_register(LPM0_bits);
        dim = hperiod - mult(dimtab[i], hperiod);
    }

    while(1) __bis_SR_register(LPM0_bits);

    return 0;
}
