#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define MAXPOWER 1.570796327
#define MAXERROR 0.0000001

inline double angle2power(double angle)
{
    return 0.5*angle - 0.25*sin(angle*2.0);
}

inline double angle2powerslope(double angle)
{
    double s = sin(angle);
    return s * s;
}

double power2angle(double power)
{
    double err, estim = M_PI/2;

    while (1) {
        err = angle2power(estim) - power;
        if (err <= MAXERROR && err >= -MAXERROR) break;
        estim -= err/angle2powerslope(estim);
        //printf(">> %f, %f\n", err, estim);
    }
    return estim;
}


int main(int argc, char **argv)
{
    int i, s;

    if (argc != 2) return -1;
    s = atoi(argv[1]);

    for (i = 0; i < s; i++) {
        /* This contains stupid tweaks to try to fix the lower end so there isn't a range
         * on the pot where the lamp is basically off, but it doesn't work very well because
         * pots are non-linear.
         */
        double angle, power = ((double)i / (s - 1 + 0.55338)) + 0.545/(s-1);
        angle = power2angle(power*power*MAXPOWER);
        printf("    %.0f, /* %f: %f */\n", 65535.0-angle*65535.0/M_PI, power, angle/M_PI*180);
    }
}
