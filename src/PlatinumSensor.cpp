#include "PlatinumSensor.h"

double PlatinumSensor::tempFromPtResistance(double R, double Rzero){

    const double c0  = -245.19;
    const double c1  = 2.5293;
    const double c2  = -0.066046;
    const double c3  = 0.0040422;
    const double c4  = -0.0000020697;
    const double c5  = -0.025422;
    const double c6 = 0.0016883;
    const double c7 = -0.0000013601;
     R = R * 100.00 / Rzero;
     return c0 + R * (c1 + R * (c2 + R * (c3 + c4 * R))) / (1 + R * (c5 + R * (c6 + c7 * R)));
}
