#include "PlatinumSensor.h"

float PlatinumSensor::tempFromPtResistance(float R, float Rzero){

    const float c0  = -245.19;
    const float c1  = 2.5293;
    const float c2  = -0.066046;
    const float c3  = 0.0040422;
    const float c4  = -0.0000020697;
    const float c5  = -0.025422;
    const float c6 = 0.0016883;
    const float c7 = -0.0000013601;
     R = R * 100.00 / Rzero;
     return c0 + R * (c1 + R * (c2 + R * (c3 + c4 * R))) / (1 + R * (c5 + R * (c6 + c7 * R)));
}
