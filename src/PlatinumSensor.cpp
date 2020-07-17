#include "PlatinumSensor.h"

#define c0  -245.19
#define c1  2.5293
#define c2  -0.066046
#define c3  0.0040422
#define c4  -0.0000020697
#define c5  -0.025422
#define c6  0.0016883
#define c7 -0.0000013601

float PlatinumSensor::tempFromPtResistance(float R, float Rzero){

     R = R * 100.00 / Rzero;
     return c0 + R * (c1 + R * (c2 + R * (c3 + c4 * R))) / (1 + R * (c5 + R * (c6 + c7 * R)));
}
