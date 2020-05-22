#pragma once
/*
Formulas and constants for platinum temperature sensor calculation using Polynoms
see also 
http://www.theremino.com/wp-content/uploads/files/Theremino_ADC24_ENG.pdf
http://www.mosaic-industries.com/embedded-systems/microcontroller-projects/temperature-measurement/platinum-rtd-sensors/resistance-calibration-table
*/

class PlatinumSensor {

    public:
    // for a PT1000 Element it RZero will be 1000 for a PT100 100
    static float tempFromPtResistance(float R, float Rzero); 
};