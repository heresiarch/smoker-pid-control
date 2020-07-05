#pragma once
#include <Fuzzy.h>

class BBQFan {
private:
    Fuzzy* m_fuzzy;
    double &m_currentTemp;
    double &m_setPoint;        // Setpoint
    uint8_t &m_pwmValue;   
    float m_tempLastError;   // Last temperature error input
    float m_tempLast;        // Temperature that is bassed through a filter
    float m_lastTempChange;      // Keep
public:
    BBQFan(double &currentTemp,double &targetTemp, uint8_t &pwmValue);
    ~BBQFan();
    /**
     * Very important, call this once in 5 seconds
     */
    void handle();

    // Fuzzy inputs monitoring
    void init();
};