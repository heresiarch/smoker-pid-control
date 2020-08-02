#pragma once
#include <Fuzzy.h>

class BBQFan {
private:
    Fuzzy* m_fuzzy;
    const double &m_currentTemp;
    const double &m_setPoint;        // Setpoint
    uint8_t &m_pwmValue;   
    float m_tempLastError;   // Last temperature error input
    float m_tempLast;        // Temperature that is bassed through a filter
    float m_lastTempChange;      // Keep
public:
    BBQFan(const double &currentTemp,const double &targetTemp, uint8_t &pwmValue);
    ~BBQFan();
    /**
     * Very important, call this once in 5 seconds
     */
    void handle();
};