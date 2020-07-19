#include "bbqfan.h"


BBQFan::BBQFan(const double &currentTemp, const double &targetTemp, uint8_t &pwmValue):
    m_currentTemp(currentTemp), m_setPoint(targetTemp), m_pwmValue(pwmValue){
       
    m_tempLast = m_currentTemp;
    m_tempLastError = 0.0;
    m_pwmValue = pwmValue;
    m_fuzzy = new Fuzzy();
}

BBQFan::~BBQFan() {
}


void BBQFan::init() {
    delete m_fuzzy;
    m_fuzzy = new Fuzzy();
    // Create input for Temperature errors
    FuzzyInput* tempDiff = new FuzzyInput(1);
    m_fuzzy->addFuzzyInput(tempDiff);
    FuzzySet* cold = new FuzzySet(-120, -30, -30, -20);
    FuzzySet* warm = new FuzzySet(-25, -5, -5, 0);
    FuzzySet* ideal = new FuzzySet(-7, 0, 0, 1);
    FuzzySet* hot = new FuzzySet(-3,0.5,0.5,10);
    tempDiff->addFuzzySet(cold);
    tempDiff->addFuzzySet(warm);
    tempDiff->addFuzzySet(ideal);
    tempDiff->addFuzzySet(hot);
    m_fuzzy->addFuzzyInput(tempDiff);
    
    FuzzyOutput* pwm = new FuzzyOutput(1);
    FuzzySet* lowSpeed = new FuzzySet(0,5,5,25);
    FuzzySet* mediumSpeed = new FuzzySet(10,45,45,90);
    FuzzySet* highSpeed = new FuzzySet(60,128,128,200);
    FuzzySet* veryhighSpeed = new FuzzySet(190,200,200,255);
    pwm->addFuzzySet(lowSpeed);
    pwm->addFuzzySet(mediumSpeed);
    pwm->addFuzzySet(highSpeed);
    pwm->addFuzzySet(veryhighSpeed);
    m_fuzzy->addFuzzyOutput(pwm);

    // Rules
    // Cold 
    FuzzyRuleAntecedent* ifTemperatureCold = new FuzzyRuleAntecedent(); 
    ifTemperatureCold->joinSingle(cold);
    FuzzyRuleConsequent* thenVeryHighSpeed = new FuzzyRuleConsequent();
    thenVeryHighSpeed->addOutput(veryhighSpeed);
    FuzzyRule* fuzzyRule01 = new FuzzyRule(1, ifTemperatureCold, thenVeryHighSpeed); 
    m_fuzzy->addFuzzyRule(fuzzyRule01);

    // Warm
    FuzzyRuleAntecedent* ifTemperatureWarm = new FuzzyRuleAntecedent(); 
    ifTemperatureWarm->joinSingle(warm);
    FuzzyRuleConsequent* thenHighSpeed = new FuzzyRuleConsequent(); 
    thenHighSpeed->addOutput(highSpeed);
    FuzzyRule* fuzzyRule02 = new FuzzyRule(2, ifTemperatureWarm, thenHighSpeed);
    m_fuzzy->addFuzzyRule(fuzzyRule02);

    // Ideal
    FuzzyRuleAntecedent* ifTemperatureIdeal = new FuzzyRuleAntecedent(); 
    ifTemperatureIdeal->joinSingle(ideal);
    FuzzyRuleConsequent* thenMediumSpeed = new FuzzyRuleConsequent();
    thenMediumSpeed->addOutput(mediumSpeed);
    FuzzyRule* fuzzyRule03 = new FuzzyRule(3, ifTemperatureIdeal, thenMediumSpeed);
    m_fuzzy->addFuzzyRule(fuzzyRule03);

    // Hot
    FuzzyRuleAntecedent* ifTemperatureHot = new FuzzyRuleAntecedent(); 
    ifTemperatureHot->joinSingle(hot);
    FuzzyRuleConsequent* thenLowSpeed = new FuzzyRuleConsequent(); 
    thenLowSpeed->addOutput(lowSpeed);
    FuzzyRule* fuzzyRule04 = new FuzzyRule(4, ifTemperatureHot, thenLowSpeed); 
    m_fuzzy->addFuzzyRule(fuzzyRule04);
}

void BBQFan::handle() {
    m_fuzzy->setInput(1, m_currentTemp - m_setPoint);
    m_fuzzy->fuzzify();
    m_pwmValue = m_fuzzy->defuzzify(1);
    if(m_pwmValue < 30)
        m_pwmValue = 0;
}
