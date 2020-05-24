#pragma once
#include <inttypes.h>

class MyButton{
    public:
        MyButton(uint8_t btnPin,bool activeLow, bool pullUp);
        // call in timer ISR
        void debounce(void);
        bool buttonPressed(void);
        bool longPressed(void);
    private:
        volatile uint8_t state;
        volatile uint8_t cnt0;
        volatile uint8_t cnt1;
        uint8_t btnPin;
        bool activeLow = true;
        uint8_t bitMask = B00000001;
        uint16_t long_counter;
        uint8_t long_state = 0;
};