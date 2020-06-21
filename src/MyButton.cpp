#include <Arduino.h>
#include "MyButton.h"

MyButton::MyButton(uint8_t btnPin,bool activeLow, bool pullUp){
    this->btnPin = btnPin;
    this->activeLow = true;
    if(pullUp)
        pinMode(btnPin, INPUT_PULLUP);
    else
        pinMode(btnPin, INPUT);
    if(!activeLow)
        bitMask = B11111110;    

}

void MyButton::debounce(void){
    uint8_t delta;
    uint8_t sample = digitalRead(btnPin);
    sample ^= bitMask;
    delta = sample ^ state;
    cnt1 = (cnt1 ^ cnt0) & (delta & sample);
    cnt0 = ~cnt0 & (delta & sample);
    state ^= (delta & ~(cnt0 | cnt1));
    if(state & (1 << 0))
    {
        long_counter ++;
    }
    else
    {
        long_counter = 0;
        long_state = 0;
    }
    if(long_counter == 1024)
    {
        long_state = 1;
        //state = 0;
        //cnt0 = 0;
        //cnt1 = 0;
    }
    
}

bool MyButton::buttonPressed(void){
    
    return (state & (1<<0));
}

bool MyButton::longPressed(void){
    bool ret = false;
    if(long_state){
        long_state = 0;
        ret = true;
    }
    return ret;
}




