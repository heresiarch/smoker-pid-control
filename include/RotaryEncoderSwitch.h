#pragma once
#include <inttypes.h>


enum EncoderType{
    singleStep = 1,
    twoStep = 2,
    fourStep = 4
};

class RotaryEnoderSwitch{

public:
    public:
        RotaryEnoderSwitch(uint8_t pinA, uint8_t pinB,EncoderType type);
    public:
        // call in your loop function
        void tickDebounceDecode(void); 
        int8_t readEncoder(void);
        // if returns true it will clear the button up state
        bool buttonUpEvent(void);
        // if returns true it will clear the long press state
        bool buttonLongPressEvent(void);
    private:
        // https://www.mikrocontroller.net/topic/436244
        // https://tech.alpsalpine.com/prod/e/html/encoder/incremental/ec12e/ec12e_list.html
        // Enoder Type = EC12 = 15 pulse / 30 dentet * 4 = 2 step encoder    
        // read single step encoders
        int8_t encode_read1(void);
        // read two step encoders
        int8_t encode_read2(void);
        // read four step encoders
        int8_t encode_read4(void);
    private:
       volatile int8_t enc_delta;          // -128 ... 127
       int8_t last_val;
       uint8_t pinA;
       uint8_t pinB; 
       EncoderType type;
};        