#pragma once
#include <inttypes.h>

class RotaryEnoderSwitch{

enum EncoderType{
    singleStep = 1,
    twoStep = 2,
    fourStep = 4
};


public:
    public:
        RotaryEnoderSwitch(uint8_t pinA, uint8_t pinB, uint8_t pinSwitch,EncoderType type);
    public:
        // should be called in ISR every 1ms via Timer
        void tickDebounceDecode(void); 
        int8_t readEncoder(void);   
    private:
        // read single step encoders
        int8_t encode_read1(void);
        // read two step encoders
        int8_t encode_read2(void);
        // read four step encoders
        int8_t encode_read4(void);
        // called in ISR every 1 ms
        // digital Filter Algorithm debouncer
        // http://web.engr.oregonstate.edu/~traylor/ece473/lectures/debounce.pdf
        void debounce_switch(void);
        void rotarydecode(void);
    private:
       volatile int8_t enc_delta;          // -128 ... 127
       static int8_t last_val;
       uint8_t pinA;
       uint8_t pinB; 
       uint8_t pinSwitch;
       EncoderType type;
       int8_t (*encode)(void);
};        