#pragma once
#include "Arduino.h"
#include "RotaryEncoderSwitch.h"


RotaryEnoderSwitch::RotaryEnoderSwitch(uint8_t pinA, uint8_t pinB, uint8_t pinSwitch,EncoderType type)
{
    this->pinA = pinA;
    this->pinB = pinB;
    this->pinSwitch = pinSwitch;
    this->type = type;

    int8_t newval = 0;
    if( digitalRead(this->pinA) )
        newval = 3;
    if( digitalRead(this->pinB) )
        newval ^= 1;                   // convert gray to binary
    last_val = newval;                   // power on state
    enc_delta = 0;
}

void RotaryEnoderSwitch::tickDebounceDecode(void){
    
    this->rotarydecode();
}

int8_t RotaryEnoderSwitch::readEncoder(void)
{
    uint8_t ret = 0;
    switch(type)
    {
        case EncoderType::singleStep:
            ret = encode_read1();
            break;
        case EncoderType::twoStep:
            ret = encode_read2();
            break;
        case EncoderType::fourStep:
            ret = encode_read4();
            break;
        default:
            break;         
    }
    return ret;
}





void RotaryEnoderSwitch::rotarydecode(void)
{
  int8_t new_val, diff;
  new_val = 0;
  if( digitalRead(this->pinA) )
    new_val = 3;
  if( digitalRead(this->pinA) )
    new_val ^= 1;                   // convert gray to binary
  diff = last_val - new_val;                // difference last - new
  if( diff & 1 ){               // bit 0 = value (1)
    last_val = new_val;                 // store new as next last
    enc_delta += (diff & 2) - 1;        // bit 1 = direction (+/-)
  }
}

int8_t RotaryEnoderSwitch::encode_read1( void )         // read single step encoders
{
  int8_t val;
  val = enc_delta;
  enc_delta = 0;
  return val;                   // counts since last call
}

int8_t RotaryEnoderSwitch::encode_read2( void )         // read two step encoders
{
  int8_t val;
  val = enc_delta;
  enc_delta = val & 1;
  return val >> 1;
}

int8_t RotaryEnoderSwitch::encode_read4( void )         // read four step encoders
{
  int8_t val;
  val = enc_delta;
  enc_delta = val & 3;
  return val >> 2;
}
void RotaryEnoderSwitch::debounce_switch(void){

}