#include "Arduino.h"
#include "RotaryEncoderSwitch.h"


RotaryEnoderSwitch::RotaryEnoderSwitch(uint8_t pinA, uint8_t pinB,EncoderType type)
{
    this->pinA = pinA;
    this->pinB = pinB;
    this->type = type;
    // enable input pins
    pinMode(this->pinA,INPUT);
    pinMode(this->pinB,INPUT);
    
    int8_t newval = 0;
    if( digitalRead(this->pinA) )
        newval = 3;
    if( digitalRead(this->pinB) )
        newval ^= 1;                   // convert gray to binary
    last_val = newval;                   // power on state
    enc_delta = 0;
}

void RotaryEnoderSwitch::tickDebounceDecode(void){
  int8_t new_val, diff;
  new_val = 0;
  if( digitalRead(this->pinA) )
    new_val = 3;
  if( digitalRead(this->pinB) )
    new_val ^= 1;                   // convert gray to binary
  diff = last_val - new_val;                // difference last - new
  if( diff & 1 ){               // bit 0 = value (1)
    last_val = new_val;                 // store new as next last
    enc_delta += (diff & 2) - 1;        // bit 1 = direction (+/-)
  }   
}

int8_t RotaryEnoderSwitch::readEncoder(void)
{
    int8_t ret = 0;
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
