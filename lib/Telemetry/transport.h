#ifndef TELEMETRY_TRANSPORT_HPP_
#define TELEMETRY_TRANSPORT_HPP_

#include "HardwareSerial.h"
#include "Arduino.h"

int32_t read(uint8_t * buf, uint32_t sizeToRead)
{
    return Serial.readBytes((char*)(buf), sizeToRead);
}

int32_t write(uint8_t * buf, uint32_t sizeToWrite)
{
    Serial.write((char*)(buf),sizeToWrite);
    return 0;
}

int32_t readable()
{
    return Serial.available();
}

int32_t writeable()
{
    return Serial.availableForWrite();
}

#endif
