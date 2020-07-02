#ifndef TELEMETRY_ARDUINO_H_
#define TELEMETRY_ARDUINO_H_

#include "telemetry_utils.h"

#define TELEMETRY_ARDUINO_MAJOR 0
#define TELEMETRY_ARDUINO_MNIOR 1
#define TELEMETRY_ARDUINO_PATCH 0

class TelemetryClass
{
    public:
      TelemetryClass();

      //void attach_to(const char * topic, );
      void attach_f32_to(const char * topic, float * variable);
      void attach_u8_to(const char * topic, uint8_t * variable);
      void attach_u16_to(const char * topic, uint16_t * variable);
      void attach_u32_to(const char * topic, uint32_t * variable);
      void attach_i8_to(const char * topic, int8_t * variable);
      void attach_i16_to(const char * topic, int16_t * variable);
      void attach_i32_to(const char * topic, int32_t * variable);

      TM_transport * get_transport();

      void pub(const char * topic, const char * msg);
      void pub_u8(const char * topic, uint8_t msg);
      void pub_u16(const char * topic, uint16_t msg);
      void pub_u32(const char * topic, uint32_t msg);
      void pub_i8(const char * topic, int8_t msg);
      void pub_i16(const char * topic, int16_t msg);
      void pub_i32(const char * topic, int32_t msg);
      void pub_f32(const char * topic, float msg);

      void sub(void (*callback)(TM_state * s, TM_msg * m), TM_state* userData);

      void update();

    private:
      TM_transport transport;
};

static TelemetryClass Telemetry;

#endif
