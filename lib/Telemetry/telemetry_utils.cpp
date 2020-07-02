#include "telemetry_utils.h"

uint32_t emplace(TM_msg* m, char * buf, size_t bufSize)
{
  if(m->type != TM_string)
    return 0;

  uint32_t size = m->size;

  if(bufSize - 1 < size)
    size = bufSize - 1;

  strncpy(buf, (char*)(m->buffer), size);
  buf[size] = '\0';

  return 1;
}

uint32_t emplace_u8(TM_msg* m, uint8_t* dst)
{
  if(m->type != TM_uint8)
    return 0;

  memcpy(dst,m->buffer,1);
  return 1;
}

uint32_t emplace_u16(TM_msg* m, uint16_t* dst)
{
  if(m->type != TM_uint16)
    return 0;

  memcpy(dst,m->buffer,2);
  return 1;
}

uint32_t emplace_u32(TM_msg* m, uint32_t* dst)
{
  if(m->type != TM_uint32)
    return 0;

  memcpy(dst,m->buffer,4);
  return 1;
}

uint32_t emplace_i8(TM_msg* m, int8_t* dst)
{
  if(m->type != TM_int8)
    return 0;

  memcpy(dst,m->buffer,1);
  return 1;
}

uint32_t emplace_i16(TM_msg* m, int16_t* dst)
{
  if(m->type != TM_int16)
    return 0;

  memcpy(dst,m->buffer,2);
  return 1;
}

uint32_t emplace_i32(TM_msg* m, int32_t* dst)
{
  if(m->type != TM_int32)
    return 0;

  memcpy(dst,m->buffer,4);
  return 1;
}

uint32_t emplace_f32(TM_msg* m, float* dst)
{
  if(m->type != TM_float32)
    return 0;

  memcpy(dst,m->buffer,4);
  return 1;
}

uint32_t match(TM_msg * m, const char * topicToMatch)
{
  if(strcmp(m->topic,topicToMatch) == 0)
    return 1;

  return 0;
}

uint32_t fullmatch(TM_msg * m, const char * topicToMatch, TM_type typeToMatch)
{
  if(strcmp(m->topic,topicToMatch) == 0 && m->type == typeToMatch)
    return 1;

  return 0;
}

uint32_t update(TM_msg * msg, const char *topic, char *var, size_t bufSize)
{
   if(strcmp(topic,msg->topic) == 0)
    return emplace(msg, var, bufSize);

   return 0;
}

uint32_t update_u8(TM_msg * msg, const char *topic, uint8_t *var)
{
   if(strcmp(topic,msg->topic) == 0)
    return emplace_u8(msg, var);

   return 0;
}

uint32_t update_u16(TM_msg * msg, const char *topic, uint16_t *var)
{
   if(strcmp(topic,msg->topic) == 0)
    return emplace_u16(msg, var);

   return 0;
}

uint32_t update_u32(TM_msg * msg, const char *topic, uint32_t *var)
{
   if(strcmp(topic,msg->topic) == 0)
     return emplace_u32(msg, var);

   return 0;
}

uint32_t update_i8(TM_msg * msg, const char *topic, int8_t *var)
{
   if(strcmp(topic,msg->topic) == 0)
    return emplace_i8(msg, var);

   return 0;
}

uint32_t update_i16(TM_msg * msg, const char *topic, int16_t *var)
{
   if(strcmp(topic,msg->topic) == 0)
    return emplace_i16(msg, var);

   return 0;
}

uint32_t update_i32(TM_msg * msg, const char *topic, int32_t *var)
{
    if(strcmp(topic,msg->topic) == 0)
      return emplace_i32(msg, var);

   return 0;
}

uint32_t update_f32(TM_msg * msg, const char *topic, float *var)
{
  if(strcmp(topic,msg->topic) == 0)
    return emplace_f32(msg, var);

   return 0;
}
