#ifndef ros_compass_msg_Compass_h
#define ros_compass_msg_Compass_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "../ros/msg.h"

namespace compass_msg
{

  class Compass : public ros::Msg
  {
    public:
      float heading;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      union {
        float real;
        unsigned long base;
      } u_heading;
      u_heading.real = this->heading;
      *(outbuffer + offset + 0) = (u_heading.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_heading.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_heading.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_heading.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->heading);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        unsigned long base;
      } u_heading;
      u_heading.base = 0;
      u_heading.base |= ((typeof(u_heading.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_heading.base |= ((typeof(u_heading.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_heading.base |= ((typeof(u_heading.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_heading.base |= ((typeof(u_heading.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->heading = u_heading.real;
      offset += sizeof(this->heading);
     return offset;
    }

    const char * getType(){ return "compass_msg/Compass"; };

  };

}
#endif