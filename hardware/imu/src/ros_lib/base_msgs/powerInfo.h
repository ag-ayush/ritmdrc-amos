#ifndef _ROS_base_msgs_powerInfo_h
#define _ROS_base_msgs_powerInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace base_msgs
{

  class powerInfo : public ros::Msg
  {
    public:
      float voltageLeft;
      float voltageRight;
      float currentLeft;
      float currentRight;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_voltageLeft;
      u_voltageLeft.real = this->voltageLeft;
      *(outbuffer + offset + 0) = (u_voltageLeft.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_voltageLeft.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_voltageLeft.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_voltageLeft.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->voltageLeft);
      union {
        float real;
        uint32_t base;
      } u_voltageRight;
      u_voltageRight.real = this->voltageRight;
      *(outbuffer + offset + 0) = (u_voltageRight.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_voltageRight.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_voltageRight.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_voltageRight.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->voltageRight);
      union {
        float real;
        uint32_t base;
      } u_currentLeft;
      u_currentLeft.real = this->currentLeft;
      *(outbuffer + offset + 0) = (u_currentLeft.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_currentLeft.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_currentLeft.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_currentLeft.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->currentLeft);
      union {
        float real;
        uint32_t base;
      } u_currentRight;
      u_currentRight.real = this->currentRight;
      *(outbuffer + offset + 0) = (u_currentRight.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_currentRight.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_currentRight.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_currentRight.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->currentRight);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_voltageLeft;
      u_voltageLeft.base = 0;
      u_voltageLeft.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_voltageLeft.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_voltageLeft.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_voltageLeft.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->voltageLeft = u_voltageLeft.real;
      offset += sizeof(this->voltageLeft);
      union {
        float real;
        uint32_t base;
      } u_voltageRight;
      u_voltageRight.base = 0;
      u_voltageRight.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_voltageRight.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_voltageRight.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_voltageRight.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->voltageRight = u_voltageRight.real;
      offset += sizeof(this->voltageRight);
      union {
        float real;
        uint32_t base;
      } u_currentLeft;
      u_currentLeft.base = 0;
      u_currentLeft.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_currentLeft.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_currentLeft.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_currentLeft.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->currentLeft = u_currentLeft.real;
      offset += sizeof(this->currentLeft);
      union {
        float real;
        uint32_t base;
      } u_currentRight;
      u_currentRight.base = 0;
      u_currentRight.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_currentRight.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_currentRight.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_currentRight.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->currentRight = u_currentRight.real;
      offset += sizeof(this->currentRight);
     return offset;
    }

    const char * getType(){ return "base_msgs/powerInfo"; };
    const char * getMD5(){ return "b4c72da9ae23f39c0571bf2017b17268"; };

  };

}
#endif