#ifndef _ROS_base_msgs_motorOdom_h
#define _ROS_base_msgs_motorOdom_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace base_msgs
{

  class motorOdom : public ros::Msg
  {
    public:
      float left;
      float right;
      float leftTime;
      float rightTime;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_left;
      u_left.real = this->left;
      *(outbuffer + offset + 0) = (u_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left);
      union {
        float real;
        uint32_t base;
      } u_right;
      u_right.real = this->right;
      *(outbuffer + offset + 0) = (u_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right);
      union {
        float real;
        uint32_t base;
      } u_leftTime;
      u_leftTime.real = this->leftTime;
      *(outbuffer + offset + 0) = (u_leftTime.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_leftTime.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_leftTime.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_leftTime.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->leftTime);
      union {
        float real;
        uint32_t base;
      } u_rightTime;
      u_rightTime.real = this->rightTime;
      *(outbuffer + offset + 0) = (u_rightTime.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rightTime.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rightTime.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rightTime.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rightTime);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_left;
      u_left.base = 0;
      u_left.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left = u_left.real;
      offset += sizeof(this->left);
      union {
        float real;
        uint32_t base;
      } u_right;
      u_right.base = 0;
      u_right.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right = u_right.real;
      offset += sizeof(this->right);
      union {
        float real;
        uint32_t base;
      } u_leftTime;
      u_leftTime.base = 0;
      u_leftTime.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_leftTime.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_leftTime.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_leftTime.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->leftTime = u_leftTime.real;
      offset += sizeof(this->leftTime);
      union {
        float real;
        uint32_t base;
      } u_rightTime;
      u_rightTime.base = 0;
      u_rightTime.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rightTime.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rightTime.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rightTime.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rightTime = u_rightTime.real;
      offset += sizeof(this->rightTime);
     return offset;
    }

    const char * getType(){ return "base_msgs/motorOdom"; };
    const char * getMD5(){ return "3adcce373d2857cd80a29bd688b1f0d2"; };

  };

}
#endif