#ifndef _ROS_tas_msgs_Status_h
#define _ROS_tas_msgs_Status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace tas_msgs
{

  class Status : public ros::Msg
  {
    public:
      typedef bool _active_type;
      _active_type active;

    Status():
      active(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_active;
      u_active.real = this->active;
      *(outbuffer + offset + 0) = (u_active.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->active);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_active;
      u_active.base = 0;
      u_active.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->active = u_active.real;
      offset += sizeof(this->active);
     return offset;
    }

    const char * getType(){ return "tas_msgs/Status"; };
    const char * getMD5(){ return "2c5cfb0a2565df41de6873994aee57d2"; };

  };

}
#endif