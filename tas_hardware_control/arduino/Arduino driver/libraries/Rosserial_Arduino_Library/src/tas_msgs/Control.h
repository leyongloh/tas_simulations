#ifndef _ROS_tas_msgs_Control_h
#define _ROS_tas_msgs_Control_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ArduinoIncludes.h" // added

namespace tas_msgs
{

  class Control : public ros::Msg
  {
    public:
      typedef float _steering_angle_type;
      _steering_angle_type steering_angle;
      typedef float _rpm_fl_type;
      _rpm_fl_type rpm_fl;
      typedef float _rpm_fr_type;
      _rpm_fr_type rpm_fr;
      typedef float _rpm_bl_type;
      _rpm_bl_type rpm_bl;
      typedef float _rpm_br_type;
      _rpm_br_type rpm_br;

    Control():
      steering_angle(0),
      rpm_fl(0),
      rpm_fr(0),
      rpm_bl(0),
      rpm_br(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_steering_angle;
      u_steering_angle.real = this->steering_angle;
      *(outbuffer + offset + 0) = (u_steering_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_steering_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_steering_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_steering_angle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->steering_angle);
      union {
        float real;
        uint32_t base;
      } u_rpm_fl;
      u_rpm_fl.real = this->rpm_fl;
      *(outbuffer + offset + 0) = (u_rpm_fl.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rpm_fl.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rpm_fl.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rpm_fl.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rpm_fl);
      union {
        float real;
        uint32_t base;
      } u_rpm_fr;
      u_rpm_fr.real = this->rpm_fr;
      *(outbuffer + offset + 0) = (u_rpm_fr.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rpm_fr.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rpm_fr.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rpm_fr.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rpm_fr);
      union {
        float real;
        uint32_t base;
      } u_rpm_bl;
      u_rpm_bl.real = this->rpm_bl;
      *(outbuffer + offset + 0) = (u_rpm_bl.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rpm_bl.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rpm_bl.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rpm_bl.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rpm_bl);
      union {
        float real;
        uint32_t base;
      } u_rpm_br;
      u_rpm_br.real = this->rpm_br;
      *(outbuffer + offset + 0) = (u_rpm_br.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rpm_br.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rpm_br.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rpm_br.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rpm_br);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_steering_angle;
      u_steering_angle.base = 0;
      u_steering_angle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_steering_angle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_steering_angle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_steering_angle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->steering_angle = u_steering_angle.real;
      offset += sizeof(this->steering_angle);
      union {
        float real;
        uint32_t base;
      } u_rpm_fl;
      u_rpm_fl.base = 0;
      u_rpm_fl.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rpm_fl.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rpm_fl.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rpm_fl.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rpm_fl = u_rpm_fl.real;
      offset += sizeof(this->rpm_fl);
      union {
        float real;
        uint32_t base;
      } u_rpm_fr;
      u_rpm_fr.base = 0;
      u_rpm_fr.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rpm_fr.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rpm_fr.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rpm_fr.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rpm_fr = u_rpm_fr.real;
      offset += sizeof(this->rpm_fr);
      union {
        float real;
        uint32_t base;
      } u_rpm_bl;
      u_rpm_bl.base = 0;
      u_rpm_bl.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rpm_bl.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rpm_bl.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rpm_bl.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rpm_bl = u_rpm_bl.real;
      offset += sizeof(this->rpm_bl);
      union {
        float real;
        uint32_t base;
      } u_rpm_br;
      u_rpm_br.base = 0;
      u_rpm_br.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rpm_br.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rpm_br.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rpm_br.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rpm_br = u_rpm_br.real;
      offset += sizeof(this->rpm_br);
     return offset;
    }

    // const char * getType(){ return "tas_msgs/Control"; };
    // const char * getMD5(){ return "4eab4ebc9933e200e9179964303ebf8c"; };
    const char * getType(){ return PSTR("tas_msgs/Control"); };
    const char * getMD5(){ return PSTR("4eab4ebc9933e200e9179964303ebf8c"); };

  };

}
#endif