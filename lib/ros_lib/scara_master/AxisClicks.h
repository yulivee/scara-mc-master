#ifndef _ROS_scara_master_AxisClicks_h
#define _ROS_scara_master_AxisClicks_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace scara_master
{

  class AxisClicks : public ros::Msg
  {
    public:
      typedef int16_t _zAxis_type;
      _zAxis_type zAxis;
      typedef int16_t _SKE_type;
      _SKE_type SKE;
      typedef int16_t _SKF_type;
      _SKF_type SKF;
      typedef int16_t _SKG_type;
      _SKG_type SKG;
      typedef int16_t _UAE_type;
      _UAE_type UAE;
      typedef int16_t _UAJ_type;
      _UAJ_type UAJ;
      typedef int16_t _Shoulder_type;
      _Shoulder_type Shoulder;

    AxisClicks():
      zAxis(0),
      SKE(0),
      SKF(0),
      SKG(0),
      UAE(0),
      UAJ(0),
      Shoulder(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_zAxis;
      u_zAxis.real = this->zAxis;
      *(outbuffer + offset + 0) = (u_zAxis.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_zAxis.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->zAxis);
      union {
        int16_t real;
        uint16_t base;
      } u_SKE;
      u_SKE.real = this->SKE;
      *(outbuffer + offset + 0) = (u_SKE.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_SKE.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->SKE);
      union {
        int16_t real;
        uint16_t base;
      } u_SKF;
      u_SKF.real = this->SKF;
      *(outbuffer + offset + 0) = (u_SKF.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_SKF.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->SKF);
      union {
        int16_t real;
        uint16_t base;
      } u_SKG;
      u_SKG.real = this->SKG;
      *(outbuffer + offset + 0) = (u_SKG.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_SKG.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->SKG);
      union {
        int16_t real;
        uint16_t base;
      } u_UAE;
      u_UAE.real = this->UAE;
      *(outbuffer + offset + 0) = (u_UAE.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_UAE.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->UAE);
      union {
        int16_t real;
        uint16_t base;
      } u_UAJ;
      u_UAJ.real = this->UAJ;
      *(outbuffer + offset + 0) = (u_UAJ.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_UAJ.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->UAJ);
      union {
        int16_t real;
        uint16_t base;
      } u_Shoulder;
      u_Shoulder.real = this->Shoulder;
      *(outbuffer + offset + 0) = (u_Shoulder.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Shoulder.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Shoulder);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_zAxis;
      u_zAxis.base = 0;
      u_zAxis.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_zAxis.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->zAxis = u_zAxis.real;
      offset += sizeof(this->zAxis);
      union {
        int16_t real;
        uint16_t base;
      } u_SKE;
      u_SKE.base = 0;
      u_SKE.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_SKE.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->SKE = u_SKE.real;
      offset += sizeof(this->SKE);
      union {
        int16_t real;
        uint16_t base;
      } u_SKF;
      u_SKF.base = 0;
      u_SKF.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_SKF.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->SKF = u_SKF.real;
      offset += sizeof(this->SKF);
      union {
        int16_t real;
        uint16_t base;
      } u_SKG;
      u_SKG.base = 0;
      u_SKG.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_SKG.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->SKG = u_SKG.real;
      offset += sizeof(this->SKG);
      union {
        int16_t real;
        uint16_t base;
      } u_UAE;
      u_UAE.base = 0;
      u_UAE.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_UAE.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->UAE = u_UAE.real;
      offset += sizeof(this->UAE);
      union {
        int16_t real;
        uint16_t base;
      } u_UAJ;
      u_UAJ.base = 0;
      u_UAJ.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_UAJ.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->UAJ = u_UAJ.real;
      offset += sizeof(this->UAJ);
      union {
        int16_t real;
        uint16_t base;
      } u_Shoulder;
      u_Shoulder.base = 0;
      u_Shoulder.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Shoulder.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Shoulder = u_Shoulder.real;
      offset += sizeof(this->Shoulder);
     return offset;
    }

    const char * getType(){ return "scara_master/AxisClicks"; };
    const char * getMD5(){ return "45bfdf54e5f4822234c6076ceada68f2"; };

  };

}
#endif