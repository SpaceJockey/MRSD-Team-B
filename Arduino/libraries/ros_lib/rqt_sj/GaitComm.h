#ifndef _ROS_rqt_sj_GaitComm_h
#define _ROS_rqt_sj_GaitComm_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rqt_sj
{

  class GaitComm : public ros::Msg
  {
    public:
      char * segment;
      char * motion;
      int64_t resolution;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_segment = strlen( (const char*) this->segment);
      memcpy(outbuffer + offset, &length_segment, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->segment, length_segment);
      offset += length_segment;
      uint32_t length_motion = strlen( (const char*) this->motion);
      memcpy(outbuffer + offset, &length_motion, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->motion, length_motion);
      offset += length_motion;
      union {
        int64_t real;
        uint64_t base;
      } u_resolution;
      u_resolution.real = this->resolution;
      *(outbuffer + offset + 0) = (u_resolution.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_resolution.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_resolution.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_resolution.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_resolution.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_resolution.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_resolution.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_resolution.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->resolution);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_segment;
      memcpy(&length_segment, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_segment; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_segment-1]=0;
      this->segment = (char *)(inbuffer + offset-1);
      offset += length_segment;
      uint32_t length_motion;
      memcpy(&length_motion, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_motion; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_motion-1]=0;
      this->motion = (char *)(inbuffer + offset-1);
      offset += length_motion;
      union {
        int64_t real;
        uint64_t base;
      } u_resolution;
      u_resolution.base = 0;
      u_resolution.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_resolution.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_resolution.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_resolution.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_resolution.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_resolution.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_resolution.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_resolution.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->resolution = u_resolution.real;
      offset += sizeof(this->resolution);
     return offset;
    }

    const char * getType(){ return "rqt_sj/GaitComm"; };
    const char * getMD5(){ return "faafd0d9e5f2d0ac94eb49b18f633635"; };

  };

}
#endif