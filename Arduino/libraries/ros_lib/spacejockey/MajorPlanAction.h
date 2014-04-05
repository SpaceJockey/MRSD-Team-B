#ifndef _ROS_spacejockey_MajorPlanAction_h
#define _ROS_spacejockey_MajorPlanAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace spacejockey
{

  class MajorPlanAction : public ros::Msg
  {
    public:
      uint32_t major_id;
      uint16_t action_type;
      char * node_name;
      float x;
      float y;
      float theta;
      enum { STEP = 0 };
      enum { VIEW = 1 };

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->major_id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->major_id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->major_id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->major_id >> (8 * 3)) & 0xFF;
      offset += sizeof(this->major_id);
      *(outbuffer + offset + 0) = (this->action_type >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->action_type >> (8 * 1)) & 0xFF;
      offset += sizeof(this->action_type);
      uint32_t length_node_name = strlen( (const char*) this->node_name);
      memcpy(outbuffer + offset, &length_node_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->node_name, length_node_name);
      offset += length_node_name;
      union {
        float real;
        uint32_t base;
      } u_x;
      u_x.real = this->x;
      *(outbuffer + offset + 0) = (u_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x);
      union {
        float real;
        uint32_t base;
      } u_y;
      u_y.real = this->y;
      *(outbuffer + offset + 0) = (u_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y);
      union {
        float real;
        uint32_t base;
      } u_theta;
      u_theta.real = this->theta;
      *(outbuffer + offset + 0) = (u_theta.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_theta.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_theta.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_theta.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->theta);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->major_id =  ((uint32_t) (*(inbuffer + offset)));
      this->major_id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->major_id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->major_id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->major_id);
      this->action_type =  ((uint16_t) (*(inbuffer + offset)));
      this->action_type |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->action_type);
      uint32_t length_node_name;
      memcpy(&length_node_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_node_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_node_name-1]=0;
      this->node_name = (char *)(inbuffer + offset-1);
      offset += length_node_name;
      union {
        float real;
        uint32_t base;
      } u_x;
      u_x.base = 0;
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x = u_x.real;
      offset += sizeof(this->x);
      union {
        float real;
        uint32_t base;
      } u_y;
      u_y.base = 0;
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y = u_y.real;
      offset += sizeof(this->y);
      union {
        float real;
        uint32_t base;
      } u_theta;
      u_theta.base = 0;
      u_theta.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_theta.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_theta.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_theta.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->theta = u_theta.real;
      offset += sizeof(this->theta);
     return offset;
    }

    const char * getType(){ return "spacejockey/MajorPlanAction"; };
    const char * getMD5(){ return "3ceee5aea2b1bbfd0bc9fd06e50cc21b"; };

  };

}
#endif