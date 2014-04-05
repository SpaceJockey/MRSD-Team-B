#ifndef _ROS_spacejockey_MinorPlanAction_h
#define _ROS_spacejockey_MinorPlanAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/TransformStamped.h"

namespace spacejockey
{

  class MinorPlanAction : public ros::Msg
  {
    public:
      uint32_t major_id;
      uint32_t minor_id;
      geometry_msgs::TransformStamped tf;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->major_id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->major_id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->major_id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->major_id >> (8 * 3)) & 0xFF;
      offset += sizeof(this->major_id);
      *(outbuffer + offset + 0) = (this->minor_id >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->minor_id >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->minor_id >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->minor_id >> (8 * 3)) & 0xFF;
      offset += sizeof(this->minor_id);
      offset += this->tf.serialize(outbuffer + offset);
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
      this->minor_id =  ((uint32_t) (*(inbuffer + offset)));
      this->minor_id |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->minor_id |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->minor_id |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->minor_id);
      offset += this->tf.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "spacejockey/MinorPlanAction"; };
    const char * getMD5(){ return "13a0046a66593ad63dc7952dda397541"; };

  };

}
#endif