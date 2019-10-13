//
// Created by shivesh on 9/14/19.
//

#ifndef CONTINENTAL_RADAR_OBJECT_LIST_ROS_HPP
#define CONTINENTAL_RADAR_OBJECT_LIST_ROS_HPP

#include <ros/ros.h>

#include <cstdint>

#include "ars_40X/ObjectList.h"
#include "continental_radar/continental_radar_can.hpp"

namespace ars_40X
{
class ObjectListROS
{
 public:
  ObjectListROS(ros::NodeHandle& nh, ContinentalRadarCAN * continentalRadarCAN);

  ~ObjectListROS();

  void send_object_0_status();

  void send_object_1_general();

  void send_object_2_quality();

  void send_object_3_extended();

 private:
  ros::Publisher objects_data_pub_;

  ContinentalRadarCAN * continental_radar_can_;

  ObjectList object_list;

  object_list::Object_0_Status * object_0_status_;

  object_list::Object_1_General * object_1_general_;

  object_list::Object_2_Quality * object_2_quality_;

  object_list::Object_3_Extended * object_3_extended_;

  int object_2_quality_id_;

  int object_3_extended_id_;
};
}

#endif //CONTINENTAL_RADAR_OBJECT_LIST_ROS_HPP
