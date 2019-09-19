//
// Created by shivesh on 9/13/19.
//

#ifndef CONTINENTAL_RADAR_CONTINENTAL_RADAR_ROS_HPP
#define CONTINENTAL_RADAR_CONTINENTAL_RADAR_ROS_HPP

#include <ros/ros.h>
#include <thread>

#include "continental_radar/Object.h"
#include "continental_radar/ObjectList.h"
#include "continental_radar/continental_radar_can.hpp"

namespace continental_radar
{
class ContinentalRadarROS : public ContinentalRadarCAN {
 public:
  ContinentalRadarROS();

  ~ContinentalRadarROS();

  void receive_data();

  void run();

  void send_object_0_status() override;

  void send_object_1_general() override;

  void send_object_3_extended() override;

 private:
  ros::NodeHandle nh_;

  ros::Publisher objects_data_pub_;

  ObjectList object_list;

  int object_id_;

  std::thread receive_data_thread;
};
}

#endif //CONTINENTAL_RADAR_CONTINENTAL_RADAR_ROS_HPP
