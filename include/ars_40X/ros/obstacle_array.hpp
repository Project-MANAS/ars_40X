//
// Created by shivesh on 9/13/19.
//

#ifndef ARS_40X_OBSTACLE_ARRAY_HPP
#define ARS_40X_OBSTACLE_ARRAY_HPP

#include <ros/ros.h>

#include "ars_40X/ObjectList.h"

#include <costmap_converter/ObstacleArrayMsg.h>

namespace ars_40X {
class ObstacleArray {
 public:
  ObstacleArray();

  ~ObstacleArray();

 private:
  void object_list_callback(ars_40X::ObjectList object_list);

  ros::Publisher obstacle_array_pub_;

  ros::Subscriber object_list_sub_;
};
}

#endif //ARS_40X_OBSTACLE_ARRAY_HPP
