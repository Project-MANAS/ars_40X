//
// Created by shivesh on 9/13/19.
//

#ifndef CONTINENTAL_RADAR_OBSTACLE_ARRAY_HPP
#define CONTINENTAL_RADAR_OBSTACLE_ARRAY_HPP

#include <ros/ros.h>

#include "continental_radar/ObjectList.h"

#include <costmap_converter/ObstacleArrayMsg.h>

namespace continental_radar
{
class ObstacleArray {
 public:
  ObstacleArray();

  ~ObstacleArray();

 private:
  void object_list_callback(continental_radar::ObjectList object_list);

  ros::Publisher obstacle_array_pub_;

  ros::Subscriber object_list_sub_;
};
}

#endif //CONTINENTAL_RADAR_OBSTACLE_ARRAY_HPP
