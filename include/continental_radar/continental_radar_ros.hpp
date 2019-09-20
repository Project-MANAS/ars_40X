//
// Created by shivesh on 9/13/19.
//

#ifndef CONTINENTAL_RADAR_CONTINENTAL_RADAR_ROS_HPP
#define CONTINENTAL_RADAR_CONTINENTAL_RADAR_ROS_HPP

#include <ros/ros.h>
#include <thread>

#include <nav_msgs/Odometry.h>

#include "continental_radar/Cluster.h"
#include "continental_radar/ClusterList.h"
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

  void odom_callback(nav_msgs::Odometry msg);

  void send_cluster_0_status() override;

  void send_cluster_1_general() override;

  void send_object_0_status() override;

  void send_object_1_general() override;

  void send_object_3_extended() override;

 private:
  ros::NodeHandle nh_;

  ros::Publisher clusters_data_pub_;

  ros::Publisher objects_data_pub_;

  ros::Subscriber odom_sub_;

  ClusterList cluster_list;

  ObjectList object_list;

  int object_id_;

  double yaw_vel_prev_;

  double yaw_vel_time_prev_;

  std::thread receive_data_thread;
};
}

#endif //CONTINENTAL_RADAR_CONTINENTAL_RADAR_ROS_HPP
