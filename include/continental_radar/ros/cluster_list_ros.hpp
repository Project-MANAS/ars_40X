//
// Created by shivesh on 9/14/19.
//

#ifndef CONTINENTAL_RADAR_CLUSTER_LIST_ROS_HPP
#define CONTINENTAL_RADAR_CLUSTER_LIST_ROS_HPP

#include <ros/ros.h>

#include <cstdint>

#include "ars_40X/ClusterList.h"
#include "continental_radar/continental_radar_can.hpp"

namespace ars_40X
{
class ClusterListROS {
 public:
  ClusterListROS(
    ros::NodeHandle& nh, ContinentalRadarCAN * continental_radar_can);

  ~ClusterListROS();

  void send_cluster_0_status();

  void send_cluster_1_general();

 private:
  ros::Publisher clusters_data_pub_;

  ClusterList cluster_list;

  cluster_list::Cluster_0_Status * cluster_0_status_;

  cluster_list::Cluster_1_General * cluster_1_general_;

  cluster_list::Cluster_2_Quality * cluster_2_quality_;

  ContinentalRadarCAN * continental_radar_can_;
};
}

#endif //CONTINENTAL_RADAR_CLUSTER_LIST_ROS_HPP
