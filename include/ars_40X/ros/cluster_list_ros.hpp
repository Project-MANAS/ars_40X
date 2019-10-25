//
// Created by shivesh on 9/14/19.
//

#ifndef ARS_40X_CLUSTER_LIST_ROS_HPP
#define ARS_40X_CLUSTER_LIST_ROS_HPP

#include <ros/ros.h>

#include <cstdint>

#include "ars_40X/ClusterList.h"
#include "ars_40X/ars_40X_can.hpp"

namespace ars_40X {
class ClusterListROS {
 public:
  ClusterListROS(
      ros::NodeHandle &nh, ARS_40X_CAN *ars_40X_can);

  ~ClusterListROS();

  void set_frame_id(std::string frame_id);

  void send_cluster_0_status();

  void send_cluster_1_general();

  void send_cluster_2_quality();

 private:
  std::string frame_id_;

  ros::Publisher clusters_data_pub_;

  ClusterList cluster_list;

  cluster_list::Cluster_0_Status *cluster_0_status_;

  cluster_list::Cluster_1_General *cluster_1_general_;

  cluster_list::Cluster_2_Quality *cluster_2_quality_;

  ARS_40X_CAN *ars_40X_can_;

  int cluster_id_;
};
}

#endif //ARS_40X_CLUSTER_LIST_ROS_HPP
