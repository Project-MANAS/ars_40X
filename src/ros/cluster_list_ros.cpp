//
// Created by shivesh on 9/17/19.
//

#include "ars_40X/ros/cluster_list_ros.hpp"

#include <iostream>

namespace ars_40X {
ClusterListROS::ClusterListROS(ros::NodeHandle &nh, ARS_40X_CAN *ars_40X_can) :
    ars_40X_can_(ars_40X_can) {
  cluster_0_status_ = ars_40X_can->get_cluster_0_status();
  cluster_1_general_ = ars_40X_can->get_cluster_1_general();
  cluster_2_quality_ = ars_40X_can->get_cluster_2_quality();
  clusters_data_pub_ = nh.advertise<ars_40X::ClusterList>("ars_40X/clusters", 10);
}

ClusterListROS::~ClusterListROS() {
}

void ClusterListROS::set_frame_id(std::string frame_id) {
  frame_id_ = frame_id;
}

void ClusterListROS::send_cluster_0_status() {
  cluster_list.header.stamp = ros::Time::now();
  cluster_list.header.frame_id = frame_id_;
  clusters_data_pub_.publish(cluster_list);
  cluster_list.clusters.clear();
}

void ClusterListROS::send_cluster_1_general() {
  Cluster cluster;
  cluster.id = cluster_1_general_->get_cluster_id();
  cluster.position.x = cluster_1_general_->get_cluster_long_dist();
  cluster.position.y = cluster_1_general_->get_cluster_lat_dist();
  cluster.relative_velocity.x = cluster_1_general_->get_cluster_long_rel_vel();
  cluster.relative_velocity.y = cluster_1_general_->get_cluster_lat_rel_vel();
  cluster_list.clusters.push_back(cluster);
}
}