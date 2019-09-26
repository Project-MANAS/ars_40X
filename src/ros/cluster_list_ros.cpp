//
// Created by shivesh on 9/17/19.
//

#include "continental_radar/ros/cluster_list_ros.hpp"

#include <iostream>

namespace continental_radar
{
ClusterListROS::ClusterListROS(ros::NodeHandle& nh, ContinentalRadarCAN * continental_radar_can) :
  continental_radar_can_(continental_radar_can)
{
  cluster_0_status_ = continental_radar_can->get_cluster_0_status();
  cluster_1_general_ = continental_radar_can->get_cluster_1_general();
  cluster_2_quality_ = continental_radar_can->get_cluster_2_quality();
  clusters_data_pub_ = nh.advertise<continental_radar::ClusterList>("continental_radar/clusters", 10);
}

ClusterListROS::~ClusterListROS()
{
}

void ClusterListROS::send_cluster_0_status() {
  cluster_list.header.stamp = ros::Time::now();
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