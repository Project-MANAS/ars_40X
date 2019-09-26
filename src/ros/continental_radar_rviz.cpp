
// Created by shivesh on 9/18/19.
//

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include "continental_radar/ClusterList.h"
#include "continental_radar/ObjectList.h"
#include "continental_radar/ros/continental_radar_rviz.hpp"

namespace continental_radar
{
ContinentalRadarRViz::ContinentalRadarRViz(){
  ros::NodeHandle nh;
  clusters_pub_ = nh.advertise<visualization_msgs::Marker>("visualize_clusters", 50);
  objects_pub_ = nh.advertise<visualization_msgs::MarkerArray>("visualize_objects", 50);
  clusters_sub_ =
    nh.subscribe("continental_radar/clusters", 50, &ContinentalRadarRViz::clusters_callback, this);
  objects_sub_ =
    nh.subscribe("continental_radar/objects", 50, &ContinentalRadarRViz::objects_callback, this);
}

ContinentalRadarRViz::~ContinentalRadarRViz() {
}

void ContinentalRadarRViz::clusters_callback(continental_radar::ClusterList cluster_list) {
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.header.frame_id = "radar";
  marker.action = visualization_msgs::Marker::ADD;
  marker.header.stamp = ros::Time::now();
  for (auto cluster : cluster_list.clusters) {
    marker.points.push_back(cluster.position);
  }
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;
  marker.lifetime.nsec = 200000000;
  clusters_pub_.publish(marker);
}

void ContinentalRadarRViz::objects_callback(continental_radar::ObjectList object_list) {
  visualization_msgs::MarkerArray marker_array;
  auto time = ros::Time::now();
  int id = 0;
  for (auto object : object_list.objects) {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.header.frame_id = "radar";
    marker.action = visualization_msgs::Marker::ADD;
    marker.header.stamp = time;
    marker.id = id++;
    geometry_msgs::Point pos1, pos2, pos3, pos4;
    pos1.x = object.position.x + (object.width / 2) * tan((object.orientation_angle * M_PI) / 180);
    pos1.y = object.position.y + object.width / 2;
    pos1.z = object.length / 2;
    pos2.x = object.position.x - (object.width / 2) * tan((object.orientation_angle * M_PI) / 180);
    pos2.y = object.position.y - object.width / 2;
    pos2.z = object.length / 2;
    pos3.x = object.position.x - (object.width / 2) * tan((object.orientation_angle * M_PI) / 180);
    pos3.y = object.position.y - object.width / 2;
    pos3.z = -object.length / 2;
    pos4.x = object.position.x + (object.width / 2) * tan((object.orientation_angle * M_PI) / 180);
    pos4.y = object.position.y + object.width / 2;
    pos4.z = -object.length / 2;
    marker.points.push_back(pos1);
    marker.points.push_back(pos2);
    marker.points.push_back(pos3);
    marker.points.push_back(pos4);
    marker.points.push_back(pos1);
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    switch (object.class_type) {
      case 1:
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        break;
      case 2:
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        break;
      case 3:
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        break;
      case 4:
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
        break;
      case 5:
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        break;
      case 6:
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        break;
      case 7:
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        break;
      case 8:
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
        break;
    }
    marker.color.a = 1.0;
    marker.lifetime.nsec = 200000000;
    marker_array.markers.push_back(marker);
  }
  objects_pub_.publish(marker_array);
}
}

int main(int argc, char ** argv) {
  ros::init(argc, argv, "continental_radar_rviz");
  continental_radar::ContinentalRadarRViz continental_radar_rviz;
  ros::spin();
}