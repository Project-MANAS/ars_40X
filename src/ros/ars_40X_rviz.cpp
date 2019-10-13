
// Created by shivesh on 9/18/19.
//

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "ars_40X/ClusterList.h"
#include "ars_40X/ObjectList.h"
#include "ars_40X/ros/ars_40X_rviz.hpp"

namespace ars_40X {
ContinentalRadarRViz::ContinentalRadarRViz() {
  ros::NodeHandle nh;
  clusters_pub_ = nh.advertise<visualization_msgs::Marker>("visualize_clusters", 50);
  objects_pub_ = nh.advertise<visualization_msgs::MarkerArray>("visualize_objects", 50);
  clusters_sub_ =
      nh.subscribe("ars_40X/clusters", 50, &ContinentalRadarRViz::clusters_callback, this);
  objects_sub_ =
      nh.subscribe("ars_40X/objects", 50, &ContinentalRadarRViz::objects_callback, this);
}

ContinentalRadarRViz::~ContinentalRadarRViz() {
}

void ContinentalRadarRViz::clusters_callback(ars_40X::ClusterList cluster_list) {
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.header.frame_id = cluster_list.header.frame_id;
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

void ContinentalRadarRViz::objects_callback(ars_40X::ObjectList object_list) {
  visualization_msgs::MarkerArray marker_array;
  auto time = ros::Time::now();
  for (auto object : object_list.objects) {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.header.frame_id = object_list.header.frame_id;
    marker.action = visualization_msgs::Marker::ADD;
    marker.header.stamp = time;
    marker.id = object.id;
    geometry_msgs::Point pos1, pos2, pos3, pos4;
    tf2::Quaternion q;
    q.setValue(
        object.position.pose.orientation.x,
        object.position.pose.orientation.y,
        object.position.pose.orientation.z,
        object.position.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    pos1.x = object.position.pose.position.x + (object.width / 2) * tan((yaw * M_PI) / 180.0);
    pos1.y = object.position.pose.position.y + object.width / 2;
    pos1.z = object.length / 2;
    pos2.x = object.position.pose.position.x - (object.width / 2) * tan((yaw * M_PI) / 180.0);
    pos2.y = object.position.pose.position.y - object.width / 2;
    pos2.z = object.length / 2;
    pos3.x = object.position.pose.position.x - (object.width / 2) * tan((yaw * M_PI) / 180.0);
    pos3.y = object.position.pose.position.y - object.width / 2;
    pos3.z = -object.length / 2;
    pos4.x = object.position.pose.position.x + (object.width / 2) * tan((yaw * M_PI) / 180.0);
    pos4.y = object.position.pose.position.y + object.width / 2;
    pos4.z = -object.length / 2;
    if (yaw != 0) {
      double max = pos1.x;
      marker.points.push_back(pos1);
      if (pos2.x >= max) {
        marker.points.push_back(pos2);
        max = std::max(pos2.x, max);
      }
      if (pos3.x >= max) {
        marker.points.push_back(pos3);
        max = std::max(pos3.x, max);
      }
      if (pos4.x >= max) {
        marker.points.push_back(pos4);
        max = std::max(pos4.x, max);
      }
      if (pos1.x >= max) {
        marker.points.push_back(pos1);
      }
    } else {
      marker.points.push_back(pos1);
      marker.points.push_back(pos2);
      marker.points.push_back(pos3);
      marker.points.push_back(pos4);
      marker.points.push_back(pos1);
    }
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    switch (object.class_type) {
      case POINT: {
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        break;
      }

      case CAR: {
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        break;
      }

      case TRUCK: {
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        break;
      }

      case PEDESTRIAN: {
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
        break;
      }

      case MOTORCYCLE: {
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        break;
      }

      case BICYCLE: {
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        break;
      }

      case WIDE: {
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        break;
      }

      case RESERVED: {
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
        break;
      }
    }
    marker.color.a = 1.0;
    marker.lifetime.fromSec(0.1);
    marker_array.markers.push_back(marker);
  }
  objects_pub_.publish(marker_array);
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ars_40X_rviz");
  ars_40X::ContinentalRadarRViz ars_40X_rviz;
  ros::spin();
}