
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
  clusters_pub_ = nh.advertise<visualization_msgs::MarkerArray>("visualize_clusters", 50);
  objects_pub_ = nh.advertise<visualization_msgs::MarkerArray>("visualize_objects", 50);
  velocity_pub_ = nh.advertise<visualization_msgs::MarkerArray>("visualize_velocity", 50);
  clusters_sub_ =
      nh.subscribe("ars_40X/clusters", 50, &ContinentalRadarRViz::clusters_callback, this);
  objects_sub_ =
      nh.subscribe("ars_40X/objects", 50, &ContinentalRadarRViz::objects_callback, this);
}

ContinentalRadarRViz::~ContinentalRadarRViz() {
}

void ContinentalRadarRViz::clusters_callback(ars_40X::ClusterList cluster_list) {
  visualization_msgs::MarkerArray marker_array;
  for (auto cluster : cluster_list.clusters) {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.header.frame_id = cluster_list.header.frame_id;
    marker.action = visualization_msgs::Marker::ADD;
    marker.header.stamp = cluster_list.header.stamp;
    marker.id = cluster.id;
    marker.points.push_back(cluster.position.pose.position);
    switch (cluster.prob_of_exist) {
      case INVALID: {
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.ns = "Invalid";
        break;
      }

      case PERCENT_25: {
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.ns = "25%";
        break;
      }

      case PERCENT_50: {
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.ns = "50%";
        break;
      }

      case PERCENT_75: {
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
        marker.ns = "75%";
        break;
      }

      case PERCENT_90: {
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.ns = "90%";
        break;
      }

      case PERCENT_99: {
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.ns = "99%";
        break;
      }

      case PERCENT_99_9: {
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.ns = "99.9%";
        break;
      }

      case PERCENT_100: {
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
        marker.ns = "100%";
        break;
      }
    }
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.color.a = 1.0;
    marker.lifetime.fromSec(0.1);
    marker_array.markers.push_back(marker);
  }
  clusters_pub_.publish(marker_array);
}

void ContinentalRadarRViz::objects_callback(ars_40X::ObjectList object_list) {
  visualization_msgs::MarkerArray marker_array, velocity_array;
  for (auto object : object_list.objects) {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.header.frame_id = object_list.header.frame_id;
    marker.action = visualization_msgs::Marker::ADD;
    marker.header.stamp = object_list.header.stamp;
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
    if (isnan(yaw)) {
      continue;
    }
    marker.pose = object.position.pose;
    pos1.x = object.length / 2;
    pos1.y = object.width / 2;
    pos2.x = object.length / 2;
    pos2.y = -object.width / 2;
    pos3.x = -object.length / 2;
    pos3.y = -object.width / 2;
    pos4.x = -object.length / 2;
    pos4.y = object.width / 2;
    marker.points.push_back(pos1);
    marker.points.push_back(pos2);
    marker.points.push_back(pos3);
    marker.points.push_back(pos4);
    marker.points.push_back(pos1);
    marker.scale.x = 0.1;

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

    switch (object.prob_of_exist) {
      case INVALID: {
        marker.ns = "Invalid";
        break;
      }

      case PERCENT_25: {
        marker.ns = "25%";
        break;
      }

      case PERCENT_50: {
        marker.ns = "50%";
        break;
      }

      case PERCENT_75: {
        marker.ns = "75%";
        break;
      }

      case PERCENT_90: {
        marker.ns = "90%";
        break;
      }

      case PERCENT_99: {
        marker.ns = "99%";
        break;
      }

      case PERCENT_99_9: {
        marker.ns = "99.9%";
        break;
      }

      case PERCENT_100: {
        marker.ns = "100%";
        break;
      }
    }
    marker.color.a = 1.0;
    marker.lifetime.fromSec(0.1);
    marker_array.markers.push_back(marker);

    visualization_msgs::Marker velocity;
    velocity.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    velocity.header.frame_id = object_list.header.frame_id;
    velocity.action = visualization_msgs::Marker::ADD;
    velocity.header.stamp = object_list.header.stamp;
    velocity.id = object.id;
    velocity.text = std::to_string(hypot(object.relative_velocity.twist.linear.x,
                                         object.relative_velocity.twist.linear.y));
    velocity.pose = object.position.pose;
    velocity.scale.z = 1;
    velocity.color.a = 1;

    velocity_array.markers.push_back(velocity);
  }
  objects_pub_.publish(marker_array);
  velocity_pub_.publish(velocity_array);
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ars_40X_rviz");
  ars_40X::ContinentalRadarRViz ars_40X_rviz;
  ros::spin();
}