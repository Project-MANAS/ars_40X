//
// Created by shivesh on 9/18/19.
//

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "ars_40X/ros/ars_40X_obstacle_array.hpp"

namespace ars_40X {
ObstacleArray::ObstacleArray() {
  ros::NodeHandle nh;
  object_list_sub_ =
      nh.subscribe("ars_40X/objects", 50, &ObstacleArray::object_list_callback, this);
  obstacle_array_pub_ = nh.advertise<costmap_converter::ObstacleArrayMsg>("obstacles", 50);
}

ObstacleArray::~ObstacleArray() {
}

void ObstacleArray::object_list_callback(ars_40X::ObjectList object_list) {
  costmap_converter::ObstacleArrayMsg obstacle_array_msg;
  obstacle_array_msg.header.frame_id = object_list.header.frame_id;
  obstacle_array_msg.header.stamp = ros::Time::now();
  for (auto object : object_list.objects) {
    costmap_converter::ObstacleMsg obstacle;
    geometry_msgs::Point32 pos1, pos2, pos3, pos4;
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
    tf2::convert(obstacle.orientation, q);
    auto width_sin = (object.width / 2) * sin((yaw * M_PI) / 180.0);
    auto width_cos = (object.width / 2) * cos((yaw * M_PI) / 180.0);
    auto length_sin = (object.length / 2) * sin((yaw * M_PI) / 180.0);
    auto length_cos = (object.length / 2) * cos((yaw * M_PI) / 180.0);
    pos1.x = object.position.pose.position.x - width_sin - length_cos;
    pos1.y = object.position.pose.position.y + width_cos - length_sin;
    pos2.x = object.position.pose.position.x + width_sin - length_cos;
    pos2.y = object.position.pose.position.y - width_cos - length_sin;
    pos3.x = object.position.pose.position.x + width_sin + length_cos;
    pos3.y = object.position.pose.position.y - width_cos + length_sin;
    pos4.x = object.position.pose.position.x - width_sin + length_cos;
    pos4.y = object.position.pose.position.y + width_cos + length_sin;
    obstacle.polygon.points.push_back(pos1);
    obstacle.polygon.points.push_back(pos2);
    obstacle.polygon.points.push_back(pos3);
    obstacle.polygon.points.push_back(pos4);
    obstacle.velocities.twist = object.relative_velocity.twist;
    obstacle_array_msg.obstacles.push_back(obstacle);
  }
  obstacle_array_pub_.publish(obstacle_array_msg);
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "obstacle_array");
  ars_40X::ObstacleArray obstacle_array;
  ros::spin();
}