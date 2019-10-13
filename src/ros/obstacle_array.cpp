//
// Created by shivesh on 9/18/19.
//

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "continental_radar/ros/obstacle_array.hpp"

namespace continental_radar
{
ObstacleArray::ObstacleArray(){
  ros::NodeHandle nh;
  object_list_sub_ = nh.subscribe("visualize_objects", 50, &ObstacleArray::object_list_callback, this);
  obstacle_array_pub_ = nh.advertise<costmap_converter::ObstacleArrayMsg>("obstacles", 50);
}

ObstacleArray::~ObstacleArray() {
}

void ObstacleArray::object_list_callback(continental_radar::ObjectList object_list) {
  costmap_converter::ObstacleArrayMsg obstacle_array_msg;
  obstacle_array_msg.header.frame_id = "radar";
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
    pos1.x = object.position.pose.position.x + (object.width / 2) * tan((yaw * M_PI) / 180);
    pos1.y = object.position.pose.position.y + object.width / 2;
    pos1.z = object.length / 2;
    pos2.x = object.position.pose.position.x - (object.width / 2) * tan((yaw * M_PI) / 180);
    pos2.y = object.position.pose.position.y - object.width / 2;
    pos2.z = object.length / 2;
    pos3.x = object.position.pose.position.x - (object.width / 2) * tan((yaw * M_PI) / 180);
    pos3.y = object.position.pose.position.y - object.width / 2;
    pos3.z = -object.length / 2;
    pos4.x = object.position.pose.position.x + (object.width / 2) * tan((yaw * M_PI) / 180);
    pos4.y = object.position.pose.position.y + object.width / 2;
    pos4.z = -object.length / 2;
    obstacle.polygon.points.push_back(pos1);
    obstacle.polygon.points.push_back(pos2);
    obstacle.polygon.points.push_back(pos3);
    obstacle.polygon.points.push_back(pos4);
    obstacle_array_msg.obstacles.push_back(obstacle);
  }
  obstacle_array_pub_.publish(obstacle_array_msg);
}
}

int main(int argc, char ** argv) {
  ros::init(argc, argv, "obstacle_array");
  continental_radar::ObstacleArray obstacle_array;
  ros::spin();
}