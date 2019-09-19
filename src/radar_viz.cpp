//
// Created by shivesh on 9/18/19.
//

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include "continental_radar/ObjectList.h"

ros::Publisher pub;

void callback(continental_radar::ObjectList object_list) {
  visualization_msgs::MarkerArray pub_msg;
  auto time = ros::Time::now();
  int id = 0;
  for (auto msg : object_list.objects) {
    visualization_msgs::Marker pub_msg_ele;
    pub_msg_ele.type = visualization_msgs::Marker::LINE_STRIP;
    pub_msg_ele.header.frame_id = "radar";
    pub_msg_ele.action = visualization_msgs::Marker::ADD;
    pub_msg_ele.header.stamp = time;
    pub_msg_ele.id = id++;
    geometry_msgs::Point a1, a2, a3, a4, a5;
    a1.x = msg.position.x + (msg.width / 2) * tan((msg.orientation_angle * M_PI) / 180);
    a1.y = msg.position.y + msg.width / 2;
    a1.z = msg.length / 2;
    a2.x = msg.position.x - (msg.width / 2) * tan((msg.orientation_angle * M_PI) / 180);
    a2.y = msg.position.y - msg.width / 2; 
    a2.z = msg.length / 2;
    a3.x = msg.position.x - (msg.width / 2) * tan((msg.orientation_angle * M_PI) / 180);
    a3.y = msg.position.y - msg.width / 2;
    a3.z = - msg.length / 2;
    a4.x = msg.position.x + (msg.width / 2) * tan((msg.orientation_angle * M_PI) / 180);
    a4.y = msg.position.y + msg.width / 2;
    a4.z = - msg.length / 2;
    a5.x = msg.position.x + (msg.width / 2) * tan((msg.orientation_angle * M_PI) / 180);
    a5.y = msg.position.y + msg.width / 2;
    a5.z = msg.length / 2;
    pub_msg_ele.points.push_back(a1);
    pub_msg_ele.points.push_back(a2);
    pub_msg_ele.points.push_back(a3);
    pub_msg_ele.points.push_back(a4);
    pub_msg_ele.points.push_back(a5);
    pub_msg_ele.scale.x = 0.1;
    pub_msg_ele.scale.y = 0.1;
    pub_msg_ele.scale.z = 0.1;
    pub_msg_ele.color.r = 1.0f;
    pub_msg_ele.color.g = 1.0f;
    pub_msg_ele.color.b = 1.0f;
    pub_msg_ele.color.a = 1.0;
    pub_msg_ele.lifetime.nsec = 200000000;
    pub_msg.markers.push_back(pub_msg_ele);
  }
  pub.publish(pub_msg);
}

int main(int argc, char ** argv) {
  ros::init(argc, argv, "continental_radar_rviz");
  ros::NodeHandle nh;
  pub = nh.advertise<visualization_msgs::MarkerArray>("visualize", 50);
  ros::Subscriber subscription = nh.subscribe("continental_radar/objects", 50, callback);
  ros::spin();
}