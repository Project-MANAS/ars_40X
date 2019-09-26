//
// Created by shivesh on 9/17/19.
//

#include "continental_radar/ros/object_list_ros.hpp"

#include <iostream>

namespace continental_radar
{
ObjectListROS::ObjectListROS(ros::NodeHandle& nh, ContinentalRadarCAN * continental_radar_can) :
  continental_radar_can_(continental_radar_can)
{
  object_0_status_ = continental_radar_can_->get_object_0_status();
  object_1_general_ = continental_radar_can_->get_object_1_general();
  object_2_quality_ = continental_radar_can_->get_object_2_quality();
  object_3_extended_ = continental_radar_can_->get_object_3_extended();
  objects_data_pub_ = nh.advertise<continental_radar::ObjectList>("continental_radar/objects", 10);
}

ObjectListROS::~ObjectListROS()
{
}

void ObjectListROS::send_object_0_status() {
  object_list.header.stamp = ros::Time::now();
  objects_data_pub_.publish(object_list);
  object_list.objects.clear();
  object_id_ = 0;
}

void ObjectListROS::send_object_1_general() {
  Object object;
  object.id = object_1_general_->get_object_id();
  object.position.x = object_1_general_->get_object_long_dist();
  object.position.y = object_1_general_->get_object_lat_dist();
  object.relative_velocity.x = object_1_general_->get_object_long_rel_vel();
  object.relative_velocity.y = object_1_general_->get_object_lat_rel_vel();
  object_list.objects.push_back(object);
}

void ObjectListROS::send_object_3_extended(){
  object_list.objects[object_id_].length = object_3_extended_->get_object_length();
  object_list.objects[object_id_].width = object_3_extended_->get_object_width();
  object_list.objects[object_id_].orientation_angle = object_3_extended_->get_object_orientation_angle();
  object_list.objects[object_id_].class_type = object_3_extended_->get_object_class();
  ++object_id_;
}
}