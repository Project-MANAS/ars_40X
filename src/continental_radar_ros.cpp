//
// Created by shivesh on 9/13/19.
//

#include "continental_radar/continental_radar_ros.hpp"

namespace continental_radar {
ContinentalRadarROS::ContinentalRadarROS() {
}

ContinentalRadarROS::~ContinentalRadarROS() {
}

void ContinentalRadarROS::receive_data() {
  get_radar_cfg()->set_output_type(radar_cfg::SEND_OBJECTS);
  send_radar_data(0x200);
  while (ros::ok()) {
    receive_radar_data();
  }
}

void ContinentalRadarROS::run() {
  objects_data_pub_ = nh_.advertise<continental_radar::ObjectList>("continental_radar/objects", 10);
  receive_data_thread = std::thread(std::bind(&ContinentalRadarROS::receive_data, this));
  receive_data_thread.detach();
}

void ContinentalRadarROS::send_object_0_status() {
  object_list.header.stamp = ros::Time::now();
  objects_data_pub_.publish(object_list);
  object_list.objects.clear();
  object_id_ = 0;
}

void ContinentalRadarROS::send_object_1_general() {
  Object object;
  object.id = get_object_1_general()->get_object_id();
  object.position.x = get_object_1_general()->get_object_long_dist();
  object.position.y = get_object_1_general()->get_object_lat_dist();
  object.relative_velocity.x = get_object_1_general()->get_object_long_rel_vel();
  object.relative_velocity.y = get_object_1_general()->get_object_lat_rel_vel();
  object_list.objects.push_back(object);
}

void ContinentalRadarROS::send_object_3_extended(){
  object_list.objects[object_id_].length = get_object_3_extended()->get_object_length();
  object_list.objects[object_id_].width = get_object_3_extended()->get_object_width();
  object_list.objects[object_id_].orientation_angle = get_object_3_extended()->get_object_orientation_angle();
  ++object_id_;
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "continental_radar");
  continental_radar::ContinentalRadarROS continental_radar_ros;
  continental_radar_ros.run();
  ros::spin();
}
