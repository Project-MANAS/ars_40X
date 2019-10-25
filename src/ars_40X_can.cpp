//
// Created by shivesh on 9/13/19.
//

#include "ars_40X/ars_40X_can.hpp"

namespace ars_40X {
ARS_40X_CAN::ARS_40X_CAN() :
    can_("can0") {
}

ARS_40X_CAN::ARS_40X_CAN(std::string port) :
    can_(port.c_str()) {
}

ARS_40X_CAN::~ARS_40X_CAN() {
}

bool ARS_40X_CAN::receive_radar_data() {
  uint32_t frame_id;
  uint8_t dlc;
  uint8_t data[8] = {0};
  bool read_status = can_.read(&frame_id, &dlc, data);
  if (!read_status) {
    return false;
  }
  switch (frame_id) {
    case RadarState:memcpy(radar_state_.get_radar_state()->raw_data, data, dlc);
      send_radar_state();
      break;

    case Cluster_0_Status:memcpy(cluster_0_status_.get_cluster_0_status()->raw_data, data, dlc);
      send_cluster_0_status();
      break;

    case Cluster_1_General:memcpy(cluster_1_general_.get_cluster_1_general()->raw_data, data, dlc);
      send_cluster_1_general();
      break;

    case Cluster_2_Quality:memcpy(cluster_2_quality_.get_cluster_2_quality()->raw_data, data, dlc);
      send_cluster_2_quality();
      break;

    case Object_0_Status:memcpy(object_0_status_.get_object_0_status()->raw_data, data, dlc);
      send_object_0_status();
      break;

    case Object_1_General:memcpy(object_1_general_.get_object_1_general()->raw_data, data, dlc);
      send_object_1_general();
      break;

    case Object_2_Quality:memcpy(object_2_quality_.get_object_2_quality()->raw_data, data, dlc);
      send_object_2_quality();
      break;

    case Object_3_Extended:memcpy(object_3_extended_.get_object_3_extended()->raw_data, data, dlc);
      send_object_3_extended();
      break;

    default: {
#if DEBUG
      printf("Unidentified Message: %d\n", frame_id);
#endif
      break;
    }
  }
  return true;
}

bool ARS_40X_CAN::send_radar_data(uint32_t frame_id) {
  switch (frame_id) {
    case RadarCfg:can_.write(frame_id, 8, radar_cfg_.get_radar_cfg()->raw_data);
      break;
    case SpeedInformation:
      can_.write(frame_id,
                 2,
                 speed_information_.get_speed_information()->raw_data);
      break;
    case YawRateInformation:
      can_.write(frame_id,
                 2,
                 yaw_rate_information_.get_yaw_rate_information()->raw_data);
      break;
#if DEBUG
      default: printf("Frame ID not supported\n");
#endif
  }
  return true;
}

cluster_list::Cluster_0_Status *ARS_40X_CAN::get_cluster_0_status() {
  return &cluster_0_status_;
}

cluster_list::Cluster_1_General *ARS_40X_CAN::get_cluster_1_general() {
  return &cluster_1_general_;
}

cluster_list::Cluster_2_Quality *ARS_40X_CAN::get_cluster_2_quality() {
  return &cluster_2_quality_;
}

motion_input_signals::SpeedInformation *ARS_40X_CAN::get_speed_information() {
  return &speed_information_;
}

motion_input_signals::YawRateInformation *ARS_40X_CAN::get_yaw_rate_information() {
  return &yaw_rate_information_;
}

object_list::Object_0_Status *ARS_40X_CAN::get_object_0_status() {
  return &object_0_status_;
}

object_list::Object_1_General *ARS_40X_CAN::get_object_1_general() {
  return &object_1_general_;
}

object_list::Object_2_Quality *ARS_40X_CAN::get_object_2_quality() {
  return &object_2_quality_;
}

object_list::Object_3_Extended *ARS_40X_CAN::get_object_3_extended() {
  return &object_3_extended_;
}

radar_state::RadarState *ARS_40X_CAN::get_radar_state() {
  return &radar_state_;
}

radar_cfg::RadarCfg *ARS_40X_CAN::get_radar_cfg() {
  return &radar_cfg_;
}
}
