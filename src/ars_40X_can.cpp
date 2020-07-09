//
// Created by shivesh on 9/13/19.
//

#include "ars_40X/ars_40X_can.hpp"

namespace ars_40X {
ARS_40X_CAN::ARS_40X_CAN() :
    can_("can0"),
    sensor_id_offset_(0) {
}

ARS_40X_CAN::ARS_40X_CAN(std::string port, uint8_t sensor_id) :
    can_(port.c_str()),
    sensor_id_offset_(0x010 * sensor_id) {
      if (sensor_id > 7) throw std::invalid_argument("Invalid sensor id, must be in [0, 7].");
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

  if (frame_id == RadarState + sensor_id_offset_) {
    memcpy(radar_state_.get_radar_state()->raw_data, data, dlc);
    send_radar_state();
  }
  else if (frame_id == Cluster_0_Status + sensor_id_offset_) {
    memcpy(cluster_0_status_.get_cluster_0_status()->raw_data, data, dlc);
    send_cluster_0_status();
  }
  else if (frame_id == Cluster_1_General + sensor_id_offset_) {
    memcpy(cluster_1_general_.get_cluster_1_general()->raw_data, data, dlc);
    send_cluster_1_general();
  }
  else if (frame_id == Cluster_2_Quality + sensor_id_offset_) {
    memcpy(cluster_2_quality_.get_cluster_2_quality()->raw_data, data, dlc);
    send_cluster_2_quality();
  }
  else if (frame_id == Object_0_Status + sensor_id_offset_) {
    memcpy(object_0_status_.get_object_0_status()->raw_data, data, dlc);
    send_object_0_status();
  }
  else if (frame_id == Object_1_General + sensor_id_offset_) {
    memcpy(object_1_general_.get_object_1_general()->raw_data, data, dlc);
    send_object_1_general();
  }
  else if (frame_id == Object_2_Quality + sensor_id_offset_) {
    memcpy(object_2_quality_.get_object_2_quality()->raw_data, data, dlc);
    send_object_2_quality();
  }
  else if (frame_id == Object_3_Extended + sensor_id_offset_) {
    memcpy(object_3_extended_.get_object_3_extended()->raw_data, data, dlc);
    send_object_3_extended();
  }
#if DEBUG
  else printf("Unidentified Message: %d\n", frame_id);
#endif
  return true;
}

bool ARS_40X_CAN::send_radar_data(uint32_t frame_id) {
  switch (frame_id) {
    case RadarCfg:can_.write(frame_id + sensor_id_offset_, 8, radar_cfg_.get_radar_cfg()->raw_data);
      break;
    case SpeedInformation:
      can_.write(frame_id + sensor_id_offset_,
                 2,
                 speed_information_.get_speed_information()->raw_data);
      break;
    case YawRateInformation:
      can_.write(frame_id + sensor_id_offset_,
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

void ARS_40X_CAN::update_sensor_id(uint8_t sensor_id) {
  if (sensor_id > 7) throw std::invalid_argument("Invalid sensor id, must be in [0, 7].");
  sensor_id_offset_ = 0x010 * sensor_id;
}
}
