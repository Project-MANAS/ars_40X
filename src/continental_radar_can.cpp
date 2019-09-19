//
// Created by shivesh on 9/13/19.
//

#include "continental_radar/continental_radar_can.hpp"

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c |"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

namespace continental_radar
{
ContinentalRadarCAN::ContinentalRadarCAN() :
  can_("can0")
{
}

ContinentalRadarCAN::ContinentalRadarCAN(std::string port) :
  can_(port.c_str())
{
}

ContinentalRadarCAN::~ContinentalRadarCAN()
{
}

bool ContinentalRadarCAN::receive_radar_data()
{
  uint32_t frame_id;
  uint8_t dlc;
  uint8_t data[8] = {0};
  bool read_status = can_.read(&frame_id, &dlc, data);
  if (!read_status) {
    return false;
  }
  switch (frame_id) {
    case 0x201:
      memcpy(radar_state_.get_radar_state()->raw_data, data, dlc);
      printf("Read status %d\n", radar_state_.get_read_status());
      printf("Write status %d\n", radar_state_.get_write_status());
      printf("Max distance %d\n", static_cast<int>(radar_state_.get_max_distance()));
      printf("Persistence %d\n", radar_state_.get_persistent_error_status());
      printf("Interference %d\n", radar_state_.get_interference_status());
      printf("Temperature %d\n", radar_state_.get_temperature_error_status());
      printf("Temporary %d\n", radar_state_.get_temporary_error_status());
      printf("Voltage %d\n", radar_state_.get_voltage_error_status());
      printf("Sensor id %d\n", radar_state_.get_sensor_id());
      printf("Sensor sort index %d\n", radar_state_.get_sort_index());
      printf("Power cfg %d\n", radar_state_.get_power_cfg());
      printf("Control relay cfg %d\n", radar_state_.get_ctrl_relay_cfg());
      printf("Output type cfg %d\n", radar_state_.get_output_type_cfg());
      printf("Send quality cfg %d\n", radar_state_.get_send_quality_cfg());
      printf("Ext info cfg %d\n", radar_state_.get_ext_info_cfg());
      printf("Motion rx cfg %d\n", radar_state_.get_motion_rx_state());
      printf("RCS cfg %d\n", radar_state_.get_rcs_threshold());
      printf("\n\n");
      break;

    case 0x600:
      memcpy(cluster_0_status_.get_cluster_0_status()->raw_data, data, dlc);
      printf("num near clusters %d\n", cluster_0_status_.get_no_of_clusters_near());
      printf("num far clusters %d\n", cluster_0_status_.get_no_of_clusters_far());
      printf("mesurement cycle%d\n", cluster_0_status_.get_measurement_cycle_counter());
      printf("interface version%d\n", cluster_0_status_.get_interface_version());
      printf("\n");
      break;

    case 0x701:
      memcpy(cluster_1_general_.get_cluster_1_general()->raw_data, data, dlc);
      printf("cluster id%d\n", cluster_1_general_.get_cluster_id());
      printf("long dist%f\n", cluster_1_general_.get_cluster_long_dist());
      printf("lat dist%f\n", cluster_1_general_.get_cluster_lat_dist());
      printf("Long Rel vel%f\n", cluster_1_general_.get_cluster_long_rel_vel());
      printf("Lat rel vel%f\n", cluster_1_general_.get_cluster_lat_rel_vel());
      printf("Dyn Prop%d\n", cluster_1_general_.get_cluster_dyn_prop());
      printf("RCS %f\n", cluster_1_general_.get_cluster_rcs());
      printf("\n");
      break;

    case 0x60A:
      memcpy(object_0_status_.get_object_0_status()->raw_data, data, dlc);
      printf("num near objects %d\n", object_0_status_.get_no_of_objects());
      printf("mesurement cycle%d\n", object_0_status_.get_measurement_cycle_counter());
      printf("interface version%d\n", object_0_status_.get_interface_version());
      printf("\n");
      send_object_0_status();
      break;

    case 0x60B:
      memcpy(object_1_general_.get_object_1_general()->raw_data, data, dlc);
//      for (int i = 0; i < 8; ++i) {
//        printf(BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(object_1_general_.get_object_1_general()->raw_data[i]));
//      }
//      printf("\n");
//      printf("object id%d\n", object_1_general_.get_object_id());
//      printf("long dist%f\n", object_1_general_.get_object_long_dist());
//      printf("lat dist%f\n", object_1_general_.get_object_lat_dist());
//      printf("Long Rel vel%f\n", object_1_general_.get_object_long_rel_vel());
//      printf("Lat rel vel%f\n", object_1_general_.get_object_lat_rel_vel());
//      printf("Dyn Prop%d\n", object_1_general_.get_object_dyn_prop());
//      printf("RCS %f\n", object_1_general_.get_object_rcs());
//      printf("\n");
      send_object_1_general();
      break;

    case 0x60D:
      memcpy(object_3_extended_.get_object_3_extended()->raw_data, data, dlc);
//      for (int i = 0; i < 8; ++i) {
//        printf(BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(object_3_extended_.get_object_3_extended()->raw_data[i]));
//      }
//      printf("\n");
//      printf("object id%d\n", object_3_extended_.get_object_id());
//      printf("long Accel%f\n", object_3_extended_.get_object_long_rel_accel());
//      printf("lat Accel%f\n", object_3_extended_.get_object_lat_rel_accel());
//      printf("Orientational angle%f\n", object_3_extended_.get_object_orientation_angle());
//      printf("Class%d\n", object_3_extended_.get_object_class());
//      printf("Length%f\n", object_3_extended_.get_object_length());
//      printf("Width %f\n", object_3_extended_.get_object_width());
//      printf("\n");
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

bool ContinentalRadarCAN::send_radar_data(uint32_t frame_id) {
  switch (frame_id) {
    case 0x200:
      can_.write(frame_id, 8, radar_cfg_.get_radar_cfg()->raw_data);
      break;
#if DEBUG
    default: printf("Frame ID not supported\n");
#endif
  }
  return true;
}

radar_state::RadarState * ContinentalRadarCAN::get_radar_state() {
  return & radar_state_;
}

radar_cfg::RadarCfg * ContinentalRadarCAN::get_radar_cfg(){
  return & radar_cfg_;
}

cluster_list::Cluster_0_Status * ContinentalRadarCAN::get_cluster_0_status() {
  return & cluster_0_status_;
}

cluster_list::Cluster_1_General * ContinentalRadarCAN::get_cluster_1_general() {
  return & cluster_1_general_;
}

cluster_list::Cluster_2_Quality * ContinentalRadarCAN::get_cluster_2_quality() {
  return & cluster_2_quality_;
}

object_list::Object_0_Status * ContinentalRadarCAN::get_object_0_status() {
  return & object_0_status_;
}

object_list::Object_1_General * ContinentalRadarCAN::get_object_1_general() {
  return & object_1_general_;
}

object_list::Object_3_Extended * ContinentalRadarCAN::get_object_3_extended() {
  return & object_3_extended_;
}
}
