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
  can("can0")
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
  bool read_status = can.read(&frame_id, &dlc, data);
  if (!read_status) {
    return false;
  }
  switch (frame_id) {
    case 0x201:
      memcpy(radar_state.get_radar_state()->raw_data, data, dlc);
      printf("Read status %d\n", radar_state.get_read_status());
      printf("Write status %d\n", radar_state.get_write_status());
      printf("Max distance %d\n", static_cast<int>(radar_state.get_max_distance()));
      printf("Persistence %d\n", radar_state.get_persistent_error_status());
      printf("Interference %d\n", radar_state.get_interference_status());
      printf("Temperature %d\n", radar_state.get_temperature_error_status());
      printf("Temporary %d\n", radar_state.get_temporary_error_status());
      printf("Voltage %d\n", radar_state.get_voltage_error_status());
      printf("Sensor id %d\n", radar_state.get_sensor_id());
      printf("Sensor sort index %d\n", radar_state.get_sort_index());
      printf("Power cfg %d\n", radar_state.get_power_cfg());
      printf("Control relay cfg %d\n", radar_state.get_ctrl_relay_cfg());
      printf("Output type cfg %d\n", radar_state.get_output_type_cfg());
      printf("Send quality cfg %d\n", radar_state.get_send_quality_cfg());
      printf("Ext info cfg %d\n", radar_state.get_ext_info_cfg());
      printf("Motion rx cfg %d\n", radar_state.get_motion_rx_state());
      printf("RCS cfg %d\n", radar_state.get_rcs_threshold());
      printf("\n\n");
      break;

    case 0x600:
      memcpy(cluster_0_status.get_cluster_0_status()->raw_data, data, dlc);
      printf("num near clusters %d\n", cluster_0_status.get_no_of_clusters_near());
      printf("num far clusters %d\n", cluster_0_status.get_no_of_clusters_far());
      printf("mesurement cycle%d\n", cluster_0_status.get_measurement_cycle_counter());
      printf("interface version%d\n", cluster_0_status.get_interface_version());
      printf("\n");
      break;

    case 0x701:
      memcpy(cluster_1_general.get_cluster_1_general()->raw_data, data, dlc);
      printf("cluster id%d\n", cluster_1_general.get_cluster_id());
      printf("long dist%f\n", cluster_1_general.get_cluster_long_dist());
      printf("lat dist%f\n", cluster_1_general.get_cluster_lat_dist());
      printf("Long Rel vel%f\n", cluster_1_general.get_cluster_long_rel_vel());
      printf("Lat rel vel%f\n", cluster_1_general.get_cluster_lat_rel_vel());
      printf("Dyn Prop%d\n", cluster_1_general.get_cluster_dyn_prop());
      printf("RCS %f\n", cluster_1_general.get_cluster_rcs());
      printf("\n");
      break;

    case 0x60A:
      memcpy(object_0_status.get_object_0_status()->raw_data, data, dlc);
      printf("num near objects %d\n", object_0_status.get_no_of_objects());
      printf("mesurement cycle%d\n", object_0_status.get_measurement_cycle_counter());
      printf("interface version%d\n", object_0_status.get_interface_version());
      printf("\n");
      break;

    case 0x60B:
      memcpy(object_1_general.get_object_1_general()->raw_data, data, dlc);
      for (int i = 0; i < 8; ++i) {
        printf(BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(object_1_general.get_object_1_general()->raw_data[i]));
      }
      printf("\n");
      printf("object id%d\n", object_1_general.get_object_id());
      printf("long dist%f\n", object_1_general.get_object_long_dist());
      printf("lat dist%f\n", object_1_general.get_object_lat_dist());
      printf("Long Rel vel%f\n", object_1_general.get_object_long_rel_vel());
      printf("Lat rel vel%f\n", object_1_general.get_object_lat_rel_vel());
      printf("Dyn Prop%d\n", object_1_general.get_object_dyn_prop());
      printf("RCS %f\n", object_1_general.get_object_rcs());
      printf("\n");
      break;

    case 0x60D:
      memcpy(object_3_extended.get_object_3_extended()->raw_data, data, dlc);
      for (int i = 0; i < 8; ++i) {
        printf(BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(object_3_extended.get_object_3_extended()->raw_data[i]));
      }
      printf("\n");
      printf("object id%d\n", object_3_extended.get_object_id());
      printf("long Accel%f\n", object_3_extended.get_object_long_rel_accel());
      printf("lat Accel%f\n", object_3_extended.get_object_lat_rel_accel());
      printf("Orientational angle%f\n", object_3_extended.get_object_orientation_angle());
      printf("Class%d\n", object_3_extended.get_object_class());
      printf("Length%f\n", object_3_extended.get_object_length());
      printf("Width %f\n", object_3_extended.get_object_width());
      printf("\n");
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

bool ContinentalRadarCAN::send_radar_data(){
  uint32_t frame_id = 0x200;
  uint8_t dlc = 8;
  uint8_t data[8] = {0};
  radar_cfg.set_output_type(radar_cfg::SEND_OBJECTS);
  radar_cfg.set_max_distance(198);
  memcpy(data, radar_cfg.get_radar_cfg()->raw_data, 8);
  for(int i = 0; i < 8; ++i)
    printf(BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(data[i]));
  can.write(frame_id, dlc, data);
  return true;
}

}

int main() {
  continental_radar::ContinentalRadarCAN continentalRadarCAN;
  continentalRadarCAN.receive_radar_data();
  continentalRadarCAN.send_radar_data();
  while(true) {
    continentalRadarCAN.receive_radar_data();
  }
}