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
      memcpy(radar_state.get_radar_state()->raw_data, data, 8);
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
//    default: {
//#if DEBUG
//      printf("Unidentified Message: %d\n", frame_id);
//#endif
//      break;
  }
  return true;
}

bool ContinentalRadarCAN::send_radar_data(){
  uint32_t frame_id = 0x200;
  uint8_t dlc = 8;
  uint8_t data[8] = {0};
  radar_cfg.set_max_distance(120);
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