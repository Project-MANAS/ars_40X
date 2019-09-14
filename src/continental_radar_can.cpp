//
// Created by shivesh on 9/13/19.
//

#include "continental_radar/continental_radar_can.hpp"

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
  memcpy(radar_status_msg.raw_data, data, 8);
  printf("%s\n", radar_status_msg.raw_data);
//  switch (frame_id) {
//    case 0x201:
//      memcpy(radar_status_msg.raw_data, data, 8);
//      printf("%s\n", radar_status_msg.raw_data);
//      break;
//    default:
//#if DEBUG
//      printf("Unidentified Message: %d\n", frame_id);
//#endif
//      break;
//  }
  return true;
}
}


int main() {
  continental_radar::ContinentalRadarCAN continentalRadarCAN;
  while (true) {
    continentalRadarCAN.receive_radar_data();
    sleep(1);
  }
}