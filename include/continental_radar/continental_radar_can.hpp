//
// Created by shivesh on 9/13/19.
//

#ifndef CONTINENTAL_RADAR_CONTINENTAL_RADAR_CAN_HPP
#define CONTINENTAL_RADAR_CONTINENTAL_RADAR_CAN_HPP

#include <socket_can/socket_can.hpp>

#include "continental_radar/continental_radar_msg.hpp"

#include <stdint.h>

namespace continental_radar
{
class ContinentalRadarCAN {
 public:
  ContinentalRadarCAN();

  ~ContinentalRadarCAN();

  virtual bool receive_radar_data();

 private:
  socket_can::SocketCAN can;

  radar_status radar_status_msg;
};
}

#endif //CONTINENTAL_RADAR_CONTINENTAL_RADAR_CAN_HPP
