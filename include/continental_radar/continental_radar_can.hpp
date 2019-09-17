//
// Created by shivesh on 9/13/19.
//

#ifndef CONTINENTAL_RADAR_CONTINENTAL_RADAR_CAN_HPP
#define CONTINENTAL_RADAR_CONTINENTAL_RADAR_CAN_HPP

#include <socket_can/socket_can.hpp>

#include "continental_radar/radar_state.hpp"
#include "continental_radar/radar_cfg.hpp"

#include <stdint.h>

namespace continental_radar
{
class ContinentalRadarCAN {
 public:
  ContinentalRadarCAN();

  ~ContinentalRadarCAN();

  virtual bool receive_radar_data();

  virtual bool send_radar_data();

 private:
  socket_can::SocketCAN can;

  radar_state::RadarState radar_state;

  radar_cfg::RadarCfg radar_cfg;
};
}

#endif //CONTINENTAL_RADAR_CONTINENTAL_RADAR_CAN_HPP
