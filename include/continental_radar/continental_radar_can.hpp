//
// Created by shivesh on 9/13/19.
//

#ifndef CONTINENTAL_RADAR_CONTINENTAL_RADAR_CAN_HPP
#define CONTINENTAL_RADAR_CONTINENTAL_RADAR_CAN_HPP

#include <socket_can/socket_can.hpp>

#include "continental_radar/radar_state.hpp"
#include "continental_radar/radar_cfg.hpp"
#include "continental_radar/cluster_list.hpp"
#include "continental_radar/object_list.hpp"

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

  cluster_list::Cluster_0_Status cluster_0_status;

  cluster_list::Cluster_1_General cluster_1_general;

  object_list::Object_0_Status object_0_status;

  object_list::Object_1_General object_1_general;

  object_list::Object_3_Extended object_3_extended;
};
}

#endif //CONTINENTAL_RADAR_CONTINENTAL_RADAR_CAN_HPP
