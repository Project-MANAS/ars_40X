//
// Created by shivesh on 9/13/19.
//

#ifndef CONTINENTAL_RADAR_CONTINENTAL_RADAR_CAN_HPP
#define CONTINENTAL_RADAR_CONTINENTAL_RADAR_CAN_HPP

#include <socket_can/socket_can.hpp>

#include "continental_radar/cluster_list.hpp"
#include "continental_radar/motion_input_signals.hpp"
#include "continental_radar/object_list.hpp"
#include "continental_radar/radar_cfg.hpp"
#include "continental_radar/radar_state.hpp"

#include <cstdint>
#include <string>

namespace continental_radar
{
class ContinentalRadarCAN {
 public:
  ContinentalRadarCAN();

  ContinentalRadarCAN(std::string port);

  ~ContinentalRadarCAN();

  virtual bool receive_radar_data();

  virtual bool send_radar_data(uint32_t frame_id);

  cluster_list::Cluster_0_Status * get_cluster_0_status();

  cluster_list::Cluster_1_General * get_cluster_1_general();

  cluster_list::Cluster_2_Quality * get_cluster_2_quality();

  motion_input_signals::SpeedInformation * get_speed_information();

  motion_input_signals::YawRateInformation * get_yaw_rate_information();

  object_list::Object_0_Status * get_object_0_status();

  object_list::Object_1_General * get_object_1_general();

  object_list::Object_3_Extended * get_object_3_extended();

  radar_state::RadarState * get_radar_state();

  radar_cfg::RadarCfg * get_radar_cfg();

  virtual void send_cluster_0_status() {};

  virtual void send_cluster_1_general() {};

  virtual void send_object_0_status() {};

  virtual void send_object_1_general() {};

  virtual void send_object_3_extended() {};

 private:
  socket_can::SocketCAN can_;

  cluster_list::Cluster_0_Status cluster_0_status_;

  cluster_list::Cluster_1_General cluster_1_general_;

  cluster_list::Cluster_2_Quality cluster_2_quality_;

  motion_input_signals::SpeedInformation speed_information_;

  motion_input_signals::YawRateInformation yaw_rate_information_;

  object_list::Object_0_Status object_0_status_;

  object_list::Object_1_General object_1_general_;

  object_list::Object_3_Extended object_3_extended_;

  radar_state::RadarState radar_state_;

  radar_cfg::RadarCfg radar_cfg_;
};
}

#endif //CONTINENTAL_RADAR_CONTINENTAL_RADAR_CAN_HPP
