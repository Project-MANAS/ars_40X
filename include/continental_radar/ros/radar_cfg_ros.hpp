//
// Created by shivesh on 9/14/19.
//

#ifndef CONTINENTAL_RADAR_RADAR_CFG_ROS_HPP
#define CONTINENTAL_RADAR_RADAR_CFG_ROS_HPP

#include <ros/ros.h>
#include <std_srvs/SetBool.h>

#include <cstdint>

#include "continental_radar/continental_radar_can.hpp"
#include "continental_radar/MaxDistance.h"
#include "continental_radar/OutputType.h"
#include "continental_radar/RadarPower.h"
#include "continental_radar/RCSThreshold.h"
#include "continental_radar/SensorID.h"
#include "continental_radar/SortIndex.h"

namespace continental_radar
{
class RadarCfgROS {
 public:
  RadarCfgROS(ros::NodeHandle& nh, ContinentalRadarCAN * continental_radar_can);

  ~RadarCfgROS();

  bool set_max_distance(
    MaxDistance::Request& req,
    MaxDistance::Response& /*res*/);

  bool set_sensor_id(
    SensorID::Request& req,
    SensorID::Response& /*res*/);

  bool set_radar_power(
    RadarPower::Request& req,
    RadarPower::Response& /*res*/);

  bool set_output_type(
    OutputType::Request& req,
    OutputType::Response& /*res*/);

  bool set_send_quality(
    std_srvs::SetBool::Request& req,
    std_srvs::SetBool::Response& /*res*/);

  bool set_send_ext_info(
    std_srvs::SetBool::Request& req,
    std_srvs::SetBool::Response& /*res*/);

  bool set_sort_index(
    SortIndex::Request& req,
    SortIndex::Response& /*res*/);

  bool set_ctrl_relay_cfg(
    std_srvs::SetBool::Request& req,
    std_srvs::SetBool::Response& /*res*/);

  bool set_store_in_nvm(
    std_srvs::SetBool::Request& req,
    std_srvs::SetBool::Response& /*res*/);

  bool set_rcs_threshold(
    RCSThreshold::Request& req,
    RCSThreshold::Response& /*res*/);

 private:
  ContinentalRadarCAN * continental_radar_can_;

  radar_cfg::RadarCfg * radar_cfg_;

  ros::ServiceServer set_max_distance_service_;

  ros::ServiceServer set_sensor_id_service_;

  ros::ServiceServer set_radar_power_service_;

  ros::ServiceServer set_output_type_service_;

  ros::ServiceServer set_send_quality_service_;

  ros::ServiceServer set_send_ext_info_service_;

  ros::ServiceServer set_sort_index_service_;

  ros::ServiceServer set_ctrl_relay_cfg_service_;

  ros::ServiceServer set_store_in_nvm_service_;

  ros::ServiceServer set_rcs_threshold_service_;
};
}

#endif //CONTINENTAL_RADAR_RADAR_CFG_ROS_HPP
