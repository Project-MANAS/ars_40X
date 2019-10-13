//
// Created by shivesh on 9/14/19.
//

#ifndef ARS_40X_RADAR_CFG_ROS_HPP
#define ARS_40X_RADAR_CFG_ROS_HPP

#include <ros/ros.h>
#include <std_srvs/SetBool.h>

#include <cstdint>

#include "ars_40X/ars_40X_can.hpp"
#include "ars_40X/MaxDistance.h"
#include "ars_40X/OutputType.h"
#include "ars_40X/RadarPower.h"
#include "ars_40X/RCSThreshold.h"
#include "ars_40X/SensorID.h"
#include "ars_40X/SortIndex.h"

namespace ars_40X {
class RadarCfgROS {
 public:
  RadarCfgROS(ros::NodeHandle &nh, ARS_40X_CAN *ars_40X_can);

  ~RadarCfgROS();

  bool set_max_distance(
      MaxDistance::Request &req,
      MaxDistance::Response & /*res*/);

  bool set_sensor_id(
      SensorID::Request &req,
      SensorID::Response & /*res*/);

  bool set_radar_power(
      RadarPower::Request &req,
      RadarPower::Response & /*res*/);

  bool set_output_type(
      OutputType::Request &req,
      OutputType::Response & /*res*/);

  bool set_send_quality(
      std_srvs::SetBool::Request &req,
      std_srvs::SetBool::Response & /*res*/);

  bool set_send_ext_info(
      std_srvs::SetBool::Request &req,
      std_srvs::SetBool::Response & /*res*/);

  bool set_sort_index(
      SortIndex::Request &req,
      SortIndex::Response & /*res*/);

  bool set_ctrl_relay_cfg(
      std_srvs::SetBool::Request &req,
      std_srvs::SetBool::Response & /*res*/);

  bool set_store_in_nvm(
      std_srvs::SetBool::Request &req,
      std_srvs::SetBool::Response & /*res*/);

  bool set_rcs_threshold(
      RCSThreshold::Request &req,
      RCSThreshold::Response & /*res*/);

 private:
  ARS_40X_CAN *ars_40X_can_;

  radar_cfg::RadarCfg *radar_cfg_;

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

#endif //ARS_40X_RADAR_CFG_ROS_HPP
