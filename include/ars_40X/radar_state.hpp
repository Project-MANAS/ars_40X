//
// Created by shivesh on 9/14/19.
//

#ifndef ARS_40X_RADAR_STATE_HPP
#define ARS_40X_RADAR_STATE_HPP

#include <cstdint>

namespace ars_40X {
namespace radar_state {
typedef union radar_state {
  struct {
    uint64_t Reserved:6;
    uint64_t RadarState_NVMReadStatus:1;
    uint64_t RadarState_NVMwriteStatus:1;
    uint64_t RadarState_MaxDistanceCfg1:8;
    uint64_t Reserved2:1;
    uint64_t RadarState_Voltage_Error:1;
    uint64_t RadarState_Temporary_Error:1;
    uint64_t RadarState_Temperature_Error:1;
    uint64_t RadarState_Interference:1;
    uint64_t RadarState_Persistent_Error:1;
    uint64_t RadarState_MaxDistanceCfg2:2;
    uint64_t RadarState_RadarPowerCfg1:2;
    uint64_t Reserved3:6;
    uint64_t RadarState_SensorID:3;
    uint64_t Reserved4:1;
    uint64_t RadarState_SortIndex:3;
    uint64_t RadarState_RadarPowerCfg2:1;
    uint64_t Reserved5:1;
    uint64_t RadarState_CtrlRelayCfg:1;
    uint64_t RadarState_OutputTypeCfg:2;
    uint64_t RadarState_SendQualityCfg:1;
    uint64_t RadarState_SendExtInfoCfg:1;
    uint64_t RadarState_MotionRxState:2;
    uint64_t Reserved6:8;
    uint64_t Reserved7:2;
    uint64_t RadarState_RCS_Threshold:3;
    uint64_t Reserved8:3;
  } data = {};

  uint8_t raw_data[8];
} radar_state;

class RadarState {
 public:
  RadarState();

  ~RadarState();

  bool get_read_status();

  bool get_write_status();

  uint64_t get_max_distance();

  bool get_persistent_error_status();

  bool get_interference_status();

  bool get_temperature_error_status();

  bool get_temporary_error_status();

  bool get_voltage_error_status();

  int get_sensor_id();

  int get_sort_index();

  int get_radar_power_cfg();

  bool get_ctrl_relay_cfg();

  int get_output_type_cfg();

  bool get_send_quality_cfg();

  bool get_ext_info_cfg();

  int get_motion_rx_state();

  int get_rcs_threshold();

  radar_state *get_radar_state();

 private:
  radar_state radar_state_msg;
};
}
}

#endif //ARS_40X_RADAR_STATE_HPP
