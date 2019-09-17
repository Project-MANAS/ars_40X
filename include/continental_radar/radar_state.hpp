//
// Created by shivesh on 9/14/19.
//

#ifndef CONTINENTAL_RADAR_RADAR_STATE_HPP
#define CONTINENTAL_RADAR_RADAR_STATE_HPP

#include <cstdint>

namespace continental_radar
{
namespace radar_state
{
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

typedef enum RadarState_SortIndex {
  NO_SORTING = 0x0,
  SORTED_BY_RANGE = 0x1,
  SORTED_BY_RCS = 0x2,
} RadarState_SortIndex;

typedef enum RadarState_RadarPowerCfg {
  STANDARD = 0x0,
  TX_GAIN_3dB = 0x1,
  TX_GAIN_6dB = 0x2,
  TX_GAIN_9dB = 0x3,
} RadarState_RadarPowerCfg;

typedef enum RadarState_OutputTypeCfg {
  NONE = 0x0,
  SEND_OBJECTS = 0x1,
  SEND_CLUSTERS = 0x2,
} RadarState_OutputTypeCfg;

typedef enum RadarState_MotionRxState {
  INPUT_OK = 0x0,
  SPEED_MISSING = 0x1,
  YAW_RATE_MISSING = 0x2,
  SPEED_AND_YAW_RATE_MISSING = 0x3,
} RadarState_MotionRxState;

typedef enum RadarState_RCS_Threshold {
  STANDARD_SENSITIVITY = 0x0,
  HIGH_SENSITIVITY = 0x1,
} RadarState_RCS_Threshold;

class RadarState
{
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

  RadarState_SortIndex get_sort_index();

  RadarState_RadarPowerCfg get_power_cfg();

  bool get_ctrl_relay_cfg();

  RadarState_OutputTypeCfg get_output_type_cfg();

  bool get_send_quality_cfg();

  bool get_ext_info_cfg();

  RadarState_MotionRxState get_motion_rx_state();

  RadarState_RCS_Threshold get_rcs_threshold();

  radar_state * get_radar_state();

 private:
  radar_state radar_state_msg;
};
}
}

#endif //CONTINENTAL_RADAR_RADAR_STATE_HPP
