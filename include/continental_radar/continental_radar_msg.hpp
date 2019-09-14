//
// Created by shivesh on 9/14/19.
//

#ifndef CONTINENTAL_RADAR_CONTINENTAL_RADAR_MSG_HPP
#define CONTINENTAL_RADAR_CONTINENTAL_RADAR_MSG_HPP

#include <cstdint>

typedef enum Status {
  INACTIVE = 0b0,
  ACTIVE = 0b1,
} Status;

// RadarState (0x201)
typedef enum RadarState_Status {
  FAILED = 0b0,
  SUCCESSFUL = 0b1,
} RadarState_Status;

typedef enum RadarState_Interference {
  NO_INTERFERENCE = 0b0,
  INTERFERENCE_DETECTED = 0b1,
} RadarState_Interference;

typedef enum RadarState_Error {
  NO_ERROR = 0b0,
  ERROR_ACTIVE = 0b1,
} RadarState_Error;

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

typedef union radar_status {
  struct {
    uint64_t Reserved:5;
    uint64_t RadarState_NVMReadStatus:1;
    uint64_t RadarState_NVMwriteStatus:1;
    uint64_t RadarState_MaxDistanceCfg:10;
    uint64_t RadarState_Persistent_Error:1;
    uint64_t RadarState_Interference:1;
    uint64_t RadarState_Temperature_Error:1;
    uint64_t RadarState_Temporary_Error:1;
    uint64_t RadarState_Voltage_Error:1;
    uint64_t RadarState_SensorID:3;
    uint64_t RadarState_SortIndex:3;
    uint64_t RadarState_RadarPowerCfg:3;
    uint64_t RadarState_CtrlRelayCfg:1;
    uint64_t RadarState_OutputTypeCfg:2;
    uint64_t RadarState_SendQualityCfg:1;
    uint64_t RadarState_SendExtInfoCfg:1;
    uint64_t RadarState_MotionRxState:2;
    uint64_t RadarState_RCS_Threshold:3;
  } data = {};

  uint8_t raw_data[64];
} radar_status;

#endif //CONTINENTAL_RADAR_CONTINENTAL_RADAR_MSG_HPP
