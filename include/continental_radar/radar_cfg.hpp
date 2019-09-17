//
// Created by shivesh on 9/14/19.
//

#ifndef CONTINENTAL_RADAR_RADAR_CFG_HPP
#define CONTINENTAL_RADAR_RADAR_CFG_HPP

#include <cstdint>

namespace continental_radar
{
namespace radar_cfg
{
typedef union radar_cfg {
  struct {
    uint64_t RadarCfg_MaxDistance_valid:1;
    uint64_t RadarCfg_SensorID_valid:1;
    uint64_t RadarCfg_RadarPower_valid:1;
    uint64_t RadarCfg_OutputType_valid:1;
    uint64_t RadarCfg_SendQuality_valid:1;
    uint64_t RadarCfg_SendExtInfo_valid:1;
    uint64_t RadarCfg_SortIndex_valid:1;
    uint64_t RadarCfg_StoreInNVM_valid:1;
    uint64_t RadarCfg_MaxDistance1:8;
    uint64_t Reserved:6;
    uint64_t RadarCfg_MaxDistance2:2;
    uint64_t Reserved2:8;
    uint64_t RadarCfg_SensorID:3;
    uint64_t RadarCfg_OutputType:2;
    uint64_t RadarCfg_RadarPower:3;
    uint64_t RadarCfg_CtrlRelay_valid:1;
    uint64_t RadarCfg_CtrlRelay:1;
    uint64_t RadarCfg_SendQuality:1;
    uint64_t RadarCfg_SendExtInfo:1;
    uint64_t RadarCfg_SortIndex:3;
    uint64_t RadarCfg_StoreInNVM:1;
    uint64_t RadarCfg_RCS_Threshold_valid:1;
    uint64_t RadarCfg_RCS_Threshold:3;
    uint64_t Reserved3:4;
    uint64_t Reserved4:8;
  } data = {};

  uint8_t raw_data[8];
} radar_cfg;

typedef enum RadarCfg_RadarPower {
  STANDARD = 0x0,
  TX_GAIN_3dB = 0x1,
  TX_GAIN_6dB = 0x2,
  TX_GAIN_9dB = 0x3,
} RadarCfg_RadarPower;

typedef enum RadarCfg_OutputType {
  NONE = 0x0,
  SEND_OBJECTS = 0x1,
  SEND_CLUSTERS = 0x2,
} RadarCfg_OutputType;

typedef enum RadarCfg_SortIndex {
  NO_SORTING = 0x0,
  SORTED_BY_RANGE = 0x1,
  SORTED_BY_RCS = 0x2,
} RadarCfg_SortIndex;

typedef enum RadarCfg_RCS_Threshold {
  STANDARD_SENSITIVITY = 0x0,
  HIGH_SENSITIVITY = 0x1,
} RadarCfg_RCS_Threshold;

class RadarCfg {
 public:
  RadarCfg();

  ~RadarCfg();

  void set_max_distance(uint64_t distance, bool valid = true);

  void set_sensor_id(uint64_t id, bool valid = true);

  void set_radar_power(RadarCfg_RadarPower power, bool valid = true);

  void set_output_type(RadarCfg_OutputType output_type, bool valid = true);

  void set_send_quality(bool quality, bool valid = true);

  void set_send_ext_info(bool send_ext, bool valid = true);

  void set_sort_index(bool sort_index, bool valid = true);

  void set_ctrl_relay_cfg(bool ctrl_relay, bool valid = true);

  void set_store_in_nvm(bool store_in_nvm, bool valid = true);

  void set_rcs_threshold(RadarCfg_RCS_Threshold rcs_threshold, bool valid = true);

  radar_cfg * get_radar_cfg();

 private:
  radar_cfg radar_cfg_msg;
};
}
}

#endif //CONTINENTAL_RADAR_RADAR_CFG_HPP
