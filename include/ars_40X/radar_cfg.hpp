//
// Created by shivesh on 9/14/19.
//

#ifndef ARS_40X_RADAR_CFG_HPP
#define ARS_40X_RADAR_CFG_HPP

#include <algorithm>
#include <cstdint>

namespace ars_40X {
namespace radar_cfg {
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

class RadarCfg {
 public:
  RadarCfg();

  ~RadarCfg();

  bool set_max_distance(uint64_t distance, bool valid = true);

  bool set_sensor_id(int id, bool valid = true);

  bool set_radar_power(int power, bool valid = true);

  bool set_output_type(int output_type, bool valid = true);

  void set_send_quality(bool quality, bool valid = true);

  void set_send_ext_info(bool send_ext, bool valid = true);

  bool set_sort_index(int sort_index, bool valid = true);

  void set_ctrl_relay_cfg(bool ctrl_relay, bool valid = true);

  void set_store_in_nvm(bool store_in_nvm, bool valid = true);

  bool set_rcs_threshold(int rcs_threshold, bool valid = true);

  radar_cfg *get_radar_cfg();

 private:
  radar_cfg radar_cfg_msg;
};
}
}

#endif //ARS_40X_RADAR_CFG_HPP
