//
// Created by shivesh on 9/14/19.
//

#ifndef CONTINENTAL_RADAR_CLUSTER_LIST_HPP
#define CONTINENTAL_RADAR_CLUSTER_LIST_HPP

#include <cstdint>

namespace continental_radar
{

typedef struct Cluster_DynProp {
  MOVING = 0x0,
  STATIONARY = 0x1,
  ONCOMING = 0x2,
  STATIONARY_CANDIDATE = 0x3,
  UNKNOWN = 0x4,
  CROSSING_STATIONARY = 0x5,
  CROSSING_MOVING = 0x6,
  STOPPED = 0x7,
} Cluster_DynProp;

typedef struct Cluster_AmbigState {
  INVALID = 0x0,
  AMBIGUOUS = 0x1,
  STAGGERED_RAMP = 0x2,
  UNAMBIGUOUS = 0x3,
  STATIONARY = 0x4,
};

typedef struct Cluster_InvalidState {
  VALID = 0x00,
  INVALID_LOW_RCS = 0x01,
  INVALID_NEAR_FIELD_ARTEFACT = 0x02,
  INVALID_FAR_RANGE_CLUSTER = 0x03,
  VALID_LOW_RCS = 0x04,
  RESERVED = 0x05,
  INVALID_HIGH_MIRROR_PROBABILITY = 0x06,
  INVALID_SENSOR_FIELD_OF_VIEW = 0x07,
  VALID_AZIMUTH_CORRECTION = 0x08,
  VALID_HIGH_CHILD_PROBALITY = 0x09,
  VALID_HIGH_PROBABILITY = 0x0A,
  VALID_NO_LOCAL_MAX = 0x0B,
  VALID_HIGH_ARTEFACT_PROBABILITY = 0x0C,
  RESERVED2 = 0x0D,
  INVALID_HARMONICS = 0x0E,
  VALID_ABOVE_95_M = 0x0F,
  VALID_HIGH_MULTI_TARGET_PROBABILITY = 0x10,
  VALID_SUSPICIOUS_ANGLE = 0x11,
};

typedef union cluster_0_status {
  struct {
    uint64_t Cluster_NofClustersNear:8;
    uint64_t Cluster_NofClustersFar:8;
    uint64_t Cluster_MeasCounter1:8;
    uint64_t Cluster_MeasCounter2:8;
    uint64_t Reserved:4;
    uint64_t Cluster_InterfaceVersion:4;
  } data = {};

  uint8_t raw_data[5];
} cluster_0_status;

typedef union cluster_1_general {
  struct {
    uint64_t Cluster_ID:8;
    uint64_t Cluster_DistLong1:8;
    uint64_t Cluster_DistLat1:2;
    uint64_t Reserved:1
    uint64_t Cluster_DistLong2:5;
    uint64_t Cluster_DistLat2:8;
    uint64_t Cluster_VrelLong1:8;
    uint64_t Cluster_VrelLat1:6;
    uint64_t Cluster_VrelLong2:2;
    uint64_t Cluster_DynProp:3;
    uint64_t Reserved2:2;
    uint64_t Cluster_VrelLat2:3;
    uint64_t Cluster_RCS:8;
  } data = {};
  uint8_t raw_data[8];
} cluster_1_general;

typedef union cluster_2_quality {
  struct {
    uint64_t Cluster_ID:8;
    uint64_t Cluster_DistLat_rms1:3;
    uint64_t Cluster_DistLong_rms:5;
    uint64_t Cluster_VrelLat_rms1:1;
    uint64_t Cluster_VrelLong_rms:5;
    uint64_t Cluster_DistLat_rms2:2;
    uint64_t Cluster_Pdh0:3;
    uint64_t Reserved:1;
    uint64_t Cluster_VrelLat_rms2:5;
    uint64_t Cluster_AmbigState:3;
    uint64_t Cluster_InvalidState:5;
  } data = {};
  uint8_t raw_data[5];
} cluster_2_quality;

class Cluster_0_Status
{
 public:
  Cluster_0_Status();

  ~Cluster_0_Status();

  int get_no_of_clusters_near();

  int get_no_of_clusters_far();

  int get_measurement_cycle_counter();

  int get_interface_version();

  cluster_0_status get_cluster_0_status();

 private:
  cluster_0_status cluster_0_status_msg;
};

class Cluster_1_General
{
 public:
  Cluster_1_General();

  ~Cluster_1_General();

  int get_cluster_id();

  double get_cluster_long_dist();

  double get_cluster_lat_dist();

  double get_cluster_long_rel_vel();

  double get_cluster_lat_rel_vel();

  double get_cluster_lat_rel_vel();

  Cluster_DynProp get_dyn_prop();

  double get_rcs();

 private:
  cluster_1_general cluster_1_general_msg;
};

class Cluster_2_Quality
{
 public:
  Cluster_2_Quality();

  ~Cluster_2_Quality();

  int get_cluster_id();

  int get_cluster_long_dist_rms();

  int get_cluster_long_rel_vel_rms();

  int get_cluster_lat_dist_rms();

  int get_cluster_pdh0();

  double get_cluster_lat_rel_vel_rms();

  Cluster_AmbigState get_cluster_ambiguity_state();

  Cluster_InvalidState get_cluster_validity_state();

 private:
  cluster_2_quality cluster_2_quality_msg;
};
}

#endif //CONTINENTAL_RADAR_CLUSTER_LIST_HPP
