//
// Created by shivesh on 9/14/19.
//

#ifndef ARS_40X_CLUSTER_LIST_HPP
#define ARS_40X_CLUSTER_LIST_HPP

#include <cstdint>

namespace ars_40X {
namespace cluster_list {
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
    uint64_t Reserved:1;
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
    uint64_t Cluster_VrelLat_rms2:4;
    uint64_t Cluster_AmbigState:3;
    uint64_t Cluster_InvalidState:5;
  } data = {};
  uint8_t raw_data[5];
} cluster_2_quality;

class Cluster_0_Status {
 public:
  Cluster_0_Status();

  ~Cluster_0_Status();

  int get_no_of_clusters_near();

  int get_no_of_clusters_far();

  int get_measurement_cycle_counter();

  int get_interface_version();

  cluster_0_status *get_cluster_0_status();

 private:
  cluster_0_status cluster_0_status_msg;
};

class Cluster_1_General {
 public:
  Cluster_1_General();

  ~Cluster_1_General();

  int get_cluster_id();

  double get_cluster_long_dist();

  double get_cluster_lat_dist();

  double get_cluster_long_rel_vel();

  double get_cluster_lat_rel_vel();

  int get_cluster_dyn_prop();

  double get_cluster_rcs();

  cluster_1_general *get_cluster_1_general();

 private:
  cluster_1_general cluster_1_general_msg;
};

class Cluster_2_Quality {
 public:
  Cluster_2_Quality();

  ~Cluster_2_Quality();

  int get_cluster_id();

  double get_cluster_long_dist_rms();

  double get_cluster_long_rel_vel_rms();

  double get_cluster_lat_dist_rms();

  int get_cluster_pdh0();

  double get_cluster_lat_rel_vel_rms();

  int get_cluster_ambiguity_state();

  int get_cluster_validity_state();

  cluster_2_quality *get_cluster_2_quality();

 private:
  cluster_2_quality cluster_2_quality_msg;

  double signal_value_table[32] = {
      0.005, 0.006, 0.008, 0.011,
      0.014, 0.018, 0.023, 0.029,
      0.038, 0.049, 0.063, 0.081,
      0.105, 0.135, 0.174, 0.224,
      0.288, 0.371, 0.478, 0.616,
      0.794, 1.023, 1.317, 1.697,
      2.187, 2.817, 3.630, 4.676,
      6.025, 7.762, 10.000
  };
};
}
}

#endif //ARS_40X_CLUSTER_LIST_HPP
