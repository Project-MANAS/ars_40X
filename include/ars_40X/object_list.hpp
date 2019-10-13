//
// Created by shivesh on 9/14/19.
//

#ifndef ARS_40X_OBJECT_LIST_HPP
#define ARS_40X_OBJECT_LIST_HPP

#include <cstdint>

namespace ars_40X {
namespace object_list {
typedef union object_0_status {
  struct {
    uint64_t Object_NofObjects:8;
    uint64_t Object_MeasCounter1:8;
    uint64_t Object_MeasCounter2:8;
    uint64_t Reserved:4;
    uint64_t Object_InterfaceVersion:4;
  } data = {};

  uint8_t raw_data[4];
} object_0_status;

typedef union object_1_general {
  struct {
    uint64_t Object_ID:8;
    uint64_t Object_DistLong1:8;
    uint64_t Object_DistLat1:3;
    uint64_t Object_DistLong2:5;
    uint64_t Object_DistLat2:8;
    uint64_t Object_VrelLong1:8;
    uint64_t Object_VrelLat1:6;
    uint64_t Object_VrelLong2:2;
    uint64_t Object_DynProp:3;
    uint64_t Reserved:2;
    uint64_t Object_VrelLat2:3;
    uint64_t Object_RCS:8;
  } data = {};
  uint8_t raw_data[8];
} object_1_general;

typedef union object_2_quality {
  struct {
    uint64_t Obj_ID:8;
    uint64_t Obj_DistLat_rms1:3;
    uint64_t Obj_DistLong_rms:5;
    uint64_t Obj_VrelLat_rms1:1;
    uint64_t Obj_VrelLong_rms:5;
    uint64_t Obj_DistLat_rms2:2;
    uint64_t Obj_ArelLong_rms1:4;
    uint64_t Obj_VrelLat_rms2:4;
    uint64_t Obj_Orientation_rms1:2;
    uint64_t Obj_ArelLat_rms:5;
    uint64_t Obj_ArelLong_rms2:1;
    uint64_t Reserved1:5;
    uint64_t Obj_Orientation_rms2:3;
    uint64_t Reserved2:2;
    uint64_t Obj_MeasState:3;
    uint64_t Obj_ProbOfExist:3;
  } data = {};
  uint8_t raw_data[8];
} object_2_quality;

typedef union object_3_extended {
  struct {
    uint64_t Object_ID:8;
    uint64_t Object_ArelLong1:8;
    uint64_t Object_ArelLat1:5;
    uint64_t Object_ArelLong2:3;
    uint64_t Object_Class:3;
    uint64_t Reserved:1;
    uint64_t Object_ArelLat2:4;
    uint64_t Object_OrientationAngle1:8;
    uint64_t Reserved2:6;
    uint64_t Object_OrientationAngle2:2;
    uint64_t Object_Length:8;
    uint64_t Object_Width:8;
  } data = {};
  uint8_t raw_data[8];
} object_3_extended;

class Object_0_Status {
 public:
  Object_0_Status();

  ~Object_0_Status();

  int get_no_of_objects();

  int get_measurement_cycle_counter();

  int get_interface_version();

  object_0_status *get_object_0_status();

 private:
  object_0_status object_0_status_msg;
};

class Object_1_General {
 public:
  Object_1_General();

  ~Object_1_General();

  int get_object_id();

  double get_object_long_dist();

  double get_object_lat_dist();

  double get_object_long_rel_vel();

  int get_object_dyn_prop();

  double get_object_lat_rel_vel();

  double get_object_rcs();

  object_1_general *get_object_1_general();

 private:
  object_1_general object_1_general_msg;
};

class Object_2_Quality {
 public:
  Object_2_Quality();

  ~Object_2_Quality();

  int get_object_id();

  double get_object_long_dist_rms();

  double get_object_long_rel_vel_rms();

  double get_object_lat_dist_rms();

  double get_object_lat_rel_vel_rms();

  double get_object_long_rel_accel_rms();

  double get_object_lat_rel_accel_rms();

  double get_object_orientation_rms();

  int get_object_meas_state();

  int get_object_prob_of_exist();

  object_2_quality *get_object_2_quality();

 private:
  object_2_quality object_2_quality_msg;

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

  double orientation_signal_value_table[32] = {
      0.005, 0.007, 0.010, 0.014,
      0.020, 0.029, 0.041, 0.058,
      0.082, 0.116, 0.165, 0.234,
      0.332, 0.471, 0.669, 0.949,
      1.346, 1.909, 2.709, 3.843,
      5.451, 7.734, 10.971, 15.565,
      22.081, 31.325, 44.439, 63.044,
      89.437, 126.881, 180.000
  };
};

class Object_3_Extended {
 public:
  Object_3_Extended();

  ~Object_3_Extended();

  int get_object_id();

  double get_object_long_rel_accel();

  double get_object_lat_rel_accel();

  int get_object_class();

  double get_object_orientation_angle();

  double get_object_length();

  double get_object_width();

  object_3_extended *get_object_3_extended();

 private:
  object_3_extended object_3_extended_msg;
};
}
}

#endif //ARS_40X_OBJECT_LIST_HPP
