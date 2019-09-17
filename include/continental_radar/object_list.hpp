//
// Created by shivesh on 9/14/19.
//

#ifndef CONTINENTAL_RADAR_OBJECT_LIST_HPP
#define CONTINENTAL_RADAR_OBJECT_LIST_HPP

#include <cstdint>

namespace continental_radar
{
namespace object_list
{
typedef enum Object_DynProp {
  MOVING = 0x0,
  STATIONARY = 0x1,
  ONCOMING = 0x2,
  STATIONARY_CANDIDATE = 0x3,
  UNKNOWN = 0x4,
  CROSSING_STATIONARY = 0x5,
  CROSSING_MOVING = 0x6,
  STOPPED = 0x7,
} Object_DynProp;

typedef enum Object_Class {
  POINT = 0x0,
  CAR = 0x1,
  TRUCK = 0x2,
  PEDESTRIAN = 0x3,
  MOTORCYCLE = 0x4,
  BICYCLE = 0x5,
  WIDE = 0x6,
  RESERVED = 0x7,
} Object_Class;

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

class Object_0_Status
{
 public:
  Object_0_Status();

  ~Object_0_Status();

  int get_no_of_objects();

  int get_measurement_cycle_counter();

  int get_interface_version();

  object_0_status * get_object_0_status();

 private:
  object_0_status object_0_status_msg;
};

class Object_1_General
{
 public:
  Object_1_General();

  ~Object_1_General();

  int get_object_id();

  double get_object_long_dist();

  double get_object_lat_dist();

  double get_object_long_rel_vel();

  Object_DynProp get_object_dyn_prop();

  double get_object_lat_rel_vel();

  double get_object_rcs();

  object_1_general * get_object_1_general();

 private:
  object_1_general object_1_general_msg;
};

class Object_2_Quality
{
 public:
  Object_2_Quality();

  ~Object_2_Quality();

  int get_object_id();

  int get_object_long_dist_rms();

  int get_object_long_rel_vel_rms();

  int get_object_lat_dist_rms();

  int get_object_pdh0();

  double get_object_lat_rel_vel_rms();
//
//  Object_AmbigState get_object_ambiguity_state();
//
//  Object_InvalidState get_object_validity_state();
//
//  object_2_quality * get_object_2_quality();
//
// private:
//  object_2_quality object_2_quality_msg;
};

class Object_3_Extended
{
 public:
  Object_3_Extended();

  ~Object_3_Extended();

  int get_object_id();

  double get_object_long_rel_accel();

  double get_object_lat_rel_accel();

  Object_Class get_object_class();

  double get_object_orientation_angle();

  double get_object_length();

  double get_object_width();

  object_3_extended * get_object_3_extended();

 private:
  object_3_extended object_3_extended_msg;
};
}
}

#endif //CONTINENTAL_RADAR_OBJECT_LIST_HPP
