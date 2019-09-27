//
// Created by shivesh on 9/17/19.
//

#include "continental_radar/object_list.hpp"

#include <iostream>
#include <continental_radar/object_list.hpp>

namespace continental_radar
{
namespace object_list
{
Object_0_Status::Object_0_Status()
{
}

Object_0_Status::~Object_0_Status()
{
}

int Object_0_Status::get_no_of_objects()
{
  return static_cast<int>(object_0_status_msg.data.Object_NofObjects);
}

int Object_0_Status::get_measurement_cycle_counter()
{
  return static_cast<int>(object_0_status_msg.data.Object_MeasCounter1 << 8
      | object_0_status_msg.data.Object_MeasCounter2);
}

int Object_0_Status::get_interface_version()
{
  return static_cast<int>(object_0_status_msg.data.Object_InterfaceVersion);
}

object_0_status * Object_0_Status::get_object_0_status()
{
  return & object_0_status_msg;
}

Object_1_General::Object_1_General()
{
}

Object_1_General::~Object_1_General()
{
}

int Object_1_General::get_object_id()
{
  return static_cast<int>(object_1_general_msg.data.Object_ID);
}

double Object_1_General::get_object_long_dist()
{
  return (object_1_general_msg.data.Object_DistLong1 << 5 |
    object_1_general_msg.data.Object_DistLong2) * 0.2 - 500.0;
}

double Object_1_General::get_object_lat_dist()
{
  return (object_1_general_msg.data.Object_DistLat1 << 8 |
      object_1_general_msg.data.Object_DistLat2) * 0.2 - 204.6;
}

double Object_1_General::get_object_long_rel_vel()
{
  return (object_1_general_msg.data.Object_VrelLong1 << 2 |
      object_1_general_msg.data.Object_VrelLong2) * 0.25 - 128.0;
}

double Object_1_General::get_object_lat_rel_vel()
{
  return (object_1_general_msg.data.Object_VrelLat1 << 3 |
      object_1_general_msg.data.Object_VrelLat2)  * 0.25 - 64.0;
}

int Object_1_General::get_object_dyn_prop()
{
  return object_1_general_msg.data.Object_DynProp;
}

double Object_1_General::get_object_rcs()
{
  return object_1_general_msg.data.Object_RCS  * 0.5 - 64.0;
}

object_1_general * Object_1_General::get_object_1_general()
{
  return & object_1_general_msg;
}

Object_2_Quality::Object_2_Quality()
{
}

Object_2_Quality::~Object_2_Quality()
{
}

Object_3_Extended::Object_3_Extended()
{
}

Object_3_Extended::~Object_3_Extended()
{
}

double Object_3_Extended::get_object_lat_rel_accel()
{
  return (object_3_extended_msg.data.Object_ArelLat1 << 4 |
      object_3_extended_msg.data.Object_ArelLat2) * 0.01 - 2.50;
}

double Object_3_Extended::get_object_orientation_angle()
{
  return (object_3_extended_msg.data.Object_OrientationAngle1 << 2 |
      object_3_extended_msg.data.Object_OrientationAngle2) * 0.4 - 180.0;
}

int Object_3_Extended::get_object_class()
{
  return object_3_extended_msg.data.Object_Class;
}

double Object_3_Extended::get_object_length()
{
  return object_3_extended_msg.data.Object_Length * 0.2;
}

double Object_3_Extended::get_object_width()
{
  return object_3_extended_msg.data.Object_Width * 0.2;
}

object_3_extended * Object_3_Extended::get_object_3_extended()
{
  return & object_3_extended_msg;
}
}
}