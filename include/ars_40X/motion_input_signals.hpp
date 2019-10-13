//
// Created by shivesh on 9/14/19.
//

#ifndef ARS_40X_MOTION_INPUT_SIGNALS_HPP
#define ARS_40X_MOTION_INPUT_SIGNALS_HPP

#include <cstdint>

namespace ars_40X {
namespace motion_input_signals {
typedef union speed_information {
  struct {
    uint64_t RadarDevice_Speed1:5;
    uint64_t Reserved:1;
    uint64_t RadarDevice_SpeedDirection:2;
    uint64_t RadarDevice_Speed2:8;
  } data = {};

  uint8_t raw_data[2];
} speed_information;

typedef enum RadarDevice_SpeedDirection {
  STANDSTILL = 0x0,
  FORWARD = 0x1,
  BACKWARD = 0x2,
} RadarDevice_SpeedDirection;

typedef union yaw_rate_information {
  struct {
    uint64_t RadarDevice_YawRate1:8;
    uint64_t RadarDevice_YawRate2:8;
  } data = {};

  uint8_t raw_data[2];
} yaw_rate_information;

class SpeedInformation {
 public:
  SpeedInformation();

  ~SpeedInformation();

  void set_speed(double speed);

  void set_speed_direction(RadarDevice_SpeedDirection direction);

  speed_information *get_speed_information();

 private:
  speed_information speed_information_msg;
};

class YawRateInformation {
 public:
  YawRateInformation();

  ~YawRateInformation();

  void set_yaw_rate(double yaw_rate);

  yaw_rate_information *get_yaw_rate_information();

 private:
  speed_information speed_information_msg;

  yaw_rate_information yaw_rate_information_msg;
};
}
}

#endif //ARS_40X_MOTION_INPUT_SIGNALS_HPP
