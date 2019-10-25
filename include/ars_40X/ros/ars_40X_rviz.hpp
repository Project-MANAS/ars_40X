//
// Created by shivesh on 9/13/19.
//

#ifndef ARS_40X_ARS_40X_RVIZ_HPP
#define ARS_40X_ARS_40X_RVIZ_HPP

#include <ros/ros.h>

namespace ars_40X {
enum {
  POINT,
  CAR,
  TRUCK,
  PEDESTRIAN,
  MOTORCYCLE,
  BICYCLE,
  WIDE,
  RESERVED
};

enum {
  INVALID,
  PERCENT_25,
  PERCENT_50,
  PERCENT_75,
  PERCENT_90,
  PERCENT_99,
  PERCENT_99_9,
  PERCENT_100
};

class ContinentalRadarRViz {
 public:
  ContinentalRadarRViz();

  ~ContinentalRadarRViz();

 private:
  void clusters_callback(ars_40X::ClusterList cluster_list);

  void objects_callback(ars_40X::ObjectList object_list);

  ros::Publisher clusters_pub_;

  ros::Publisher objects_pub_;

  ros::Publisher velocity_pub_;

  ros::Subscriber clusters_sub_;

  ros::Subscriber objects_sub_;
};
}

#endif //ARS_40X_ARS_40X_RVIZ_HPP
