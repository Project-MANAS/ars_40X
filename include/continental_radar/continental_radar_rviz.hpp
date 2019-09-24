//
// Created by shivesh on 9/13/19.
//

#ifndef CONTINENTAL_RADAR_CONTINENTAL_RADAR_RVIZ_HPP
#define CONTINENTAL_RADAR_CONTINENTAL_RADAR_RVIZ_HPP

#include <ros/ros.h>

namespace continental_radar
{
class ContinentalRadarRViz {
 public:
  ContinentalRadarRViz();

  ~ContinentalRadarRViz();

 private:
  void clusters_callback(continental_radar::ClusterList cluster_list);

  void objects_callback(continental_radar::ObjectList object_list);

  ros::Publisher clusters_pub_;

  ros::Publisher objects_pub_;

  ros::Subscriber clusters_sub_;

  ros::Subscriber objects_sub_;
};
}

#endif //CONTINENTAL_RADAR_CONTINENTAL_RADAR_RVIZ_HPP
