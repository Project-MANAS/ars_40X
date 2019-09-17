//
// Created by shivesh on 9/17/19.
//

#include "continental_radar/cluster_list.hpp"

namespace continental_radar
{
Cluster_0_Status::Cluster_0_Status()
{
}

Cluster_0_Status::~Cluster_0_Status()
{
}

int Cluster_0_Status::get_no_of_clusters_near()
{
  return cluster_0_status_msg.data.Cluster_NofClustersNear;
}

int Cluster_0_Status::get_no_of_clusters_far()
{
  return cluster_0_status_msg.data.Cluster_NofClustersFar;
}
}