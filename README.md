# ARS_40X

 ARS_40X package contains a driver for the Continental radar ARS_404 / ARS_408.
 The package also contains a ROS Wrapper for the driver.

#### Requirements

- [socket_can](https://github.com/Project-MANAS/socket_can)

#### Launching with arguments

```bash
roslaunch ars_40X ars_40X.launch visualize:=true obstacle_array:=true
```

#### Arguments available

- **visualize** *(default:"true")* : Launches RViz to display the clusters/obstacles as markers.
- **obstacle_array** *(default:"false")* : Launches ars_40X_obstacle_array node which publishes obstacles as geometry_msgs/Polygon

#### Publications

|Message|Type|Description|Message Box|
|---|---|---|---|
|/radar_status|ars_40X/RadarStatus|Describe the radar configuration|0x201|
|/ars_40X/clusters|ars_40X/ClusterList|Raw clusters data from radar|0x600, 0x701|
|/ars_40X/objects|ars_40X/ObjectList|Raw objects data from radar|0x60A, 0x60B, 0x60C, 0x60D|
|/visualize_clusters|visualization_msgs/MarkerArray|Clusters markers for RViz visualization| - |
|/visualize_objects|visualization_msgs/MarkerArray|Object markers for RViz visualization| - |

#### Subscription

|Message|Type|Description|Message Box|
|---|---|---|---|
|/odom|nav_msgs/Odometry|Velocity and accleration information|0x300, 0x301|


#### Services
The following services are available for configuring the radar options available in 0x200

|Services|
|---|
|/set_ctrl_relay_cfg|
|/set_max_distance|
|/set_output_type|
|/set_radar_power|
|/set_rcs_threshold|
|/set_send_ext_info|
|/set_send_quality|
|/set_sensor_id|
|/set_sort_index|
|/set_store_in_nvm|

