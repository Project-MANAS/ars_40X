#!/usr/bin/env python

import rospy
from ars_40X.msg import Object, ObjectList, Cluster, ClusterList
from radar_msgs.msg import RadarDetection, RadarDetectionArray

def object_list_callback(object_list, detection_array_publisher):
    radar_detection_array = RadarDetectionArray()
    radar_detection_array.header = object_list.header
    for obj in object_list.objects:
        radar_detection = RadarDetection()
        radar_detection.detection_id = obj.id
        radar_detection.position = obj.position.pose.position
        radar_detection.velocity = obj.relative_velocity.twist.linear
        radar_detection.amplitude = obj.rcs
        radar_detection_array.detections.append(radar_detection)
    detection_array_publisher.publish(radar_detection_array)


if __name__ == '__main__':
    rospy.init_node('object_list_converter')
    publisher = rospy.Publisher("/radar_converter/detections", RadarDetectionArray, queue_size=10)
    callback_closure = lambda object_list: object_list_callback(object_list, publisher)
    rospy.Subscriber('/ars_40X/objects', ObjectList, callback_closure)
    rospy.spin()