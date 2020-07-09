#!/usr/bin/env python

import rospy
from ars_40X.msg import Object, ObjectList, Cluster, ClusterList
from radar_msgs.msg import RadarDetection, RadarDetectionArray


def convert_to_detection(obj):
    radar_detection = RadarDetection()
    radar_detection.detection_id = obj.id
    radar_detection.position = obj.position.pose.position
    radar_detection.velocity = obj.relative_velocity.twist.linear
    radar_detection.amplitude = obj.rcs
    return radar_detection

def object_list_callback(object_list, detection_array_publisher):
    radar_detection_array = RadarDetectionArray()
    radar_detection_array.header = object_list.header
    try:
        objects = getattr(object_list, 'objects')
    except AttributeError:
        objects = getattr(object_list, 'clusters')
    radar_detection_array.detections = [convert_to_detection(obj) for obj in objects]
    detection_array_publisher.publish(radar_detection_array)


if __name__ == '__main__':
    rospy.init_node('object_list_converter')
    publisher = rospy.Publisher("/radar_converter/detections123", RadarDetectionArray, queue_size=10)
    
    object_list_callback_closure = lambda object_list: object_list_callback(object_list, publisher)
    rospy.Subscriber('/ars_40X/objects', ObjectList, object_list_callback_closure)
    
    cluster_list_callback_closure = lambda cluster_list: object_list_callback(cluster_list, publisher)
    rospy.Subscriber('/ars_40X/clusters', ClusterList, cluster_list_callback_closure)

    rospy.spin()