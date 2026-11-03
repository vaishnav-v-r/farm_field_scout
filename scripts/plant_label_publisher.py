#!/usr/bin/env python3
"""
plant_label_publisher.py

Publishes a MarkerArray of floating text labels ("#1", "#2", ...) positioned
above each known plant's real-world coordinates, read from plant_map.yaml.
Purely a visualization aid for RViz - has no effect on navigation or
inspection logic. Published once, latched, since plant positions are fixed
for the duration of a run.
"""

import rospy
import rospkg
import yaml
from visualization_msgs.msg import Marker, MarkerArray


def main():
    rospy.init_node('plant_label_publisher')

    pkg_path = rospkg.RosPack().get_path('farm_field_scout')
    with open(pkg_path + '/config/plant_map.yaml') as f:
        plants = yaml.safe_load(f)['plants']

    pub = rospy.Publisher('/plant_labels', MarkerArray, queue_size=1, latch=True)

    # Give the publisher a moment to register with roscore before sending -
    # a very common ROS gotcha: publishing immediately at startup can get
    # missed if no subscriber/connection exists yet, even with latch=True
    # needing the initial send to succeed.
    rospy.sleep(1.0)

    marker_array = MarkerArray()
    for plant in plants:
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "plant_labels"
        marker.id = plant['id']
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = plant['x']
        marker.pose.position.y = plant['y']
        marker.pose.position.z = 0.55  # float just above the foliage sphere
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3  # text height in meters

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.text = f"#{plant['id']}"
        marker.lifetime = rospy.Duration(0)  # 0 = never expire

        marker_array.markers.append(marker)

    pub.publish(marker_array)
    rospy.loginfo("plant_label_publisher: published %d plant labels to /plant_labels", len(plants))
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
