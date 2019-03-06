#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA

def marker_pub():
    marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rospy.init_node('turtlebot_marker', anonymous=True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
    	marker = Marker(
            type=Marker.CYLINDER,
            id=0,
            lifetime=rospy.Duration(1.5),
            pose=Pose(Point(0.0, 0.0, 0.0), Quaternion(0, 0, 0, 1)),
            scale=Vector3(0.15, 0.15, 0.20),
            header=Header(frame_id='base_footprint'),
            color=ColorRGBA(1.0, 0.0, 0.0, 1.0))
    	marker_publisher.publish(marker)

if __name__ == '__main__':
    try:
        marker_pub()
    except rospy.ROSInterruptException:
        pass
