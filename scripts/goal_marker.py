#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose2D, PoseStamped
import tf

# how is nav_cmd being decided -- human manually setting it, or rviz
rviz = rospy.get_param("rviz")
# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = rospy.get_param("map")

def marker_pub():
    rospy.init_node('goal_marker', anonymous=True)
    # if using rviz, we can subscribe to nav goal click
    if rviz:
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, rviz_goal_callback)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()
    	

def rviz_goal_callback(msg):
    """ callback for a pose goal sent through rviz """
    origin_frame = "/map" if mapping else "/odom"
    try:
        trans_listener = tf.TransformListener()
        nav_pose_origin = trans_listener.transformPose(origin_frame, msg)
        x_g = nav_pose_origin.pose.position.x
        y_g = nav_pose_origin.pose.position.y
        quaternion = (
                nav_pose_origin.pose.orientation.x,
                nav_pose_origin.pose.orientation.y,
                nav_pose_origin.pose.orientation.z,
                nav_pose_origin.pose.orientation.w)

        marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        marker = Marker(
            type=Marker.ARROW,
            id=1,
            lifetime=rospy.Duration(5),
            pose=Pose(Point(x_g, y_g, 0.0), Quaternion(*quaternion)),
            scale=Vector3(0.5, 0.05, 0.20),
            header=Header(frame_id='map'),
            color=ColorRGBA(0.0, 1.0, 0.0, 0.8))
        marker_publisher.publish(marker)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass


if __name__ == '__main__':
    try:
        marker_pub()
    except rospy.ROSInterruptException:
        pass
