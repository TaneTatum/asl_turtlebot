#!/usr/bin/env python

import rospy
import tf
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from asl_turtlebot.msg import ObjectList

# if sim is True/using gazebo, therefore want to subscribe to /gazebo/model_states\
# otherwise, they will use a TF lookup
use_gazebo = rospy.get_param("sim")
# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = rospy.get_param("map")

class Object_Marker:

    def __init__(self):
        rospy.init_node('object_marker', anonymous=True)
        self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        rospy.Subscriber('object_list', ObjectList, self.object_list_callback)

        self.trans_listener = tf.TransformListener()

    def object_list_callback(self,msg):
        name = msg.name

        if name == 'stop_sign':
            N = len(msg.x)
            for i in range(N):
                x = msg.x[i]
                y = msg.y[i]
                th = msg.th[i]
                # Put marker ahead of robot
                dist = 0.5
                x = x + dist*np.cos(th)
                y = y + dist*np.sin(th)

                marker = Marker(
                    type=Marker.CYLINDER,
                    id=i+10,
                    lifetime=rospy.Duration(5),
                    pose=Pose(Point(x, y, 0.0), Quaternion(0, 0, 0, 1)),
                    scale=Vector3(0.05, 0.05, 1.0),
                    header=Header(frame_id='map'),
                    color=ColorRGBA(1.0, 0.0, 0.0, 1.0))
                self.marker_publisher.publish(marker)

    def loop(self):
        # Get current location from gmapping
        if not use_gazebo:
            try:
                origin_frame = "/map" if mapping else "/odom"
                (translation,rotation) = self.trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
                self.x = translation[0]
                self.y = translation[1]
                euler = tf.transformations.euler_from_quaternion(rotation)
                self.theta = euler[2]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass


    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    objectMarker = Object_Marker()
    objectMarker.run()