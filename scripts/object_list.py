#!/usr/bin/env python

import rospy
import tf
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from asl_turtlebot.msg import DetectedObject, ObjectList

# if sim is True/using gazebo, therefore want to subscribe to /gazebo/model_states\
# otherwise, they will use a TF lookup
use_gazebo = rospy.get_param("sim")
# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = rospy.get_param("map")

class Object_List:

	def __init__(self, objectName=None):
		rospy.init_node('object_list', anonymous=True)

		#Initialize Variables
		self.x = None
		self.y = None
		self.theta = None
		self.trans_listener = tf.TransformListener()
		self.objectName = objectName
		self.xList = []
		self.yList = []
		self.thetaList = []
	

		# if using gazebo, we have access to perfect state
		if use_gazebo:
			rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
		# stop sign detector
		rospy.Subscriber('/detector/stop_sign', DetectedObject, self.object_detected_callback)

		#Object list publisher
		self.object_list_publisher = rospy.Publisher('object_list', ObjectList, queue_size=10)


	# Not used if not using gazebo
	def gazebo_callback(self, msg):
		pose = msg.pose[msg.name.index("turtlebot3_burger")]
		twist = msg.twist[msg.name.index("turtlebot3_burger")]
		self.x = pose.position.x
		self.y = pose.position.y
		quaternion = (
		    pose.orientation.x,
		    pose.orientation.y,
		    pose.orientation.z,
		    pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		self.theta = euler[2]


	def object_detected_callback(self, msg):
		""" callback for when the detector has found an object."""

		dist_threshold = 0.25
		
		print 'Object Found'

		#Get current position
		x = self.x
		y = self.y
		theta = self.theta

		N = len(self.xList)
		if N == 0:
			self.xList = [x]
			self.yList = [y]
			self.thetaList = [theta]
		else:
			dists = []
			for i in range(N):
				dist = ((x-self.xList[i])**2 + (y-self.yList[i])**2)**0.5
				dists.append(dist)
			if min(dists) > dist_threshold:
				self.xList.append(x)
				self.yList.append(y)
				self.thetaList.append(theta)

		obList = ObjectList()
		obList.name = self.objectName
		obList.x = self.xList
		obList.y = self.yList
		obList.th = self.thetaList

		self.object_list_publisher.publish(obList)



	def loop(self):
		# Set current location from gmapping
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
	stopSignList = Object_List('stop_sign')
	stopSignList.run()