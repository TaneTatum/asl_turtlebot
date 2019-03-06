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
		self.th = None
		self.trans_listener = tf.TransformListener()

		# if using gazebo, we have access to perfect state
		if use_gazebo:
			rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
		# stop sign detector
		rospy.Subscriber('/detector/stop_sign', DetectedObject, self.object_detected_callback)

		#Object list publisher
		self.object_list_publisher = rospy.Publisher('object_list', ObjectList, queue_size=10)


		self.objectList = ObjectList()
		self.objectList.name = objectName
		self.objectList.x = []
		self.objectList.y = []
		self.objectList.th = []

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

		distance_threshold = 10

		print('Object found')
		# Get current robot location
		x = self.x
		y = self.y
		th = self.th

		# If list is empty
		print self.objectList.x
		if self.objectList.x == []:
			print 'empty'
			# Add location to list
			self.objectList.name = 'stop_sign'
			self.objectList.x.append(x)
			self.objectList.y.append(y)
			self.objectList.th.append(th)
			# Publish new list
			self.object_list_publisher.publish(self.objectList)

		else:
			# Check distance to other objects in list
			N = len(self.objectList.x)	#Number of items in list
			dists = []
			for i in range(N):
				dist = ((x-self.objectList.x[i])**2 - (y-self.objectList.y[i])**2)**0.5
				dists.append(dist)
			# Check if closer than threshold to other objects in list
			if min(dists) < distance_threshold:
				# Add location to list
				self.objectList.x.append(x)
				self.objectList.y.append(y)
				self.objectList.th.append(th)
				# Publish new list
				self.object_list_publisher.publish(self.objectList)




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