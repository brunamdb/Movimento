#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty

def takeoff():
	global pub_TakeOff
	pub_TakeOff.publish(Empty())


def land():
	global pub_land
	pub_Land.publish(Empty())

if __name__ == '__main__':
	rospy.init_node('drone')
	pub_TakeOff = rospy.Publisher("bebop/takeoff", Empty, queue_size=10)
	pub_Land = rospy.Publisher("bebop/land", Empty, queue_size=10)
	rate = rospy.Rate(10) # 10hz
	count = 0
	while not rospy.is_shutdown():
		if count < 100:
			print("takeoff2")
			takeoff()
		else:
			print("land")
			land()
		rate.sleep()
		count += 1
