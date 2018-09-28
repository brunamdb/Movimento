#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
from std_msgs.msg import Twist, Vector3

def takeoff(): #Publica nas vari'aveis para realizar a funcao
	global pub_TakeOff
	pub_TakeOff.publish(Empty())


def land():
	global pub_land
	pub_Land.publish(Empty())

def veloc(vel):
	global pub_Vel
	pub_Vel.publish(vel)

if __name__ == '__main__':
	rospy.init_node('drone')
	pub_TakeOff = rospy.Publisher("bebop/takeoff", Empty, queue_size=10) #Define variaveis
	pub_Land = rospy.Publisher("bebop/land", Empty, queue_size=10)
	pub_Vel = rospy.Publisher("bebop/cmd_vel", Twist, queue_size=10)
	rate = rospy.Rate(10) # 10hz
	count = 0
	estado = 0
	while not rospy.is_shutdown():
		if estado == 0:
			if count < 100:
				print("takeoff")
				takeoff()
			elif count == 100:
				estado += 1
		elif estado == 1:
			for i in range(0,100):
				x_linear = 0.15#float(-(2*math.sin(2*teta))/7)
				y_linear = 0.15#float((2*math.cos(2*teta))/7)
				z_linear = 0

				x_angular = 0
				y_angular = 0
				z_angular = 0 #Usar isso para que a frente do drone também vire.

				rospy.sleep(0.5)
				vel = Twist(Vector3(x_linear, y_linear, z_linear), Vector3(x_angular, y_angular, z_angular))
				veloc(vel)

			for i in range(0,10):
				x_linear = 0
				y_linear = 0
				z_linear = 0

				x_angular = 0
				y_angular = 0
				z_angular = 0

				rospy.sleep(0.5)
				vel = Twist(Vector3(x_linear, y_linear, z_linear), Vector3(x_angular, y_angular, z_angular))
				veloc(vel)
			estado += 1
		elif estado == 2:
			print("land")
			land()
		rate.sleep()
		count += 1
