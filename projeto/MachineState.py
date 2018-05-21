import rospy
import smach
import smach_ros

import time
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String, Empty
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
################################################################################

bridge = CvBridge()


countCam = 0  # Variavel para teste de Camera
atraso = 1.5  # Atraso aceitavel pela Camera


# Range de Cor a ser procurado (Tempoarario para testes)
cor_maior = np.array([70, 255, 255])
cor_menor = np.array([50, 50, 50])

################################################################################

def identifica_cor(frame):
	global cor_maior, cor_menor, frame_hsv
	frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)

	segmentado_cor = cv2.morphologyEx(segmentado_cor,cv2.MORPH_CLOSE,np.ones((7, 7)))

	img_out, contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	maior_contorno = None
	maior_contorno_area = 0


	for cnt in contornos:
		area = cv2.contourArea(cnt)
		if area > maior_contorno_area:
			maior_contorno = cnt
			maior_contorno_area = area

	if not maior_contorno is None :
		cv2.drawContours(frame, [maior_contorno], -1, [0, 0, 255], 5)
		maior_contorno = np.reshape(maior_contorno, (maior_contorno.shape[0], 2))
		media = maior_contorno.mean(axis=0)
		media = media.astype(np.int32)
		cv2.circle(frame, tuple(media), 5, [0, 255, 0])
	else:
		media = (0, 0)
	cv2.imshow('video', frame)
	cv2.imshow('seg', segmentado_cor)
	cv2.waitKey(1)

	centro = (frame.shape[0]//2, frame.shape[1]//2)
	return media, centro

def showImage(dado):
	global countCam
	countCam += 1
	now = rospy.get_rostime()
	imgtime = dado.header.stamp
	lag = now-imgtime
	delay = lag.secs
	if delay > atraso and check_delay==True:
		return
	antes = time.clock()
	cv_image = bridge.compressed_imgmsg_to_cv2(dado, "bgr8")
	media, centro = identifica_cor(cv_image)
	depois = time.clock()
	#  cv2.imshow("Camera", cv_image)

################################################################################

class TakeOff(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['Decolar','Pousar', "Mover"])
		self.count = 0
		self.countMax = 5000

	def execute(self, userdata):
		global pub_TakeOff
		rospy.loginfo('Executing state TAKEOFF')

		#comando para aprender a ler a cor
		self.count+=1
		pub_TakeOff.publish(Empty())
		if self.count > self.countMax: return 'Pousar'
		else: return 'Decolar'

class Land(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['Pousar','Decolar', "Mover"])
		self.Landed = 1

	def execute(self, userdata):
		global pub_Land
		rospy.loginfo('Executing state LAND')

		#comando para aprender a ler a cor
		pub_Land.publish(Empty())
		if self.Landed:
			self.Landed = 0
			return "Decolar"
		else: return 'Pousar'

class Move(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['Pousar','Decolar', "Mover"])

	def execute(self, userdata):
		global pub_Move
		rospy.loginfo('Executing state MOVE')

		#comando para aprender a ler a cor
		vel = Twist(Vector3(100,0,0), Vector3(0,0,0))
		pub_Move.publish(vel)
		return 'Mover'


def maquina():
	global pub_TakeOff, pub_Land, pub_Move
	rospy.init_node('Drone_State_Machine')
	pub_TakeOff = rospy.Publisher("bebop/takeoff", Empty, queue_size = 10)
	pub_Land = rospy.Publisher("bebop/land", Empty, queue_size = 10)
	pub_Move = rospy.Publisher("bebop/cmd_vel", Twist, queue_size = 10)

	sub_Cam = rospy.Subscriber("bebop/image_raw/compressed", CompressedImage, showImage, queue_size = 10)
	rate = rospy.Rate(10) # 10hz

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['terminei'])

	# Open the container
	with sm:

		# Add states to the container


		smach.StateMachine.add('TAKEOFF', TakeOff(),
								transitions = {'Decolar':'TAKEOFF',
												'Pousar':'MOVE',
												'Mover': 'MOVE'})

		smach.StateMachine.add('LAND', Land(),
								transitions = {'Pousar':'LAND',
												'Decolar':'TAKEOFF',
												'Mover':'MOVE'})

		smach.StateMachine.add('MOVE', Move(),
								transitions = {'Pousar':'LAND',
												'Decolar':'TAKEOFF',
												'Mover':'MOVE'})

	# Execute SMACH plan
	outcome = sm.execute()

if __name__=="__main__":
	maquina()
