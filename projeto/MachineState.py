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

atraso = 1.5  # Atraso aceitavel pela Camera

roiBox = None
roi_hist = None

PegarFundo = False
Fundo_Atual = []
Fundo_ComObjeto = []

frame_global = []

term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)

################################################################################



def CamShiftTrack(frame):
	global roiBox, roi_hist, term_crit
	print(roiBox)
	if roiBox is not None:
		# Making the frame into HSV and backproject the HSV frame
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		dst = cv2.calcBackProject([hsv],[0],roi_hist,[0,180],1)

		# Apply meanshift to get the new location
		ret, roiBox = cv2.CamShift(dst, roiBox, term_crit)

		# Draw it on image
		pts = cv2.boxPoints(ret)
		pts = np.int0(pts)
		cv2.polylines(frame,[pts],True, 255,2)

		# Draw the center
		cx = (pts[0][0] + pts[1][0]) / 2
		cy = (pts[0][1] + pts[2][1]) / 2
		cv2.circle(frame, (cx, cy), 4, (0, 255, 0), 2)
	cv2.imshow('CameraCam',frame)
	cv2.waitKey(1)



def showImage(dado):
	global Fundo_Atual, Fundo_ComObjeto
	global PegarFundo
	global frame_global

	now = rospy.get_rostime()
	imgtime = dado.header.stamp
	lag = now - imgtime
	delay = lag.secs
	if delay > atraso and check_delay == True:
		return
	antes = time.clock()
	cv_image = bridge.compressed_imgmsg_to_cv2(dado, "bgr8")
	CamShiftTrack(cv_image)
	# media, centro = identifica_cor(cv_image)
	depois = time.clock()
	if PegarFundo: Fundo_Atual = cv_image
	else: Fundo_ComObjeto = cv_image

	frame_global = cv_image

	cv2.imshow("Camera", cv_image)
	cv2.waitKey(1)



################################################################################


################################################################################

class TakeOff(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['Decolar','Sobreviver'])
		self.count = 0
		self.countMax = 5000

	def execute(self, userdata):
		global pub_TakeOff
		rospy.loginfo('Executing state TAKEOFF')

		#comando para aprender a ler a cor
		self.count+=1
		pub_TakeOff.publish(Empty())
		if self.count > self.countMax:
			return 'Sobreviver'
		else:
			return 'Decolar'

class StandBy(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['StandBy','Decolar'])
		self.count = 0
		self.countMax = 3000

	def execute(self, userdata):
		global pub_Move
		rospy.loginfo('Executing state STANDBY')

		self.count += 1
		if self.count > self.countMax:  # Gesto para levantar
			self.count = 0
			return 'Decolar'
		return 'StandBy'

class Survive(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['Sobreviver','AnalisarI',"Aprender"])
		self.count = 0
		self.countMax = 3000
		self.flagDono = False

	def execute(self, userdata):
		global pub_Move
		rospy.loginfo('Executing state SURVIVE')

		self.count += 1
		if self.count > self.countMax:  # Altura < 2,5m
			vel = Twist(Vector3(0,0,0.2), Vector3(0,0,0))
			pub_Move.publish(vel)
			self.count = 0
			if self.flagDono: return 'AnalisarI'
			else:
				self.flagDono = True
				self.count = 0
				return "Aprender"

		return 'Sobreviver'

class AnalisandoI(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['AnalisarI','Pousar','Manobra'])
		self.count = 0
		self.countMax = 3000

	def execute(self, userdata):
		global pub_Move
		rospy.loginfo('Executing state ANALISANDOI')

		self.count += 1
		vel = Twist(Vector3(0,0,0.2), Vector3(0,0,0))
		pub_Move.publish(vel)
		if self.count > self.countMax: # Gesto para pousar
			self.count = 0
			return 'Pousar'
		elif 1:  # Gesto para manobra
			self.count = 0
			return 'Manobra'
		return 'AnalisandoI'

class AnalisandoII(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['AnalisandoII','Mover','Girar'])
		self.count = 0
		self.countMax = 3000

	def execute(self, userdata):
		global pub_Move
		rospy.loginfo('Executing state ANALISANDOII')

		self.count += 1
		if self.count > self.countMax:  # Se nao estiver alinhado
			self.count = 0
			return 'Girar'
		elif 1:  # Se estiver alinhado
			self.count = 0
			return 'Mover'
		return 'AnalisandoII'

class Move(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['Mover','Sobreviver'])
		self.count = 0
		self.countMax = 3000

	def execute(self, userdata):
		global pub_Move
		rospy.loginfo('Executing state MOVE')

		self.count += 1
		vel = Twist(Vector3(0.2,0,0), Vector3(0,0,0))
		pub_Move.publish(vel)
		if self.count > self.countMax: # Segue o humano
			self.count = 0
			return 'Mover'
		return 'Sobreviver'

class Gira(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['Girar','Sobreviver'])
		self.count = 0
		self.countMax = 3000

	def execute(self, userdata):
		global pub_Move
		rospy.loginfo('Executing state MOVE')

		self.count += 1
		if self.count > self.countMax:
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0.2))
			pub_Move.publish(vel)
			return 'Girar'
		return 'Sobreviver'

class Manobra(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['Manobra','Sobreviver'])
		self.count = 0
		self.countMax = 3000

	def execute(self, userdata):
		global pub_Move
		rospy.loginfo('Executing state MANOBRA')

		self.count += 1
		if self.count > self.countMax: # Se teve um gesto de manobra
			return 'Manobrar'
		return 'Sobreviver'

class Land(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['Pousar','StandBy'])
		self.Landed = 1

	def execute(self, userdata):
		global pub_Land
		rospy.loginfo('Executing state LAND')

		pub_Land.publish(Empty())
		if self.Landed:
			self.Landed = 0
			return "StandBy"
		else:
			return 'Pousar'

class Aprender(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['Aprender','Sobreviver'])
		self.Dono = 0
		self.count = 0
		self.countLimit = 10000 # 10 segundos

	def execute(self, userdata):
		global pub_Land
		global PegarFundo
		rospy.loginfo('Executing state APRENDENDO')
		PegarFundo = True
		if self.count < self.countLimit:
			# bipa
			PegarFundo = True
		else:
			self.subtrairFundo()
			self.Dono = 1

		self.count += 1
		if self.Dono:
			self.count = 0
			return 'Sobreviver'
		else:
			return "Aprender"

	def subtrairFundo(self):
		global Fundo_Atual, Fundo_ComObjeto
		GrayLimit = 10


		grayFundo = cv2.GaussianBlur(Fundo_Atual, (5, 5), 0)
		grayComObjeto = cv2.GaussianBlur(Fundo_ComObjeto, (5, 5), 0)

		diferenca2 = cv2.subtract(grayComObjeto, grayFundo)
		diferenca = cv2.subtract(grayFundo, grayComObjeto)

		or_img = cv2.bitwise_or(diferenca, diferenca2)
		ret, limiar = cv2.threshold(or_img, np.percentile(or_img, 97), 255, cv2.THRESH_BINARY)

		kernel = np.ones((6, 6))
		limiar_open = cv2.morphologyEx(limiar, cv2.MORPH_OPEN, kernel)
		limiar_close = cv2.morphologyEx(limiar, cv2.MORPH_CLOSE, kernel)


		cv2.imwrite("grayFundo.jpg", grayComObjeto)
		cv2.imwrite("grayObjeto.jpg", grayFundo)
		cv2.imwrite("open.jpg", limiar_open)
		cv2.imwrite("close.jpg", limiar_close)

		#################################################################

		img = cv2.inRange(limiar_close, 0, 255)

		segmentado_cor = cv2.morphologyEx(img,cv2.MORPH_CLOSE,np.ones((7, 7)))

		# Esse segundo inRange faz com que nao precisamos criar os arrays de HSV
		segmentado_cor = cv2.inRange(segmentado_cor, -1, 200)

		# Outra hipotese eh que os contornos estejam saindo errado (menos provavel)
		cv2.imwrite("segCor.jpg", segmentado_cor)

		img_out, contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

		maior_contorno = None
		maior_contorno_area = 0


		for cnt in contornos:
			area = cv2.contourArea(cnt)
			if area > maior_contorno_area:
				maior_contorno = cnt
				maior_contorno_area = area

		print("len: " + str(len(contornos)))
		if not maior_contorno is None :
			self.addRoiPoints(maior_contorno, grayFundo)

	def addRoiPoints(self, roiPts, imagem):
		global roiBox, roi_hist
		roiPts = np.array(roiPts)
		s = roiPts.sum(axis=1)
		xs = s[:,1]
		ys = s[:,0]

		min_x = np.min(xs)
		max_x = np.max(xs)
		min_y = np.min(ys)
		max_y = np.max(ys)
		print("x: " + str(min_x) + " : " + str(max_x))

		print("y: " + str(min_y) + " : " + str(max_y))
		orig = imagem.copy()
		print("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n")
		print(len(orig))
		cv2.imwrite("orig.jpg", orig)
		roi = orig[min_x : max_x, min_y : max_y]
		cv2.imwrite("roi.jpg", roi)
		print(len(roi))

		roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
		cv2.imwrite("roiHSV.jpg", roi)

		roi_hist = cv2.calcHist([roi], [0], None, [16], [0, 180])
		roi_hist = cv2.normalize(roi_hist, roi_hist, 0, 255, cv2.NORM_MINMAX)

		roiBox = (min_x, min_y, max_x, max_y)


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
												'Sobreviver':'SURVIVE'})

		smach.StateMachine.add('STANDBY', StandBy(),
								transitions = {'StandBy':'STANDBY',
												'Decolar':'TAKEOFF'})

		smach.StateMachine.add('SURVIVE', Survive(),
								transitions = {'Sobreviver':'SURVIVE',
												'AnalisarI':'ANALISANDOI',
												'Aprender':'LEARN'})

		smach.StateMachine.add('ANALISANDOI', AnalisandoI(),
								transitions = {'AnalisarI':'ANALISANDOI',
												'Pousar':'LAND',
												'Manobra': 'MANOBRA'})

		smach.StateMachine.add('ANALISANDOII', AnalisandoII(),
								transitions = {'AnalisandoII':'ANALISANDOII',
												'Mover':'MOVE',
												'Girar':'GIRA'})

		smach.StateMachine.add('MOVE', Move(),
								transitions = {'Mover':'MOVE',
												'Sobreviver':'SURVIVE'})

		smach.StateMachine.add('GIRA', Gira(),
								transitions = {'Girar':'GIRA',
												'Sobreviver':'SURVIVE'})

		smach.StateMachine.add('MANOBRA', Manobra(),
								transitions = {'Manobra':'MANOBRA',
												'Sobreviver':'SURVIVE'})

		smach.StateMachine.add('LAND', Land(),
								transitions = {'Pousar':'LAND',
												'StandBy':'STANDBY'})

		smach.StateMachine.add("LEARN",Aprender(),
								transitions = {'Aprender':'LEARN',
												'Sobreviver':'SURVIVE'})



	# Execute SMACH plan
	outcome = sm.execute()

if __name__=="__main__":
	maquina()
