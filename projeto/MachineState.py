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

roiPts = []
roiBox = None
inputPointMode = False

cv2.setMouseCallback("image", click_and_crop)

# Range de Cor a ser procurado (Tempoarario para testes)
cor_maior = np.array([70, 255, 255])
cor_menor = np.array([50, 50, 50])

################################################################################

def click_and_crop(event, x, y, flags, param):
	 # grab the reference to the current frame, list of ROI
	 # points and whether or not it is ROI selection mode
	 global frame, roiPts, inputPointMode

	 # if we are in ROI selection mode, the mouse was clicked,
	 # and we do not already have four points, then update the
	 # list of ROI points with the (x, y) location of the click
	 # and draw the circle
	 if inputPointMode and event == cv2.EVENT_LBUTTONDOWN and len(roiPts) < 4:
		  roiPts.append((x, y))
		  cv2.circle(frame, (x, y), 4, (0, 255, 0), 2)
		  cv2.imshow("image", frame)






	 # REMOVER: FUNC ANTIGA

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

def CamShiftTrack(frame):
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
		cx = (pts[0][0]+pts[1][0])/2
		cy = (pts[0][1]+pts[2][1])/2
		cv2.circle(frame, (cx, cy), 4, (0, 255, 0), 2)
		# cv2.imshow('img2',frame)

	# handle if the 'i' key is pressed, then go into ROI
	# selection mode
	cv2.imshow("image", frame)
	key = cv2.waitKey(1) & 0xFF
	if key == ord("i") and len(roiPts) < 4:
		# indicate that we are in input mode and clone the
		# frame
		inputMode = True
		orig = frame.copy()

		# keep looping until 4 reference ROI points have
		# been selected; press any key to exit ROI selction
		# mode once 4 points have been selected
		while len(roiPts) < 4:
			cv2.imshow("image", frame)
			cv2.waitKey(0)

		# determine the top-left and bottom-right points
		roiPts = np.array(roiPts)
		s = roiPts.sum(axis = 1)
		tl = roiPts[np.argmin(s)]
		br = roiPts[np.argmax(s)]

		# grab the ROI for the bounding box and convert it
		# to the HSV color space
		roi = orig[tl[1]:br[1], tl[0]:br[0]]
		roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

		# compute a HSV histogram for the ROI and store the
		# bounding box
		roi_hist = cv2.calcHist([roi], [0], None, [16], [0, 180])
		roi_hist = cv2.normalize(roi_hist, roi_hist, 0, 255, cv2.NORM_MINMAX)
		roiBox = (tl[0], tl[1], br[0], br[1])

	# k = cv2.waitKey(60) & 0xff
	if key == 27:
		break

def showImage(dado):
	global countCam
	countCam += 1
	now = rospy.get_rostime()
	imgtime = dado.header.stamp
	lag = nows-imgtime
	delay = lag.secs
	if delay > atraso and check_delay == True:
		return
	antes = time.clock()
	cv_image = bridge.compressed_imgmsg_to_cv2(dado, "bgr8")
	media, centro = CamShiftTrack(cv_image)
	depois = time.clock()
	#  cv2.imshow("Camera", cv_image)

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
			return 'Decolar'
		return 'StandBy'

class Survive(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['Sobreviver','AnalisarI'])
		self.count = 0
		self.countMax = 3000

	def execute(self, userdata):
		global pub_Move
		rospy.loginfo('Executing state SURVIVE')

		self.count += 1
		if self.count > self.countMax:  # Altura < 2,5m
			vel = Twist(Vector3(0,0,0.2), Vector3(0,0,0))
			pub_Move.publish(vel)
			return 'Pousar'
		return 'Mover'

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
			return 'Pousar'
		elif 1:  # Gesto para manobra
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
			return 'Girar'
		elif 1:  # Se estiver alinhado
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
		smach.State.__init__(self, outcomes = ['Manobrar','Sobreviver'])
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
												'AnalisarI':'ANALISANDOI'})

		smach.StateMachine.add('ANALISANDOI', AnalisandoI(),
								transitions = {'AnalisarI':'ANALISANDOI',
												'Pousar':'LAND',
												'Manobra': 'MANOBRA'})

		smach.StateMachine.add('ANALISANDOII', AnalisandoII(),
								transitions = {'AnalisarII':'ANALISANDOII',
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



	# Execute SMACH plan
	outcome = sm.execute()

if __name__=="__main__":
	maquina()
