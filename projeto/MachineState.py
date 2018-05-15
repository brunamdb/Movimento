import rospy
import smach
import smach_ros

from std_msgs.msg import String
from std_msgs.msg import Empty
################################################################################

class TakeOff(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['Decolar','Pousar'])
		self.count = 0
		self.countMax = 10000

	def execute(self, userdata):
		global pub_TakeOff
		rospy.loginfo('Executing state TAKEOFF')

		#comando para aprender a ler a cor
		pub_TakeOff.publish(Empty())

		self.count += 1
		if self.count > self.countMax: return 'Pousar'
		else: return 'Decolar'

class Land(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['Pousar','Decolar'])
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

def maquina():
	global pub_TakeOff, pub_Land
	rospy.init_node('Drone_State_Machine')
	pub_TakeOff = rospy.Publisher("bebop/takeoff", Empty, queue_size=10)
	pub_Land = rospy.Publisher("bebop/land", Empty, queue_size=10)
	rate = rospy.Rate(10) # 10hz

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['terminei'])

	# Open the container
	with sm:

		# Add states to the container

		smach.StateMachine.add('TAKEOFF', TakeOff(),
								transitions = {'Decolar':'TAKEOFF',
												'Pousar':'LAND'})

		smach.StateMachine.add('LAND', Land(),
								transitions = {'Pousar':'LAND',
												'Decolar':'TAKEOFF'})
	# Execute SMACH plan
	outcome = sm.execute()

if __name__=="__main__":
	maquina()
