import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import *
from std_srvs.srv import *
from turtlesim.msg import Pose
from math import floor, pow,atan2, sin,sqrt,cos
PI = 3.1415926535897

class Turtle:
	def __init__(self):
		self.name = 'turtle1'
		rospy.init_node('ninja_turtle', anonymous=True)
		self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
		self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.callback)
		self.pose = Pose()
		self.vel_msg = Twist()

		self.killTurtle = rospy.ServiceProxy('/kill', Kill)
		self.spawnTurtle = rospy.ServiceProxy('/spawn', Spawn)
		self.clearStage = rospy.ServiceProxy('/clear', Empty)
		self.rate = rospy.Rate(10)
		
		self.speed = 15

	def kill(self):
		try:
			self.killTurtle(self.name)
			print('Turtle killed')
		except rospy.ServiceException as e:
			rospy.loginfo('Service execution failed: %s' + str(e))
		
	
	def spawn(self, x, y, angle):
		try:
			self.spawnTurtle(x, y, angle, self.name)
			print(f'Turtle spawned at x:{str(x)} y:{str(y)}')
		except rospy.ServiceException as e:
			rospy.loginfo('Service execution failed: %s' + str(e))
		

	def clear(self):
		try:
			self.clearStage()
			print('Stage cleared')
		except rospy.ServiceException as e:
			rospy.loginfo('Service execution failed: %s' + str(e))
		


	def __repr__(self) -> str:
		print('Turtle {}'.format(self.name))


	def callback(self, data):
		self.pose = data
		self.pose.x = round(self.pose.x, 4)
		self.pose.y = round(self.pose.y, 4)

	
	def move(self, distance, isForward):
		if isForward:
			# self.vel_msg.linear.x = abs(self.speed)
			self.vel_msg.linear.x = abs(self.speed)
		else:
			self.vel_msg.linear.x = -abs(self.speed)
		self.vel_msg.linear.y = 0
		self.vel_msg.linear.z = 0
		self.vel_msg.angular.x = 0
		self.vel_msg.angular.y = 0
		self.vel_msg.angular.z = 0
		
		# Setting the current time for distance calculus
		t0 = float(rospy.Time.now().to_sec())
		current_distance = 0
		# Loop to move the turtle in a specified distance
		while(float(current_distance < float(distance))):
			# Publish the velocity
			self.velocity_publisher.publish(self.vel_msg)

			# Takes actual time to velocity calculus
			t1 = rospy.Time.now().to_sec()
			current_distance = float(self.speed*(t1-t0))
		self.vel_msg.linear.x = 0
		self.velocity_publisher.publish(self.vel_msg)
		print(f'Finish moving {str(distance)}')

	def rotate(self, angle, clockwise):
		# Receiving the user's input
		angle = float(angle)
		clockwise = bool(clockwise) # True or false

		# Converting from angles ro radians
		angular_speed = float(self.speed*2*PI/360)
		relative_angle = float(angle*2*PI/360)

		# We won't use linear components
		self.vel_msg.linear.x = 0
		self.vel_msg.linear.y = 0
		self.vel_msg.linear.z = 0
		self.vel_msg.angular.x = 0
		self.vel_msg.angular.y = 0

		# Checking if our movement is CW or CCW
		if clockwise:
			self.vel_msg.angular.z = -abs(angular_speed)
		else:
			self.vel_msg.angular.z = abs(angular_speed)

		# Setting the current time for distance calculus
		t0 = rospy.Time.now().to_sec()
		current_angle = 0

		while (current_angle < relative_angle):
			self.velocity_publisher.publish(self.vel_msg)
			t1 = rospy.Time.now().to_sec()
			current_angle = angular_speed * (t1 - t0)

		# Forcing robot to stop
		self.vel_msg.angular.z = 0
		self.velocity_publisher.publish(self.vel_msg)
		# rospy.spin()
		print(f'Finish rotating {str(angle)} degree')

	def move2goal(self, x, y):
		print(f'moving to ({str(x)},{str(y)})')
		goal_pose = Pose()
		goal_pose.x = int(x)
		goal_pose.y = int(y)
		distance_tolerance = 0

		while sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2)) >= distance_tolerance:

			#Porportional Controller
			#linear velocity in the x-axis:
			self.vel_msg.linear.x = 1.5 * sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
			self.vel_msg.linear.y = 0
			self.vel_msg.linear.z = 0

			#angular velocity in the z-axis:
			self.vel_msg.angular.x = 0
			self.vel_msg.angular.y = 0
			self.vel_msg.angular.z = 4 * (atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x) - self.pose.theta)

			#Publishing our vel_msg
			self.velocity_publisher.publish(self.vel_msg)
		#Stopping our robot after the movement is over
		self.vel_msg.linear.x = 0
		# self.vel_msg.angular.z = 0
		self.velocity_publisher.publish(self.vel_msg)
		print('Finish moving')
	
	def slide(self):
		distance = 2
		self.vel_msg.linear.x = abs(5)
		self.vel_msg.linear.y = abs(5)
		self.vel_msg.linear.z = 0
		self.vel_msg.angular.x = 0
		self.vel_msg.angular.y = 0
		self.vel_msg.angular.z = 0

		t0 = float(rospy.Time.now().to_sec())
		current_distance = 0
		# Loop to move the turtle in a specified distance
		while(float(current_distance < float(distance))):
			# Publish the velocity
			self.velocity_publisher.publish(self.vel_msg)

			# Takes actual time to velocity calculus
			t1 = rospy.Time.now().to_sec()
			current_distance = float(self.speed*(t1-t0))
		self.vel_msg.linear.y = 0
		self.vel_msg.linear.x = 0
		self.velocity_publisher.publish(self.vel_msg)
		print(f'Finish moving {str(distance)}')

		

if __name__ == '__main__':
	try:
		turtle = Turtle()
		move = turtle.move
		rotate = turtle.rotate
		# while True:
		turtle.kill()
		turtle.spawn(1, 1, 0)
		# 1
		turtle.rotate(90, 0)
		turtle.move(2, 1)
		turtle.rotate(90, 1)
		print('finish move 1')
		# 2
		turtle.rotate(30, 0)
		print('finish move 2')
		# 3
		turtle.rotate(30, 1)
		turtle.move(1, 1)
		turtle.rotate(30, 0)
		print('finish move 3')
		# 4
		turtle.rotate(60, 1)
		print('finish move 4')
		# 5
		turtle.rotate(30, 1)
		turtle.move(1, 1)
		turtle.rotate(30, 0)
		print('finish move 5')
		# 6
		turtle.rotate(60, 1)
		print('finish move 6')
		# 7
		turtle.rotate(30, 0)
		move(1, 1)
		rotate(30, 1)
		print('finish move 7')
		# 8
		turtle.rotate(105, 1)
		turtle.move(1, 1)
		turtle.rotate(105, 0)
		print('finish move 8')

		# rospy.sleep(2)

	except rospy.ROSInterruptException:
		pass