from os import kill
import rospy
from geometry_msgs.msg  import Twist
from turtlesim.msg import Pose
import turtlesim.srv as turtle
import std_srvs.srv as service
from math import pow, atan2, sqrt
import math
import sys
import time

PI = math.pi

def moveToTarget(distance, isForward):
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    speed = 5

    if isForward:
        vel_msg.linear.x = abs(speed)
    else:
        vel_msg.linear.x = -abs(speed)
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
    if not rospy.is_shutdown():
    	t0 = float(rospy.Time.now().to_sec())
    	current_distance = 0
    	#robotu belirlenmis mesafeye goturme islemi
    	while(float(current_distance) < float(distance)):
    		velocity_publisher.publish(vel_msg)
    		#gercek zamanin konuma cevrilmesi
    		t1=float(rospy.Time.now().to_sec())
    		current_distance = float(speed*(t1-t0))
    	#robotun hizi sifirlandi
    	vel_msg.linear.x = 0
    	#robot durduruldu
    	velocity_publisher.publish(vel_msg)

def rotate(angle, clockwise):
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    speed = 20 #aci/saniye

    #dereceyi radyana cevirme
    angular_speed = speed*2*PI/360
    relative_angle = float(float(angle)*2*PI/360)

    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    # robot yonu kontrolu (saat yonu- saat yonu tersi)
    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)
   
    t0 = float(rospy.Time.now().to_sec())
    current_angle = 0

    while(current_angle < relative_angle):
        velocity_publisher.publish(vel_msg)
        t1 = float(rospy.Time.now().to_sec())
        current_angle = float(angular_speed*(t1-t0))


    #robot durduruldu.
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('turtle_odev', anonymous=True)
        killTurtle = rospy.ServiceProxy('/kill', turtle.Kill)
        spawnturtle = rospy.ServiceProxy('/spawn', turtle.Spawn)
        clearStage = rospy.ServiceProxy('/clear', service.Empty)

        rate = rospy.Rate(10)

        while True:
            killTurtle('turtle1')
            spawnturtle(1, 1, 0, 'turtle1')
            moveToTarget(2, False)
            rotate(30, 1)

            rate.sleep()

        pass
    except rospy.ROSInterruptException: 
        pass