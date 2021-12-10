import rospy
from geometry_msgs.msg import Twist
PI = 3.1415926535897

if __name__ == '__main__':
    try:
        rospy.init_node('rotate_turtle', anonymous=True)
        publish sqrt(pow((x2-x1),2) + pow((y2-y1),2))er = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        tw = Twist()
        while True:
            tw.angular.z = 15*2*PI/360
            publisher.publish(tw)
    except rospy.ROSInterruptException: pass

    
