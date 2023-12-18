#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray

def main():
    # Publisher to send data to Arduino
    pub = rospy.Publisher('to_arduino', Int32MultiArray, queue_size=10)
    rospy.init_node('pi_node', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    rospy.sleep(5)

    while not rospy.is_shutdown():
        right_speed_to_send = int(input("Enter speed for right motor: "))
        left_speed_to_send = int(input("Enter speed for left motor: "))
        to_send = Int32MultiArray()
        to_send.data = [right_speed_to_send, left_speed_to_send]
        pub.publish(to_send)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

