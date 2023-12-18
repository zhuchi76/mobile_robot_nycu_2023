#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

def callback(data):
   print "received from arduino",data.data

def main():
    
    # Publisher to send data to Arduino
    pub = rospy.Publisher('to_arduino', Int32, queue_size=10)

    rospy.init_node('pi_node', anonymous=True)

    # Subscriber to receive data from Arduino
    rospy.Subscriber("from_arduino", Int32, callback)
    rospy.sleep(5)

    while not rospy.is_shutdown():
        number_to_send = int(input("user's input is: "))
        pub.publish(number_to_send)
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
