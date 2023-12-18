#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time
def ir_sensor_reader():
    rospy.init_node('ir_sensor_reader', anonymous=True)
    ratio_pub = rospy.Publisher('ir_sensor_ratio', Float32, queue_size=10)
    ir_sensor_pin = 23
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(ir_sensor_pin, GPIO.IN)
    while not rospy.is_shutdown():
        count_0 = 0
        count_1 = 0
        start_time = time.time()
        while time.time() - start_time <= 0.1:
            if GPIO.input(ir_sensor_pin) == GPIO.LOW:
                count_0 += 1
            else:
                count_1 += 1
        ratio = count_0 / float(count_0 + count_1)
        # rospy.loginfo("IR sensor ratio: %f", ratio)
        ratio_pub.publish(ratio)
    GPIO.cleanup()
if __name__ == '__main__':
    try:
        ir_sensor_reader()
    except rospy.ROSInterruptException:
        pass