#!/usr/bin/env python

import wiringpi

import rospy
from std_msgs.msg import Int32MultiArray, Float32
import RPi.GPIO as GPIO

# Initialize wiringpi in GPIO mode
wiringpi.wiringPiSetup()

# Define pin numbers
pin0 = 0 # right-front touch sensor
pin1 = 1 # left-front touch sensor
pin2 = 2 # mid-front touch sensor
pin3 = 3 # light sensor

# Set pins to input mode
wiringpi.pinMode(pin0, wiringpi.INPUT)
wiringpi.pinMode(pin1, wiringpi.INPUT)
wiringpi.pinMode(pin2, wiringpi.INPUT)
wiringpi.pinMode(pin3, wiringpi.INPUT)

# Set up pull-up resistors as these are touch sensors
wiringpi.pullUpDnControl(pin0, wiringpi.PUD_UP)
wiringpi.pullUpDnControl(pin1, wiringpi.PUD_UP)
wiringpi.pullUpDnControl(pin2, wiringpi.PUD_UP)
wiringpi.pullUpDnControl(pin3, wiringpi.PUD_UP)

gate = 0 # 1

# left_speed, right_speed, count_max
def move_forward():
    return [217, 200], 300
    # return [138, 125], 300

def move_backward():
    return [-50, -50], 100

#def turn_left_360_degree():
#    return [-30, 30], 700

def stop():
    return [0, 0], 200

def turn_left_finetune():
    return [-1, 1], 10

def turn_right_finetune():
    return [1, -1], 10

def turn_left_90_degree():
    return [-50, 50], 200

def turn_right_90_degree():
    return [50, -50], 200

count0r = 0.0  # Initialize with a default value
door_found = [0, 0]  # Assuming default values are zeros 

def find_door():

    # if count0r >= 0.01:
    if count0r >= 0.38 and count0r <= 0.51:
        # Beacon-1 600
        door_found[0] = 1

    elif count0r >= 0.15 and count0r <= 0.35:
        # Beacon-2 1500
        door_found[1] = 1


def finded_door_callback(data):
    global count0r
    count0r = data.data
    

def main():
    # Publisher to send data to Arduino
    pub = rospy.Publisher('to_arduino', Int32MultiArray, queue_size=10)
    rospy.Subscriber("ir_sensor_ratio", Float32, finded_door_callback)
    rospy.init_node('pi_node', anonymous=True)
    rate = rospy.Rate(100)  # 10hz
    rospy.sleep(5)
    
    to_send = Int32MultiArray()
    to_send.data = [0, 0]

    # rospy.Subscriber("finded_door", Float32, finded_door_callback)
    

    counter = 0

    state = 0

    obstacle_direction = None

    catch_ball = False

    while not rospy.is_shutdown():
        
        sensor_right = 1 - wiringpi.digitalRead(pin0)
        sensor_left = 1 - wiringpi.digitalRead(pin1)        
        sensor_mid = 1 - wiringpi.digitalRead(pin2)
        sensor_light = 1 - wiringpi.digitalRead(pin3)  

        if counter % 100 == 0:
            print 'sl, sr, sm, sli: ', sensor_left, sensor_right, sensor_mid, sensor_light
            print 'state: ', state
            #print 'left speed: ', to_send.data[0]
            #print 'right speed: ', to_send.data[1]

        

        if sensor_mid == 1 and state < 8:
            #print '*'*50
            state = 8

        # Obstacle Interupt
        if (sensor_left == 1 or sensor_right == 1) and not catch_ball:
            counter = 0
            if sensor_left == 1 and sensor_right == 1:
                obstacle_direction = 'M' 
            elif sensor_left == 1:
                obstacle_direction = 'L'
            elif sensor_right == 1:
                obstacle_direction = 'R'
            state = 5
        
        # Main State machine
        if state == 0: # Start
            # Move forward
            to_send.data, count_max = move_forward()
            if counter % 100 == 0:
                print 'move_forward'
            pub.publish(to_send)
        
        elif state == 1: # Exploration forward
            # Move forward
            to_send.data, count_max = move_forward()
            if counter % 100 == 0:
                print 'move_forward'
            pub.publish(to_send)

            # Next state
            if sensor_light == 1:
                counter = 0
                state = 4
            elif counter > count_max:
                counter = 0
            else:
                state = 1
        
        elif state == 2: # left Finetune
            # Turn left finetune
            to_send.data, count_max = turn_left_finetune()
            if counter % 100 == 0:
                print 'turn_left_finetune'
            pub.publish(to_send)

            # Next state
            if counter > count_max:
                counter = 0
                state = 4

        elif state == 3: # right Finetune
            # Turn right finetune
            to_send.data, count_max = turn_right_finetune()
            if counter % 100 == 0:
                print 'turn_right_finetune'
            pub.publish(to_send)

            # Next state
            if counter > count_max:
                counter = 0
                state = 4
            
        elif state == 4: # Forward to light
            # Forward
            to_send.data, count_max = move_forward()
            if counter % 100 == 0:
                print 'move_forward'
            pub.publish(to_send)

            # Next state
            if counter > count_max:
                counter = 0
                if sensor_mid == 1:
                    state = 8
                else:
                    state = 4
        
        elif state == 5: # Obstacle move_backward
            # Backward
            to_send.data, count_max = move_backward()
            if counter % 100 == 0:
                print 'move_backward'
            pub.publish(to_send)

            # Next state
            if counter > count_max:
                counter = 0
                state = 6

            
        elif state == 6: # Obstacle stop
            # Stop
            to_send.data, count_max = stop()
            if counter % 100 == 0:
                print 'stop'
            pub.publish(to_send)
            
            # Next state
            if counter > count_max:
                counter = 0
                state = 7

        elif state == 7: # Obstacle turn
            
            if obstacle_direction in ['L']:
                # Left obstacle -> Turn right 90 degree
                to_send.data, count_max = turn_right_90_degree()
                if counter % 100 == 0:
                    print 'turn_right_90_degree'
                pub.publish(to_send)

                # Next state
                if sensor_light == 1:
                    counter = 0
                    state = 3

            elif obstacle_direction in ['M', 'R']:
                # Right obstacle -> Turn left 90 degree
                to_send.data, count_max = turn_left_90_degree()
                if counter % 100 == 0:
                    print 'turn_left_90_degree'
                pub.publish(to_send)

                # Next state
                if sensor_light == 1:
                    counter = 0
                    state = 2

            # Next state
            if counter > count_max:
                counter = 0
                state = 1
        
        elif state == 8: # Turn left find door

            find_door()

            to_send.data, count_max = turn_left_90_degree()
            if counter % 100 == 0:
                print 'turn_left_90_degree'
            pub.publish(to_send)

            # Next state
            if door_found[gate] == 1:
                counter = 0
                state = 10

            if counter > count_max * 2:
                counter = 0
                state = 9

        elif state == 9: # Move forward explore
            # Move forward
            to_send.data, count_max = move_forward()
            if counter % 100 == 0:
                print 'move_forward'
            pub.publish(to_send)
            
            # Next state
            if counter > count_max * 0.8:
                counter = 0
                state = 8

        elif state == 10: # Move forward to door
            # Move forward
            to_send.data, count_max = move_forward()
            if counter % 100 == 0:
                print 'move_forward'
            pub.publish(to_send)

        counter += 1

        rate.sleep()
     

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
