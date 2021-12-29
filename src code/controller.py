#!/usr/bin/python
# import math
import rospy
# from cv_bridge import CvBridge
# import cv2
# from sensor_msgs.msg import Image
import numpy as np
import time
from geometry_msgs.msg import Twist
from beginner_tutorials.msg import custom_msg

# from tensorflow.python.ops.gen_array_ops import empty

# from numpy.core.fromnumeric import amax
# from numpy.lib.type_check import imag
# from tensorflow import keras

public_velo = rospy.Publisher("/cmd_vel", Twist, queue_size=100)
myVelo = Twist()

rospy.init_node('controller', anonymous=True)

STRAIGHT_VELO = 0.2
ANGULAR_VELO = (5/180)*3.14
ANGULAR_VELO_TURN_RIGHT = 0.25
ANGULAR_VELO_TURN_LEFT = 0.25

labelToText = { 0:"STOP",
    	1:"TURN LEFT",
    	2:"TURN RIGHT"}

def self_balance(distance_left, distance_right):
    if distance_left > -1 and distance_right > -1:
        if(distance_left < 80):
            myVelo.linear.x = STRAIGHT_VELO
            myVelo.angular.z = -ANGULAR_VELO
        if(distance_right < 80):
            myVelo.linear.x = STRAIGHT_VELO
            myVelo.angular.z = ANGULAR_VELO
        else:
            myVelo.linear.x = STRAIGHT_VELO
            myVelo.angular.z = 0
    elif distance_left > -1 or distance_right > -1:
        if(distance_left == -1):
            if(distance_right > 130):
                myVelo.linear.x = STRAIGHT_VELO
                myVelo.angular.z = -ANGULAR_VELO   
            else:    
                myVelo.linear.x = STRAIGHT_VELO
                myVelo.angular.z = 0  
        elif(distance_right == -1):
            if(distance_left > 130):
                myVelo.linear.x = STRAIGHT_VELO
                myVelo.angular.z = ANGULAR_VELO  
            else:
                myVelo.linear.x = STRAIGHT_VELO
                myVelo.angular.z = 0
    else:
        myVelo.linear.x = STRAIGHT_VELO
        myVelo.angular.z = ANGULAR_VELO
    public_velo.publish(myVelo)

def turn_right() :
    begin = time.time()
    while ((time.time() - begin) < 7.8):
        myVelo.linear.x = STRAIGHT_VELO
        myVelo.angular.z = -ANGULAR_VELO_TURN_RIGHT
        public_velo.publish(myVelo)
    # time.sleep(3)
    myVelo.angular.z = 0
    public_velo.publish(myVelo)

def turn_left() :
    begin = time.time()
    while ((time.time() - begin) < 5.5):
        myVelo.linear.x = STRAIGHT_VELO
        myVelo.angular.z = 0
        public_velo.publish(myVelo)
    begin = time.time()
    while ((time.time() - begin) < 7.5):
        myVelo.linear.x = STRAIGHT_VELO
        myVelo.angular.z = ANGULAR_VELO_TURN_LEFT
        public_velo.publish(myVelo)
    # time.sleep(3)
    myVelo.angular.z = 0
    public_velo.publish(myVelo)

def stop() :
    begin = time.time()
    while ((time.time() - begin) < 5):
        myVelo.linear.x = 0
        myVelo.angular.z = 0
        public_velo.publish(myVelo)

def most_frequent(list):
    counter = 0
    num = list[0]

    for i in list:
        curr_frequency = list.count(i)
        if (curr_frequency > counter):
            counter = curr_frequency
            num = i
    return num

DISABLED = 0
ENABLED = 1

signal_array = [] 
balance_permission = ENABLED

def callbackFunction(data):
    # distance_left = -1
    # distance_right = -1
    
    # traffic_signal = -1
    global balance_permission
    # print("Turning flag: ", turning_flag)
    # print ("Balance Permission", balance_permission)
    if (data.name == 1):
        traffic_signal = data.num1

        signal_array.append(traffic_signal)
        print ("Signal collection: ", signal_array)

        tail = signal_array[len(signal_array)-15:]
        # tail = slice(len(signal_array)-7, len(signal_array))
        # tail = signal_array[tail]
    
        head = signal_array[:len(signal_array)-16]
        # head = slice(0, len(signal_array) - 8)
        # head = signal_array[head]

        count = 0
        for i in tail:
            if i != -1: 
                count = 0
                break;
            else:
                count = count + 1
    
        if (count == 15): 
            signal = most_frequent(head)
            if (signal == 1):
                print ("READY TO TURN LEFT")
                turn_left()
                print ("Turn left: done") 
                signal_array.clear()
                
            elif (signal == 2):
                print ("READY TO TURN RIGHT")
                turn_right()
                print ("Turn right: done")    
                signal_array.clear()            
                
            elif (signal == 0):
                print ("READY TO STOP")
                stop()

            else:
                signal_array.clear()       
                # turning_flag = 0
    

    # print (permission_tag)

    if (data.name == 2):
        distance_left = data.num1 
        distance_right = data.num2
        if (balance_permission):
            print (distance_left, " --- ", distance_right)    
            self_balance(distance_left, distance_right)


while not rospy.is_shutdown():
    lis = rospy.Subscriber('chatter', custom_msg, callbackFunction) 
    rospy.spin()

