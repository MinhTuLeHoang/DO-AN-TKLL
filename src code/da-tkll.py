#! /usr/bin/python

from tensorflow.python.ops.gen_array_ops import empty
import rospy
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image

from beginer_tutorials.msg import custom_msg

import numpy as np
from numpy.core.fromnumeric import amax
from numpy.lib.type_check import imag
from tensorflow import keras
import time

index = 0
model = keras.models.load_model("/home/minhtulehoang/catkin_ws/src/rotate_turtlebot/src/tkll_model.h5")

def returnRedness(img):
	yuv = cv2.cvtColor(img,cv2.COLOR_BGR2YUV)
	y, u, v = cv2.split(yuv)
	return v

#T=145 - long range
#T=150 - short range
#T=160
#from T=128 to T=150 -> red sign detection -> best:150
#T= < T=120 -> blue sign detection -> best: 110
def threshold_RedSign(img,T=150):
	_, img = cv2.threshold(img,T,255,cv2.THRESH_BINARY)	
	#cv2.imshow("sth",img)
	return img 

def threshold_BlueSign(img,T=110):
	_, img = cv2.threshold(img,T,255,cv2.THRESH_BINARY)	
	#cv2.imshow("sth",img)
	return img 

def findContour(img):
	contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	return contours

def findBiggestContour(contours):
	m = 0
	c = [cv2.contourArea(i) for i in contours]
	return contours[c.index(max(c))]

def boundary_Green_Box(img,contours):
	x, y, w, h = cv2.boundingRect(contours)
	img = cv2.rectangle(img, (x-5,y-5), (x+w+4,y+h+4), (0,255,0), 2)
	sign = img[(y-5):(y+h+4) , (x-5):(x+w+4)]
	return img, sign

def preprocessingImageToClassifier(image=None,imageSize=32):
	# GRAYSCALE	
	image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)		
	# RESIZE
	image = cv2.resize(image,(imageSize,imageSize))
	cv2.imshow("resize", image)
	# LOCAL HISTOGRAM EQUALIZATION
	clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4, 4))	
	image = clahe.apply(image)
	image = image.astype(np.float32)/255.0	
	image = image.reshape(1,imageSize,imageSize,1)		
	return image

def predict(sign):	
	img = preprocessingImageToClassifier(sign,imageSize=32)	
	return model.predict(img)

def probility(sign):
	img = preprocessingImageToClassifier(sign, imageSize=32)
	return np.amax(model.predict(img))


labelToText = { 0:"STOP",
    	1:"TURN LEFT",
    	2:"TURN RIGHT"}


def publish(traffic_sign):
	pub = rospy.Publisher('chatter', custom_msg, queue_size=10)
	
	customMsg = custom_msg()
	customMsg.name = 1
	customMsg.num1 = traffic_sign
	customMsg.num2 = -1
	
	pub.publish(customMsg)



def callbackFunction(data):
	print("---------------oke---------------")
	global index
	begin = time.time()
	bridge = CvBridge()
	print("Start Convert")
	imgMatrix = bridge.imgmsg_to_cv2(data, "bgr8")	#data.encoding
	# basePath = "/home/minhtulehoang/catkin_ws/src/rotate_turtlebot/src/img"
	# baseFileName = "/pic" + str(index) + ".jpeg"
	print("Start Create File")
	#cv2.imwrite(basePath + baseFileName, imgMatrix)
	print("Successful!")
	# index +=1
	cv2.waitKey(3)
	#cv2.imshow("Origin",imgMatrix)
	final_sign = []
	redness = returnRedness(imgMatrix)
	try:
		#### RED SIGN ####
		thresh = threshold_RedSign(redness)
		contours = findContour(thresh)
		for contour in contours:
			area = cv2.contourArea(contour)
			x, y, w, h = cv2.boundingRect(contour)
			if( w <= 20 or h <= 20):
				continue
			elif w/h >= 1.2 or h/w >= 1.2:
				continue
			elif area > 100 and area < 15000:
				area = cv2.contourArea(contour)
				x, y, w, h = cv2.boundingRect(contour)
				print("NEARLY OKE, w=", w, " - h=",h, " - area= ", area)
				final_sign.append(contour)
			else:
				area = cv2.contourArea(contour)
				x, y, w, h = cv2.boundingRect(contour)
				print("NEARLY OKE, w=", w, " - h=",h, " - area= ", area)
				continue

		#### BLUE SIGN ####
		thresh = threshold_BlueSign(redness)
		contours = findContour(thresh)
		for contour in contours:
			area = cv2.contourArea(contour)
			x, y, w, h = cv2.boundingRect(contour)
			if( w <= 20 or h <= 20):
				continue
			elif w/h >= 1.2 or h/w >= 1.2:
				continue
			elif area > 100 and area < 15000:
				final_sign.append(contour)
			else:
				continue
        
		#### FINAL SIGN ####
		if final_sign:  #non-empty
			big = findBiggestContour(final_sign)
			img, sign = boundary_Green_Box(imgMatrix, big)
			cv2.imshow('final',img)
			prediction = predict(sign)
			print("Now,I see:" + labelToText[np.argmax(prediction)])
			print(np.amax(prediction))
			x, y, w, h = cv2.boundingRect(big)
			print("width= ",w," - height= ",h)
			
			publish(np.argmax(prediction))
		else:
			cv2.imshow('final',imgMatrix)
			print("I can't see anything !")
	except:
		print("err")


while not rospy.is_shutdown():
	print("---------------begin---------------")
	rospy.init_node("topic_receiver_img")
	lis = rospy.Subscriber("/camera/rgb/image_raw", Image, callbackFunction)
	rospy.spin()












