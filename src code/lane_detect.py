import math
import rospy
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
import numpy as np
import time
from geometry_msgs.msg import Twist
from beginer_tutorials.msg import custom_msg

CENTER_POINT_COOR_TUPLE = (320,290)
CENTER_POINT_COOR_LIST = [320, 290]
STRAIGHT_VELO = 0.2
ANGULAR_VELO = (5/180)*3.14

# public_velo = rospy.Publisher("/cmd_vel", Twist, queue_size=100)
# myVelo = Twist()

customMsg = custom_msg()
distance_left = -1
distance_right = -1

rospy.init_node('lane_detection', anonymous=True)
pub = rospy.Publisher('chatter', custom_msg, queue_size=100)

def distance(x1, y1, x2, y2):
    return int(math.sqrt((x1 - x2)**2 + (y1 - y2)**2))

def make_points(image, line):
    slope, intercept = line
    y1 = int(image.shape[0])# bottom of the image
    y2 = int(y1*2.85/5)         # slightly lower than the middle
    x1 = int((y1 - intercept)/slope)
    x2 = int((y2 - intercept)/slope)
    return [[x1, y1, x2, y2]]
 
def average_slope_intercept(image, lines):
    left_fit    = []
    right_fit   = []
    global distance_left
    global distance_right

    distance_left = -1
    distance_right = -1

    if lines is None:
        return None
    for line in lines:
        for x1, y1, x2, y2 in line:                 
            fit = np.polyfit((x1,x2), (y1,y2), 1)            
            slope = fit[0]
            intercept = fit[1]            
            if(abs(slope) > 0.2):
                if slope < 0: # y is reversed in image
                    left_fit.append((slope, intercept))
                else:
                    right_fit.append((slope, intercept))                            
    # add more weight to longer lines
    averaged_lines = []
    if(len(left_fit)):
        left_fit_average = np.average(left_fit, axis=0)            
        left_line  = make_points(image, left_fit_average)
        distance_left = distance(CENTER_POINT_COOR_TUPLE[0], 
                                 CENTER_POINT_COOR_TUPLE[1],
                                 left_line[0][2],
                                 left_line[0][3])
        
        cv2.line(image, CENTER_POINT_COOR_TUPLE, (left_line[0][2], left_line[0][3]), (0, 0, 0), 5)
        
        text_pos = (int((CENTER_POINT_COOR_TUPLE[0] + left_line[0][2])/2), 
                    int((CENTER_POINT_COOR_TUPLE[1] + left_line[0][3])/2))
        font = cv2.FONT_HERSHEY_SIMPLEX
        color_text = (255, 0, 0)
        
        cv2.putText(image, str(distance_left), text_pos, font, 
                   0.5, color_text, 2, cv2.LINE_AA)
        
        averaged_lines.append(left_line)
    if(len(right_fit)):
        right_fit_average = np.average(right_fit, axis=0)
        right_line = make_points(image, right_fit_average)     
        distance_right = distance(CENTER_POINT_COOR_TUPLE[0], 
                            CENTER_POINT_COOR_TUPLE[1],
                            right_line[0][2],
                            right_line[0][3])
        
        cv2.line(image, CENTER_POINT_COOR_TUPLE, (right_line[0][2], right_line[0][3]), (0, 0, 0), 5)
        
        text_pos = (int((CENTER_POINT_COOR_TUPLE[0] + right_line[0][2])/2), 
                    int((CENTER_POINT_COOR_TUPLE[1] + right_line[0][3])/2))
        font = cv2.FONT_HERSHEY_SIMPLEX
        color_text = (255, 0, 0)
        
        cv2.putText(image, str(distance_right), text_pos, font, 
                   0.5, color_text, 2, cv2.LINE_AA)
        averaged_lines.append(right_line)    
    return averaged_lines
 
def canny(img):
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    kernel = 5
    blur = cv2.GaussianBlur(gray,(kernel, kernel),0)
    canny = cv2.Canny(gray, 50, 150)
    return canny
 
def display_lines(img,lines):
    line_image = np.zeros_like(img)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image,(x1,y1),(x2,y2),(255,0,0),10)    
    return line_image
 
def region_of_interest(canny):
    height = canny.shape[0]
    width = canny.shape[1]
    
    # IMAGE FULL BLACK 
    mask = np.zeros_like(canny)    
 
    # triangle = np.array([[
    # (200, height),
    # (550, 250),
    # (1100, height),]], np.int64)    
    
    triangle = np.array([
        [50,300], 
        [230,255], 
        [470, 255],
        [625, 300]
        ])        

    cv2.fillPoly(mask, pts=[triangle], color =(255,255,255))    
    masked_image = cv2.bitwise_and(canny, mask)    
    return masked_image

def callbackFunction(data):
    begin = time.time()
    bridge = CvBridge() 
    frame = bridge.imgmsg_to_cv2(data, "bgr8")	#data.encoding
    cv2.waitKey(3)
    canny_image = canny(frame)    
    # plt.imshow(canny_image)
    # plt.show()
    # # GET REGION OF INTEREST
    cropped_canny = region_of_interest(canny_image)    
    # cv2.imshow("result", cropped_canny)     
    # HOUGH TRANSFORM            
    # ref https://docs.opencv.org/3.4/dd/d1a/group__imgproc__feature.html#ga8618180a5948286384e3b7ca02f6feeb
    lines = cv2.HoughLinesP(image=cropped_canny, rho=2, theta=np.pi/180, threshold=60 , minLineLength=5,maxLineGap=250)    
    # #     
    averaged_lines = average_slope_intercept(frame, lines)
    end = time.time()    
    line_image = display_lines(frame, averaged_lines)
    combo_image = cv2.addWeighted(frame, 1, line_image, 1, 1)        
    cv2.circle(combo_image, CENTER_POINT_COOR_TUPLE, 8, (149, 53, 83), -1)
    cv2.imshow("result", combo_image)       
    print(int(1/(end-begin)), " fps")   
    customMsg.name = 2
    customMsg.num1 = distance_left
    customMsg.num2 = distance_right
    pub.publish(customMsg)

    # if distance_left > -1 and distance_right > -1:
    #     if(distance_left < 80):
    #         myVelo.linear.x = STRAIGHT_VELO
    #         myVelo.angular.z = -ANGULAR_VELO
    #     if(distance_right < 80):
    #         myVelo.linear.x = STRAIGHT_VELO
    #         myVelo.angular.z = ANGULAR_VELO
    #     else:
    #         myVelo.linear.x = STRAIGHT_VELO
    #         myVelo.angular.z = 0
    # elif distance_left > -1 or distance_right > -1:
    #     if(distance_left == -1):
    #         if(distance_right > 130):
    #             myVelo.linear.x = STRAIGHT_VELO
    #             myVelo.angular.z = -ANGULAR_VELO   
    #         else:    
    #             myVelo.linear.x = STRAIGHT_VELO
    #             myVelo.angular.z = 0  
    #     elif(distance_right == -1):
    #         if(distance_left > 130):
    #             myVelo.linear.x = STRAIGHT_VELO
    #             myVelo.angular.z = ANGULAR_VELO  
    #         else:
    #             myVelo.linear.x = STRAIGHT_VELO
    #             myVelo.angular.z = 0
    # else:
    #     myVelo.linear.x = STRAIGHT_VELO
    #     myVelo.angular.z = ANGULAR_VELO
    # public_velo.publish(myVelo)

while not rospy.is_shutdown():
    lis = rospy.Subscriber("/camera/rgb/image_raw", Image, callbackFunction)
    rospy.spin()
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cv2.destroyAllWindows()