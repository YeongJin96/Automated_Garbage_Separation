#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
from detection_msgs.msg import BoundingBox, BoundingBoxes
import sys
import math
import time
from time import sleep
from sensor_msgs.msg import JointState
import copy

DE2RA = math.pi / 180

class pub_to_dofbot:
    def __init__(self):
        rospy.init_node("pub_test", anonymous=True)
        
        #message
        self.joint_state = JointState()
        self.joint_state.name = ['joint1','joint2','joint3','joint4','joint5','joint6']
        self.ready1 = [-1.27, -0.08, -0.72, -1.32, 0.0, -1.50]
        self.joint_state.position = copy.deepcopy(self.ready1)
        self.pub = rospy.Publisher('move_arm', JointState, queue_size=1)
        self.pub.publish(self.joint_state)
        sleep(2.0)
        
        #follow
        resolution_x = 640
        resolution_y = 480
        threshold = 50
        cam_centerX = resolution_x/2
        cam_centerY = resolution_y/2
        
        self.minX = cam_centerX - threshold
        self.maxX = cam_centerX + threshold
        self.minY = cam_centerY - threshold
        self.maxY = cam_centerY + threshold
        self.speed = 0.01
        self.flag = True
        
        #etc
        self.start_time = time.time()
        
        #yolov5
        rospy.Subscriber("/yolov5/detections", BoundingBoxes, self.my_callback, queue_size=1)
        
    def Point2D(self, p1, p2):
      x = p2[0] - p1[0]
      y = p2[1] - p1[1]
      middle_line = math.sqrt((x**2) + (y**2))
      #print(middle_line)
      
      return middle_line
  
    def grap(self, y_max):
        # if x<210:
        #     self.joint_state.position[1:5] = [-1.21, -0.38, -0.51, 1.52]
        #     # sleep(4.0)
        #     self.pub.publish(self.joint_state)
        #     sleep(0.5)
        #     self.joint_state.position[5] = 1.3
        #     self.pub.publish(self.joint_state)
        
        # elif x>500:
        #     self.joint_state.position[1:5] = [-0.88, -0.82, -0.87, 0.62]
        #     # sleep(6.0)
        #     self.pub.publish(self.joint_state)
        #     sleep(0.5)
        #     self.joint_state.position[5] = 1.3
        #     self.pub.publish(self.joint_state)
        
        # else:
        if y_max > 465:
            if self.joint_state.position[0] < -1.2:
                self.joint_state.position[0] = -1.2
            self.joint_state.position[4] = 3.11
            self.pub.publish(self.joint_state)
            sleep(0.5)
            self.joint_state.position[:4] = [self.joint_state.position[0]+0.15,-1.10, -0.83, -0.14]
            self.pub.publish(self.joint_state)
            sleep(0.5)
            self.joint_state.position[5] = 1.3
            self.pub.publish(self.joint_state)
            
            self.move_object()

            self.flag = True

        
    def push_object(self):
        sleep(0.2)
        push_ready = [self.joint_state.position[0]+0.20,-0.81, -0.89, -1.55, 0.0, 0.0]
        self.joint_state.position = copy.deepcopy(push_ready)
        print('ready')
        self.pub.publish(self.joint_state)
        sleep(0.5)
        
        self.joint_state.position[1:] = [0.0, 0.0, 0.0, 0.0, 0.0]
        print('push')
        self.pub.publish(self.joint_state)
        sleep(1.0)
        
        self.joint_state.position = copy.deepcopy(self.ready1)
        
        
        
  
    def follow_object(self, b_x, b_y):
        #joint 0
        if self.joint_state.position[0] > -1.50 and self.joint_state.position[0] < 1.50:
            if self.minX>b_x:
                self.joint_state.position[0] += (self.speed*2)
            # elif self.maxX<b_x:
            #     self.joint_state.position[0] -= (self.speed/2)
            else: pass
        else: print("joint0 value is maximum, so you don't move joint0")

        #joint 3, 4
        if self.joint_state.position[2] > -1.50 and self.joint_state.position[2] < 1.50:
            if self.maxY<b_y:
                self.joint_state.position[2] -= self.speed * 2
            # elif self.minY>b_y:
                # self.joint_state.position[2] += self.speed
            else: pass
        elif self.joint_state.position[3] > -1.50 and self.joint_state.position[3] < 1.50:
            if self.maxY<b_y:
                self.joint_state.position[3] -= self.speed * 2
            # elif self.minY>b_y:
                # self.joint_state.position[3] += self.speed
            else: pass
        else: print("joint3,4 value is maximum, so you don't move joint3,4")
        

        
    def move_object(self):
        sleep(0.3)
        self.joint_state.position[:4] = [-1.55, -0.17, -0.62, -0.90]
        self.pub.publish(self.joint_state)
        sleep(0.5)
        self.joint_state.position[5] = -1.5
        self.pub.publish(self.joint_state)
        sleep(1.5)
        self.joint_state.position = copy.deepcopy(self.ready1)
        self.pub.publish(self.joint_state)
        sleep(1.0)

        
    def my_callback(self, msg):
        self.pub = rospy.Publisher('move_arm', JointState, queue_size=1)
        self.pub.publish(self.joint_state)
        if msg.bounding_boxes != []:
            ob_class = str(msg.bounding_boxes[0].Class)
            ob_proba = float(msg.bounding_boxes[0].probability)
            ob_xmin = int(msg.bounding_boxes[0].xmin)
            ob_ymin = int(msg.bounding_boxes[0].ymin)
            ob_xmax = int(msg.bounding_boxes[0].xmax)
            ob_ymax = int(msg.bounding_boxes[0].ymax)

            bbox_Xcenter = ob_xmin + (ob_xmax - ob_xmin)/2
            bbox_Ycenter = (ob_ymin + (ob_ymax - ob_ymin)/2) + 50
            
            if self.flag:
                self.firstX = copy.deepcopy(bbox_Xcenter)
                self.flag = False

            if ob_class != 'vinyl':
            
                self.follow_object(bbox_Xcenter, bbox_Ycenter) #grab
                
            
                # self.grap(self.firstX, ob_ymax)
                
            if self.joint_state.position[0] > -0.9:#push
                self.push_object()

            ob_class, ob_ymax = None, None
            self.start_time = time.time()
            
        else: 
            self.flag = True
            current_time = time.time()
            end_time = current_time - self.start_time
            if end_time > 5:
                print("didn't find object!")
                self.joint_state.position = copy.deepcopy(self.ready1)
                self.pub.publish(self.joint_state)
                sleep(2.0)
                
                
            
def main(args):
    start = pub_to_dofbot()
        
    try:
        rospy.spin()
        
    except KeyboardInterrupt:
        print("Shutting down")

    print('Bye')
        
        
if __name__ == '__main__':
    main(sys.argv)