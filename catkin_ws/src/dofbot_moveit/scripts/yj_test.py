#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
from detection_msgs.msg import BoundingBox, BoundingBoxes
import sys
import math
import time
from time import sleep
from geometry_msgs.msg import Pose
from moveit_commander.move_group import MoveGroupCommander
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import JointState
import copy


DE2RA = math.pi / 180

class get_cood:
    def __init__(self):
        rospy.init_node("test", anonymous=True)
        self.box_info = [None,None,None,None,None,None]
        

        #dofbot
        global dofbot
        dofbot = MoveGroupCommander("dofbot")
        dofbot.allow_replanning(True)
        dofbot.set_planning_time(1)
        dofbot.set_num_planning_attempts(1)
        dofbot.set_goal_position_tolerance(0.01)
        dofbot.set_goal_orientation_tolerance(0.01)
        dofbot.set_goal_tolerance(0.01)
        dofbot.set_max_velocity_scaling_factor(1.0)
        dofbot.set_max_acceleration_scaling_factor(1.0)
        # dofbot.set_named_target("ready")
        # #ready 값 (바꿔야함) [-1.21, -0.51, -0.64, -0.98, 0.0]
        # dofbot.go()

        #message
        self.joint_state = JointState()
        self.joint_state.name = dofbot.get_active_joints()
        self.joint_state.position = dofbot.get_current_joint_values()
        
        self.joint_state.position = [-0.91, -0.53, -0.64, -0.98, 0.0]
        dofbot.go(self.joint_state)

        #cood
        resolution_x = 640
        resolution_y = 480
        self.centerX = resolution_x/2
        self.centerY = resolution_y/2
        tolerance_x = 80
        tolerance_y = 30
        self.lower_x = (resolution_x - tolerance_x)/2
        self.upper_x = (resolution_x + tolerance_x)/2
        self.lower_y = (resolution_y - tolerance_y)/2
        self.upper_y = (resolution_y + tolerance_y)/2

        self.value = 0.1

        self.cnt = 1
        self.vec_flag = True
        self.vec_lists = []
        
        self.garbage_list = ['can', 'plastic', 'paper', 'bottle']
        
        rospy.Subscriber("/yolov5/detections", BoundingBoxes, self.my_callback, queue_size=1)

    def move_arm(self, target_centerX, target_centerY):
      
      # if self.lower_x < target_centerX < self.upper_x and self.lower_y < target_centerY < self.upper_y:

      # if self.lower_y < target_centerY:
      #   self.joint_state.position[3] += self.spped

      # elif self.upper_y < target_centerY:
      #   self.joint_state.position[3] -= self.spped
  
      if self.lower_x < target_centerX:
        self.joint_state.position[0] -= self.value - 0.05

      elif self.lower_x > target_centerX:
        self.joint_state.position[0] += self.value


      dofbot.set_joint_value_target(self.joint_state)
      dofbot.go(self.joint_state)

    def Point2D(self, p1, p2):
      x = p2[0] - p1[0]
      y = p2[1] - p1[1]
      middle_line = math.sqrt((x**2) + (y**2))
      #print(middle_line)
      
      return middle_line
    
    def my_callback(self, msg):
      box_info = [None,None,None,None,None,None]
      if msg.bounding_boxes != []:
          ob_class = str(msg.bounding_boxes[0].Class)
          ob_proba = float(msg.bounding_boxes[0].probability)
          ob_xmin = int(msg.bounding_boxes[0].xmin)
          ob_ymin = int(msg.bounding_boxes[0].ymin)
          ob_xmax = int(msg.bounding_boxes[0].xmax)
          ob_ymax = int(msg.bounding_boxes[0].ymax)

          bbox_Xcenter = ob_xmin + (ob_xmax - ob_xmin)/2
          bbox_Ycenter = ob_ymin + (ob_ymax - ob_ymin)/2
          
          vec = (bbox_Xcenter, bbox_Ycenter)
  
          # if self.cnt % 2 == 0:
          if self.vec_flag:
            self.prev_vec = vec
            self.vec_flag = False
            self.start_time = time.time()
            
          else: 
            self.vec_flag = True
            pred_line = self.Point2D(self.prev_vec, vec)
            self.vec_lists.append(pred_line)
            if len(self.vec_lists) == 5:
              end_time = time.time()
              elapsed_time = end_time - self.start_time
              avg_line = 0
              for vec_list in self.vec_lists:
                avg_line += vec_list
              avg_line /= 5
              print('AVG',avg_line,elapsed_time)
              self.vec_lists = []
          
          info_list= [ob_class, ob_proba, ob_xmin, ob_ymin, ob_xmax, ob_ymax]

          # for i in range(len(box_info)):
              # box_info[i] = info_list[i]
          # print('In Callback',box_info)

          # self.move_arm(bbox_Xcenter, bbox_Ycenter)

          # if self.lower_x < bbox_Xcenter < self.upper_x and bbox_Ycenter > 150 and ob_class=='paper':
          
          # print("ob_xmax - ob_xmin ", ob_xmax - ob_xmin) # 205
          # print("ob_ymax - ob_ymin ", ob_ymax - ob_ymin) # 235
          
          x_sub = ob_xmax - ob_xmin
          y_sub = ob_ymax - ob_ymin
          
          if ob_class=='paper':
          # if ob_class=='paper' and x_sub > 205 and y_sub > 235:
            
            sleep(3.0)
          #   # (잡기용)
          #   # self.joint_state.position = [0.0, -1.21, -0.64, -0.66, 3.10]
          #   # dofbot.go(self.joint_state)
            
            # (밀기)
            self.joint_state.position = [0.0, -1.53, -0.06, -1.0, 0.0]
            dofbot.go(self.joint_state)
            
            self.joint_state.position = [0.0, -1.53, -0.06, 0.20, 0.0]
            dofbot.go(self.joint_state)
            
          #   # 버리기 ( jetson 수정 해야함 )
          #   #self.joint_state.position[-1] = [-1.5, -0.40, -0.37, -1.42]
          #   #dofbot.go(self.joint_state)
          
            self.joint_state.position = [-0.91, -0.53, -0.64, -0.98, 0.0]
            dofbot.go(self.joint_state)
            
            # dofbot.set_named_target("ready")
            # dofbot.go()
          
      # self.cnt+=1
      # print(self.cnt)  
            


def main(args):
  
  obc = get_cood()
  try:
    # rospy.sleep(0.3)
    rospy.spin()

  except KeyboardInterrupt:
    print("Shutting down")
  
  print('Bye')


  
if __name__ == '__main__':
    main(sys.argv)

# a = get_cood()
# print(a)
