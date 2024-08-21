#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
from detection_msgs.msg import BoundingBox, BoundingBoxes
import sys
from math import pi
from time import sleep
from geometry_msgs.msg import Pose
from moveit_commander.move_group import MoveGroupCommander
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import JointState
import Arm_Lib

# import Arm_Lib

DE2RA = pi / 180

ready = [-1.17,-0.40,-0.36,-1.42,0.0,0.0]

class get_cood:
    def __init__(self):
        rospy.init_node("subscriber")
        self.box_info = [None,None,None,None,None,None]
        rospy.Subscriber("/yolov5/detections", BoundingBoxes, self.my_callback, queue_size=1)
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
        dofbot.Arm_serial_servo_write6_array(ready, 100)

        self.joint_state = JointState()
        joint = dofbot.get_active_joints()
        self.joint_state.name = joint
        self.joint_state.position = dofbot.get_current_joint_values()

        
        resolution_x = 640
        resolution_y = 480
        self.centerX = resolution_x/2
        self.centerY = resolution_y/2
        tolerance_x = 100
        tolerance_y = 30
        self.lower_x = (resolution_x - tolerance_x)/2
        self.upper_x = (resolution_x + tolerance_x)/2
        self.lower_y = (resolution_y - tolerance_y)/2
        self.upper_y = (resolution_y + tolerance_y)/2

        self.value = 0.03

        self.garbage_list = ['can', 'plastic', 'paper', 'bottle']

    def move_arm(self, target_centerX, target_centerY):
      # if self.lower_x < target_centerX < self.upper_x and self.lower_y < target_centerY < self.upper_y:

      # if self.lower_y < target_centerY:
      #   self.joint_state.position[3] += self.spped

      # elif self.upper_y < target_centerY:
      #   self.joint_state.position[3] -= self.spped

      if self.lower_x < target_centerX:
        self.joint_state.position[0] -= self.value

      elif self.lower_x > target_centerX:
        self.joint_state.position[0] += self.value


      dofbot.set_joint_value_target(self.joint_state)
      dofbot.go(self.joint_state)
      
      print('in move_arm')

    # def grap(self):
    #   self.joint_state.effort = [42.0]

    def my_callback(self, msg):
      box_info = [None,None,None,None,None,None]
      if msg.bounding_boxes[0].Class:        
          ob_class = str(msg.bounding_boxes[0].Class)
          ob_proba = float(msg.bounding_boxes[0].probability)
          ob_xmin = int(msg.bounding_boxes[0].xmin)
          ob_ymin = int(msg.bounding_boxes[0].ymin)
          ob_xmax = int(msg.bounding_boxes[0].xmax)
          ob_ymax = int(msg.bounding_boxes[0].ymax)
      

          bbox_Xcenter = ob_xmin + (ob_xmax - ob_xmin)/2
          bbox_Ycenter = ob_ymin + (ob_ymax - ob_ymin)/2

          info_list = [ob_class, ob_proba, ob_xmin, ob_ymin, ob_xmax, ob_ymax]
          

          for i in range(len(box_info)):
              box_info[i] = info_list[i]
          print('In Callback',box_info)

          self.move_arm(bbox_Xcenter, bbox_Ycenter)
          print(self.joint_state.velocity)
          
          if self.lower_x < bbox_Xcenter < self.upper_x and ob_class=='paper':
            self.joint_state.position[1:] = [-1.19, -0.57, -0.04, 3.14]
            dofbot.go(self.joint_state)
            sleep(1.5)

            dofbot.Arm_serial_servo_write6_array(ready, 100)

          elif self.joint_state.position[0] > 0:
            dofbot.Arm_serial_servo_write6_array(ready, 100)
        
          # if self.centerX < bbox_Xcenter:
          #   self.joint_state.position[0] += 0.05

          # elif self.centerX > bbox_Xcenter:
          #   self.joint_state.position[0] -= 0.05

          # else:
          #   pass
          # dofbot.set_joint_value_target(self.joint_state)
          # dofbot.go(self.joint_state)

          # sleep(1.5)



    def get_data(self):
        return self.box_info

def main(args):
  obc = get_cood()
  try:
    # rospy.sleep(0.3)
    rospy.spin()

  except KeyboardInterrupt:
    print("Shutting down")

  print('hi')
  print(obc.get_data())


  
if __name__ == '__main__':
    main(sys.argv)

# a = get_cood()
# print(a)
