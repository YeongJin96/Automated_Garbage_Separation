#!/usr/bin/env python
import rospy

def start_node():
    rospy.init_node('image_pub')
    rospy.loginfo('image_pub node started')

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
