#!/usr/bin/env python3

import rospy

def main():
    rospy.init_node('test_node')
    rospy.loginfo("ROS Gym Automation test node started")
    
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        rospy.loginfo("Test node is running")
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass 