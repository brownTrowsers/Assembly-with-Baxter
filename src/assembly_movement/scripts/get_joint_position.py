#!/usr/bin/env python

import argparse
import rospy
import numpy as np

# from sensor_msgs.msg import JointState

import baxter_interface

def callback(data):
    rate = rospy.Rate(2)
    pos = np.zeros(len(data.name))
    for c in range(len(data.name)):
        pos[c] = data.position[c]
        rospy.loginfo("%s: %s", data.name[c] ,data.position[c])
    rate.sleep()

def Monitor():
    # rospy.Subscriber("/robot/joint_states",JointState,callback)
    
    left_arm = baxter_interface.limb.Limb('left')
    right_arm = baxter_interface.limb.Limb('right')
    current_left_arm = left_arm.endpoint_pose()
    current_right_arm = right_arm.endpoint_pose()
    rospy.loginfo("left_arm pose: \n %s", current_left_arm)
    rospy.loginfo("right_arm pose: \n %s", current_right_arm)

def main():
    """
    Commands get joint position 
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("joint_monitor")

    Monitor()
    rospy.spin()

    print("Done")

if __name__ == '__main__':
    main()