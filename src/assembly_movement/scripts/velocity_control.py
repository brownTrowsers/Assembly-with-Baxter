#!/usr/bin/env python

import numpy as np
import struct
import rospy

import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest
)

from std_msgs.msg import (
    UInt16,
    Header,
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    WrenchStamped,
)

from netft_utils.srv import (
    SetBias,
)

#Costants Definitions
PI = np.pi

#Distance Offsets (m)
L0 = 0.27035
L1 = 0.069
L2 = 0.36435
L3 = 0.069
L4 = 0.37429
L5 = 0.01
L6 = 0.3945


class VelControl(object):
    def __init__(self):
        self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16, queue_size=10)
        self._right_arm = baxter_interface.limb.Limb("right")
        self._left_arm = baxter_interface.limb.Limb("left")
        # control parameters
        self._rate = 500.0  # Hz

        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

        # set joint state publishing to 500Hz
        self._pub_rate.publish(self._rate)

        # Sets force limit flag to false
        self.max_wrench = False

    def _reset_control_modes(self):
        rate = rospy.Rate(self._rate)
        for _ in np.arange(100):
            if rospy.is_shutdown():
                return False
            self._right_arm.exit_control_mode()
            self._left_arm.exit_control_mode()
            self._pub_rate.publish(100)
            rate.sleep()

        return True

    def set_neutral(self):
        print("Moving to neutral pose...")
        self._right_arm.move_to_neutral()
        self._left_arm.move_to_neutral()

    def clean_shutdown(self):
        print("\nExiting...")
        self._reset_control_modes()
        #  self.set_neutral()
        '''
        if not self._init_state:
            print("Disabling robot...")
            self._rs.disable()
        '''
        return True

    def _ik_request(self, limb, pose):
        """
        Return: IK joint angle solutions for a given pose, as list
        Argument: Limb, as a string. Pose, as defined by RR SDK/Wiki
        """
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            rospy.wait_for_service(ns,5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False

        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        if resp_seeds[0] != resp.RESULT_INVALID:
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False

        return limb_joints


    def set_wrench_bias(self):
        """
        resets the ATI force sesnor to a 0 force bias and sets the 
        warning limits 50 for force and 10 for torque
        """
        rospy.wait_for_service('/bias')
        try:
            bias_cmd = rospy.ServiceProxy('/bias', SetBias)
            resp1 = bias_cmd(True,50.0,10.0)
            return resp1
        except rospy.ServiceException as e:
            print ("Service call failed: %s"%e)
    
    def wrench_current(self):
        """
        Current wrench values from ATI force-torque sensor. 
        '/transformed_world' is the biased topic
        source from Mats Nilsson
        """

        while not rospy.is_shutdown():
            msg = rospy.wait_for_message("/transformed_world", WrenchStamped)
            fx = msg.wrench.force.x
            fy = msg.wrench.force.y
            fz = msg.wrench.force.z
            tx = msg.wrench.torque.x
            ty = msg.wrench.torque.y
            tz = msg.wrench.torque.z
            wrench = np.array([[fx,fy,fz,tx,ty,tz]]).T
            if fz <= -40:
                self.max_wrench = True
            break
        
        return wrench

    def create_dh_array(self, joints):
        # DH parameter matrix
        dh = np.array([[L1, -PI/2, joints[0], L0],      #S0
                       [0, PI/2, joints[1] + PI/2, 0],  #S1
                       [L3, -PI/2, joints[2], L2],      #E0
                       [0, PI/2, joints[3], 0],         #E1
                       [L5, -PI/2, joints[4], L4],      #W0
                       [0, PI/2, joints[5], 0],         #W1
                       [0, 0, joints[6], L6]])          #W2
        return dh


    def homo_mats(self, arm, dh, start, finish):
        # Homogenous transformation matrices between joints
        H = np.zeros((finish, 4, 4))
        if(arm == "left"):
            H[start] = np.array([[np.cos(PI/4), -np.sin(PI/4), 0, 0],
                             [np.sin(PI/4), np.cos(PI/4), 0, 0],
                             [0, 0, 1, 0],
                             [0, 0, 0, 1]])
        elif(arm == "right"):
            H[start] = np.array([[np.cos(-PI/4), -np.sin(-PI/4), 0, 0],
                             [np.sin(-PI/4), np.cos(-PI/4), 0, 0],
                             [0, 0, 1, 0],
                             [0, 0, 0, 1]])
        for i in range(start+1, finish):
            H[i] = np.array([[np.cos(dh[i][2]), -np.sin(dh[i][2])*np.cos(dh[i][1]), np.sin(dh[i][2])*np.sin(dh[i][1]), dh[i][0]*np.cos(dh[i][2])],
                            [np.sin(dh[i][2]), np.cos(dh[i][2])*np.cos(dh[i][1]), -np.cos(dh[i][2])*np.sin(dh[i][1]), dh[i][0]*np.sin(dh[i][2])],
                            [0, np.sin(dh[i][1]), np.cos(dh[i][1]), dh[i][3]],
                            [0, 0, 0, 1]])

        # Transformations to joints from base frame
        H_mats = np.zeros((finish,4,4))
        H_mats[0] = H[0]
        for i in range((start + 1), finish):
            H_mats[i] = np.dot(H_mats[i-1], H[i])

        return H_mats

    def get_jacobian(self, H_mats):
        J = []
        #Note need to apply base transformation
        #Get Displacement [x, y, z]
        D07 = [H_mats[len(H_mats) - 1][0][3],
               H_mats[len(H_mats) - 1][1][3],
               H_mats[len(H_mats) - 1][2][3]]

        #Calc base case
        rot = [0, 0, 1]
        lin = np.cross(rot, D07)
        J.append(np.append(lin, rot)) #First Column of Jacobian

        for i in range(len(H_mats)- 1): #Other 6 columns of Jacobian
            rot = [H_mats[i][0][2], H_mats[i][1][2], H_mats[i][2][2]]
            D0i = [H_mats[i][0][3], H_mats[i][1][3], H_mats[i][2][3]]
            lin = np.cross(rot, (np.subtract(D07, D0i)))
            J.append(np.append(lin, rot))
        J = np.transpose(J)

        return J

    def vel_cmd(self, arm):
        rate = rospy.Rate(self._rate)
        itera = 0

        while (itera <= 6000):
            self._pub_rate.publish(self._rate)

            if(arm == "left"):
                names = self._left_arm.joint_names()
                # names = ["left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"]
                jald = self._left_arm.joint_angles() #Gets joint angles of left arm
                q = [jald["left_s0"], jald["left_s1"], jald["left_e0"],
                            jald["left_e1"], jald["left_w0"], jald["left_w1"],
                            jald["left_w2"]]
            elif(arm == "right"):
                names = self._right_arm.joint_names()
                jard = self._right_arm.joint_angles() #Gets joint angles of right arm
                q = [jard["right_s0"], jard["right_s1"], jard["right_e0"],
                            jard["right_e1"], jard["right_w0"], jard["right_w1"],
                            jard["right_w2"]]
            # print(q)


            # target end effector velocity
            x = 0.0
            y = 0.01
            z = 0.0
            w_x = 0
            w_y = 0
            w_z = 0
            X = [x, y, z, w_x, w_y, w_z]

            # Calculate required joint velocities
            dh = self.create_dh_array(q)
            h_trans = self.homo_mats(arm, dh, 0, 7)
            j = self.get_jacobian(h_trans)
            j_inv = np.linalg.pinv(j)
            joint_vel = np.dot(j_inv, X)

            # Create joint velocity dict
            vel_dict = dict()
            for idx, name in enumerate(names):
                vel_dict[name] = joint_vel[idx]

            #Set Joint Velocities
            if(arm == "left"):
                self._left_arm.set_joint_velocities(vel_dict)
                # print(vel_dict)
                # print(cur_pos)
            if(arm == "right"):
                self._right_arm.set_joint_velocities(vel_dict)
                # self.wrench_current()
                # print(force)
                # print(vel_dict)
                # print(cur_pos)

            itera += 1
            rate.sleep()

        print(itera)
        print(joint_vel)



def main():
    print("Initializing node... ")
    rospy.init_node("arm_velocity_controller")

    move = VelControl()
    exiting = move.clean_shutdown
    rospy.on_shutdown(exiting)

    if(exiting != True):
        print("In Main Loop")
        move.vel_cmd("right")

    print("Done.")
    return 0

if __name__ == '__main__':
    main()