#!/usr/bin/env python

import baxter_interface
import baxter_external_devices
import rospy
import rospkg
import re
import itertools

from baxter_interface import CHECK_VERSION
from std_srvs.srv import Empty, EmptyResponse

if __name__ == '__main__':

    rospy.init_node('baxter_sequence_calibration_ids')

    sensor_name = rospy.get_param("sensor_name")

    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)

    print("Enabling robot... ")
    rs.enable()

    left_arm = baxter_interface.Limb("left")
    left_arm_joint_names = left_arm.joint_names()
    
    right_arm = baxter_interface.Limb("right")
    right_arm_joint_names = right_arm.joint_names()


    #arm.move_to_neutral()

    print ("Waiting for servers... ")
    rospy.wait_for_service('read_tfs')
    rospy.wait_for_service('compute_frames')
    print ("Connected to servers. ")

    rospack = rospkg.RosPack()
    file_path = rospack.get_path('calibration_routine')
    fr = open (file_path+'/data/'+sensor_name+'_calib_sequence_right.txt')
    fl = open (file_path+'/data/'+sensor_name+'_calib_sequence_left.txt')
    
    rospy.sleep(15.0)

    print ("Executing "+sensor_name+" calibration sequence... ")
    for liner, linel in itertools.izip(fr, fl):
        left_joint_list = re.sub(r'\n', '', linel).split(',')
        right_joint_list = re.sub(r'\n', '', liner).split(',')

        joint_command_r = {right_arm_joint_names[2]: float(right_joint_list[0]), right_arm_joint_names[3]: float(right_joint_list[1]), right_arm_joint_names[0]: float(right_joint_list[2]),
            right_arm_joint_names[1]: float(right_joint_list[3]), right_arm_joint_names[4]: float(right_joint_list[4]), right_arm_joint_names[5]: float(right_joint_list[5]), 
            right_arm_joint_names[6]: float(right_joint_list[6])}
            
        joint_command_l = {left_arm_joint_names[2]: float(left_joint_list[0]), left_arm_joint_names[3]: float(left_joint_list[1]), left_arm_joint_names[0]: float(left_joint_list[2]),
            left_arm_joint_names[1]: float(left_joint_list[3]), left_arm_joint_names[4]: float(left_joint_list[4]), left_arm_joint_names[5]: float(left_joint_list[5]), 
            left_arm_joint_names[6]: float(left_joint_list[6])}
            
        left_arm.move_to_joint_positions(joint_command_l)
        right_arm.move_to_joint_positions(joint_command_r)

        rospy.sleep(4.0)
        try:
            rtfs = rospy.ServiceProxy('read_tfs', Empty)
            resp = rtfs()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        rospy.sleep(2.0)

    try:
        cf = rospy.ServiceProxy('compute_frames', Empty)
        resp = cf()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    print ("Done.")




