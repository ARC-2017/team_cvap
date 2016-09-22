#!/usr/bin/env python

import baxter_interface
import baxter_external_devices
import rospy
import rospkg
import re

from baxter_interface import CHECK_VERSION
from std_srvs.srv import Empty, EmptyResponse

if __name__ == '__main__':

    rospy.init_node('baxter_sequence_calibration')

    arm_name = rospy.get_param("arm_name")
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

    arm = baxter_interface.Limb(arm_name)
    arm_joint_names = arm.joint_names()

    #arm.move_to_neutral()

    print ("Waiting for servers... ")
    rospy.wait_for_service('read_tfs')
    rospy.wait_for_service('compute_frames')
    print ("Connected to servers. ")

    rospack = rospkg.RosPack()
    file_path = rospack.get_path('calibration_routine')
    f = open (file_path+'/data/'+sensor_name+'_calib_sequence.txt')

    print ("Executing "+sensor_name+" calibration sequence... ")
    for line in f:
        joint_list = re.sub(r'\n', '', line).split(',')

        joint_command = {arm_joint_names[2]: float(joint_list[0]), arm_joint_names[3]: float(joint_list[1]), arm_joint_names[0]: float(joint_list[2]), arm_joint_names[1]: float(joint_list[3]), 
            arm_joint_names[4]: float(joint_list[4]), arm_joint_names[5]: float(joint_list[5]), arm_joint_names[6]: float(joint_list[6])}
        arm.move_to_joint_positions(joint_command)

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




