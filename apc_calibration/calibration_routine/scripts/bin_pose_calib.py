#!/usr/bin/env python

import yaml
import rospy
import rospkg
import baxter_interface
import tf

bin_A = "bin_A"
bin_B = "bin_B"
bin_C = "bin_C"
bin_D = "bin_D"
bin_E = "bin_E"
bin_F = "bin_F"
bin_G = "bin_G"
bin_H = "bin_H"
bin_I = "bin_I"
bin_J = "bin_J"
bin_K = "bin_K"
bin_L = "bin_L"
tote = "tote"
GOOD_BINS = [bin_A, bin_B, bin_C, bin_D, bin_E, bin_F, bin_G, bin_H, bin_I, bin_J, bin_K, bin_L, tote]

left_gripper = "left_gripper_base"
right_gripper = "right_gripper_base"


class BinCalibrator:
    """Calibrates robot poses"""

    def __init__(self):
        rospk = rospkg.RosPack()
        rospy.init_node("bin_calibration")

        self._data_file_path = rospk.get_path('calibration_routine') + "/data/"
        self._data_left_arm = {}
        self._data_right_arm = {}
        self._config_mode = None
        self._listener = tf.TransformListener()

    def getPoseDict(self, arm_name, bin_name):
        if arm_name == "right":
            gripper = right_gripper
        elif arm_name == "left":
            gripper = left_gripper

        try:
            self._listener.waitForTransform(
                bin_name, gripper, rospy.Time(0),
                rospy.Duration(1))
            (trans, rot) = self._listener.lookupTransform(bin_name,
                                                          gripper,
                                                          rospy.Time(0))
        except:
            rospy.logerr("%s failed!" % self._action_name)
            return False

        return {'position': {'x': float(trans[0]), 'y': float(trans[1]), 'z': float(trans[2])}, 'orientation': {'x': float(rot[0]), 'y': float(rot[1]), 'z': float(rot[2]), 'w': float(rot[3])}}

    def calibrateArm(self):
        arm_name = raw_input("Arm name (left / right): ")
        pose_name = raw_input("Bin name and pose (bin_A_1, bin_B_3, ...): ")
        bin_name = pose_name[0:5]

        if (arm_name == "right" or arm_name == "left") and (bin_name in GOOD_BINS):
            arm = baxter_interface.limb.Limb(arm_name)

            raw_input("Press enter when arm in desired pose")

            if self._config_mode == "joint":
	            temp_data = {pose_name : arm.joint_angles()}
            elif self._config_mode == "pose":
                temp_data = {pose_name : self.getPoseDict(arm_name, bin_name)}
            else:
                temp_data = {pose_name + "_pose" : self.getPoseDict(arm_name, bin_name), pose_name + "_joint" : self.getPoseDict(arm_name, bin_name)}

    	    print "Will add the following data: " + str(yaml.dump(temp_data))
    	    dump = raw_input("Type 'no' to discard")

    	    if dump != 'no':
    	        if arm_name == "right":
                    data = self._data_right_arm
    	        else:
                    data = self._data_left_arm

                if self._config_mode == "joint":
        	         data[pose_name] = arm.joint_angles()
                elif self._config_mode == "pose":
                     data[pose_name] = self.getPoseDict(arm_name, bin_name)
                else:
                    data[pose_name + "_joint"] = arm.joint_angles()
                    data[pose_name + "_pose"] = self.getPoseDict(arm_name, bin_name)
    	    else:
    		    print "Discarded data"
    	else:
            if bin_name not in GOOD_BINS:
                print "wrong bin name!"
            else:
	            print "Wrong arm name!"



    def calibrationCycle(self):
        print "Welcome to the APC bin calibrator! This program will accept new arm poses in a cycle and save the output as a yaml file"

        finished = False
        self._config_mode = raw_input("Config mode (joint or pose): ")
        if self._config_mode !="joint" and self._config_mode != "pose" and self._config_mode != "both":
            rospy.logerr("Config mode should be 'joint' or 'pose'")
            return False

        while finished != True:
                self.calibrateArm()
                dump = raw_input("Type 'yes' to finish, anything else to continue: ")

            	if dump == 'yes':
                   finished = True


        print "Will save datafile"
        data = {"left_arm": self._data_left_arm, "right_arm": self._data_right_arm}

        with open(self._data_file_path + "binConfigurations.yaml", "w") as file:
            file.write(yaml.dump(data))


if __name__ == "__main__":
    calibrator = BinCalibrator()

    calibrator.calibrationCycle()
