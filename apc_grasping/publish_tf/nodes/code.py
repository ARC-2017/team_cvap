#!/usr/bin/env python  
import roslib
roslib.load_manifest('publish_tf')
import rospy
import tf
import yaml


def publish_bin_pose(binname, data):
    br = tf.TransformBroadcaster()
    br.sendTransform((data["position"]["x"], data["position"]["x"], data["position"]["x"]),
                     (data["orientation"]["w"], data["orientation"]["x"], data["orientation"]["y"], data["orientation"]["z"]),
               	     rospy.Time.now(),
		     binname,
		     "world")
    #print data["position"]["x"], data["position"]["x"], data["position"]["x"], data["orientation"]["w"], data["orientation"]["x"], data["orientation"]["y"], data["orientation"]["z"]


if __name__ == '__main__':
    rospy.init_node('code')
    bins = ["bin_A", "bin_B", "bin_C", "bin_D", "bin_E", "bin_F", "bin_G", "bin_H", "bin_I", "bin_J", "bin_K", "bin_L"]
    f = open('shelf_bin_pre_poses.yaml', 'r')
    data = yaml.load(f)
    bin_info = data[bins[0]]
    for i in range(12):  
	binname = bins[i]        
    	publish_bin_pose(binname, data[binname])
    

    rospy.spin()
