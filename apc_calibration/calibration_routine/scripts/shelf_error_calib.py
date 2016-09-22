#!/usr/bin/env python

import sys
import PyKDL as kdl
import argparse
import numpy as np
import tf
import sys
import rospy
import rospkg
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, MoveGroupAction
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import threading
import Tkinter as tk

pitch = 0.0
roll  = 0.0
yaw   = 0.0

posx  = 0.0
posy  = 0.0
posz  = 0.0

def writeOutputFile(array_shelf_front):
    global pitch, roll, yaw, posx, posy, posz

    rootDir= rospkg.RosPack().get_path('calibration_data') + '/extrinsics/'
    file_shelf_front_output = rootDir + 'shelf_front_manually_calibrated.txt'
    file_output = open(file_shelf_front_output, "w" )
    
    ## transform F1 between the base and the shelf_front
    x1 = array_shelf_front[0]
    y1 = array_shelf_front[1]
    z1 = array_shelf_front[2]
    q1 = array_shelf_front[3:7]
    position_xyz1 = array_shelf_front[0:3]
    orientation_xyzw1 = array_shelf_front[3:7]
    poseq1 = np.array(array_shelf_front[0:7])

    R1 = kdl.Rotation.Quaternion(*q1)
    rpy1 = R1.GetRPY()


    F1 = kdl.Frame(kdl.Rotation.RPY(rpy1[0]+roll, rpy1[1]+pitch,rpy1[2]+yaw), 
                                    kdl.Vector(x1+posx, y1+posy, z1+posz   ))


    position_xyz_out = F1.p
    orientation_xyzw_out = F1.M.GetQuaternion()
    
    line_output = (str(position_xyz_out[0]) + '\t' + str(position_xyz_out[1]) + '\t' + str(position_xyz_out[2]) + '\t'
                 + str(orientation_xyzw_out[0]) + '\t' + str(orientation_xyzw_out[1]) + '\t' + str(orientation_xyzw_out[2]) + '\t' + str(orientation_xyzw_out[3]))

    print('writing: ', line_output, ' to the file: ', file_shelf_front_output)    
    
    file_output.write(line_output)
    file_output.close()

def calculateComposeTransform(array_shelf_front, array_shelf):
    global pitch, roll, yaw, posx, posy, posz
    ## transform F1 between the base and the shelf_front
    x1 = array_shelf_front[0]
    y1 = array_shelf_front[1]
    z1 = array_shelf_front[2]
    q1 = array_shelf_front[3:7]
    position_xyz1 = array_shelf_front[0:3]
    orientation_xyzw1 = array_shelf_front[3:7]
    poseq1 = np.array(array_shelf_front[0:7])

    R1 = kdl.Rotation.Quaternion(*q1)
    rpy1 = R1.GetRPY()
    pose1 = np.array((x1, y1, z1) + rpy1)
#    print('[x1, y1, z1, qx1, qy1, qz1, qw1]: ')
#    print(np.array_str(poseq1, precision=4))
#    print('[x1, y1, z1, roll1, pitch1, yaw1]: ')
#    print(np.array_str(pose1, precision=4))

    ## transform F2 between the shelf_front and the shelf_base
    x2 = array_shelf[0]
    y2 = array_shelf[1]
    z2 = array_shelf[2]
    q2 = array_shelf[3:7]
    position_xyz2 = array_shelf[0:3]
    orientation_xyzw2 = array_shelf[3:7]
    poseq2 = np.array(array_shelf[0:7])

    R2 = kdl.Rotation.Quaternion(*q2)
    rpy2 = R2.GetRPY()
    pose2 = np.array((x2, y2, z2) + rpy2)
#    print('[x2, y2, z2, qx2, qy2, qz2, qw2]: ')
#    print(np.array_str(poseq2, precision=4))
#    print('[x2, y2, z2, roll2, pitch2, yaw2]: ')
#    print(np.array_str(pose2, precision=4))

    ## combining the transforms
    F1 = kdl.Frame(kdl.Rotation.RPY(rpy1[0]+roll, rpy1[1]+pitch,rpy1[2]+yaw), 
                                    kdl.Vector(x1+posx, y1+posy, z1+posz   ))

    F2 = kdl.Frame(kdl.Rotation.RPY(rpy2[0],rpy2[1],rpy2[2]), kdl.Vector(x2,y2,z2))
    
    F3 = F1*F2
    #print("F3")
    #print F3
    position_xyz3 = F3.p
    orientation_xyzw3 = F3.M.GetQuaternion()
    return position_xyz3, orientation_xyzw3


def createROSPose(position, rotation, frame_id='/base'):
        """ Creates a stamped pose from a list position and a list rotation.
            @param position - [x, y, z]
            @param rotation - [x, y, z, w]
            @param frame_id - the frame
        """
        header = Header(stamp=rospy.Time.now(), frame_id=frame_id)
        stampedPose = PoseStamped(header=header)
        stampedPose.pose.position.x = position[0]
        stampedPose.pose.position.y = position[1]
        stampedPose.pose.position.z = position[2]
        stampedPose.pose.orientation.x = rotation[0]
        stampedPose.pose.orientation.y = rotation[1]
        stampedPose.pose.orientation.z = rotation[2]
        stampedPose.pose.orientation.w = rotation[3]
        return stampedPose


class ShelfErrorCalibration(object):
    def __init__(self, argv, configFileName=None):
        self._objectsInScene = ["shelf"]
        self._tfListener = tf.TransformListener()
        #self._planningScene = []
        self._namedConfigurations = {}
        self._co_publisher = rospy.Publisher('/collision_object', CollisionObject, queue_size=1)
        #self._lock = threading.RLock()
        #self._moveitScene = moveit_commander.PlanningSceneInterface()
        if configFileName is not None:
            try:
                configFile = open(configFileName, 'r')
                self._namedConfigurations = yaml.load(configFile)
                configFile.close()
            except IOError as err:
                rospy.logerr('Could not load configurations from file ' + configFileName)
        rospy.loginfo('Shelf error Calibration started successfully.')
        
    
    def getTransform(self):
        for trial in range(10):
               try:
                    (position_xyz, orientation_xyzw) = self._tfListener.lookupTransform('/base', '/shelf_front', rospy.Time(0))
                    break
               except Exception:
                    print('not found tf yet, waiting 0.5 seconds with 10 attempts')
                    rospy.sleep(0.5)
        #print('target frame')
        #print(position_xyz, orientation_xyzw)
        return createROSPose(position_xyz, orientation_xyzw)

 
    def _moveObject(self, objectName, objectPose):
        co = CollisionObject()
        co.operation = co.MOVE
        co.id = objectName
        co.mesh_poses = [objectPose.pose]
        co.header.stamp = objectPose.header.stamp
        co.header.frame_id = objectPose.header.frame_id
        self._co_publisher.publish(co)

    def moveObjectInPlanningScene(self, objectName, filePose):
        objectPose = self.getTransform()
        #print("object pose")
        #print(objectPose)
#        self._moveObject(objectName=objectName, objectPose=objectPose)
        self._moveObject(objectName=objectName, objectPose=filePose)
 


def onKeyPress(event):
    text.insert('end', 'You pressed %s\n' % (event.char, ))
    global pitch, roll, yaw, posx, posy, posz
    global shelf_calibrator
    global array_shelf_front, array_shelf

    ## section for adjusting the angle of the shelf
    step_size_angle = 1 * 2*np.pi/360
    if event.char == '7':
        pitch = pitch + step_size_angle
    if event.char == '4':
        pitch = pitch - step_size_angle
    if event.char == '1':
        pitch = 0.0

    if event.char == '8':
        roll = roll + step_size_angle
    if event.char == '5':
        roll = roll - step_size_angle
    if event.char == '2':
        roll = 0.0
    
    if event.char == '9':
        yaw = yaw + step_size_angle
    if event.char == '6':
        yaw = yaw - step_size_angle
    if event.char == '3':
        yaw = 0.0

    ## section for adjusting the position of the shelf
    step_size_position = 0.01
    if event.char == 'q':
        posx = posx + step_size_position
    if event.char == 'a':
        posx = posx - step_size_position
    if event.char == 'z':
        posx = 0.0

    if event.char == 'w':
        posy = posy + step_size_position
    if event.char == 's':
        posy = posy - step_size_position
    if event.char == 'x':
        posy = 0.0
    
    if event.char == 'e':
        posz = posz + step_size_position
    if event.char == 'd':
        posz = posz - step_size_position
    if event.char == 'c':
        posz = 0.0

    if event.char == '0':
        writeOutputFile(array_shelf_front)
 
    print('pitch, roll, yaw: ', pitch, roll, yaw)
    print('posx , posy, posz: ', posx, posy, posz)

    (position_xyz3, orientation_xyzw3) = calculateComposeTransform(array_shelf_front, array_shelf)
    shelf_pose3 = createROSPose(position_xyz3, orientation_xyzw3)
    shelf_calibrator.moveObjectInPlanningScene('shelf', shelf_pose3)
    


# show the calib file in [X, Y, Z, Roll, Pitch, Yaw] format instead of horrendous quaternions
if __name__=="__main__":
    print("INSTRUCTIONS")
    print("use keypad to change roll, pitch, yaw and letters qaz, wsx, edc to change the position")
    print("the offset values can be increased or decreased by a fixed amount or reseted to 0.0")
    print("after you are done calibrating press 0 to write the output file.")
    print("")

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('shelf_error_calibration_node')

    shelf_calibrator = ShelfErrorCalibration('shelf')
    
    rootDir= rospkg.RosPack().get_path('calibration_data') + '/extrinsics/'
    file_shelf_front = rootDir + 'shelf_front.txt'
    file_shelf = rootDir + 'shelf.txt'

    array_shelf_front = np.loadtxt(file_shelf_front)
    array_shelf = np.loadtxt(file_shelf)

    (position_xyz3, orientation_xyzw3) = calculateComposeTransform(array_shelf_front, array_shelf)
    shelf_pose3 = createROSPose(position_xyz3, orientation_xyzw3)
    shelf_calibrator.moveObjectInPlanningScene('shelf', shelf_pose3)
       
    root = tk.Tk()
    root.geometry('600x600')
    text = tk.Text(root, background='black', foreground='white', font=('Comic Sans MS', 12))
    text.pack()
    root.bind('<KeyPress>', onKeyPress)

    thr = threading.Thread(target=root.mainloop(), args=(), kwargs={})
    #thr.start()
    #root.mainloop()



    rospy.spin()
    
