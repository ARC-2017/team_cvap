#ifndef MOVEIT_INTERFACE_H_
#define MOVEIT_INTERFACE_H_
/****************************************************************************
    This class provides synchronized access to the Robot.

    @author: Joshua Haustein (haustein@kth.se)
 *****************************************************************************/

// TODO: import tf
// TODO: import rospy
// TODO moveit_msgs.msg import CollisionObject, AttachedCollisionObject

// std includes
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <mutex>
// MoveIt! includes
#include "moveit/move_group_interface/move_group.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
// tf
#include "tf/transform_listener.h"
// Eigen
#include <Eigen/Geometry>
// apc
#include "apc_manipulation/BaxterGripper.h"
// ROS
#include "geometry_msgs/PoseStamped.h"
#include "geometric_shapes/shape_messages.h"

namespace apc {
typedef std::map<std::string, double> Configuration;
typedef std::shared_ptr<moveit::planning_interface::MoveGroup> MoveGroupPtr;
enum MoveResult { PlanFail, ExecuteFail, Success };

class RobotInterface {
  /**
     Interface to MoveIt. Get the lock before doing anything and
     release it as soon as you don't use it anymore! You can not execute
     any of the method without acquiring the lock. This prevents multiple
     threads from sending commands to MoveIt.
   **/
public:
  RobotInterface();
  ~RobotInterface();
  /**
     Acquire this lock before doing anything with the robot!
     @return true iff lock acquired.
   */
  bool acquireLock();
  /**
     Release this lock as soon as other threads are allowed to operate the
     robot! ONLY CALL THIS IF THE CALLING THREAD HAS ACQUIRED THE LOCK BEFORE!
   */
  void releaseLock();
  /**
     Reset the planning scene. Blocks until either the scene is successfully
     reset or
     the ros node is killed.
     NOTE: Acquire the lock before calling this function!
     @return True iff the scene is successfully reset.
   */
  bool resetPlanningScene();

  /**
     Adds the given set of objects to the planning scene.
     If any of the objects is already in the planning scene, its
     pose is updated to the most recent one.

     @param objectNames - a list of object names to add.
   */
  void addObjectsToPlanningScene(const std::vector<std::string> &objectNames);

  /**
     Attach the specified object to the given arm.
     @param objectName - name of the object to attach
     @param armName - name of the arm (either left_arm or right_arm)
   */
  void attachObject(const std::string &objectName, const std::string &armName);

  /** Releases the specified object. **/
  void releaseObject(const std::string &objectName);

  /** Get a named configuration for the given arm. **/
  Configuration getNamedConfiguration(const std::string &name,
                                      const std::string &arm) const;

  /**
     Removes the given objects from the planning scene.
     @param objectNames: a list of object names to remove.
   **/
  void
  removeObjectsFromPlanningScene(const std::vector<std::string> &objectNames);

  /**
     Move the end effector of the given arm to the given position.
     @param position - end effector position in global frame
     @param arm - name of the arm to move
     @param execute - whether to execute the trajectory or not
     @param plan - if a path is found, it is stored in here
     @return MoveResult
   */
  MoveResult moveToPosition(const Eigen::Vector3d &position,
                            const std::string &arm,
                            moveit::planning_interface::MoveGroup::Plan &plan,
                            bool execute = true);

  /**
     Move the end effector of the given arm to the given position.
     @param position - end effector position
     @param arm - name of the arm to move
     @param execute - whether to execute the trajectory or not
     @param plan - if a path is found, it is stored in here
     @return MoveResult
   */
  MoveResult moveToPosition(const geometry_msgs::PointStamped &position,
                            const std::string &arm,
                            moveit::planning_interface::MoveGroup::Plan &plan,
                            bool execute = true);

  /**
     Plans/moves the specified arm to the given pose.
     @param pose - goal pose in global frame.
     @param arm - either left_arm or right_arm
     @param execute - if true, try to execute, otherwise just plan
     @param plan - if a path is found, it is stored in here
     @return MoveResult
   */
  MoveResult moveToPose(const Eigen::Affine3d &pose, const std::string &arm,
                        moveit::planning_interface::MoveGroup::Plan &plan,
                        bool execute = true);

  /**
     Plans/moves the specified arm to the given pose.
     @param pose - goal pose of type PoseStamped. Can be defined in any known
     frame.
     @param arm - either left_arm or right_arm
     @param execute - if true, try to execute, otherwise just plan
     @param plan - if a path is found, it is stored in here
     @return MoveResult
   */
  MoveResult moveToPose(const geometry_msgs::PoseStamped &pose,
                        const std::string &arm,
                        moveit::planning_interface::MoveGroup::Plan &plan,
                        bool execute = true);

  /**
     Plans/moves the specified arm to the given configuration.
     @param config - configuration to move to (dictionary name -> value).
     @param arm - either left_arm or right_arm
     @param execute - if true, try to execute, otherwise just plan
     @return (feedback, trajectory), where feedback is one of the values in
     MoveResult
     and trajectory is the MoveIt trajectory.
   */
  MoveResult
  moveToConfiguration(const Configuration &config, const std::string &arm,
                      moveit::planning_interface::MoveGroup::Plan &plan,
                      bool execute = true);

  /**
     Moves the head aside such that the given arm can move freely.
    */
  void moveHeadAside(const std::string &arm);

  // TODO: type
  /**
     Returns the specified MoveGroup.
     @param name - name of the move group (left_arm or right_arm)
     @param clearGroup - whether to clear all set targets.
   */
  MoveGroupPtr getMoveGroup(const std::string &name, bool clearGroup = true);
  BaxterGripperPtr getGripper(const std::string &name);
  std::string getGripperName(const std::string &armName) const;
  std::string getGripperFrameName(const std::string &armName) const;

  /**
     The frame of objectName is assumed to be $objectName_final.
   */
  std::string getObjectFrameName(const std::string &objectName) const;

  /**
     Returns the transform of the given object.
     This is a convenience function that calls self.getTransform.
   */
  bool getObjectPose(const std::string &objectName,
                     geometry_msgs::PoseStamped &outPose,
                     std::string base_frame = "/base") const;

  /**
     Returns the pose of frame_id in the base_frame, if known.
   */
  bool getPose(const std::string &frame_id, geometry_msgs::PoseStamped &outPose,
               std::string base_frame = "/base") const;

  /**
     Returns the transform from frame_id to base_frame, if known.
   */
  Eigen::Affine3d getEigenTransform(const std::string &frame_id,
                                    std::string base_frame = "/base") const;

  /**
    Transforms the given pose to the given target frame.
  */
  bool transformPose(const geometry_msgs::PoseStamped &inPose,
                     geometry_msgs::PoseStamped &outPose,
                     const std::string &targetFrame) const;

  /**
    Transforms the given pose to the given target frame.
  */
  Eigen::Affine3d transformEigenPose(const Eigen::Affine3d &pose,
                                     const std::string &sourceFrame,
                                     const std::string &targetFrame) const;

  bool addObject(const std::string &objectName,
                 const geometry_msgs::PoseStamped &objectPose);
  void moveObject(const std::string &objectName,
                  const geometry_msgs::PoseStamped &objectPose);

private:
  // TODO
  mutable std::recursive_mutex _mutex;
  MoveGroupPtr _leftArmGroup;
  MoveGroupPtr _rightArmGroup;
  BaxterGripperPtr _leftGripper;
  BaxterGripperPtr _rightGripper;
  moveit::planning_interface::PlanningSceneInterface _planningSceneInterface;
  ros::NodeHandle _nodeHandle;
  tf::TransformListener _tfListener;

  // self._robotCommander = moveit_commander.RobotCommander()
  // self._moveitScene = moveit_commander.PlanningSceneInterface()
  // self._leftArmGroup = moveit_commander.MoveGroupCommander("left_arm")
  // self._rightArmGroup = moveit_commander.MoveGroupCommander("right_arm")
  // self._co_publisher = rospy.Publisher('/collision_object', CollisionObject,
  // queue_size=1)
  // self._co_attach_publisher = rospy.Publisher('/attached_collision_object',
  // AttachedCollisionObject, queue_size=1)
  // self._objectsInScene = []
  // self._lock = threading.RLock()
  // self._tfListener = tf.TransformListener()
  // self._leftGripper = baxter_interface.Gripper('left')
  // self._rightGripper = baxter_interface.Gripper('right')
  // self._head = baxter_interface.Head()
  std::string getMeshPath(const std::string &objName) const;
  bool loadMesh(const std::string &filePath,
                shapes::ShapeMsg &meshMessage) const;
};
}

#endif
