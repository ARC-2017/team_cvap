#ifndef APC_GRASPING_SHELF_POSE_OPTIMIZER_H
#define APC_GRASPING_SHELF_POSE_OPTIMIZER_H
// STL includes
#include <string>
#include <vector>
// Moveit includes
#include "moveit/planning_scene/planning_scene.h"
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/collision_detection/collision_common.h"
// Eigen includes
#include <Eigen/Geometry>
// ROS includes
#include "ros/ros.h"
// geometric_shapes
#include "geometric_shapes/shape_messages.h"


#define SIDE_COL_WIDTH 0.275
#define CENTER_COL_WIDTH 0.305
#define BOTTOM_ROW_HEIGHT 0.265
#define MIDDLE_ROW_HEIGHT 0.23
#define TOP_ROW_HEIGHT 0.265
#define BIN_DEPTH 0.4
#define NUM_COLUMNS 3
#define NUM_ROWS 4

namespace moveit {
  namespace core {
    class JointModelGroup;
    class RobotState;
  }
}

namespace apc_grasping {
    namespace shelf_pose {
        struct PoseRange {
          float xmin;
          float ymin;
          float zmin;
          float rxmin;
          float rymin;
          float rzmin;
          float xmax;
          float ymax;
          float zmax;
          float rxmax;
          float rymax;
          float rzmax;
          PoseRange()
              : xmin(0.0), ymin(0.0), zmin(0.0), rxmin(0.0), rymin(0.0), rzmin(0.0) 
              , xmax(0.0), ymax(0.0), zmax(0.0),rxmax(0.0), rymax(0.0), rzmax(0.0) {}
        };

        struct Arguments {
          bool paramsFromServer;
          std::string paramName;
          std::string urdf;
          std::string srdf;
          std::string shelfMeshPath;
          PoseRange shelfPoseRange;
          PoseRange eefPoseRange;
          unsigned int numSamplesBinX;
          unsigned int numSamplesBinY;
          unsigned int numSamplesBinZ;
          unsigned int maxXSamples;
          unsigned int maxYSamples;
          unsigned int maxRotSamples;
          unsigned int numEEFWidthSamples;
          unsigned int numEEFHeightSamples;
          unsigned int numEEFDepthSamples;
          unsigned int numEEFRotSamples;
        };

        struct StateValidatorArgs {
          StateValidatorArgs(planning_scene::PlanningScenePtr ps, std::string name) {
            _armGroupName = name;
            _planningScene = ps;
          }
          std::string _armGroupName;
          planning_scene::PlanningScenePtr _planningScene;
        };

        class ShelfPoseOptimizer {
        public:
          ShelfPoseOptimizer();
          ~ShelfPoseOptimizer();
          bool initialize(const Arguments& args);
          float evaluateShelfPose(float x, float y, float theta, bool publishInitialScene=false, bool publishRobotStates=false);
          Eigen::Affine3d run();
          void setScenePublisher(ros::Publisher& publisher);
          void setScenePublisher(std::string name);
          void setPosesPublisher(ros::Publisher& publisher);
          void setPosesPublisher(std::string name);
        protected:
          float evaluateBasePose(const Eigen::Affine3d& shelfPose, bool publishInitialScene=false, bool publishRobotStates=false);
          bool checkPoseForIk(const std::string& armGroupName, const Eigen::Affine3d& globalPose,
                              unsigned int& numIksFound, unsigned int& numReachable);
          void computeSamplePoses(EigenSTL::vector_Affine3d& poses);
          void computeBinPoses(const Eigen::Affine3d& basePose, const std::vector<float>& columnWidths,
                               const std::vector<float>& rowHeights, std::vector< EigenSTL::vector_Affine3d >& binPoses);
          void computePosesWithinBin(const Eigen::Affine3d& binPose, EigenSTL::vector_Affine3d& poses, 
                                     float binWidth, float binHeight, float binDepth,
                                     float rotMin=-0.78, float rotMax=0.78,
                                     float padding=0.0, unsigned int numWidthSamples=3, 
                                     unsigned int numHeightSamples=3, unsigned int numDepthSamples=3,
                                     unsigned int numRotSamples=3);

        private:
          robot_model::RobotModelPtr _robotModel;
          planning_scene::PlanningScenePtr _planningScene;
          robot_model_loader::RobotModelLoaderPtr _robotLoader;
          ros::Publisher _scenePublisher;
          ros::Publisher _posesPublisher;
          bool _initialized;
          bool _scenePublisherAvailable;
          bool _posesPublisherAvailable;
          bool _firstPosePublishing;
          Arguments _args;
          std::vector<double> _armRestingConfigurationLeft;
          std::vector<double> _armRestingConfigurationRight;
          std::string _groupNameLeft;
          std::string _groupNameRight;
          EigenSTL::vector_Affine3d _eefSamplePoses;
          Eigen::Affine3d _shelfBasePose;

          void publishPoses();
          void publishScene();
          bool loadMesh(std::string filePath, shapes::ShapeMsg& meshMessage);

        };
    }
}
        // <joint name="left_e0" value="0" />
        // <joint name="left_e1" value="0.75" />
        // <joint name="left_s0" value="0" />
        // <joint name="left_s1" value="-0.55" />
        // <joint name="left_w0" value="0" />
        // <joint name="left_w1" value="1.26" />
        // <joint name="left_w2" value="0" />

        // <joint name="left_s0" />
        // <joint name="left_s1" />
        // <joint name="left_e0" />
        // <joint name="left_e1" />
        // <joint name="left_w0" />
        // <joint name="left_w1" />
        // <joint name="left_w2" />
        // <joint name="left_hand" />
        // <joint name="left_gripper_base" />
        // <joint name="left_endpoint" />
#endif