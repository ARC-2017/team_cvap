#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <json/json.hpp>
#include <std_srvs/Empty.h>
#include <apc_bt_comms/ManagerAction.h>
#include <actionlib/server/simple_action_server.h>

using json = nlohmann::json;

namespace apc
{
const std::string bin_A = "bin_A";
const std::string bin_B = "bin_B";
const std::string bin_C = "bin_C";
const std::string bin_D = "bin_D";
const std::string bin_E = "bin_E";
const std::string bin_F = "bin_F";
const std::string bin_G = "bin_G";
const std::string bin_H = "bin_H";
const std::string bin_I = "bin_I";
const std::string bin_J = "bin_J";
const std::string bin_K = "bin_K";
const std::string bin_L = "bin_L";
const std::string left_arm = "left_arm";
const std::string right_arm = "right_arm";

enum Desperation {level_1 = 1, level_2 = 2, level_3 = 3, level_4 = 4};

class TaskManager
{
public:
  TaskManager(std::string actionlib_name);

  bool parseFile();

private:
  ros::NodeHandle nh_;
  std::string task_;
  std::string current_object_;
  std::string current_bin_;
  std::string current_arm_, suction_arm_, gripper_arm_;
  std::string file_name_;
  actionlib::SimpleActionServer<apc_bt_comms::ManagerAction> action_server_;
  ros::ServiceClient shelf_calibration_client_;
  apc_bt_comms::ManagerResult result_;
  // contains the desserialized JSON file
  json document_root_;

  std::vector<std::map<std::string, std::string>> mission_;
  std::vector<std::map<std::string, std::string>> mission_impossible_;
  std::map<std::string, std::vector<std::string>> bin_contents_;
  std::vector<std::string> tote_contents_;
  std::vector<std::string> object_priorities_;
  std::vector<std::string> grippable_objects_;
  std::vector<std::string> bin_priorities_;
  int bin_cycling_num_;
  Desperation desperation_mode_;

  std::string getNextInMission();
  /*
    Marks bin as impossible in pick task,
    otherwise marks item as impossible in stow task
  */
  void markAsImpossible(std::string name);
  /*
    Marks bin as solved in pick task,
    otherwise marks item as solved in stow task

    Returns true when everything is solved
  */
  bool markAsSolved(std::string name);
  bool isArmBin(std::string arm_name, std::string bin_name);
  bool isObjectInBin(std::string bin_name, std::string object_name);
  bool isBinAcceptable(std::string bin_name);
  double getNumberOfBinItems(std::string bin_name);
  bool objectIsGrippable(std::string object_name);
  std::string getOrderForBin(std::string bin_name);
  std::string getBinWithFewerItems();
  std::string getBinWithFewerItems(std::string arm);
  std::string getOtherBin();
  std::string selectArm(std::string object, std::string bin);

  void setSystemTarget();
  void writeOutputFile();
  void raiseDesperationLevel();
  void callShelfCalibration();
  void swapMission();

  /*
    Rosparam getters. Will print error message.
  */
  template<typename T>
  bool getParam(std::string param_name, T &param);

  template<typename T>
  void setParam(std::string param_name, T param);

  void actionlib_callback(const apc_bt_comms::ManagerGoalConstPtr &goal);
};
}
