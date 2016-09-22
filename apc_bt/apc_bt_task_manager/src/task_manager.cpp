#include <apc_bt_task_manager/task_manager.h>

namespace apc
{
TaskManager::TaskManager(std::string actionlib_name)
  : action_server_(nh_, actionlib_name, boost::bind(&TaskManager::actionlib_callback, this, _1), false)
{
  nh_ = ros::NodeHandle("~");
  bin_cycling_num_ = 0;
  desperation_mode_ = level_1;

  ros::param::del("/apc/task_manager/memory");
  setParam("/apc/task_manager/desperation", level_1);
  shelf_calibration_client_ = nh_.serviceClient<std_srvs::Empty>("/shelf_publisher/calibrate");
  callShelfCalibration();
  action_server_.start();
}

template<typename T>
bool TaskManager::getParam(std::string param_name, T &param)
{
    if (nh_.hasParam(param_name))
    {
      nh_.getParam(param_name, param);
    }
    else
    {
      ROS_ERROR("%s is not set!", param_name.c_str());
      return false;
    }
}

bool TaskManager::parseFile()
{
  if (!getParam("/apc/task_manager/json_filename", file_name_) || !getParam("/apc/task", task_)
      || !getParam("/apc/task_manager/bin_priorities", bin_priorities_)
      || !getParam("/apc/task_manager/object_priorities", object_priorities_))
  {
      return false;
  }

  if (task_ == "stow")
  {
    if(!getParam("/apc/task_manager/suction_arm", suction_arm_)
       || !getParam("/apc/task_manager/gripper_arm", gripper_arm_)
       || !getParam("/apc/task_manager/grippable_objects", grippable_objects_))
    {
      return false;
    }
  }

  std::string package_path = ros::package::getPath("apc_bt_launcher");
  std::string file_path = package_path + "/data/" + file_name_ + ".json";
  std::vector<std::string> mission_objects;

  ROS_INFO("Opening file: %s", file_path.c_str());

  // Deserialize the JSON file
  std::ifstream file;
  file.open(file_path);
  file >> document_root_;
  file.close();

  if (task_ == "pick")
  {
    if (document_root_["bin_contents"].empty() || document_root_["work_order"].empty())
    {
      ROS_ERROR("Work order must have 'bin_contents' and 'work_order' for pick task!");
      return false;
    }

    std::cout << document_root_["work_order"] << std::endl << std::endl;

    for (int i = 0; i < document_root_["work_order"].size(); i++)
    {
      json::iterator it = document_root_["work_order"][i].begin();

      std::string bin = (it++).value();
      std::string item = it.value();

      if (std::find(object_priorities_.begin(), object_priorities_.end(), item) != object_priorities_.end())
      {
          std::map<std::string, std::string> order;
          order.emplace(bin, item);

          ROS_INFO("Adding %s to mission", item.c_str());

          if (getNumberOfBinItems(bin) == 1)
          {
            mission_objects.push_back(item);
          }
          mission_.push_back(order);

      }
    }

    ROS_INFO("Setting the mission list objects");
    setParam("/apc/task_manager/mission_objects", mission_objects);

  }
  else
  {
    ROS_INFO("Parsing input for stowing task");
    if (document_root_["bin_contents"].empty() || document_root_["tote_contents"].empty())
    {
      ROS_ERROR("Work order must have 'bin_contents' and 'tote_contents' for stow task!");
      return false;
    }

    std::cout << std::endl << document_root_["tote_contents"] << std::endl << std::endl;

    for (int i = 0; i < document_root_["tote_contents"].size(); i++)
    {
      std::string bin = document_root_["tote_contents"][i];
      std::string item = document_root_["tote_contents"][i];

      if (std::find(object_priorities_.begin(), object_priorities_.end(), item) != object_priorities_.end())
      {
          std::map<std::string, std::string> order;
          order.emplace(bin, item);

          ROS_INFO("Adding %s to mission", item.c_str());
          mission_.push_back(order);
      }
    }
  }

  std::cout << std::endl << document_root_["bin_contents"] << std::endl << std::endl;

  for (json::iterator it = document_root_["bin_contents"].begin(); it != document_root_["bin_contents"].end(); ++it)
  {
    std::string bin = it.key();
    std::vector<std::string> items = it.value();
    bin_contents_.emplace(bin, items);
  }
  for (int i = 0; i < document_root_["tote_contents"].size(); i++)
  {
    tote_contents_.push_back(document_root_["tote_contents"][i]);
  }

  return true;
}

bool TaskManager::markAsSolved(std::string name)
{
  if (task_ == "pick")
  {
    for (int i = 0; i < bin_contents_[current_bin_].size(); i++)
    {
      if (bin_contents_[current_bin_][i] == current_object_)
      {
        bin_contents_[current_bin_].erase(bin_contents_[current_bin_].begin() + i);
        break;
      }
    }

    if (!current_object_.empty())
    {
      tote_contents_.push_back(current_object_);
    }

    for (int i = 0; i < mission_.size(); i++)
    {
      if (mission_[i].begin()->first == name)
      {
        mission_.erase(mission_.begin() + i);

        if (mission_.empty())
        {
          if (!mission_impossible_.empty())
          {
            swapMission();
          }
          else
          {
            ROS_INFO("Successfully finished mission! wow");
            return true;
          }
        }

        current_object_.clear();
        current_bin_.clear();
        return false;
      }
    }
  }
  else
  {
    for (int i = 0; i < tote_contents_.size(); i++)
    {
      if (tote_contents_[i] == current_object_)
      {
        tote_contents_.erase(tote_contents_.begin() + i);
        break;
      }
    }

    if (!current_object_.empty() && !current_bin_.empty())
    {
      bin_contents_[current_bin_].push_back(current_object_);
    }

    for (int i = 0; i < mission_.size(); i++)
    {
      if (mission_[i].begin()->second == name)
      {
        mission_.erase(mission_.begin() + i);

        if (mission_.empty())
        {
          if (!mission_impossible_.empty())
          {
            swapMission();
          }
          else
          {
            ROS_INFO("Successfully finished mission! wow");
            return true;
          }
        }

        current_object_.clear();
        current_bin_.clear();
        return false;
      }
    }
  }

  ROS_ERROR("Asked to mark %s as solved, but could not find it in the mission", name.c_str());
}

void TaskManager::swapMission()
{
  ROS_INFO("No elements left in the mission! Will swap to mission impossible");
  ROS_WARN("This increases our desperation level!");
  raiseDesperationLevel();
  mission_ = mission_impossible_;
  mission_impossible_.clear();
}

std::string TaskManager::getOrderForBin(std::string bin_name)
{
  for (int i = 0; i < mission_.size(); i++)
  {
    if (mission_[i].begin()->first == bin_name)
    {
      return mission_[i].begin()->second;
    }
  }

  ROS_ERROR("Tried to get order for bin %s and could not find anything", bin_name.c_str());
}

double TaskManager::getNumberOfBinItems(std::string bin_name)
{
  for (std::map<std::string, std::vector<std::string>>::iterator it = bin_contents_.begin(); it != bin_contents_.end();
       it++)
  {
    if(it->first == bin_name)
    {
      return it->second.size();
    }
  }

  return -1;
}

std::string TaskManager::getBinWithFewerItems()
{
  int min_num = 100;
  std::string min_key;

  ROS_INFO("Getting bin with fewer items");

  for (std::map<std::string, std::vector<std::string>>::iterator it = bin_contents_.begin(); it != bin_contents_.end();
       it++)
  {
    if (it->second.size() < min_num)
    {
      min_num = it->second.size();
      min_key = it->first;
    }
  }

  ROS_INFO("Bin with fewer items is: %s", min_key.c_str());

  return min_key;
}

bool TaskManager::isObjectInBin(std::string bin_name, std::string object_name)
{
  std::vector<std::string> this_bin_contents;

  this_bin_contents = bin_contents_[bin_name];

  for (int i = 0; i < this_bin_contents.size(); i++)
  {
    if (this_bin_contents[i] == object_name)
    {
      return true;
    }
  }

  return false;
}

bool TaskManager::isBinAcceptable(std::string bin_name)
{
  if (current_object_ == "kleenex_tissue_box")
  {
    if (bin_name != bin_A && bin_name != bin_B && bin_name != bin_C)
    {
      return false;
    }
  }

  if (current_object_ == "cherokee_easy_tee_shirt")
  {
    if (bin_name != bin_J && bin_name != bin_K && bin_name != bin_L)
    {
      return false;
    }
  }

  if (isObjectInBin(bin_name, "kleenex_paper_towels") || isObjectInBin(bin_name, "hanes_tube_socks"))
  {
    return false;
  }

  return true;
}

std::string TaskManager::getBinWithFewerItems(std::string arm)
{
  int min_num = 100;
  std::string min_key = bin_E;

  ROS_INFO("Getting bin with fewer items");

  for (std::map<std::string, std::vector<std::string>>::iterator it = bin_contents_.begin(); it != bin_contents_.end();
       it++)
  {
    if (it->second.size() < min_num && isArmBin(arm, it->first) && isBinAcceptable(it->first))
    {
      min_num = it->second.size();
      min_key = it->first;
    }
  }

  ROS_INFO("Bin with fewer items is: %s", min_key.c_str());

  return min_key;
}

bool TaskManager::isArmBin(std::string arm_name, std::string bin_name)
{
  if (arm_name == left_arm)
  {
    if (bin_name != bin_C && bin_name != bin_F && bin_name != bin_I && bin_name != bin_L)
    {
      return true;
    }
  }
  else
  {
    if (arm_name == right_arm)
    {
      if (bin_name != bin_A && bin_name != bin_D && bin_name != bin_G && bin_name != bin_J)
      {
        return true;
      }
    }
  }

  return false;
}

void TaskManager::writeOutputFile()
{
  std::string package_path = ros::package::getPath("apc_bt_launcher");
  std::string file_path = package_path + "/data/" + file_name_ + "_output.json";

  std::ofstream file(file_path);

  if (file.is_open())
  {
    json out;

    out["bin_contents"] = bin_contents_;
    out["tote_contents"] = tote_contents_;
    file << out.dump(4);
    file.close();
  }
  else
  {
    ROS_ERROR("Error writing output file!!!");
  }
}

void TaskManager::actionlib_callback(const apc_bt_comms::ManagerGoalConstPtr &goal)
{
  ROS_INFO("BT Task Manager got a request!");
  sleep(0.1);

  if (goal->success)
  {
    if (task_ == "pick")
    {
      ROS_INFO("Will mark bin as solved!");
      if (markAsSolved(current_bin_))
      {
        nh_.setParam("/apc/task_manager/running", false);
      }
    }
    else
    {
      ROS_INFO("Will mark object as solved!");
      if (markAsSolved(current_object_))
      {
        nh_.setParam("/apc/task_manager/running", false);
      }
    }
  }
  else
  {
    if (goal->pop)
    {
      if (task_ == "pick")
      {
        if (!current_bin_.empty())
        {
          ROS_INFO("Current bin is not empty! Will mark as impossible");
          markAsImpossible(current_bin_);
          current_bin_.clear();
          current_object_.clear();
        }
        ROS_INFO("Will pop next mission bin!");
        current_bin_ = getNextInMission();
        current_object_ = getOrderForBin(current_bin_);
        current_arm_ = selectArm(current_object_, current_bin_);
      }
      else
      {
        if (!current_object_.empty())
        {
          ROS_INFO("Current object is not empty! Will mark as impossible");
          markAsImpossible(current_object_);
          current_bin_.clear();
          current_object_.clear();
        }
        ROS_INFO("Will pop next mission object!");
        current_object_ = getNextInMission();
        current_arm_ = selectArm(current_object_, current_bin_);
        current_bin_ = getBinWithFewerItems(current_arm_);
      }
    }
    else
    {
        if (task_ == "stow") // Failed to place an object
        {
            setParam("/apc/task_manager/place_failure", true);
            current_bin_ = getOtherBin();
        }
    }
  }

  setSystemTarget();
  writeOutputFile();
  result_.result = true;
  action_server_.setSucceeded(result_);
}

template<typename T>
void TaskManager::setParam(std::string param_name, T param)
{
    ros::param::set(param_name, param);
}

std::string TaskManager::getOtherBin()
{
    std::string bin;
    ROS_INFO("Task manager getting another bin");
    do
    {
      ROS_INFO("Current bin num: %d", bin_cycling_num_);
      bin = bin_priorities_[bin_cycling_num_];
      bin_cycling_num_++;

      if (bin_cycling_num_ >= bin_priorities_.size())
      {
          ROS_INFO("Bin num %d is bigger than bin priorities size %d", bin_cycling_num_, (int) bin_priorities_.size());
          bin_cycling_num_ = 0;
      }
    }while (!isArmBin(current_arm_, bin) && bin != current_bin_);

    return bin;
}

void TaskManager::setSystemTarget()
{
  ROS_INFO("Setting system target to:\nObject: %s\nBin: %s\nArm: %s", current_object_.c_str(), current_bin_.c_str(),
           current_arm_.c_str());
  nh_.setParam("/apc/task_manager/target_object", current_object_);
  nh_.setParam("/apc/task_manager/target_bin", current_bin_);
  nh_.setParam("/apc/task_manager/arm", current_arm_);

  std::vector<std::string> non_target_objects;

  if (task_ == "pick")
  {
      non_target_objects = bin_contents_[current_bin_];
  }
  else
  {
      non_target_objects = tote_contents_;
  }

  int target_index = -1;
  for (int i = 0; i < non_target_objects.size(); i++)
  {
      if (non_target_objects[i] == current_object_)
      {
          target_index = i;
          break;
      }
  }

  if (target_index >= 0)
  {
      non_target_objects.erase(non_target_objects.begin() + target_index);
  }

  nh_.setParam("/apc/task_manager/non_target_bin_items", non_target_objects);
}

std::string TaskManager::selectArm(std::string object, std::string bin)
{
  if (task_ == "pick")
  {
    if (bin == bin_A || bin == bin_D || bin == bin_G || bin == bin_J)
    {
      return left_arm;
    }

    return right_arm;
  }
  else
  {
    if (objectIsGrippable(object))
    {
      return gripper_arm_;
    }
    return suction_arm_;
  }
}

bool TaskManager::objectIsGrippable(std::string object_name)
{
  for (int i = 0; i < grippable_objects_.size(); i++)
  {
      if (grippable_objects_[i] == object_name)
      {
          return true;
      }
  }

  return false;
}

void TaskManager::markAsImpossible(std::string name)
{
  if (task_ == "pick")
  {
    if (desperation_mode_ != level_1)
    {
      for (int i = 0; i < bin_priorities_.size(); i++)
      {
          if (bin_priorities_[i] == name)
          {
              bin_priorities_.erase(bin_priorities_.begin() + i);
              break;
          }
      }
    }
    for (int i = 0; i < mission_.size(); i++)
    {
      if (mission_[i].begin()->first == name)
      {
        mission_impossible_.push_back(mission_[i]);
        mission_.erase(mission_.begin() + i);
        return;
      }
    }
  }
  else
  {
    for (int i = 0; i < mission_.size(); i++)
    {
      if (mission_[i].begin()->second == name)
      {
        mission_impossible_.push_back(mission_[i]);
        mission_.erase(mission_.begin() + i);
        return;
      }
    }
  }

  ROS_ERROR("Tried to mark %s as impossible, but it was not in the mission!", name.c_str());
}

std::string TaskManager::getNextInMission()
{
  std::string next;

  if (mission_.empty())
  {
    swapMission();
  }

  if (task_ == "pick")
  {
    // 1 - See if there is a mission item on an easy bin
    // for (int i = 0; i < mission_.size(); i++)
    // {
    //     if (std::find(bin_priorities_.begin(), bin_priorities_.end(), mission_[i].begin()->first) != bin_priorities_.end())
    //     {
    //         return mission_[i].begin()->first;
    //     }
    // }

    ROS_INFO("Task manager could not find an item on an easy bin!");

    // 2 - If not, get the easiest mission item possible
    for (int i = 0; i < object_priorities_.size(); i++)
    {
        for (int j = 0; j < mission_.size(); j++)
        {
            if (mission_[j].begin()->second == object_priorities_[i])
            {
                return mission_[j].begin()->first;
            }
        }
    }

    // We shouldn't reach this point, just here to catch stupid bugs
    ROS_WARN("Task manager could not find a mission item in the object priorities list! Resorting to default behavior.");
    return mission_[0].begin()->first;
  }
  else
  {
    // 1 - Get the easiest mission item possible
    for (int i = 0; i < object_priorities_.size(); i++)
    {
        for (int j = 0; j < mission_.size(); j++)
        {
            if (mission_[j].begin()->second == object_priorities_[i])
            {
                return mission_[j].begin()->second;
            }
        }
      }

    // We shouldn't reach this point, just here to catch stupid bugs
    ROS_WARN("Task manager could not find a mission item in the object priorities list! Resorting to default behavior.");
    return mission_[0].begin()->second;
  }

  return next;
}

void TaskManager::raiseDesperationLevel()
{
    if (desperation_mode_ == level_1)
    {
        desperation_mode_ = level_2;
    }
    else
    {
      if (desperation_mode_ == level_2 && task_ != "pick")
      {
        desperation_mode_ = level_3;
      }
      else
      {
        if (desperation_mode_ == level_3 && task_ != "pick")
        {
          desperation_mode_ = level_4;
        }
        else
        {
          desperation_mode_ = level_1;
        }
      }
    }

    setParam("/apc/task_manager/desperation", desperation_mode_);
}

void TaskManager::callShelfCalibration()
{
  std_srvs::Empty srv;

  ros::service::waitForService("/shelf_publisher/calibrate");
  shelf_calibration_client_.call(srv);
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "task_manager");
  apc::TaskManager parser("/apc/task_manager");

  parser.parseFile();
  ros::spin();
  return -1;
}
