#ifndef BAXTER_GRIPPER_H_
#define BAXTER_GRIPPER_H_
#include <memory>
#include <string>

namespace apc {
class BaxterGripper {
public:
  BaxterGripper(const std::string &arm);
  ~BaxterGripper();
  /**
    Calibrate the gripper.
  */
  void calibrate();
  /**
    Close the gripper.
  */
  void close();
  /**
    Open the gripper.
  */
  void open();
};
typedef std::shared_ptr<BaxterGripper> BaxterGripperPtr;
}
#endif
