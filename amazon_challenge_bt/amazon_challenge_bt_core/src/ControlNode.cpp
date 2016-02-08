#include <amazon_challenge_bt_core/ControlNode.h>

using namespace BT;

ControlNode::ControlNode(std::string Name) : TreeNode::TreeNode(Name)
{
    Type = Control;
}

ControlNode::~ControlNode() {}

void ControlNode::AddChild(TreeNode* Child)
{
    // Checking if the Child is not already present
    for (int i=0; i<ChildNodes.size(); i++)
    {
        if (ChildNodes[i] == Child)
        {
            throw BehaviorTreeException("'" + Child->Name + "' is already a '" + Name + "' child.");
        }
    }

    ChildNodes.push_back(Child);
    ChildStates.push_back(Idle);
}

unsigned int ControlNode::GetChildrenNumber()
{
    return ChildNodes.size();
}

bool ControlNode::Halt()
{
    // Lock acquistion
    boost::lock_guard<boost::mutex> LockGuard(StateMutex);

    State = Halted;
    return true;
}

bool ControlNode::WriteState(NodeState StateToBeSet)
{
    // Lock acquistion
    boost::lock_guard<boost::mutex> LockGuard(StateMutex);

    State = StateToBeSet;
    return true;
}


std::vector<TreeNode*> ControlNode::GetChildren()
{
    return ChildNodes;
}


void ControlNode::ResetColorState()
{

    SetColorState(Idle);
    for(int i = 0; i < ChildNodes.size(); i++)
    {
        ChildNodes[i]->ResetColorState();
    }
}
