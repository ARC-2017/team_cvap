#include <amazon_challenge_bt_core/BehaviorTree.h>

using namespace BT;

BehaviorTree::BehaviorTree(std::string Name)
{
    // Name initialization
    this->Name = Name;

    // Flag initialization
    Ticked = false;

    // Tick period initialization
    TickPeriod = 1000;

    // RootNode initialization
    RootNode = NULL;
}

BehaviorTree::~BehaviorTree()
{
    // Deallocating all the nodes
    for (std::map<std::string, TreeNode*>::iterator Pair=NodeMap.begin(); Pair!=NodeMap.end(); ++Pair)
    {
        Pair->second->WriteState(Exit);
        Pair->second->Semaphore.Signal();
        Pair->second->Thread.join();

        delete Pair->second;
    }

    // Deallocating all the subtrees
    for (std::map<std::string, BehaviorTree*>::iterator Pair=SubTreeMap.begin(); Pair!=SubTreeMap.end(); ++Pair)
    {
        delete Pair->second;
    }
}

std::string& BehaviorTree::CreateNewControlNode(std::string NodeName, ControlNodeType Type)
{
    try
    {
        // If the NodeName is "free", an exception will be thrown
        NodeMap.at(NodeName);

        // The NodeName is already present in the map
        throw BehaviorTreeException("'" + NodeName + "'" + " is already being used for another node.");
    }
    catch (std::out_of_range&)
    {
        try
        {
            switch (Type)
            {
                case Parallel:
                {
                    NodeMap[NodeName] = new ParallelNode(NodeName);
                    break;
                }
                case Selector:
                {
                    NodeMap[NodeName] = new SelectorNode(NodeName);
                    break;
                }
                case Sequence:
                {
                    NodeMap[NodeName] = new SequenceNode(NodeName);
                    break;
                }
                default:
                {
                    break;
                }
            }
        }
        catch (std::bad_alloc&)
        {
            throw BehaviorTreeException("A bac_alloc error occured while allocating memory for '" + NodeName + "'");
        }

        // Insertion succeded
        return NodeMap[NodeName]->Name;
    }
}

std::string& BehaviorTree::CreateNewActionNode(std::string NodeName, ActionNodeType Type)
{
    try
    {
        // If the NodeName is "free", an exception will be thrown
        NodeMap.at(NodeName);

        // The NodeName is already present in the map
        throw BehaviorTreeException("'" + NodeName + "'" + " is already being used for another node.");
    }
    catch (std::out_of_range&)
    {
        try
        {
            switch (Type)
            {
                case ActionTest:
                {
                    NodeMap[NodeName] = new ActionTestNode(NodeName);
                    break;
                }
                default:
                {
                    break;
                }
            }
        }
        catch (std::bad_alloc&)
        {
            throw BehaviorTreeException("A bac_alloc error occured while allocating memory for '" + NodeName + "'");
        }

        // Insertion succeded
        return NodeMap[NodeName]->Name;
    }
}

std::string& BehaviorTree::CreateNewConditionNode(std::string NodeName, ConditionNodeType Type)
{
    try
    {
        // If the NodeName is "free", an exception will be thrown
        NodeMap.at(NodeName);

        // The NodeName is already present in the map
        throw BehaviorTreeException("'" + NodeName + "'" + " is already being used for another node.");
    }
    catch (std::out_of_range&)
    {
        try
        {
            switch (Type)
            {
                case ConditionTest:
                {
                    NodeMap[NodeName] = new ConditionTestNode(NodeName);
                    break;
                }
                default:
                {
                    break;
                }
            }
        }
        catch (std::bad_alloc&)
        {
            throw BehaviorTreeException("A bac_alloc error occured while allocating memory for '" + NodeName + "'");
        }

        // Insertion succeded
        return NodeMap[NodeName]->Name;
    }
}

BehaviorTree& BehaviorTree::CreateNewSubTree(std::string TreeName)
{
    try
    {
        // If the TreeName is "free", an exception will be thrown
        SubTreeMap.at(TreeName);

        // The TreeName is already present in the map
        throw BehaviorTreeException("'" + TreeName + "'" + " is already being used for another subtree.");
    }
    catch (std::out_of_range&)
    {
        try
        {
            SubTreeMap[TreeName] = new BehaviorTree(TreeName);

            return *SubTreeMap[TreeName];
        }
        catch (std::bad_alloc&)
        {
            throw BehaviorTreeException("A bac_alloc error occured while allocating memory for '" + TreeName + "'");
        }
    }
}

std::string& BehaviorTree::SetRootNode(std::string NodeName)
{
    try
    {
        TreeNode* OldRootNode = RootNode;
        RootNode = NodeMap.at(NodeName);

        return OldRootNode->Name;
    }
    catch (std::out_of_range&)
    {
        throw BehaviorTreeException("'" + NodeName + "' can't be found in the node list.");
    }
}

std::pair<std::string&, std::string&> BehaviorTree::SetFatherChildRelationship(std::string FatherNodeName, std::string ChildName)
{
    ControlNode* FatherNode;

    try
    {
        NodeMap.at(FatherNodeName);

        FatherNode = dynamic_cast<ControlNode*> (NodeMap.at(FatherNodeName));

        if (FatherNode == 0)
        {
            throw BehaviorTreeException("'" + FatherNodeName + "' is not a control node, and therefore it can't have children.");
        }
    }
    catch (std::out_of_range&)
    {
        throw BehaviorTreeException("'" + FatherNodeName + "' can't be found in the node list.");
    }

    try
    {
        NodeMap.at(ChildName);

        if (FatherNodeName == ChildName)
        {
            throw BehaviorTreeException("'" + FatherNodeName + "' can not be father of itself.");
        }

        FatherNode->AddChild(NodeMap.at(ChildName));
    }
    catch (std::out_of_range&)
    {
        try
        {
            SubTreeMap.at(ChildName);

            if (SubTreeMap.at(ChildName)->RootNode == NULL)
            {
                throw BehaviorTreeException("No root node has been defined for '" + ChildName + "' yet, and therefore it can't be added as a child.");
            }

            FatherNode->AddChild(SubTreeMap.at(ChildName)->RootNode);
        }
        catch (std::out_of_range&)
        {
            throw BehaviorTreeException("'" + ChildName + "' can't be found in the node/subtree list.");
        }
    }

    return std::pair<std::string&, std::string&>(FatherNodeName, ChildName);
}

void BehaviorTree::SetParallelNodeThreshold(std::string NodeName, unsigned int N)
{
    ParallelNode* FatherNode;

    try
    {
        NodeMap.at(NodeName);

        FatherNode = dynamic_cast<ParallelNode*> (NodeMap.at(NodeName));

        if (FatherNode == 0)
        {
            throw BehaviorTreeException("'" + NodeName + "' is not a parallel node, and therefore it can't have an N threashold.");
        }
    }
    catch (std::out_of_range&)
    {
        throw BehaviorTreeException("'" + NodeName + "' can't be found in the node list.");
    }

    FatherNode->SetThreashold(N);
}

void BehaviorTree::Tick()
{
    if (RootNode == NULL)
    {
        throw BehaviorTreeException("No root node has been defined for '" + Name + "' yet, and therefore it can't be ticked.");
    }

    Ticked = true;
    RootNode->Semaphore.Signal();
}

NodeState BehaviorTree::GetTreeState()
{
    if (Ticked = false)
    {
        throw BehaviorTreeException("No tick has been sent to '" + Name + "' yet, and therefore no state may be retriven.");
    }

    Ticked = false;
    return RootNode->GetNodeState();
}
