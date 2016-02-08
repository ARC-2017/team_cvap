#ifndef BEHAVIORTREE_H
#define BEHAVIORTREE_H
#include <amazon_challenge_bt_core/Draw.h>
#include <amazon_challenge_bt_core/ParallelNode.h>
#include <amazon_challenge_bt_core/SelectorNode.h>
#include <amazon_challenge_bt_core/SequenceNode.h>

#include <amazon_challenge_bt_core/SelectorStarNode.h>
#include <amazon_challenge_bt_core/SequenceStarNode.h>
#include <amazon_challenge_bt_core/DecoratorRetryNode.h>

#include <amazon_challenge_bt_core/Actions/ActionTestNode.h>
#include <amazon_challenge_bt_core/Conditions/ConditionTestNode.h>
#include <amazon_challenge_bt_core/Actions/ROSAction.h>
#include <amazon_challenge_bt_core/Conditions/ROSCondition.h>

#include <amazon_challenge_bt_core/Conditions/BumpersOK.h>


#include <amazon_challenge_bt_core/Exceptions.h>



#include <string>
#include <map>

namespace BT
{
    // Enumerates the possible types of a control node
    enum ControlNodeType {Parallel, Selector, Sequence};

    // Enumerates the possible types of an action node
    enum ActionNodeType {ActionTest};

    // Enumerates the possible types of a condition node
    enum ConditionNodeType {ConditionTest};

    // Nodes container class
    class BehaviorTree
    {
    private:
        // Nodes map
        std::map<std::string, TreeNode*> NodeMap;

        // Subtrees map
        std::map<std::string, BehaviorTree*> SubTreeMap;

        // Root node
        TreeNode* RootNode;

        // To avoid deadlocks
        bool Ticked;
    public:
        // Tree name
        std::string Name;

        // Tick period (1000 as default)
        unsigned long int TickPeriod;

        BehaviorTree(std::string Name);
        ~BehaviorTree();

        /// It creates a new control node:
        /// - "NodeName" should be unique among all the nodes;
        /// - "Type" should be either ParallelNode (or 0),
        ///   or SelectorNode (or 1), or SequenceNode (or 2).
        /// It throws a BehaviorTreeException if NodeName is already being
        /// used for another node or if a bad_alloc exception occurred while
        /// trying to create the new node.
        /// It returns the new node name.
        std::string& CreateNewControlNode(std::string NodeName, ControlNodeType Type);

        /// It creates a new action node:
        /// - "NodeName" should be unique among all the nodes;
        /// - "Type" can only be Test (or 0).
        /// It throws a BehaviorTreeException if NodeName is already being
        /// used for another node or if a bad_alloc exception occurred while
        /// trying to create the new node.
        /// It returns the new node name.
        std::string& CreateNewActionNode(std::string NodeName, ActionNodeType Type);

        /// It creates a new condition node:
        /// - "NodeName" should be unique among all the nodes;
        /// - "Type" can only be Test (or 0).
        /// It throws a BehaviorTreeException if NodeName is already being
        /// used for another node or if a bad_alloc exception occurred while
        /// trying to create the new node.
        /// It returns the new node name.
        std::string& CreateNewConditionNode(std::string NodeName, ConditionNodeType Type);

        /// It creates a new subtree:
        /// - "NodeName" should be unique among all the subtrees;
        /// It throws a BehaviorTreeException if NodeName is already being
        /// used for another subtree or if a bad_alloc exception occurred while
        /// trying to create the new subtree.
        /// It returns the new node subtree.
        BehaviorTree &CreateNewSubTree(std::string TreeName);

        /// It deletes the node with that specific NodeName.
        /// Beware, if the specified node is a control node,
        /// all of its children will be deleted too.
        /// It returns false if the NodeName does not belong
        /// to an existing node.
        /// bool DeleteNode(std::string NodeName);

        /// It sets the root node of the tree (if a root node already
        /// exists, it will be changed to be the new one).
        /// It throws a BehaviorTreeNodeNotFoundException if NodeName does not name
        /// an existing tree node.
        /// It returns the name of the old root node.
        std::string& SetRootNode(std::string NodeName);

        /// It sets a new father-child relationship (the children are
        /// added from left to right: the first child to be set would
        /// be the most left one, ecc...).
        /// It returns false if at least one Name does not belong
        /// to an existing node, or if FatherNodeName does not belong
        /// to a control node.
        std::pair<std::string&, std::string&> SetFatherChildRelationship(std::string FatherNodeName, std::string ChildName);

        /// It sets the N threashold for the specified parallels node.
        /// It throws a BehaviorTreeException if NodeName does not
        /// correspond to an existing parallel node or if the N threshold
        /// is grater then the number of child M.
        /// Use it only after having set all the node children.
        void SetParallelNodeThreshold(std::string NodeName, unsigned int N);

        /// It ticks the RootNode of the BehaviorTree
        /// It returns false if the RootNode doesn't exist.
        void Tick();

        /// Waits untill the RootNode has updated its state and then
        /// returns it (Blocking call).
        /// Return Success (or 0), Failure (or 1) or Running (or 2).
        /// Returns Idle (or 3) if the RootNode hasn't been ticked yet.
        NodeState GetTreeState();
    };
}

#endif
