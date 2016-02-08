#include <amazon_challenge_bt_core/LeafNode.h>

using namespace BT;

LeafNode::LeafNode(std::string Name) : TreeNode(Name) {}

LeafNode::~LeafNode() {}


void LeafNode::ResetColorState()
{
    // Lock acquistion

    ColorState = Idle;
}
