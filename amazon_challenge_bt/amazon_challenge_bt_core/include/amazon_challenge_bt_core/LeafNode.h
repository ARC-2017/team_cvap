#ifndef LEAFNODE_H
#define LEAFNODE_H

#include <vector>

#include <amazon_challenge_bt_core/TreeNode.h>

namespace BT
{
    class LeafNode : public TreeNode
    {
    protected:
    public:
        LeafNode(std::string Name);
        ~LeafNode();
	void ResetColorState();
    };
}

#endif
