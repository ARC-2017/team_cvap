#ifndef SEQUENCESTARNODE_H
#define SEQUENCESTARNODE_H

#include <amazon_challenge_bt_core/ControlNode.h>

namespace BT
{
    class SequenceStarNode : public ControlNode
    {
    public:
        // Constructor
        SequenceStarNode(std::string Name);
        ~SequenceStarNode();
	int GetType();
        // The method that is going to be executed by the thread
        void Exec();
    };
}

#endif
