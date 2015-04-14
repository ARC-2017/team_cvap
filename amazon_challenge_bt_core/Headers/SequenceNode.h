#ifndef SEQUENCENODE_H
#define SEQUENCENODE_H

#include "ControlNode.h"

namespace BT
{
    class SequenceNode : public ControlNode
    {
    public:
        // Constructor
        SequenceNode(std::string Name);
        ~SequenceNode();

        // The method that is going to be executed by the thread
        void Exec();
    };
}

#endif
