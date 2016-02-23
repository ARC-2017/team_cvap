#ifndef DECORATORRETRYNODE_H
#define DECORATORRETRYNODE_H

#include <amazon_challenge_bt_core/ControlNode.h>

namespace BT
{
    class DecoratorRetryNode : public ControlNode
    {
    public:
        // Constructor
        DecoratorRetryNode(std::string Name);
        ~DecoratorRetryNode();
	int GetType();
        // The method that is going to be executed by the thread
        void Exec();
    };
}

#endif
