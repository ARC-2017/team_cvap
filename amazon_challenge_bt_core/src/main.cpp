#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include <amazon_challenge_bt_core/BehaviorTree.h>
#include <typeinfo>
#include <math.h>       /* pow */



using namespace BT;

// Global robot variable
//NAO::RobotAction DesiredAction = NAO::None;
//std::mutex DesiredActionMutex;


ActionNode* detect;
ActionNode* grasp;
ActionNode* drop;
ActionNode* posearm;
ActionNode* removeobject;

ActionTestNode* test1;
ActionTestNode* test2;
ActionTestNode* test3;
ActionTestNode* test4;



ConditionNode* detected;
ConditionNode* grasped;
ConditionNode* armposed;

ConditionNode* dropped;
ConditionNode* done;
float x = 0.0;
float y = 0.4;
float x_offset = 0.1;
float r_color = 1;
float g_color = 1;
float b_color = 1;

bool reactiveBT = false;


SequenceStarNode* sequence1;
//SelectorStarNode* root;

SequenceStarNode* root;
DecoratorRetryNode* dec;


int GetDepth(TreeNode* tree )
{
    ControlNode* d = dynamic_cast<ControlNode*> (tree);
    if (d != NULL)
    {
        //the node is a control flow node
        int M = d->GetChildrenNumber();
        std::vector<TreeNode*> children = d->GetChildren();

        int depMax = 0;
        int dep = 0;
        for (int i = 0; i < M; i++)
        {
           dep = GetDepth(children[i]);
           if (dep > depMax)
           {
               depMax = dep;
           }

        }
      return 1 + depMax;
    }
    else
    {
        //the node is a leaf node
        return 0;
    }

}

void updateTree(TreeNode* tree, GLfloat x_pos, GLfloat y_pos, GLfloat x_offset, GLfloat y_offset )
{
    ControlNode* d = dynamic_cast<ControlNode*> (tree);
    if (d == NULL)
    {//if it is a leaf node, draw it

        draw_node((GLfloat) x_pos, (GLfloat) y_pos, tree->GetType(), tree->Name.c_str(), tree->ReadColorState());
    }
    else
    {//if it is a control flow node, draw it and its children
        draw_node((GLfloat) x_pos, (GLfloat) y_pos, tree->GetType(), tree->Name.c_str(), tree->ReadColorState());
        std::vector<TreeNode*> children = d->GetChildren();
        int M = d->GetChildrenNumber();
        for (int i = M-1; i >= 0; i--)
        {
            updateTree(children[i], x_pos - x_offset * (M-1) + 2*x_offset*(i) , y_pos - y_offset , x_offset/2.0  ,y_offset );
            draw_edge(x_pos, y_pos, 0.02, x_pos - x_offset * (M-1) + 2*x_offset*(i) , y_pos - y_offset, 0.02);
        }
    }
}


void display()
{

    glClearColor( r_color, g_color, b_color, 0.1);

    // clear the draw buffer .
    glClear(GL_COLOR_BUFFER_BIT);   // Erase everything
    updateTree(root, x , y, x_offset*pow(2,GetDepth(root)-1) , 0.1 );
    glutSwapBuffers();
    glutPostRedisplay();

}


void processSpecialKeys(int key, int xx, int yy) {

    float fraction = 0.1f;

    switch (key) {
        case GLUT_KEY_UP :
            y +=  fraction;
            break;
        case GLUT_KEY_DOWN :
            y -=  fraction;
            break;
        case GLUT_KEY_LEFT:
            x -=  fraction;
            break;
        case GLUT_KEY_RIGHT:
            x +=  fraction;
            break;
        case  GLUT_KEY_PAGE_UP:
         x_offset +=  fraction;
            break;
        case  GLUT_KEY_PAGE_DOWN:
        if (x_offset > 0.1+fraction) x_offset -=  fraction; //Avoid negative offset
            break;
        case  GLUT_KEY_F1:
        if (r_color < 1)  r_color +=  fraction;
             break;
        case  GLUT_KEY_F2:
        if (r_color > 0) r_color -=  fraction;
            break;
        case  GLUT_KEY_F3:
        if (g_color < 1) g_color +=  fraction;
             break;
        case  GLUT_KEY_F4:
        if (g_color > 0) g_color -=  fraction;
            break;
        case  GLUT_KEY_F5:
        if (b_color < 1) b_color +=  fraction;
             break;
        case  GLUT_KEY_F6:
        if (b_color > 0) b_color -=  fraction;
            break;


    }
}



void drawTree()
{
    //***************************BT VISUALIZATION****************************
    int argc = 1;
    char *argv[1] = {(char*)"Something"};
    glutInit(&argc, argv);      // Initialize GLUT
    glutInitWindowSize(1024,1024);

    glutCreateWindow("Behavior Tree");  // Create a window
    glutReshapeFunc(resize);
    glClearColor( 0, 0.71, 0.00, 0.1);
    glutDisplayFunc(display);   // Register display callback


    glutKeyboardFunc(keyboard); // Register keyboard callback
    glutSpecialFunc(processSpecialKeys); //Register keyboard arrow callback

    glutMainLoop();             // Enter main event loop

    //***************************ENDOF BT VISUALIZATION ****************************

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Main");
/*

    detect = new ROSAction("detector_node");
    grasp = new ROSAction("grasp_object");
    drop = new ROSAction("drop_object");
    posearm = new ROSAction("arm_position_server");
    removeobject = new ROSAction("remove_object");


    detected = new ROSCondition("object_detected");
    grasped = new ROSCondition("object_grasped");
    armposed = new ROSCondition("arm_posed");
    dropped = new ROSCondition("object_dropped");
    done = new ROSCondition("list_done");


     sequence1 = new SequenceStarNode("seq1");
    root = new SelectorStarNode("root");



*/
    test1 = new ActionTestNode("A1");
    test2 = new ActionTestNode("A2");
    test3 = new ActionTestNode("A3");
    test4 = new ActionTestNode("A4");
    sequence1 = new SequenceStarNode("seq1");

    dec = new DecoratorRetryNode("retry");


    test1->SetBehavior(Failure);
    test2->SetBehavior(Success);
    test3->SetBehavior(Success);



    test4->SetBehavior(Failure);

    root = new SequenceStarNode("root");



    std::cout << "Start Drawing!" << std::endl;

    boost::thread t(&drawTree);

    std::cout << "Drawing Done!" << std::endl;



    // ros::NodeHandle Handle;

    // ros::Subscriber Subscriber = Handle.subscribe<std_msgs::UInt8>("Inputs", 10, chatterCallback);

    // ros::spin();
    int TickPeriod_milliseconds = 1000;



    try
    {

/*

        sequence1->AddChild(posearm);
        sequence1->AddChild(grasp);
        sequence1->AddChild(drop);
        sequence1->AddChild(removeobject);

        root->AddChild(done);
        root->AddChild(sequence1);


*/
        sequence1->AddChild(test2);
        sequence1->AddChild(test1);
        sequence1->AddChild(test3);

        dec->AddChild(sequence1);


        root->AddChild(dec);
        root->AddChild(test4);

        std::cout << "Depth !"<< GetDepth(root) << std::endl << std::endl;


        root->ResetColorState();

        while(ros::ok())
        {


            std::cout << "Ticking the root node !" << std::endl << std::endl;

            // Ticking the root node
            root->Semaphore.Signal();
            // Printing its state
            //std::cout << sequence1->Name << " returned " << sequence1->GetNodeState() << "!" << std::endl << std::endl;
            root->GetNodeState();

            if(root->ReadState() != Running  )
            {
                root->ResetColorState();

            }
            // Simulating the tick period

            boost::this_thread::sleep(boost::posix_time::milliseconds(TickPeriod_milliseconds));

        }

       // std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    }
    catch (BehaviorTreeException& Exception)
    {
        std::cout << Exception.what() << std::endl;
    }

    return 0;
}


