#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include <amazon_challenge_bt_core/BehaviorTree.h>
#include <typeinfo>
#include <math.h>       /* pow */
#include <std_srvs/Empty.h>
#include <signal.h>

using namespace BT;

// Global robot variable
//NAO::RobotAction DesiredAction = NAO::None;
//std::mutex DesiredActionMutex;


ActionNode* detect;
ActionNode* grasp;
ActionNode* drop;
ActionNode* pregrasp;
ActionNode* base_torso_head_init;

ActionNode* removeobject;
ActionNode* nextobject;



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
SequenceStarNode* sequence2;
SelectorStarNode* root;
SelectorStarNode* selector1;
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

void timerCB(int millisec)
{
glutTimerFunc(millisec, timerCB, millisec);
glutPostRedisplay();
}
void display()
{

    glClearColor( r_color, g_color, b_color, 0.1);
    glutTimerFunc(500, timerCB, 500); // draw every 500 ms
    ros::Duration(0.2).sleep();
    // clear the draw buffer .
    glClear(GL_COLOR_BUFFER_BIT);   // Erase everything
    updateTree(root, x , y, x_offset*pow(2,GetDepth(root)-1) , 0.1 );
    glutSwapBuffers();
    glutPostRedisplay();

}


void processSpecialKeys(int key, int xx, int yy) {

    float pos_fraction = 0.05f;
    float col_fraction = 0.1f;

    switch (key) {
        case GLUT_KEY_UP :
            y +=  pos_fraction;
            break;
        case GLUT_KEY_DOWN :
            y -=  pos_fraction;
            break;
        case GLUT_KEY_LEFT:
            x -=  pos_fraction;
            break;
        case GLUT_KEY_RIGHT:
            x +=  pos_fraction;
            break;
        case  GLUT_KEY_PAGE_UP:
         x_offset +=  pos_fraction;
            break;
        case  GLUT_KEY_PAGE_DOWN:
        if (x_offset > 0.1+pos_fraction) x_offset -=  pos_fraction; //Avoid negative offset
            break;
        case  GLUT_KEY_F1:
        if (r_color < 1)  r_color +=  col_fraction;
             break;
        case  GLUT_KEY_F2:
        if (r_color > 0) r_color -=  col_fraction;
            break;
        case  GLUT_KEY_F3:
        if (g_color < 1) g_color +=  col_fraction;
             break;
        case  GLUT_KEY_F4:
        if (g_color > 0) g_color -=  col_fraction;
            break;
        case  GLUT_KEY_F5:
        if (b_color < 1) b_color +=  col_fraction;
             break;
        case  GLUT_KEY_F6:
        if (b_color > 0) b_color -=  col_fraction;
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

bool start;

bool kickAssSrvCallback(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    ROS_INFO("[Behavior Tree]: Starting to kick ass!!!");
    start = true;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Main");
    start = false;

    ros::NodeHandle nh("~");
    ros::ServiceServer srv = nh.advertiseService("/kick_ass", kickAssSrvCallback);
    ros::Rate r(1.0);

    ROS_INFO("Waiting for input to start the behavior tree!");
    while(!start && ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    detect = new ROSAction("amazon_detector");
    grasp = new ROSAction("grasp_object");
    drop = new ROSAction("drop_object");
    pregrasp = new ROSAction("pregrasp_server");
    base_torso_head_init = new ROSAction("base_torso_head_init_server");


    removeobject = new ROSAction("remove_object");
    nextobject = new ROSAction("next_object");


    detected = new ROSCondition("object_detected");
    grasped = new ROSCondition("object_grasped");
    armposed = new ROSCondition("arm_posed");
    dropped = new ROSCondition("object_dropped");
    done = new ROSCondition("list_done");


    sequence1 = new SequenceStarNode("seq1");
    sequence2 = new SequenceStarNode("seq2");
    selector1 = new SelectorStarNode("selector1");
    root = new SelectorStarNode("root");



    std::cout << "Start Drawing!" << std::endl;

    boost::thread t(&drawTree);

    std::cout << "Drawing Done!" << std::endl;



    // ros::NodeHandle Handle;

    // ros::Subscriber Subscriber = Handle.subscribe<std_msgs::UInt8>("Inputs", 10, chatterCallback);

    // ros::spin();
    int TickPeriod_milliseconds = 1000;



    try
    {



        sequence2->AddChild(detect);
        sequence2->AddChild(pregrasp);
        sequence2->AddChild(grasp);


        selector1->AddChild(sequence2);
        selector1->AddChild(nextobject);
        sequence1->AddChild(base_torso_head_init);
        sequence1->AddChild(selector1);
        sequence1->AddChild(drop);
        sequence1->AddChild(removeobject);
        root->AddChild(done);
        root->AddChild(sequence1);

        root->ResetColorState();

//        system("pkill -STOP rviz");

        ros::Rate loop_rate(1.0);
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

            loop_rate.sleep();

        }

       // std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    }
    catch (BehaviorTreeException& Exception)
    {
        std::cout << Exception.what() << std::endl;
    }

//    system("pkill -CONT base_torso_head");
//    system("pkill -CONT detector");
//    system("pkill -CONT pregrasp");
//    system("pkill -CONT grasping");
//    system("pkill -CONT dropping");
//    system("pkill -CONT pr2_cam_switch");
//    system("pkill -CONT periodic_cloud");
//    system("pkill -CONT rviz");
//    system("pkill -CONT rqt_amazon");

    return 0;
}


