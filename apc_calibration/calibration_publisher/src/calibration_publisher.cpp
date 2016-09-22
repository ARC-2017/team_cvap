#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include <fstream>

using namespace std;

int main(int argc, char** argv) {


    ros::init(argc, argv, "calibration_publisher");
    ros::NodeHandle n("~");

    string calibration_file;
    string source_frame;
    string target_frame;

    n.param<string>("calibration_file",calibration_file, "");
    n.param<string>("source_frame",source_frame, "");
    n.param<string>("target_frame",target_frame, "");
    
    cout<<"Calibration file "<<calibration_file<<";   source frame "<<source_frame<<";  target frame "<<target_frame<<endl;

    ifstream file_in;
    file_in.open(calibration_file.c_str());

    double tx = 0.0, ty = 0.0, tz = 0.0, qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;

    tf::TransformBroadcaster br;
    tf::Transform transform;

    // initialize transform with identity
    transform.setOrigin( tf::Vector3(tx, ty, tz) );
    tf::Quaternion q(qx, qy, qz, qw);
    transform.setRotation(q);

    if (!file_in.is_open()){
        ROS_ERROR_STREAM("Could not open calibration file  "<<calibration_file);
            ROS_INFO_STREAM("Publishing identity transform from between "<<source_frame<<" and "<<target_frame);
    } else {
        ROS_INFO_STREAM("Publishing transform from "<< calibration_file<<"between "<<source_frame<<" and "<<target_frame);
        file_in >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        transform.setOrigin( tf::Vector3(tx, ty, tz) );
        tf::Quaternion q(qx, qy, qz, qw);
        transform.setRotation(q);
    }

    ros::Rate rate(50.0f);

    while (n.ok()) {
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), source_frame, target_frame));
        rate.sleep();
    }
    
    ROS_INFO_STREAM("Stopping publishing transform between "<<source_frame<<" and "<<target_frame);

    return 0;
}
