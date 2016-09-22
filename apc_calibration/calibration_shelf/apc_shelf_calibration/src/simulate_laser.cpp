#include "ros/ros.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include "sensor_msgs/LaserScan.h"
#include "pcl_ros/impl/transforms.hpp"
#include <pcl/filters/passthrough.h>
#include <tf/transform_broadcaster.h>
#include <math.h>

using namespace std;

ros::Publisher  pub_laser, put_select;
geometry_msgs::TransformStamped odom_trans;
tf::TransformListener* tfListener = NULL;

#define ROBOT_TO_SHELF_DIST 1 // 1 meter
#define ROBOT_TO_SHELF_DIST_MARGIN ROBOT_TO_SHELF_DIST * 5.0/100 // 5 percent
#define POSSIBLE_SHELF_DISPLACEMENT 3.0/100 // 3 cm for each leg

#define MAX_DISCARD_THRESHOLD_X ROBOT_TO_SHELF_DIST + ROBOT_TO_SHELF_DIST_MARGIN + POSSIBLE_SHELF_DISPLACEMENT
#define MIN_DISCARD_THRESHOLD_X ROBOT_TO_SHELF_DIST - ROBOT_TO_SHELF_DIST_MARGIN - POSSIBLE_SHELF_DISPLACEMENT

#define MAX_DISCARD_THRESHOLD_Z -0.2
#define MIN_DISCARD_THRESHOLD_Z -0.3

float max_discard_threshold_z;
float min_discard_threshold_z;
float max_discard_threshold_x;
float min_discard_threshold_x;

void callback_kinect2(const sensor_msgs::PointCloud2ConstPtr &cloud_in)
{
    sensor_msgs::PointCloud2 cloud_base;
    tf::StampedTransform kinect_to_base;
    tfListener->waitForTransform("/base", cloud_in->header.frame_id, ros::Time::now(), ros::Duration(1.0));
    tfListener->lookupTransform("/base", cloud_in->header.frame_id, ros::Time(0), kinect_to_base);
    // cout << "frame " <<cloud_in->header.frame_id <<endl;
    Eigen::Matrix4f eigen_transform;
    pcl_ros::transformAsMatrix (kinect_to_base, eigen_transform);
    pcl_ros::transformPointCloud (eigen_transform, *cloud_in, cloud_base);

    cloud_base.header.frame_id = "base";
    sensor_msgs::PointCloud cloud_selected;
    sensor_msgs::PointCloud cloud_out_base, cloud_out_sensor;
    sensor_msgs::convertPointCloud2ToPointCloud(cloud_base, cloud_out_base);
    cloud_out_base.header.frame_id = "base";

    for(int i = 0; i<cloud_out_base.points.size(); i++)
    {
        float x = cloud_out_base.points[i].x;
        float y = cloud_out_base.points[i].y;
        float z = cloud_out_base.points[i].z;
        //float r = sqrt(x*x + y*y);
        //float angle = abs(atan2(z,r));
        //if(angle < 3.1415926/180 * 2)
        //if(angle > 0 && angle < 3.1415926/180 * 2)
        if(z < max_discard_threshold_z && z > min_discard_threshold_z) // z points upwards from the robot
        {
            if (x < max_discard_threshold_x && x > min_discard_threshold_x) // x points forward from the robot
            {
                cloud_out_base.points[i].x = x;
                cloud_out_base.points[i].y = y;
                cloud_out_base.points[i].z = z;
                cloud_selected.points.push_back(cloud_out_base.points[i]);
            }
        }
    }

    cloud_selected.header = cloud_out_base.header;
    // cout << "number of points remaining after filtering; " << cloud_selected.points.size() << endl;
    put_select.publish(cloud_selected);

    ///////////////////////////////////////////////////////////////////////////////
    sensor_msgs::LaserScan output;
    output.header = cloud_selected.header;
    output.header.frame_id = "base";
    output.angle_min = -M_PI_4;
    output.angle_max = M_PI_4;
    output.angle_increment = M_PI_2/1000;
    output.time_increment = 0.0;
    output.scan_time = 0.3333;
    output.range_min = 0.5;
    output.range_max = 2;

    uint32_t ranges_size = M_PI_2/output.angle_increment;
    output.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());

    for(int i = 0; i<cloud_selected.points.size(); i++)
    {
        float x = cloud_selected.points[i].x;
        float y = cloud_selected.points[i].y;
        float r = sqrt(x*x + y*y);

        double angle = atan2(y, x);
        if (angle < output.angle_min || angle > output.angle_max)
        {
            continue;
        }

        int index = round((angle - output.angle_min) / output.angle_increment);
        if (r < output.ranges[index])
        {
            output.ranges[index] = r;
        }
    }

    // cout << "publishing laser simulation" << endl;
    pub_laser.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simulate_laser");

    ros::NodeHandle node;
    ros::Rate rate(10.0);

    ros::Subscriber sub_kinect2 = node.subscribe<sensor_msgs::PointCloud2>("/kinect_chest/hd/points", 1, callback_kinect2);
    put_select = node.advertise<sensor_msgs::PointCloud>("/simu_scan", 1);
    pub_laser = node.advertise<sensor_msgs::LaserScan>("/scan", 1);

    //tfListener = new (tf::TransformListener);
    tf::TransformBroadcaster broadcaster;
    tfListener = new (tf::TransformListener);

    while (node.ok())
    {
        try {
            node.getParam("apc/shelf_calibration/max_discard_threshold_z", max_discard_threshold_z);
            node.getParam("apc/shelf_calibration/min_discard_threshold_z", min_discard_threshold_z);
            node.getParam("apc/shelf_calibration/max_discard_threshold_x", max_discard_threshold_x);
            node.getParam("apc/shelf_calibration/min_discard_threshold_x", min_discard_threshold_x);
            break;
            }
        catch(const std::exception& e)
        {
            sleep(1.0);
            continue;
        }
    }

    //node.getParam("apc/shelf_calibration/max_discard_threshold_z", max_discard_threshold_z);
    //node.getParam("apc/shelf_calibration/min_discard_threshold_z", min_discard_threshold_z);
    //node.getParam("apc/shelf_calibration/max_discard_threshold_x", max_discard_threshold_x);
    //node.getParam("apc/shelf_calibration/min_discard_threshold_x", min_discard_threshold_x);

    cout << MAX_DISCARD_THRESHOLD_X << endl;
    cout << MIN_DISCARD_THRESHOLD_X << endl;

    cout << "max z: " << max_discard_threshold_z << " min z: " << min_discard_threshold_z << endl;
    cout << "max x: " << max_discard_threshold_x << " min x: " << min_discard_threshold_x << endl;
    while (node.ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
};
