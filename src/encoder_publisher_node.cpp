#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include <math.h>
#include <boost/bind.hpp>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <fstream>
#include <numeric>

ros::Publisher pub_encoder;

void generateEncoderMeasurement(double& _d_angle_right, double& _d_angle_left)
{
    _d_angle_right = 0.36;
    _d_angle_left = 0.18;
}

void publish(const double& _d_angle_right, const double& _d_angle_left)
{
    sensor_msgs::PointCloudPtr encoder_msg(new sensor_msgs::PointCloud);
    
    geometry_msgs::Point32 p;
    p.x = _d_angle_right;
    p.y = _d_angle_left;
    p.z = 1;    
    encoder_msg->points.push_back(p);
    
    pub_encoder.publish(encoder_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Encoder_Publisher");
    ros::NodeHandle n;

    pub_encoder = n.advertise<sensor_msgs::PointCloud>("encoder", 1000);
    
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();   
	ROS_INFO("Publishing encoder measurements...");
	
	double d_angle_left, d_angle_right;
	
	generateEncoderMeasurement(d_angle_left, d_angle_right);
	publish(d_angle_left, d_angle_right);
	
        loop_rate.sleep();
    }
    return(0);
}