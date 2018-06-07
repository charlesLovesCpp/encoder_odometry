#pragma once

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include <math.h>
#include <boost/bind.hpp>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <fstream>
#include <numeric>

using namespace std;
using namespace Eigen;

class EncoderOdometry
{
public:
    EncoderOdometry();
    ~EncoderOdometry();
    
    std::string		encoder_topic;
    std::string		odom_topic;
    std::string		base_frame_id;
    std::string         odom_frame_id;
    bool                publish_tf;
    bool 		new_encoder_available;
    bool		first_encoder_meas;
    double              freq;
    double		wheel_diameter;		//! diameter of wheels(cm)
    double 		angle2arclength;
     
    double 		dx;
    double 		dy;
    double 		d_right_angle;		//! angle difference of right wheel
    double		d_left_angle;		//! angle difference of left wheel
    double 		d_xy_average;		//! linear speed
    double 		d_theta;		//! angle of yaw
    double 		dt;			//! time interal(s)
 
    double 		x;
    double		y;
    double 		theta;
    double 		v_xy;			//! velocity of xy
    double 		v_theta;		//! angular rate
    
    double 		cur_timestamp;
    double 		last_timestamp;
    
    void encoderCallBack(const sensor_msgs::PointCloudConstPtr &encoder_msg);
    
    void odometryCalculation();
    
    bool is_available();
    
    void publishOdom();    
    
    void initialOdom();
  
protected:
    ros::NodeHandle		n;
    std_msgs::Float32    	encoder_meas;
    tf::TransformListener       tf_listener; 
    tf::TransformBroadcaster    odom_broadcaster;
    nav_msgs::Odometry          initial_robot_pose;
    
    //Subscriptions & Publishers
    ros::Subscriber encoder_sub, initPose_sub;
    ros::Publisher odom_pub;
    

};