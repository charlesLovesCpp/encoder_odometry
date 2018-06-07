#include <ros/ros.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int16MultiArray.h>
#include <eigen3/Eigen/Dense>

using namespace Eigen;

ros::Publisher pub_pose;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cartographer");
    ros::NodeHandle n;
    pub_pose = n.advertise<std_msgs::Int16MultiArray>("slam", 50);
    
    tf::TransformListener listener;
    ros::Rate rate(10.0);
    while (n.ok()) {
	tf::StampedTransform transform;
	try {
	    // Providing ros::Time(0) will just get us the latest available transform.
	    listener.lookupTransform("map", "laser", ros::Time(0), transform);// base_link->map,base_link represented in map frame 
	}
	catch (tf::TransformException ex) {
	    ROS_ERROR("%s", ex.what());
	    ros::Duration(1.0).sleep();
	}

	tf::Quaternion q = transform.getRotation();
	double yaw = tf::getYaw(q);	
	Vector2d xy = Vector2d(transform.getOrigin().x(), transform.getOrigin().y());

	ROS_INFO_STREAM("x: " << xy.x() * 100 << " y: " << xy.y() * 100 << " yaw: " << (yaw * 180 / M_PI));
	// Publish slam topic to serial port
// 	pub_pose.publish(msg_int);
	
	rate.sleep();
    }
    

    
    return(0);
}