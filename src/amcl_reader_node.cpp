#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int16MultiArray.h>
#include <eigen3/Eigen/Dense>

using namespace Eigen;

ros::Publisher pub_pose;

Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
{
    Eigen::Vector3d n = R.col(0);
    Eigen::Vector3d o = R.col(1);
    Eigen::Vector3d a = R.col(2);

    Eigen::Vector3d ypr(3);
    double y = atan2(n(1), n(0));
    double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;

    return ypr / M_PI * 180.0;
}
    
void amclCallBack(const geometry_msgs::PoseWithCovarianceStamped& msg)// this is a msg, so we have to use msg->
{
        
//     ros::Subscriber sub_pose = n.subscribe("/amcl_pose", 50, amclCallBack);
//     ros::spin();
    
    std::string frame_id = msg.header.frame_id;
    ROS_INFO_STREAM("Frame_id: " << frame_id);
    
    // covariance
    float cov[36];
    for (int i = 0; i < 36; ++i) {
	cov[i] = msg.pose.covariance[i];
    }   
    
    // position
    double x, y, z;
    x = msg.pose.pose.position.x;
    y = msg.pose.pose.position.y;
    z = msg.pose.pose.position.z;
    
    // quaternion
    geometry_msgs::Quaternion odom_quat;
    odom_quat = msg.pose.pose.orientation;
    Eigen::Quaterniond q;
    q.x() = odom_quat.x;
    q.y() = odom_quat.y;
    q.z() = odom_quat.z;
    q.w() = odom_quat.w;
    
    // yaw
    Vector3d ypr = R2ypr(q.toRotationMatrix());
    double angle_min = tf::getYaw(odom_quat);  
    
    
    std_msgs::Int16MultiArray msg_int;
    msg_int.data[0] = x;
    msg_int.data[1] = y;
    msg_int.data[2] = ypr.z();
    
    pub_pose.publish(msg_int);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "AMCL_Reader");
    ros::NodeHandle n;
    pub_pose = n.advertise<std_msgs::Int16MultiArray>("slam", 50);
    
    tf::TransformListener listener;
    ros::Rate rate(10.0);
    while (n.ok()) {
	tf::StampedTransform transform;
	try {
	    // Providing ros::Time(0) will just get us the latest available transform.
	    listener.lookupTransform("map", "base_link", ros::Time(0), transform);// base_link->map,base_link represented in map frame 
	}
	catch (tf::TransformException ex) {
	    ROS_ERROR("%s", ex.what());
	    ros::Duration(1.0).sleep();
	}

	tf::Quaternion q = transform.getRotation();
	double yaw = tf::getYaw(q);
	
	Vector2d xy = Vector2d(transform.getOrigin().x(), transform.getOrigin().y());

	double angle = atan2(transform.getOrigin().y(), transform.getOrigin().x());
	double distance = sqrt(pow(transform.getOrigin().x(), 2) + 
				pow(transform.getOrigin().y(), 2));

	// Publish slam topic to serial port
// 	pub_pose.publish(msg_int);
	
	rate.sleep();
    }
    

    
    return(0);
}