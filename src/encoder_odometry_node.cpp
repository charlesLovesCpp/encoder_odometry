#include "encoder_odometry_node.h"

EncoderOdometry::EncoderOdometry():
    first_encoder_meas(true),
    new_encoder_available(false)
{
    ROS_INFO("Initializing EncoderOdometry node...");
    
    ros::NodeHandle pn("~");
    pn.param<std::string>("encoder_topic",encoder_topic,"/encoder");
    pn.param<std::string>("odom_topic", odom_topic, "/odom");
    pn.param<std::string>("base_frame_id", base_frame_id, "/base_link");
    pn.param<std::string>("odom_frame_id", odom_frame_id, "/odom");    
    pn.param<bool>("publish_tf", publish_tf, true);
    pn.param<double>("freq",freq, 10.0);
    pn.param<double>("wheel_diameter",wheel_diameter, 15.25);
    pn.param<double>("dt",dt, 0.01);
    
    odom_pub = pn.advertise<nav_msgs::Odometry>(odom_topic, 5);
    encoder_sub = n.subscribe(encoder_topic, 1, &EncoderOdometry::encoderCallBack, this);
//     encoder_sub = n.subscribe<sensor_msgs::PointCloudConstPtr>(encoder_topic, 1, &EncoderOdometry::encoderCallBack, this);

    angle2arclength = M_PI * wheel_diameter / 360.0;
}

EncoderOdometry::~EncoderOdometry()
{
}

void EncoderOdometry::initialOdom()
{
    x = 0;
    y = 0;
    theta = 0;
    v_xy = 0;
    v_theta = 0;
}


void EncoderOdometry::encoderCallBack(const sensor_msgs::PointCloudConstPtr& encoder_msg)
{
    if (first_encoder_meas) {
	initialOdom();
	first_encoder_meas = false;
    }
    
    d_right_angle = encoder_msg->points[0].x / 10;		// distance that left wheel rotates
    d_left_angle = encoder_msg->points[0].y / 10;		// distance that right wheel rotates
    
    new_encoder_available = true;
}

double D2R(const double& deg)
{
    return deg / 180.0 * M_PI;
}

void EncoderOdometry::odometryCalculation()
{
    d_xy_average = (d_right_angle + d_left_angle) / 2.0 * angle2arclength;
    d_theta = d_right_angle - d_left_angle;		
    v_xy = d_xy_average / dt;
    v_theta = d_theta / dt;
    
    if (d_xy_average != 0.0) {
	dx = cos(D2R(d_theta)) * d_xy_average;
	dy = -sin(D2R(d_theta)) * d_xy_average;
	x += cos(D2R(d_theta)) * dx - sin(D2R(d_theta)) * dy;
	y += sin(D2R(d_theta)) * dx + cos(D2R(d_theta)) * dy;
    }
    
    if (d_theta != 0.0) {
	theta += d_theta;
    }
    
    publishOdom();
    new_encoder_available = false;
}

void EncoderOdometry::publishOdom()
{
    if (publish_tf) {
        geometry_msgs::TransformStamped odom_trans; //base_link → odom , base_link is represented in odom, so odom is parant frame 
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.header.frame_id = odom_frame_id;// base_frame_id represented in odom frame, odom -> base
        odom_trans.child_frame_id = base_frame_id;
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(theta);
        //send the transform
        odom_broadcaster.sendTransform(odom_trans);	
    }
    
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = odom_frame_id;
    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
    //set the velocity
    odom.child_frame_id = base_frame_id;
    odom.twist.twist.linear.x = v_xy;    	//linear speed
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = v_theta;	//angular speed
    //publish the message
    odom_pub.publish(odom);
    
    ROS_INFO_STREAM("x: "<< x << " y: " << y << " theta: " << theta);
}

bool EncoderOdometry::is_available()
{
    return new_encoder_available;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "Encoder_Odom");
 
    EncoderOdometry encoder_odom;
    //Main Loop
    //----------
    ROS_INFO("encoder_odom initialization complete...Looping");
    ros::Rate loop_rate(encoder_odom.freq);
    while (ros::ok())
    {
        ros::spinOnce();        

        if( encoder_odom.is_available() ) {            
            //Process odometry estimation
            encoder_odom.odometryCalculation();
	    ROS_INFO("Receive encoder measurement!");
        }
        else {
            ROS_WARN("Waiting for encoder measurement....") ;
        }

        loop_rate.sleep();
    }
    return(0);
}