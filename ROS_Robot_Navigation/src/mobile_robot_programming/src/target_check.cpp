#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float32MultiArray.h>
#include "tf/transform_listener.h"

// function declarations
void callback(const nav_msgs::OdometryConstPtr &ptr);
// callback function that gets the map
void map_update(const nav_msgs::OccupancyGrid new_map);
bool get_position(float &x, float &y);
// global variables
ros::Publisher pub;
float xTarget,yTarget;
bool map_valid = false;

// main function
int main(int argc, char** argv)
{
    // read target parameters
    if(argc < 5) {
        ROS_ERROR("Cannot read coordinates of target!");
        //ROS_INFO("Setting target to x:0, y:0");
        xTarget = 0;
        yTarget = 0;
    }
    else {
        xTarget = atof(argv[1]);
        yTarget = atof(argv[2]);
        ROS_INFO("Setting target to x:%.2f, y:%2f", xTarget, yTarget);
        //here transform the map coordinates to odom coordinates to be uniform with the odom coordinates in the callback function
        //xTarget = xTarget + 1.194684577112486;
        //yTarget = yTarget - 23.335950614726176;
    }

    // initialize node
    ros::init(argc, argv, "target_check");
    ros::NodeHandle * nh = new ros::NodeHandle();

    // subscribe to the odometry topic
    ros::Subscriber sub = nh->subscribe("/odom",10,callback);

    // subscribe to the map topic
    ros::Subscriber map_sub = nh->subscribe("/map", 1, map_update);
    // wait for map
    while(map_valid == false){
        ros::Duration(1).sleep();
        ros::spinOnce(); // check for map update
    }

    // advertise found topic
    pub = nh->advertise<std_msgs::Float32MultiArray>("/found", 1000);

    ros::Duration(1).sleep();

    tf::TransformListener tf_listener;
    tf::StampedTransform position;
    
    while(nh->ok())
    {
      try
      {
        tf_listener.lookupTransform("map", "odom", ros::Time(), position);
        break;
      }   
      catch(tf::LookupException ex)
      {
        ROS_WARN("%s", ex.what());
        //ros::Duration(10).sleep();
        ros::Duration(0.1).sleep();
        ros::spinOnce();
      }
    }
    // How to get the current position of the robot with respect to the map of objects
    // I want to know Robot Current position w.r.t the bottom left of image=1376x1376 
    xTarget = position.getOrigin().x() - xTarget;
    yTarget = position.getOrigin().y() - yTarget ;
    
	if(get_position(xTarget, yTarget))
    {
    	//ROS_INFO("Setting target (odom) to x:%.2f, y:%2f", xTarget, yTarget);
    	ROS_INFO("TF map - odom x:%.2f, y:%2f", position.getOrigin().x(), position.getOrigin().y());
    }
    // keep running and check for odometry updates
    ros::spin();

    return 0;
}

// callback function for odometry updates
void callback(const nav_msgs::OdometryConstPtr& ptr )
{
    // get position from odometry
    nav_msgs::Odometry odom = *ptr.get();
    float x,y; // units are m, not grid cells
    x = odom.pose.pose.position.x;
    y = odom.pose.pose.position.y;

    float range = 1.5;
    // if target is in range, publish found message
    if(x < xTarget+range && x > xTarget-range)
    {
        if(y < yTarget+range && y > yTarget-range)
        {
            std_msgs::Float32MultiArray target;
            target.data.push_back(x);
            target.data.push_back(y);
            pub.publish(target);
        }
    }
}

// get the current position of the robot in x,y coordinates (meters)
bool get_position(float &x, float &y)
{
    tf::StampedTransform position;
    tf::TransformListener tf_listener;

    try{
        // sleeping to avoid errors
        // increase if the transformation lookup is not working
        ros::Duration(0.5).sleep();

        tf_listener.lookupTransform("map", "base_footprint", ros::Time(), position);

        x = position.getOrigin().x();
        y = position.getOrigin().y();

        return true;
    }
    catch(tf::TransformException tx){
        ROS_ERROR("Error while trying to get position: %s", tx.what());
        return false;
    }
}



// callback function that gets the map
void map_update(const nav_msgs::OccupancyGrid new_map)
{
    map_valid = true;
}
