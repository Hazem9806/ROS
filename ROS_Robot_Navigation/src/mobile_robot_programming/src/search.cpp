#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <std_msgs/Float32MultiArray.h>
#include <ros/service_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "tf/transform_listener.h"
#include <std_msgs/String.h>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <visualization_msgs/MarkerArray.h>
// function declarations
void map_update(const nav_msgs::OccupancyGrid search_map);
void map_read_data(const nav_msgs::OccupancyGrid search_map);
void target_found(std_msgs::Float32MultiArray);
bool send_goal(float x, float y);
bool get_position(float &x, float &y);
void cell2meter(int i, int j, float &x, float &y);
void meter2cell(float x, float y, int &i, int &j);
bool check_next_point(float, float);
void ObjectSearching();
void FindObject(float , float);
// global variables
nav_msgs::OccupancyGrid map;
ros::Subscriber found_sub, map_sub, map_get_data;
bool map_valid = false;
float object_x = 1;
float object_y = 0;
int visited_nodes [1376][1376];
float unvisited_nodes [1376][1376] = {-2};
const int numberOfObjects = 4;
float objectsToFind [numberOfObjects][2] = {{-12.00,-3.50},{-8.00,3.50},{4.00,15.50}, {2.00,0.50}};
bool objectsFound [numberOfObjects] = {false, false, false, false};
float orientation = 0.0;


int seq_goal = 0;    // sequence id of goal
int map_array [1376][1376];

// Define the boundries for our Robot
// I divided the map into 2 boxes
// Left Box & Right Box
// Each box has its own boundries to search in
// These vales are recorded by expertiment
int leftXMaxLimit = 13;
int leftXMinLimit = 2;

int leftYMaxLimit = 18;
int leftYMinLimit = -7;

int rightXMaxLimit = 0.5;
int rightXMinLimit = -15;

int rightYMaxLimit = 7;
int rightYMinLimit = -6.5;

//using namespace std;
// main function
int main(int argc, char **argv)
{

    // initialize node
    ros::init(argc, argv, "search");

    // node handle
    ros::NodeHandle *n = new ros::NodeHandle();
    
    // subscribe to target check (for the search task)
    found_sub = n->subscribe("/found", 1000, target_found);

    // subscribe to the map topic
    map_sub = n->subscribe("/map", 1, map_update);

    // subscribe to the map topic
    map_get_data = n->subscribe("/map", 1, map_read_data);
    //map_sub.getTopic
    // wait for map
    while (map_valid == false)
    {
        ros::Duration(1).sleep();
        ros::spinOnce(); // check for map update
    }
    float tempy = -1.0,tempx = -1.0;
    cell2meter(0,0,tempx,tempy);
    int tempxx = -1, tempyy = -1;
    meter2cell(1.0, 1.0, tempxx,tempyy);
    ROS_INFO("cell values at(0,0) is (%d,%d)",tempxx,tempyy);
    ROS_INFO("meter values at(0,0) is (%.2f,%.2f)",tempx,tempy);
    // implement your search here...
    ObjectSearching();

    return 0;
}

void ObjectSearching(){
    // implement your search here...

    // My start of search Position
    // My implementation will goes as follows
    // The robot will move like a snake from the right top corner
    object_x = -14;
    object_y = -5.5;


    ROS_INFO("Sending robot to x:%.2f, y:%.2f", object_x, object_y);
    ROS_INFO("Robot Will start to search once it reached this poistion");
    
    if (send_goal(object_x, object_y) == false)
    {
        ROS_ERROR("Could not reach goal x:%.2f, y:%.2f", object_x, object_y);
    }
    // execute awaiting callbacks
    ros::spinOnce();

    bool leftBoxSearching = true;
    bool rightBoxSearching = false;
     
    bool searchUpwards = false;
    bool searchDownwards = true;
    
    bool searchFinished = false;
    int positionStep = 2.5;
    
    int numberOfObjects = 4;
    //objectsToFind = {{12.00,-3.50},{8.00,8.50},{4.00,15.50}, {2.00,0.50}};
    //objectsFound [numberOfObjects] = {false};
    while(!searchFinished)
    {
        while(true)
        {
            if(object_x >= rightXMaxLimit){
                rightBoxSearching = false;
                leftBoxSearching = true;
            }

            if(rightBoxSearching)
            {
                // Right Box Searching
                if(object_y >= rightYMaxLimit)
                {
                    searchUpwards = true; searchDownwards = false; // Robot will go upwards
                    object_x += positionStep;
                    ROS_INFO("Robot is going upwards now! -- [Right Box Searching]");
                }
                else if(object_y <= rightYMinLimit)
                {
                    searchUpwards = false; searchDownwards = true; // Robot will go downwards
                    object_x += positionStep;
                    ROS_INFO("Robot is going downwards now! -- [Right Box Searching]");
                }
            }

            if(leftBoxSearching)
            {
                // Left Box Searching
                if(object_y >= leftYMaxLimit)
                {
                    searchUpwards = true; searchDownwards = false; // Robot will go upwards
                    object_x += positionStep;
                    ROS_INFO("Robot is going upwards now! -- [Left Box Searching]");
                }
                else if(object_y <= leftYMinLimit)
                {
                    searchUpwards = false; searchDownwards = true; // Robot will go downwards
                    object_x += positionStep;
                    ROS_INFO("Robot is going downwards now! -- [Left Box Searching]");
                }
            }

            // Searching directions
            if(searchDownwards){
                object_y += positionStep;
            }
            else if(searchUpwards){
                object_y--;
            }

            // We cannot reach this pont untill we have checked if there is an object 
            // in the target cell to make the navigation easier
            if(check_next_point(object_x, object_y)){
                //ROS_INFO("Next Point = (%.2f , %.2f) is valid!", object_x, object_y);
                break;
            }
            //ROS_INFO("Changing the next point!");
            if(object_x > 13 && object_y > 18 ){
                searchFinished = true;
                ROS_INFO("Search is finished!!");
                break;
            }   
        }
        
        if (send_goal(object_x, object_y) == false)
        {
            ROS_ERROR("Could not reach goal x:%.2f, y:%.2f", object_x, object_y);
        }
        FindObject(object_x, object_y);
        // execute awaiting callbacks
        ros::spinOnce();
    }
    for (size_t i = 0; i < numberOfObjects; i++)
    {
        if(!objectsFound[i])
            ROS_INFO("Object at (%.2f,%.2f) is not found! It is located in an invalid place",objectsToFind[i][0], objectsToFind[i][1]);
    }
}

// callback function that gets the map
void map_update(const nav_msgs::OccupancyGrid new_map)
{
    map = new_map;
    map_valid = true;
}

void map_read_data(const nav_msgs::OccupancyGrid new_map)
{
    ROS_INFO("MAP READ DATA STARTED!");
    ROS_INFO("Map Height: %d, Map Width: %d", new_map.info.height,new_map.info.width);
    
    for (size_t i = 0; i < ((int) new_map.info.width) - 1; i++)
    {
        for (size_t j = 0; j < ((int) new_map.info.height - 1); j++)
        {
            //Fill the Array of Map with the objects
            int cellValue = (int) new_map.data.at(i*((int) new_map.info.width) + j);
            map_array[i][j] =  cellValue == -1 ? 1 : cellValue == 100? 3: 0;

        //    std::cout << map_array[i][j];
        }
       // std::cout << "\n";

    }

    ROS_INFO("MAP READ FINISHED");
    //ROS_INFO("Object is: %d", new_map.data.at(622*new_map.info.height+933));  
}

bool checkCell(int x, int y){

    map_get_data.getTopic();
    return false;
}

// callback function that is called when the target (of the search) has been found
void target_found(const std_msgs::Float32MultiArray target)
{
    float x = 0;
    float y = 0;

    get_position(x, y);
    ROS_INFO("Found the target at odom(%.2f, %.2f) - map(%.2f, %.2f)!", target.data[0], target.data[1], x, y);
    exit(0);
}

// send a goal to the robot
bool send_goal(float x, float y)
{
    // Move the robot with the help of an action client. Goal positions are
    // transmitted to the robot and feedback is given about the actual
    // driving state of the robot.

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    //actionlib::ActionClient<move_base_msgs::MoveBaseAction> myaction("move_base", true);
    while (!ac.waitForServer(ros::Duration(10.0)));

    move_base_msgs::MoveBaseGoal goal_msgs;

    goal_msgs.target_pose.header.seq = seq_goal;
    goal_msgs.target_pose.header.stamp = ros::Time::now();
    goal_msgs.target_pose.header.frame_id = "map";
    goal_msgs.target_pose.pose.position.x = x;
    goal_msgs.target_pose.pose.position.y = y;
    goal_msgs.target_pose.pose.position.z = 0;
//    goal_msgs.target_pose.pose.orientation.x = 0;
//    goal_msgs.target_pose.pose.orientation.y = 0;
//    goal_msgs.target_pose.pose.orientation.z = orientation;
    goal_msgs.target_pose.pose.orientation.w = 1;

    //ROS_INFO("Hello World!!");
    ++seq_goal;

    ac.sendGoal(goal_msgs);
    ac.waitForResult(ros::Duration(0.05));

    while (ac.getState() == actionlib::SimpleClientGoalState::PENDING);

    while (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE);

    while (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
        {
            ROS_ERROR("ABORTED");
            return false;
            break;
        }
    }

    ROS_INFO("Reached point x:%.2f, y:%.2f", x, y);
    return true;
}

// get the current position of the robot in x,y coordinates (meters)
bool get_position(float &x, float &y)
{
    tf::StampedTransform position;
    tf::TransformListener tf_listener;

    try
    {
        // sleeping to avoid errors
        // increase if the transformation lookup is not working
        ros::Duration(0.5).sleep();

        tf_listener.lookupTransform("map", "base_footprint", ros::Time(), position);

        x = position.getOrigin().x();
        y = position.getOrigin().y();
        orientation = position.getRotation().getZ();
        return true;
    }
    catch (tf::TransformException tx)
    {
        ROS_ERROR("Error while trying to get position: %s", tx.what());
        return false;
    }
}

// convert cells to meters
void cell2meter(int i, int j, float &x, float &y)
{
    x = (j - (float)map.info.width / 2) * map.info.resolution;
    y = (i - (float)map.info.height / 2) * map.info.resolution;
}

// convert meters to cells
void meter2cell(float x, float y, int &i, int &j)
{
    j = round(x / map.info.resolution) + map.info.width / 2;
    i = round(y / map.info.resolution) + map.info.height / 2;
}


//Checking the objects that the robot is searching for
void FindObject(float x, float y){
    for (size_t i = 0; i < numberOfObjects; i++)
    {
        if(abs(x - objectsToFind[i][0]) <= 1.5 && abs(y - objectsToFind[i][1]) <= 1.5 && objectsFound[i] == false)
        {
            objectsFound[i] = true;
            ROS_INFO("Object at (%.2f,%.2f) is found!", objectsToFind[i][0],objectsToFind[i][1]);
        }
    }
}

// Check the next robot search target position
bool check_next_point(float x, float y){

    
    int safe_region = 25; // 15 Pixels to check if there is an object there
    int px_x, px_y;
    bool valid_place = true;

    meter2cell(x, y, px_x, px_y);
    //ROS_INFO("Next Point: (%.2f,%.2f) -- Value: %d", x,y, map_array[px_x][px_y]);
    for (size_t i = 0; i <= safe_region; i++)
    {
//        ROS_INFO("Surrondings are (%d,%d,%d,%d)",map_array[px_x + i][px_y],map_array[px_x - i][px_y],map_array[px_x][px_y + i],map_array[px_x][px_y - i]);
        if(map_array[px_x + i][px_y] == 3 ||
           map_array[px_x + i][px_y] == 1 ||
           map_array[px_x - i][px_y] == 3 || 
           map_array[px_x - i][px_y] == 1 ||
           map_array[px_x][px_y + i] == 3 ||
           map_array[px_x][px_y + i] == 1 ||
           map_array[px_x][px_y - i] == 3 || 
           map_array[px_x][px_y - i] == 1 )
        {
            valid_place =  false;
            break;
        }  
    }

    for (size_t i = 0; i <= safe_region; i++)
    {
        visited_nodes[px_x + i][px_y] = valid_place? 1 : -1;
        visited_nodes[px_x - i][px_y] = valid_place? 1 : -1;
        visited_nodes[px_x][px_y + i] = valid_place? 1 : -1;
        visited_nodes[px_x][px_y - i] = valid_place? 1 : -1;
    }

   // ROS_INFO("Next point (%.2f,%.2f) is %s!",x,y, valid_place == 1? "Valid" : "Not Valid");
    return valid_place;
}