#include <numeric>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <std_msgs/Float32MultiArray.h>
#include <ros/service_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "tf/transform_listener.h"
#include <visualization_msgs/Marker.h>

// function declarations
void map_update(const nav_msgs::OccupancyGrid search_map);
void target_found(std_msgs::Float32MultiArray);
bool send_goal(float x, float y);
bool get_position(float &x, float &y);

// functions
void meter2cell2(float x, float y, int &width, int &height);
void cell2meter2(int i, int j, float &x, float &y);

void add_marker_to_rviz(float &x_pose, float &y_pose, int &marker_id, ros::Publisher &marker_pub);
bool check_area_around_point(std::vector<std::vector<int>> &mat, int p, int q, int &square_side);

// global variables
nav_msgs::OccupancyGrid map;
nav_msgs::OccupancyGrid map_scanned;

ros::Subscriber found_sub, map_sub;
ros::Publisher map_pub;

bool map_valid = false;
const int range = 1; // range of target detector in meters
int seq_goal = 0;    // sequence id of goal

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

    // publish map with scanned area
    map_pub = n->advertise<nav_msgs::OccupancyGrid>("/map", 50, true);
    // merged_map_publisher_ = n.advertise<nav_msgs::OccupancyGrid>("merged_map_topic", 50, true);

    /*****************************************************************/
    // RVIZ Markers
    ros::Publisher marker_pub = n->advertise<visualization_msgs::Marker>("visualization_marker", 100, true);

    /*****************************************************************/

    // wait for map
    while (map_valid == false)
    {
        ros::Duration(1).sleep();
        ros::spinOnce(); // check for map update
    }

    // implement your search here...
    /********************************************************************************/
    /* Map Corners
    at star
    x: -15.1081619263
    y: -6.741086483

    at circle
    x: 13.0703678131
    y: -7.70404243469

    at empty room
    x: 13.0703678131
    y: -7.70404243469

    at triangle
    x: 13.2215995789
    y: 18.0037002563
    */

    int i = 0;
    int j = 0;

    float x = 0;
    float y = 0;

    /********************************PROCESS THE MAP*************************************/
    // used to check area of 10x10 cells around each location opt 12
    int loc_check_threshold = 11;
    // to compansate offset given in the map opt 12
    int map_offset = 12;
    // difference between cells to check and to visit opt 35
    int loc_diff = 35;

    // unique marker for each
    int marker_id = 0;

    // vector to store the x and y locations
    std::vector<std::pair<int, int>> valid_points;

    // instantiate vector object of type `std::vector<int>` and
    // use `push_back()` function to resize it
    std::vector<std::vector<int>> matrix;

    auto w = map.info.width;  // x M
    auto h = map.info.height; // y N

    // Map to 2D Matrix
    for (int i = 0; i < w; i++)
    {
        // construct a vector of ints with the given default value
        std::vector<int> v;
        for (int j = 0; j < h; j++)
        {
            v.push_back(map.data[j * w + i]);
        }

        // push back above one-dimensional vector
        matrix.push_back(v);
    }

    ROS_INFO("Map to 2D Matrix UPDATED");

    map_scanned = map;

    for (int i = 0; i < w; i += loc_diff)
    {
        for (int j = 0; j < h; j += loc_diff)
        {
            map_scanned.data[j * w + i] = matrix[i][j] * 1;
        }
    }

    ROS_INFO("2D Matrix to Map UPDATED");
    // Pub to new map to topic /map

    map_pub.publish(map_scanned);

    ROS_INFO("MAP UPDATED");

    /******************Get Valid Points To Visit and visualize it*************************/

    for (int i = 0; i < w; i += loc_diff)
    {
        for (int j = 0; j < h; j += loc_diff)
        {
            // check each area around the index if its valid location to visit later
            if (check_area_around_point(matrix, i, j, loc_check_threshold))
            {
                cell2meter2(i - map_offset, j - map_offset, x, y);

                // added the valid point to visited later
                valid_points.push_back({x, y});

                ROS_INFO("valid_point  x:%0.2f,y:%0.2f", x, y);

                // add red markers to rviz at the valid location
                add_marker_to_rviz(x, y, marker_id, marker_pub);

                marker_id += 1; //unique id
            }
        }
    }

    ROS_INFO("Done Getting the Points and adding the RViz markers");

    // ROS_INFO("valid_points[50]i:%d", valid_points[50].first);
    // ROS_INFO("valid_points[50]j:%d", valid_points[50].second);

    /********************send the robot to valid locations********************************/

    // while (ros::ok())
    // {
    ros::Rate loop_rate(5); // 5Hz

    for (int index = 0; index < valid_points.size(); ++index)
    {
        auto x = valid_points[index].first;
        auto y = valid_points[index].second;

        ROS_INFO("Sending robot to x:%.2f, y:%.2f", x, y);

        if (send_goal(x, y) == false)
        {
            ROS_ERROR("Could not reach goal x:%.2f, y:%.2f", x, y);
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    // }
    
    // x = 8;
    // y = 15;

    // ROS_INFO("Sending robot to x:%.2f, y:%.2f", x, y);

    // if (send_goal(x, y) == false)
    // {
    //     ROS_ERROR("Could not reach goal x:%.2f, y:%.2f", x, y);
    // }
    // ros::spinOnce;
    /********************************************************************************/

    return 0;
}

// callback function that gets the map
void map_update(const nav_msgs::OccupancyGrid new_map)
{
    map = new_map;
    map_valid = true;
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
    while (!ac.waitForServer(ros::Duration(10.0)))
        ;

    move_base_msgs::MoveBaseGoal goal_msgs;

    goal_msgs.target_pose.header.seq = seq_goal;
    goal_msgs.target_pose.header.stamp = ros::Time::now();
    goal_msgs.target_pose.header.frame_id = "map";
    goal_msgs.target_pose.pose.position.x = x;
    goal_msgs.target_pose.pose.position.y = y;
    goal_msgs.target_pose.pose.position.z = 0;
    goal_msgs.target_pose.pose.orientation.x = 0;
    goal_msgs.target_pose.pose.orientation.y = 0;
    goal_msgs.target_pose.pose.orientation.z = 0;
    goal_msgs.target_pose.pose.orientation.w = 1;

    ++seq_goal;

    ac.sendGoal(goal_msgs);
    ac.waitForResult(ros::Duration(7.0));

    while (ac.getState() == actionlib::SimpleClientGoalState::PENDING)
        ;

    while (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE)
        ;

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

        return true;
    }
    catch (tf::TransformException tx)
    {
        ROS_ERROR("Error while trying to get position: %s", tx.what());
        return false;
    }
}

/* CUSTOM FUNCTION */

// convert cells to meters
void cell2meter2(int i, int j, float &x, float &y)
{
    x = (i - (float)map.info.width / 2) * map.info.resolution;
    y = (j - (float)map.info.height / 2) * map.info.resolution;
}

// convert meters to cells
void meter2cell2(float x, float y, int &width, int &height)
{
    width = round(x / map.info.resolution) + map.info.width / 2;
    height = round(y / map.info.resolution) + map.info.height / 2;
}

void add_marker_to_rviz(float &x_pose, float &y_pose, int &marker_id, ros::Publisher &marker_pub)
{

    visualization_msgs::Marker marker;

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    // marker.ns = "marker_i_" + std::to_string(i)+"_j_"+ std::to_string(j);
    marker.ns = "search";
    marker.id = marker_id;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::SPHERE;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x_pose;
    marker.pose.position.y = y_pose;
    marker.pose.position.z = 0.5;
    marker.pose.orientation = tf::createQuaternionMsgFromYaw(-1.57);
    // 00029     marker.pose.orientation.x = 0.0;
    // 00030     marker.pose.orientation.y = 0.0;
    // 00031     marker.pose.orientation.z = 0.0;
    // 00032     marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.30;
    marker.scale.y = 0.30;
    marker.scale.z = 0.30;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    ROS_INFO("marker  x:%0.2f,y:%0.2f", marker.pose.position.x, marker.pose.position.y);
    ROS_INFO("marker_id:%i ", marker.id);

    marker_pub.publish(marker);
}

bool check_area_around_point(std::vector<std::vector<int>> &mat, int p, int q, int &square_side)
{
    int sum_of_elems = 0;

    for (int i = 0; i < square_side; i++)
    {
        for (int j = 0; j < square_side; j++)
        {
            sum_of_elems += mat[p + i][q + j];

            if (p > square_side)
            {
                sum_of_elems += mat[p - i][q + j];
            }
            if (q > square_side)
            {
                sum_of_elems += mat[p + i][q - j];
            }
            if (p > square_side && q > square_side)
            {
                sum_of_elems += mat[p - i][q - j];
            }

            if (!sum_of_elems == 0)
            {
                return false;
            }
        }
    }

    if (sum_of_elems == 0)
    {
        return true;
    }

    return false;
}