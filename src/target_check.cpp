#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float32MultiArray.h>
#include "tf/transform_listener.h"
#include <visualization_msgs/Marker.h>

// function declarations
void callback(const nav_msgs::OdometryConstPtr &ptr);
// callback function that gets the map
void map_update(const nav_msgs::OccupancyGrid new_map);

// marker for visualizing the object
void add_marker_to_rviz(float &x_pose, float &y_pose, int &marker_id, ros::Publisher &marker_pub);

// global variables
ros::Publisher pub;
float xTarget, yTarget;
bool map_valid = false;

// main function
int main(int argc, char **argv)
{

    // read target parameters
    if (argc < 5)
    {
        ROS_ERROR("Cannot read coordinates of target!");
        ROS_INFO("Setting target to x:0, y:0");
        xTarget = 0;
        yTarget = 0;
    }
    else
    {
        xTarget = atof(argv[1]);
        yTarget = atof(argv[2]);
        ROS_INFO("Setting target to x:%.2f, y:%2f", xTarget, yTarget);

        //here transform the map coordinates to odom coordinates to be uniform with the odom coordinates in the callback function
        //xTarget = xTarget + 1.194684577112486;
        //yTarget = yTarget - 23.335950614726176;
    }

    // initialize node
    ros::init(argc, argv, "target_check");
    ros::NodeHandle *nh = new ros::NodeHandle();

    // subscribe to the odometry topic
    ros::Subscriber sub = nh->subscribe("/odom", 10, callback);

    // subscribe to the map topic
    ros::Subscriber map_sub = nh->subscribe("/map", 1, map_update);
    // wait for map
    while (map_valid == false)
    {
        ros::Duration(1).sleep();
        ros::spinOnce(); // check for map update
    }

    /*****************************************************************/
    // RVIZ Markers
    ros::Publisher marker_obj_pub = nh->advertise<visualization_msgs::Marker>("visualization_marker", 10, true);
    /*****************************************************************/

    // Add object marker to RViz
    int marker_id = 10000;
    add_marker_to_rviz(xTarget, yTarget, marker_id, marker_obj_pub);

    // advertise found topic
    pub = nh->advertise<std_msgs::Float32MultiArray>("/found", 1000);

    ros::Duration(1).sleep();

    tf::TransformListener tf_listener;
    tf::StampedTransform position;

    while (nh->ok())
    {
        try
        {
            tf_listener.lookupTransform("map", "odom", ros::Time(), position);
            break;
        }
        catch (tf::LookupException ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(10).sleep();
            ros::spinOnce();
        }
    }
    xTarget = position.getOrigin().x() - xTarget;
    yTarget = position.getOrigin().y() - yTarget;
    ROS_INFO("Setting target (odom) to x:%.2f, y:%2f", xTarget, yTarget);
    ROS_INFO("TF map - odom x:%.2f, y:%2f", position.getOrigin().x(), position.getOrigin().y());
    // keep running and check for odometry updates

    ros::spin();

    return 0;
}

// callback function for odometry updates
void callback(const nav_msgs::OdometryConstPtr &ptr)
{
    // get position from odometry
    nav_msgs::Odometry odom = *ptr.get();
    float x, y; // units are m, not grid cells
    x = odom.pose.pose.position.x;
    y = odom.pose.pose.position.y;

    // if target is in range, publish found message
    if (x < xTarget + 1.5 && x > xTarget - 1.5)
    {
        if (y < yTarget + 1.5 && y > yTarget - 1.5)
        {
            std_msgs::Float32MultiArray target;
            target.data.push_back(x);
            target.data.push_back(y);
            pub.publish(target);
        }
    }
}

// callback function that gets the map
void map_update(const nav_msgs::OccupancyGrid new_map)
{
    map_valid = true;
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
    uint32_t shape = visualization_msgs::Marker::CUBE;

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
    marker.scale.x = 0.60;
    marker.scale.y = 0.60;
    marker.scale.z = 0.60;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    ROS_INFO("marker  x:%0.2f,y:%0.2f", marker.pose.position.x, marker.pose.position.y);
    ROS_INFO("marker_id:%i ", marker.id);

    marker_pub.publish(marker);
}