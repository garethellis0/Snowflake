/*
 * Created By: Gareth Ellis
 * Created On: June 1, 2017
 * Description: TODO
 */

#ifndef PATHFINDING_PATHFINDING_H
#define PATHFINDING_PATHFINDING_H

#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sb_utils.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <occupancy_grid_utils/shortest_path.h>
#include <occupancy_grid_utils/coordinate_conversions.h>
#include <geometry_msgs/PointStamped.h>
#include <boost/optional.hpp>
#include <nav_msgs/Path.h>

class Pathfinding {
public:
    Pathfinding(int argc, char **argv, std::string node_name);

private:
    /**
     * Callback function for the global map
     *
     * @param map the global map
     */
    void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& map);

    /**
     * Callback function for the current destination
     *
     * @param dest the current destination
     */
    void destinationCallBack(const geometry_msgs::PointStamped::ConstPtr& dest);

    ros::Subscriber map_subscriber; // Subscribes to the map topic
    ros::Subscriber destination_subscriber; // Subscribes to the current destination
    ros::Publisher path_publisher;
    ros::Publisher dest_point_publisher;

    std::string global_frame; // The frame of the map
    std::string base_frame; // The frame of the robot

    double inflation_radius; // The radius to inflate obstacles in the map by
    int path_lookahead; // How many cells into the path to look for the robot's next waypoints

    geometry_msgs::Point destination; // The current destination

    // Used for looking up transforms
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener* tfListener;
};
#endif //PATHFINDING_PATHFINDING_H
