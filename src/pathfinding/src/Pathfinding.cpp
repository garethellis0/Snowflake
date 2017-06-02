/*
 * Created By: Gareth Ellis
 * Created On: June 1, 2017
 * Description: TODO
 */

#include <Pathfinding.h>

Pathfinding::Pathfinding(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Get Params
    SB_getParam(nh, "global_frame", global_frame, (std::string)"odom_combined");
    SB_getParam(nh, "base_frame", base_frame, (std::string)"base_link");
    SB_getParam(nh, "inflation_radius", inflation_radius, 0.25);
    SB_getParam(nh, "path_lookahead", path_lookahead, 20);

    // Setup Subscriber(s)
    std::string map_topic = "map";
    int refresh_rate = 10;
    map_subscriber = nh.subscribe(map_topic, refresh_rate, &Pathfinding::mapCallBack, this);
    std::string destination_topic = "/gps_manager/current_waypoint";
    destination_subscriber = nh.subscribe(destination_topic, refresh_rate,
                                          &Pathfinding::destinationCallBack, this);


    // Setup Publisher(s)
    std::string path_topic = private_nh.resolveName("path");
    uint32_t queue_size = 3;
    path_publisher = private_nh.advertise<nav_msgs::Path>(path_topic, queue_size);
    std::string dest_point_topic = private_nh.resolveName("current_waypoint");
    dest_point_publisher = private_nh.advertise<geometry_msgs::PointStamped>
            (dest_point_topic, queue_size);

    //Initialize TFs
    tfListener = new tf2_ros::TransformListener(tfBuffer);
}

void Pathfinding::mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr &map) {
    // Get the current position from the transform tree
    geometry_msgs::Point current_location;
    try {
        geometry_msgs::TransformStamped tfStamped = tfBuffer.lookupTransform(
                global_frame, base_frame, ros::Time(0), ros::Duration(1.0));
        // Get our current heading and location from the global_frame <-> base_frame tf
        current_location.x = tfStamped.transform.translation.x;
        current_location.y = tfStamped.transform.translation.y;
    } catch (tf2::LookupException e) {
        // If we can't lookup the tf, then warn the user and tell robot to stop
        ROS_ERROR_STREAM("Could not lookup tf between " <<
                         global_frame << " and " << base_frame);
        return;
    }

    // Check that we're within the bounds of the map
    if (!occupancy_grid_utils::withinBounds(map->info, current_location)) {
        ROS_WARN_STREAM("Error, point (" << current_location.x << ", " << current_location.y
                        << ") not within map bounds");
        return;
    }

    // Inflate the map
    nav_msgs::OccupancyGrid::Ptr inflatedMapPtr = occupancy_grid_utils::inflateObstacles(
            *map, inflation_radius);

    // Get out current position and destination as cells
    occupancy_grid_utils::Cell curr_cell = occupancy_grid_utils::pointCell(map->info, current_location);
    occupancy_grid_utils::Cell dest_cell = occupancy_grid_utils::pointCell(map->info, destination);

    // Compute the shortest path to the destination
    boost::optional<occupancy_grid_utils::AStarResult> path_result =
            occupancy_grid_utils::shortestPathAStar(*inflatedMapPtr, curr_cell, dest_cell);

    // Check that we computed the path succesfully
    if (!path_result){
        ROS_WARN_STREAM("Could not compute path");
        return;
    }

    // Convert the path to nav_msgs/Path and publish it
    occupancy_grid_utils::Path path = path_result->first;
    nav_msgs::Path ros_path;
    ros_path.header.frame_id = global_frame;
    for (auto cell : path){
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = global_frame;
        pose_stamped.pose.position.x = cell.x;
        pose_stamped.pose.position.y = cell.y;
    }
    path_publisher.publish(ros_path);

    // The goal cell is either the cell at path[path_lookahead] or
    // the end of the path, whichever is lesser
    long goal_cell_index = std::min((long)path.size()-1, (long)path_lookahead);
    occupancy_grid_utils::Cell goal_cell = path[goal_cell_index];

    // Publish the goal cell as a StampedPoint
    geometry_msgs::Point goal_cell_point = occupancy_grid_utils::cellCenter(map->info, goal_cell);
    geometry_msgs::PointStamped goal_cell_stamped_point;
    goal_cell_stamped_point.header.frame_id = global_frame;
    goal_cell_stamped_point.point = goal_cell_point;

    dest_point_publisher.publish(goal_cell_stamped_point);
}

void Pathfinding::destinationCallBack(const geometry_msgs::PointStamped::ConstPtr &dest) {
    destination = dest->point;
}
