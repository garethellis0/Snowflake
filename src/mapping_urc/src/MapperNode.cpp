/*
 * Created By: Gareth Ellis
 * Created On: Nov. 10th, 2018
 * Description: A node that creates a risk map of the world based on given areas
 *              with given risks
 */

// Snowbots Includes
#include "MapperNode.h"
#include <sb_geom/Polygon2D.h>
#include <nav_msgs/OccupancyGrid.h>

using namespace mapping_msgs_urc;

MapperNode::MapperNode(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup Subscriber(s)
    std::string risk_area_topic_name = "risk_areas";
    int queue_size                    = 10;
    risk_area_array_subscriber        = nh.subscribe(
    risk_area_topic_name, queue_size, &MapperNode::riskAreaArrayCallback, this);

    // Setup Publisher(s)
    std::string topic = private_nh.resolveName("publish_topic");
    std::string occ_grid_topic_name = "occ_grid";
    occ_grid_publisher = nh.advertise<nav_msgs::OccupancyGrid>(
            occ_grid_topic_name, queue_size);
}

void MapperNode::riskAreaArrayCallback(const
                                       RiskAreaArray::ConstPtr &msg) {
    for (RiskArea risk_area : msg->areas){
        sb_geom::Polygon2D polygon2D(risk_area.area);
        mapper.addAreaWithRisk(polygon2D, risk_area.score.data);
    }

    // Generate a new map based on the new risk areas
    mapper.createNextMap();

    // Publish the map as an occupancy grid
    occ_grid_publisher.publish(mapper.getMapAsOccGrid());
}

