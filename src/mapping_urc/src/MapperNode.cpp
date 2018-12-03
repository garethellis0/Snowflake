/*
 * Created By: Gareth Ellis
 * Created On: Nov. 10th, 2018
 * Description: A node that creates a risk map of the world based on given areas
 *              with given risks
 */

// Snowbots Includes
#include "MapperNode.h"
#include <sb_geom/Polygon2D.h>

using namespace mapping_msgs_urc;

MapperNode::MapperNode(int argc, char **argv, std::string node_name) {
    // Setup NodeHandles
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Setup Subscriber(s)
    std::string topic_to_subscribe_to = "risk_areas";
    int queue_size                    = 10;
    risk_area_array_subscriber        = nh.subscribe(
    topic_to_subscribe_to, queue_size, &MapperNode::riskAreaArrayCallback, this);
}

void MapperNode::riskAreaArrayCallback(const
                                       RiskAreaArray::ConstPtr &msg) {
    for (RiskArea risk_area : msg->areas){
        sb_geom::Polygon2D polygon2D(risk_area.area);
        mapper.addAreaWithRisk(polygon2D, risk_area.score.data);
    }
}

