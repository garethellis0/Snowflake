/*
 * Created By: Gareth Ellis
 * Created On: Nov. 10th, 2018
 * Description: A node that creates a risk map of the world based on given areas
 *              with given risk
 */

#pragma once

// STD Includes
#include <iostream>

// ROS Includes
#include <ros/ros.h>

// Snowbots Includes
#include "Mapper.h"
#include <sb_utils.h>
#include <mapping_msgs_urc/RiskAreaArray.h>

class MapperNode {
public:
    MapperNode(int argc, char **argv, std::string node_name);

private:
    /**
     * Callback function for when a new RiskAreaArray is received
     *
     * @param msg the RiskAreaArray received in the callback
     */
    void riskAreaArrayCallback(
            const mapping_msgs_urc::RiskAreaArray::ConstPtr &msg);

    // The subscriber to all the risk area array messages
    ros::Subscriber risk_area_array_subscriber;

    // The mapper that handles all the map updates and produces a nice cost-map
    // for us
    Mapper mapper;
};
