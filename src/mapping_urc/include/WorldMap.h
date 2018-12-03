// TODO: start of file comment

#pragma once

// STD Includes
#include <vector>
#include <memory>

// TODO: Make this include a proper `<>` style include.........
// External library includes
#include "../multi_resolution_graph/include/multi_resolution_graph/GraphNode.h"

// TODO: This is really just a placeholder structure... probably re-name and redo...
// TODO: This should support the concept of not knowing what the risk/cost is
struct CostCell {
    double cost;
};


// TODO: Descriptive class comment here
class WorldMap {
public:
    // Delete the default constructor
    WorldMap() = delete;

    /**
     * Construct a WorldMap from a given Graph and offset
     * @param graph A graph representing what we know about the world
     * @param x_offset_meters The offset from 0 that the graph origin is at in the x-direction (in meters)
     * @param y_offset_meters The offset from 0 that the graph origin is at in the y-direction (in meters)
     */
    WorldMap(std::shared_ptr<multi_resolution_graph::GraphNode<CostCell>> graph, double x_offset_meters, double y_offset_meters);

    /**
     * Gets the cost at a given point
     * @param x TODO?
     * @param y TODO?
     * @return TODO?
     */
    CostCell getCostAtPoint(double x, double y);

    /**
     * @return the minimum X value of the graph in this map
     */
    double getGraphMinX();

    /**
     * @return the maximum X value of the graph in this map
     */
    double getGraphMaxX();

    /**
     * @return the minimum Y value of the graph in this map
     */
    double getGraphMinY();

    /**
     * @return the maximum Y value of the graph in this map
     */
    double getGraphMaxY();
};

