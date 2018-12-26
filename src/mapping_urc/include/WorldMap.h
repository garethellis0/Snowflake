// TODO: start of file comment

#pragma once

// STD Includes
#include <vector>
#include <memory>

// TODO: Make this include a proper `<>` style include.........
// External library includes
#include "../multi_resolution_graph/include/multi_resolution_graph/GraphNode.h"
#include "../multi_resolution_graph/include/multi_resolution_graph/GraphFactory.h"

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

    // Delete the copy constructor, because we can't nicely copy the
    // the multi_resolution_graph graph that this contains
    WorldMap(const WorldMap&) = delete;

    // Default the move constructor
    WorldMap(WorldMap&&) = default;
    WorldMap& operator=(WorldMap&& other)  = default;

    /**
     * Construct a WorldMap with a given size and offset
     *
     * @param size the x and y size of the map
     * @param x_offset the x offset of the origin (0,0) of the map
     * @param y_offset the y offset of the origin (0,0) of the map
     */
    WorldMap(double size, double x_offset, double y_offset);

    /**
     * Gets the cost at a given point
     *
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

    // TODO: Test me
    /**
     * Updates the CostCells of this map to match those of the given map
     *
     * @param source_map the map to copy over the cost cells from
     */
    void copyCostCellsFromMap(WorldMap &source_map);

    /**
     * Update all nodes in the given area to contain the given CostCell
     *
     * @param area the area in which we want to update all cost cells
     * @param new_value the new value we want the cost cells in the given area to take
     */
    void updateCostCellsInArea(multi_resolution_graph::Area<CostCell>& area, CostCell new_value);

private:

    // The multi_resolution_graph underlying this map
    std::shared_ptr<multi_resolution_graph::GraphNode<CostCell>> graph;

    // The offset of the origin of the underlying graph
    double x_offset;
    double y_offset;
};

