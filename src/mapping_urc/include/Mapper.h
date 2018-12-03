// TODO: start of file comment

#pragma once

// STD Includes
#include <vector>

// TODO: Make this include a proper `<>` style include.........
// External library includes
#include "../multi_resolution_graph/include/multi_resolution_graph/GraphNode.h"

// Snowbots Includes
#include <sb_geom/Polygon2D.h>
#include "WorldMap.h"

// TODO: Comment here
struct AreaWithScore {
    // TODO: Comment here
    sb_geom::Polygon2D area;

    // TODO: Comment here
    double score;
};

// TODO: Descriptive class comment here
class Mapper {
public:
    /**
     * Add a given area with a given score to the map
     *
     * @param area The area
     * @param score The score for the given area
     */
    void addAreaWithRisk(const sb_geom::Polygon2D& area, const double& risk);

    /**
     * Gets the current map, with all the added areas with given risks
     * @return the current map, with all the added areas with given risks
     */
    void createNextMap();

    // TODO: Should this be private
    /**
     * Generates a new graph big enough to contain the previous graph and all new risk areas. Also sets the resolutions on the graph appropriately.
     * @return TODO
     */
    std::shared_ptr<multi_resolution_graph::GraphNode<CostCell>> generateEmptyGraph();

private:
    // TODO: Comment here
    struct GraphBounds {
        double min_x, max_x, min_y, max_y;
    };

    /**
     * Calculates the bounds for the new graph
     *
     * The bounds are such that they will fit both the previous map and
     * and any new Risk areas we've seen since we generated the previous map
     *
     * @return The bounds for a new graph
     */
    GraphBounds getNewGraphBounds();

    // The areas with associated score we're seen so far
    std::vector<AreaWithScore> areas_with_scores;

    // The last map we generated
    std::shared_ptr<WorldMap> prev_map;
};
