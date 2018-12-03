// TODO: Start of file comment

// Snowbots Includes
#include <Mapper.h>

// External Library Includes
// TODO: Fix and change to proper `<>` syntax
#include "../multi_resolution_graph/include/multi_resolution_graph/GraphFactory.h"
#include "../multi_resolution_graph/include/multi_resolution_graph/Polygon.h"

using namespace multi_resolution_graph;

void Mapper::addAreaWithRisk(const sb_geom::Polygon2D& area, const double& risk) {
    areas_with_scores.emplace_back(
            AreaWithScore {
                area, risk
            }
            );
}

void Mapper::createNextMap() {
    // Generate the graph without any values in it, but with the required resolution everywhere we want it
    std::shared_ptr<GraphNode<CostCell>> graph = generateEmptyGraph();

    // TODO: Set resolution where we want it (probably just around robot?)

    // For every node in the current graph, find the risk score for it in by finding its location in the previous graph and copying it over
    auto nodes = graph->getAllSubNodes();
    for (const auto& node : nodes) {
        // TODO: We should really have `setContainedValue` and `getContainedValue()` instead of just a reference like this?
        Coordinates coord = node->getCoordinates();
        node->containedValue() = prev_map.getCostAtPoint(coord.x, coord.y);
    }

    // Update the risk scores for each node based on the Risk Areas we've seen since we last generated a map
    for (auto& areaAndScore : areas_with_scores) {
        sb_geom::Polygon2D& area = areaAndScore.area;
        const double& score = areaAndScore.score;

        // Convert our area into one the multi-resolution graph can understand
        std::vector<multi_resolution_graph::Coordinates> coordinates;
        for(auto boundary_point : area.getBoundaryPoints()) {
            coordinates.emplace_back((Coordinates){boundary_point.x(), boundary_point.y()});
        }
        multi_resolution_graph::Polygon<CostCell> polygon_area(coordinates);
        for(auto& node : graph->getAllNodesInArea(polygon_area)) {
            node->containedValue().cost = score;
        }
    }

    // TODO: x and y offsets for Worldmap?
    // Save this WorldMap
    prev_map = std::make_shared<WorldMap>(graph, 0.0, 0.0);
}

std::shared_ptr<multi_resolution_graph::GraphNode<CostCell>>
Mapper::generateEmptyGraph() {
    // Figure out what the bounds for the graph should be
    GraphBounds bounds = getNewGraphBounds();

    // Set the top-level graph scale
    // The graph is a square, so make sure we can fit both the x and y
    double graph_scale = std::max(bounds.max_x - bounds.min_x, bounds.max_y - bounds.min_y);
    GraphFactory<CostCell> graph_factory;
    graph_factory.setGraphScale(graph_scale);

    // TODO: Set the top level resolution to some default value

    // TODO: set the resolution in some area around us to some value

    return graph_factory.createGraph();
}

Mapper::GraphBounds Mapper::getNewGraphBounds() {
    // Figure out the min/max x and y from:
    //  - the last WorldMap we generated
    //  - all new risk areas
    std::vector<double> x_values = {prev_map.getGraphMinX(), prev_map.getGraphMaxX()};
    std::vector<double> y_values = {prev_map.getGraphMinY(), prev_map.getGraphMaxY()};
    for(const auto& area_with_score : areas_with_scores) {
        sb_geom::Polygon2D area = area_with_score.area;
        x_values.emplace_back(area.getMinX());
        x_values.emplace_back(area.getMaxX());
        y_values.emplace_back(area.getMinY());
        y_values.emplace_back(area.getMaxY());
    }
    GraphBounds graph_bounds;
    graph_bounds.min_x = *std::min_element(x_values.begin(), x_values.end());
    graph_bounds.max_x = *std::max_element(x_values.begin(), x_values.end());
    graph_bounds.min_y = *std::min_element(y_values.begin(), y_values.end());
    graph_bounds.max_y = *std::max_element(y_values.begin(), y_values.end());
    return graph_bounds;
}

