// TODO: Start of file comment

// Snowbots Includes
#include <Mapper.h>

// External Library Includes
// TODO: Fix and change to proper `<>` syntax
#include "../multi_resolution_graph/include/multi_resolution_graph/GraphFactory.h"
#include "../multi_resolution_graph/include/multi_resolution_graph/Polygon.h"

using namespace multi_resolution_graph;

void Mapper::addAreaWithRisk(const sb_geom::Polygon2D& area, const double& risk) {
    risk_areas.emplace_back(
            RiskArea {
                area, risk
            });
}

void Mapper::createNextMap() {
    // Generate a map without any values in it, but with the required resolution everywhere we want it
    WorldMap new_map = generateEmptyMap();

    // Carry over risk scores from the previous map
    new_map.copyCostCellsFromMap(prev_map);

    // Update the map with all the new risk areas we've seen since we last generated a map
    updateMapWithRiskAreas(new_map, risk_areas);

    // Clear the risk areas
    risk_areas.clear();

    // Save our new WorldMap
    prev_map = std::move(new_map);
}

WorldMap
Mapper::generateEmptyMap() {
    // Figure out what the bounds for the graph should be
    GraphBounds bounds = getNewGraphBounds();

    // Set the top-level graph scale
    // The graph is a square, so make sure we can fit both the x and y
    double dx = bounds.max_x - bounds.min_x;
    double dy = bounds.max_y - bounds.min_y;
    double map_size = std::max(dx, dy);

    GraphFactory<CostCell> graph_factory;
    graph_factory.setGraphScale(map_size);

    // TODO: Set the top level resolution to some default value

    // TODO: set the resolution in some area around us to some value

    return WorldMap(map_size, bounds.min_x, bounds.min_y);
}

void Mapper::updateMapWithRiskAreas(WorldMap &map,
                                    std::vector<RiskArea> areas) {
    for (auto& areaAndScore : areas) {
        sb_geom::Polygon2D& area = areaAndScore.area;
        const double& score = areaAndScore.score;

        // Convert our area into one the multi-resolution graph can understand
        std::vector<multi_resolution_graph::Coordinates> coordinates;
        for(auto boundary_point : area.getBoundaryPoints()) {
            coordinates.emplace_back((Coordinates){boundary_point.x(), boundary_point.y()});
        }

        multi_resolution_graph::Polygon<CostCell> polygon_area(coordinates);
        CostCell new_cost({score});

        map.updateCostCellsInArea(polygon_area, new_cost);
    }

}

Mapper::GraphBounds Mapper::getNewGraphBounds() {
    // Figure out the min/max x and y from:
    //  - the last WorldMap we generated
    //  - all new risk areas
    std::vector<double> x_values = {prev_map.getMinX(), prev_map.getMaxX()};
    std::vector<double> y_values = {prev_map.getMinY(), prev_map.getMaxY()};
    for(const auto& area_with_score : risk_areas) {
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

