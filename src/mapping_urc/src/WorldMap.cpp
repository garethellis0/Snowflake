// TODO: start of file comment

#include <WorldMap.h>

#include "WorldMap.h"

using namespace multi_resolution_graph;

WorldMap::WorldMap(
        double size, double x_offset, double y_offset) :
        x_offset(x_offset),
        y_offset(y_offset) {

    // Setup the underlying graph
    graph_factory.setGraphScale(size);
    graph = graph_factory.createGraph();
}

void WorldMap::copyCostCellsFromMap(WorldMap &source_map) {
    // Get all the base nodes for this map
    auto nodes = graph->getAllSubNodes();

    // For each node, find it's equivalent on the source map and
    // copy over the values
    for (const auto& node : nodes) {
        // TODO: We should really have `setContainedValue` and `getContainedValue()` instead of just a reference like this?
        Coordinates coord = node->getCoordinates();
        node->containedValue() = source_map.getCostAtPoint(coord.x, coord.y);
    }
}

