/*
 * Created By: Gareth Ellis
 * Created On:  Nov. 11th, 2018
 * Description: A class representing a 2D Polygon
 */

#include "sb_geom/Polygon2D.h"

using namespace sb_geom;

double Polygon2D::getMinX() {
    return std::min_element(boundary_points.begin(), boundary_points.end(),
            [&](auto p1, auto p2) {
                return (p1.x() < p2.x());
            })->x();
}

double Polygon2D::getMaxX() {
    return std::max_element(boundary_points.begin(), boundary_points.end(),
                            [&](auto p1, auto p2) {
                                return (p1.x() < p2.x());
                            })->x();
}

double Polygon2D::getMinY() {
    return std::min_element(boundary_points.begin(), boundary_points.end(),
                            [&](auto p1, auto p2) {
                                return (p1.y() < p2.y());
                            })->y();
}

double Polygon2D::getMaxY() {
    return std::max_element(boundary_points.begin(), boundary_points.end(),
                            [&](auto p1, auto p2) {
                                return (p1.y() < p2.y());
                            })->y();
}

