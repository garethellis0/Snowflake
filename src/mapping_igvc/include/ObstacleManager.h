/*
 * Created By: Gareth Ellis
 * Created On: January 9, 2018
 * Description: TODO
 */

#ifndef MAPPING_IGVC_OBSTACLEMANAGER_H
#define MAPPING_IGVC_OBSTACLEMANAGER_H

// C++ STD includes
#include <vector>

// ROS Includes
#include <ecl/geometry.hpp>

// TODO: Should we have a general geometry package for these lines? Seems like we use them a lot (in IARRC, etc)...
// TODO: (perhaps these could go in sb_utils

// TODO: Should this class be in it's own file? (probably...)
class Cone {
public:
    /**
     * Create a Cone with default x,y position and radius
     */
    Cone() : Cone(0,0,0) {};

    /**
     * Create a cone with a given x,y position and radius
     * @param x
     * @param y
     * @param radius
     */
    Cone(double x, double y, double radius)
    : x(x),
      y(y),
      radius(radius)
    {}

    double x;
    double y;
    double radius;

};

// Implementation of the `==` operator for Cone
inline bool operator==(const Cone& lhs, const Cone& rhs){
    return (
            lhs.x == rhs.x &&
            lhs.y == rhs.y &&
            lhs.radius == rhs.radius
    );
}


class ObstacleManager {
public:
    template <int T>
    using Polynomial = ecl::Polynomial<T>;
    using Spline = ecl::CubicSpline;

    /**
     * We delete the default constructor to force the user of this class to use
     * one of the other provided constructors
     */
    ObstacleManager() = delete;

    /**
     * Creates a ObstacleManager
     * @param cone_merging_tolerance the minimum distance between the center of
     * two cones for them to be considered different from each other
     * (and so not merged)
     * @param line_merging_tolerance TODO
     */
    explicit ObstacleManager(double cone_merging_tolerance, double line_merging_tolerance);

    /**
     * Add a given cone to our map of the world
     * @param cone the cone to add
     */
    void addObstacle(Cone cone);

    /**Degree
     * Add a given line to our map of the world
     * @param line the line to add
     */
    template <int T>
    void addObstacle(Polynomial<T> line, double start_point, double end_point);

    /**
     * Get all the cones in our world
     * @return a list of all known cones in our world
     */
    std::vector<Cone> getConeObstacles();

private:

    /**
     * Finds the minimum distance between a spline line and polynomial line
     * @param spline_line a spline line
     * @param poly_line a polynomial line
     * @return the minimum distance between the spline and polynomial lines
     */
    template <int T>
    double distanceBetweenSplineAndPolynomial(Spline spline, Polynomial<T> poly_line);

    /**
     * Merges the new line into the current line to come up with an updated line
     *
     * Prioritizes new_line over current_line when merging
     *
     * @param current_line our currently known line
     * @param new_line the newly found line
     * @return the merged line
     */
    template <int T>
    Spline updateLineWithNewLine(Spline current_line, Polynomial<T> new_line);

    template <int T>
    Spline splineFromPolynomial(Polynomial<T> polynomial);

    // the minimum distance between the center of two cones for them to be
    // considered different from each other (and so not merged)
    double cone_merging_tolerance;

    // the minimum distance between two lines for them to be
    // considered different from each other (and so not merged)
    double line_merging_tolerance;

    // all known cones in our world
    std::vector<Cone> cones;

    // all known lines in our world
    std::vector<Spline> lines;
};

#include "ObstacleManager.tpp"

#endif //MAPPING_IGVC_OBSTACLEMANAGER_H
