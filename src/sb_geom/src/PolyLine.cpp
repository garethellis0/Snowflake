#include <vector>

/*
 * Created By: Gareth Ellis
 * Created On: January 27, 2018
 * Description: TODO
 */

#include "sb_geom/PolyLine.h"

namespace sb_geom {

    PolyLine::PolyLine():
            coefficients({})
    {}

    PolyLine::PolyLine(std::vector<double>& coefficients)
    {
        setCoefficients(coefficients);
    }

    void PolyLine::setCoefficients(std::vector<double> coefficients) {
        this->coefficients = coefficients;
    }

    std::vector<double> PolyLine::getCoefficients() {
        return coefficients;
    }

    int PolyLine::getDegree() {
        return coefficients.size();
    }

}
