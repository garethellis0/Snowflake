/*
 * Created By: Gareth Ellis
 * Created On:  February 24th, 2018
 * Description: Test for the `PolyLine` class
 */

#include <gtest/gtest.h>
#include "sb_geom/PolyLine.h"

using namespace sb_geom;

class PolyLineTest : public testing::Test {
protected:
    PolyLineTest(){};

    virtual void SetUp() {
    }
};

TEST_F(PolyLineTest, default_constructor){
    PolyLine poly_line;
    std::vector<double> expected = {};
    EXPECT_EQ(expected, poly_line.getCoefficients());
}

TEST_F(PolyLineTest, constructor_with_coefficients){
    std::vector<double> coefficients = {1,2,3};
    PolyLine poly_line(coefficients);
    EXPECT_EQ(coefficients, poly_line.getCoefficients());
}

TEST_F(PolyLineTest, getDegree){
    PolyLine poly_line;
    std::vector<double> coefficients = {1,2,3};
    poly_line.setCoefficients(coefficients);
    EXPECT_EQ(3, poly_line.getDegree());
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}