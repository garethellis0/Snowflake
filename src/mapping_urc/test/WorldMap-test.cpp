/*
 * Created By: Gareth Ellis
 * Created On: January 7th, 2019
 * Description: Tests for MyNode
 */

#include "WorldMap.h"
#include <gtest/gtest.h>

using ::testing::TestWithParam;
using ::testing::Values;

TEST(WorldMapTest, constructor){
    WorldMap worldMap(0.312, 0.23, 0.87);

    EXPECT_EQ(0.312, worldMap.getSize());
    EXPECT_EQ(0.23, worldMap.getXOffset());
    EXPECT_EQ(0.87, worldMap.getYOffset());
}

TEST(WorldMapTest, getMinX_positive_offset) {
    WorldMap world_map(10, 5, 7);

    EXPECT_EQ(5, world_map.getMinX());
}

TEST(WorldMapTest, getMinX_negative_offset) {
    WorldMap world_map(10, -5, 7);

    EXPECT_EQ(-5, world_map.getMinX());
}

TEST(WorldMapTest, getMaxX_positive_offset) {
    WorldMap world_map(10, 5, 7);

    EXPECT_EQ(15, world_map.getMaxX());
}

TEST(WorldMapTest, getMaxX_negative_offset) {
    WorldMap world_map(10, -5, 7);

    EXPECT_EQ(5, world_map.getMaxX());
}

TEST(WorldMapTest, getMinY_positive_offset) {
    WorldMap world_map(10, 5, 7);

    EXPECT_EQ(7, world_map.getMinY());
}

TEST(WorldMapTest, getMinY_negative_offset) {
    WorldMap world_map(10, 5, -7);

    EXPECT_EQ(-7, world_map.getMinY());
}

TEST(WorldMapTest, getMaxY_positive_offset) {
    WorldMap world_map(10, 5, 7);

    EXPECT_EQ(17, world_map.getMaxY());
}

TEST(WorldMapTest, getMaxY_negative_offset) {
    WorldMap world_map(10, 5, -7);

    EXPECT_EQ(3, world_map.getMaxY());
}

// Inside the test body, fixture constructor, SetUp(), and TearDown() you
// can refer to the test parameter by GetParam().  In this case, the test
// parameter is a factory function which we call in fixture's SetUp() to
// create and store an instance of PrimeTable.
// class PrimeTableTestSmpl7 : public TestWithParam<CreatePrimeTableFunc*> {
//  public:
//   ~PrimeTableTestSmpl7() override { delete table_; }
//   void SetUp() override { table_ = (*GetParam())(); }
//   void TearDown() override {
//     delete table_;
//     table_ = nullptr;
//   }
//
//  protected:
//   PrimeTable* table_;
// };

// TODO: Is this name a _bit_ wordy?
struct WorldMapCopyCostCellsFromMapParam {
    // The source map to copy cells from
    WorldMap source_map;

    // The destination map to copy cells to
    WorldMap dest_map;

    // The resolution at which to verify that the copy was performed correctly
    double verification_resolution;
};

class WorldMapCopyCostCellsFromMapTest : public TestWithParam<
        WorldMapCopyCostCellsFromMapParam> {
};

TEST_P(WorldMapCopyCostCellsFromMapTest, maps_are_equivalent_after_copy) {
    for(double x = GetParam().dest_map.getMinX(); x < GetParam().dest_map.getMaxX())
}

// Test copyCostCellsFromMap where the source_map is smaller
// then the map we're copying to
TEST(WorldMapTest, copyCostCellsFromMap_source_map_is_smaller) {
    // TODO
}

// Test copyCostCellsFromMap where the source_map is larger
// then the map we're copying to
TEST(WorldMapTest, copyCostCellsFromMap_source_map_is_larger) {
    // TODO
}

// Test copyCostCellsFromMap where the source_map is offset
// from the map we're copying to
TEST(WorldMapTest, copyCostCellsFromMap_source_map_is_offset) {
    // TODO
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}