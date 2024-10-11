#include <gtest/gtest.h>
//#include "camera_hld/camera_hld.hpp"

// Voorbeeld van een eenvoudige test
TEST(CameraHLDTest, ExampleTest)
{
    EXPECT_TRUE(1==1);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
