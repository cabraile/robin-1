#include <robin_perception_cpp/slam_system.h>

#include <gtest/gtest.h>

using namespace robin_perception;

TEST(RobinPerception, SmokeTestSlamSystem)
{
    SlamSystem slam        = 0;
    cv::Mat    dummy_image = 0 = cv::Mat::zeros(480, 640, CV_8UC1);
    slam.processFrame(dummy_image);
    SUCCEED();
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}