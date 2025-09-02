#ifndef ROBIN_PERCEPTION_CPP_SLAM_SYSTEM_H
#define ROBIN_PERCEPTION_CPP_SLAM_SYSTEM_H
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

namespace robin_perception
{

struct Frame
{
    cv::Mat                   image;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat                   descriptors;
    cv::Mat                   R, t; // pose
};

struct Landmark
{
    cv::Point3f position;
    cv::Mat     descriptor;
    int         observations;
};

class SlamSystem
{
  public:
    void processFrame(const cv::Mat& img);

  private:
    std::vector<Frame>    frames;
    std::vector<Landmark> mapPoints;
    cv::Ptr<cv::ORB>      orb = cv::ORB::create(2000);

    cv::Mat K          = (cv::Mat_<double>(3, 3) << 525, 0, 319.5, 0, 525, 239.5, 0, 0, 1); // Example intrinsics
    cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
    std::vector<cv::Point3f> marker3DPoints = {{0, 0, 0}, {0.1, 0, 0}, {0.1, 0.1, 0}, {0, 0.1, 0}}; // Example marker

    bool isBlurry(const cv::Mat& img);

    std::vector<cv::DMatch> matchFeatures(Frame& prev, Frame& curr);

    bool detectAruco(Frame& f, cv::Mat& R, cv::Mat& t);

    bool enoughLandmarksVisible(const Frame& f);

    void solvePnPwithLandmarks(Frame& f);

    void estimateRelativePose(Frame& prev, Frame& curr, const std::vector<cv::DMatch>& matches);

    bool prevHasPose(const Frame& f);

    bool currHasPose(const Frame& f);

    std::vector<cv::Point3f> triangulatePoints(Frame& f1, Frame& f2, std::vector<cv::DMatch>& matches);

    void addLandmarks(const std::vector<cv::Point3f>& pts, Frame& f);

    void runLocalBA();

    bool isKeyframe(const Frame& f);
};

} // namespace robin_perception
#endif