#include <opencv2/aruco.hpp>
#include <robin_perception_cpp/slam_system.h>

namespace robin_perception
{

void SlamSystem::processFrame(const cv::Mat& img)
{
    Frame curr{};
    curr.image = img;

    if (isBlurry(curr.image))
        return;

    // 1. Detect features
    orb->detectAndCompute(curr.image, cv::noArray(), curr.keypoints, curr.descriptors);

    // 2. Match with previous frame
    if (!frames.empty())
    {
        Frame& prev    = frames.back();
        auto   matches = matchFeatures(prev, curr);

        bool hasAruco = detectAruco(curr, curr.R, curr.t);
        if (hasAruco)
        {
            // Pose from ArUco
            // Already set in detectAruco
        }
        else if (enoughLandmarksVisible(curr))
        {
            // Pose from PnP with 3D landmarks
            solvePnPwithLandmarks(curr);
        }
        else
        {
            // Fallback relative pose (scale uncertain)
            estimateRelativePose(prev, curr, matches);
        }

        // 3. Triangulate new points if baseline sufficient
        if (prevHasPose(prev) && currHasPose(curr))
        {
            auto newPoints = triangulatePoints(prev, curr, matches);
            addLandmarks(newPoints, curr);
        }

        // 4. Local BA
        if (isKeyframe(curr))
        {
            runLocalBA();
        }
    }

    frames.push_back(curr);
}

bool SlamSystem::isBlurry(const cv::Mat& img)
{
    cv::Mat lap;
    cv::Laplacian(img, lap, CV_64F);
    cv::Scalar mean, stddev;
    cv::meanStdDev(lap, mean, stddev);
    return stddev.val[0] < 100.0; // threshold
}

std::vector<cv::DMatch> SlamSystem::matchFeatures(Frame& prev, Frame& curr)
{
    cv::BFMatcher           matcher(cv::NORM_HAMMING, true);
    std::vector<cv::DMatch> matches;
    matcher.match(prev.descriptors, curr.descriptors, matches);
    // Filter by distance
    double                  max_dist = 50.0;
    std::vector<cv::DMatch> good_matches;
    for (auto& m : matches)
    {
        if (m.distance < max_dist)
            good_matches.push_back(m);
    }
    return good_matches;
}

bool SlamSystem::detectAruco(Frame& f, cv::Mat& R, cv::Mat& t)
{
    std::vector<int>                      ids;
    std::vector<std::vector<cv::Point2f>> corners;
    auto                                  dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::detectMarkers(f.image, dict, corners, ids);
    if (ids.empty())
        return false;

    cv::Mat rvec, tvec;
    cv::solvePnP(marker3DPoints, corners[0], K, distCoeffs, rvec, tvec);
    cv::Rodrigues(rvec, R);
    t = tvec;
    return true;
}

bool SlamSystem::enoughLandmarksVisible(const Frame& f)
{
    // Check if enough landmarks are matched in current frame
    int visible = 0;
    for (const auto& lm : mapPoints)
    {
        // Use descriptor matching for visibility (brute force)
        cv::BFMatcher           matcher(cv::NORM_HAMMING);
        std::vector<cv::DMatch> matches;
        matcher.match(lm.descriptor, f.descriptors, matches);
        if (!matches.empty() && matches[0].distance < 50.0)
        {
            visible++;
        }
    }
    return visible >= 10; // threshold
}

void SlamSystem::solvePnPwithLandmarks(Frame& f)
{
    std::vector<cv::Point3f> pts3D;
    std::vector<cv::Point2f> pts2D;
    for (const auto& lm : mapPoints)
    {
        cv::BFMatcher           matcher(cv::NORM_HAMMING);
        std::vector<cv::DMatch> matches;
        matcher.match(lm.descriptor, f.descriptors, matches);
        if (!matches.empty() && matches[0].distance < 50.0)
        {
            pts3D.push_back(lm.position);
            pts2D.push_back(f.keypoints[matches[0].trainIdx].pt);
        }
    }
    if (pts3D.size() >= 4)
    {
        cv::Mat rvec, tvec;
        cv::solvePnPRansac(pts3D, pts2D, K, distCoeffs, rvec, tvec);
        cv::Rodrigues(rvec, f.R);
        f.t = tvec;
    }
}

void SlamSystem::estimateRelativePose(Frame& prev, Frame& curr, const std::vector<cv::DMatch>& matches)
{
    // Use Essential matrix for relative pose
    std::vector<cv::Point2f> pts1, pts2;
    for (auto& m : matches)
    {
        pts1.push_back(prev.keypoints[m.queryIdx].pt);
        pts2.push_back(curr.keypoints[m.trainIdx].pt);
    }
    if (pts1.size() >= 8)
    {
        cv::Mat E = cv::findEssentialMat(pts1, pts2, K);
        cv::recoverPose(E, pts1, pts2, K, curr.R, curr.t);
    }
}

bool SlamSystem::prevHasPose(const Frame& f)
{
    return !f.R.empty() && !f.t.empty();
}
bool SlamSystem::currHasPose(const Frame& f)
{
    return !f.R.empty() && !f.t.empty();
}

std::vector<cv::Point3f> SlamSystem::triangulatePoints(Frame& f1, Frame& f2, std::vector<cv::DMatch>& matches)
{
    cv::Mat P1(3, 4, CV_64F), P2(3, 4, CV_64F);
    cv::hconcat(f1.R, f1.t, P1);
    cv::hconcat(f2.R, f2.t, P2);
    P1 = K * P1;
    P2 = K * P2;
    std::vector<cv::Point2f> pts1, pts2;
    for (auto& m : matches)
    {
        pts1.push_back(f1.keypoints[m.queryIdx].pt);
        pts2.push_back(f2.keypoints[m.trainIdx].pt);
    }
    cv::Mat points4D;
    cv::triangulatePoints(P1, P2, pts1, pts2, points4D);
    std::vector<cv::Point3f> newPoints;
    for (int i = 0; i < points4D.cols; i++)
    {
        cv::Mat x = points4D.col(i);
        x /= x.at<float>(3);
        newPoints.push_back(cv::Point3f(x.at<float>(0), x.at<float>(1), x.at<float>(2)));
    }
    return newPoints;
}

void SlamSystem::addLandmarks(const std::vector<cv::Point3f>& pts, Frame& f)
{
    // Add new landmarks to mapPoints
    for (size_t i = 0; i < pts.size() && i < f.keypoints.size(); ++i)
    {
        Landmark lm;
        lm.position     = pts[i];
        lm.descriptor   = f.descriptors.row(i).clone();
        lm.observations = 1;
        mapPoints.push_back(lm);
    }
}

void SlamSystem::runLocalBA()
{
    // Placeholder: In practice, use ceres/g2o for bundle adjustment
    // Here, just a stub
}

bool SlamSystem::isKeyframe(const Frame& f)
{
    // Simple heuristic: every N frames or if enough parallax
    return frames.size() % 10 == 0;
}

} // namespace robin_perception
