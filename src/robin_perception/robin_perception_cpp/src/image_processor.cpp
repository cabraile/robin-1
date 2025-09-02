#include <robin_perception_cpp/image_processor.h>

namespace robin_perception
{

ImageProcessor::ImageProcessor(const ImageProcessorSettings& settings) : settings_(settings) {}

cv::Mat ImageProcessor::process(const cv::Mat& image)
{
    cv::Mat processed_img;
    cv::undistort(processed_img, image, settings_.cam_calib_matrix, settings_.cam_dist_coeffs);
    return processed_img;
}

} // namespace robin_perception