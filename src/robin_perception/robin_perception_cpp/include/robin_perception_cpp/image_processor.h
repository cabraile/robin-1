#ifndef ROBIN_PERCEPTION_CPP_IMAGE_PROCESSOR_H
#define ROBIN_PERCEPTION_CPP_IMAGE_PROCESSOR_H

#include <opencv2/opencv.hpp>

namespace robin_perception
{

struct ImageProcessorSettings
{
    cv::Mat cam_calib_matrix;
    cv::Mat cam_dist_coeffs;
};

class ImageProcessor
{

  public:
    ImageProcessor() = delete;
    ImageProcessor(const ImageProcessorSettings& settings);

    cv::Mat process(const cv::Mat& image);

  private:
    ImageProcessorSettings settings_;
};

} // namespace robin_perception
#endif
