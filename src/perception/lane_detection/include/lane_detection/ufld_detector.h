#pragma once

#include "tensorrt_base/tensorrt_base.h"
#include <opencv2/opencv.hpp>

namespace ufld_ros {

class UFLDDetector : public tensorrt::TensorRTBase {
  public:
    struct Config {
        int cutHeight;
        int inputWidth;
        int inputHeight;
        int numRow;
        int numCol;
        std::vector<float> rowAnchor;
        std::vector<float> colAnchor;
    };

    UFLDDetector(const std::string &enginePath, const Config &config);
    bool detect(const cv::Mat &image, cv::Mat &outputImage);

  private:
    Config config_;
    bool preprocess(const cv::Mat &input, float *gpuInput);
    bool inference();
    std::vector<std::vector<cv::Point>> postprocess();
    void pred2coords(const std::vector<float> &locRow, const std::vector<float> &locCol,
                     const std::vector<float> &existRow, const std::vector<float> &existCol,
                     std::vector<std::vector<cv::Point>> &coords);
};

} // namespace ufld_ros