#include "lane_detection/ufld_detector.h"

namespace ufld_ros {

UFLDDetector::UFLDDetector(const std::string &enginePath, const Config &config) : config_(config) {
    if (!loadEngine(enginePath)) {
        throw std::runtime_error("Failed to load TensorRT engine");
    }
    initializeBuffers();
}

bool UFLDDetector::detect(const cv::Mat &image, cv::Mat &outputImage) {
    outputImage = image.clone();

    if (!preprocess(image, static_cast<float *>(deviceBuffers_[0]))) {
        return false;
    }

    if (!inference()) {
        return false;
    }

    auto coords = postprocess();
    for (const auto &lane : coords) {
        for (const auto &point : lane) {
            cv::circle(outputImage, point, 2, cv::Scalar(0, 255, 0), -1);
        }
    }

    return true;
}

bool UFLDDetector::preprocess(const cv::Mat &input, float *gpuInput) {
    cv::Mat cropped = input(cv::Range(config_.cutHeight, input.rows), cv::Range(0, input.cols));

    cv::Mat resized;
    cv::resize(cropped, resized, cv::Size(config_.inputWidth, config_.inputHeight));

    cv::Mat float_mat;
    resized.convertTo(float_mat, CV_32F, 1.0 / 255.0);

    std::vector<cv::Mat> channels(3);
    cv::split(float_mat, channels);

    std::vector<float> inputData(config_.inputWidth * config_.inputHeight * 3);
    float *inputPtr = inputData.data();
    for (int c = 0; c < 3; c++) {
        memcpy(inputPtr + c * config_.inputWidth * config_.inputHeight, channels[c].data,
               config_.inputWidth * config_.inputHeight * sizeof(float));
    }

    cudaMemcpy(gpuInput, inputData.data(), inputData.size() * sizeof(float), cudaMemcpyHostToDevice);

    return true;
}

bool UFLDDetector::inference() { return context_->executeV2(deviceBuffers_.data()); }

std::vector<std::vector<cv::Point>> UFLDDetector::postprocess() {
    // Get output data from GPU
    std::vector<float> locRowData(bufferSizes_[1] / sizeof(float));
    std::vector<float> locColData(bufferSizes_[2] / sizeof(float));
    std::vector<float> existRowData(bufferSizes_[3] / sizeof(float));
    std::vector<float> existColData(bufferSizes_[4] / sizeof(float));

    cudaMemcpy(locRowData.data(), deviceBuffers_[1], bufferSizes_[1], cudaMemcpyDeviceToHost);
    cudaMemcpy(locColData.data(), deviceBuffers_[2], bufferSizes_[2], cudaMemcpyDeviceToHost);
    cudaMemcpy(existRowData.data(), deviceBuffers_[3], bufferSizes_[3], cudaMemcpyDeviceToHost);
    cudaMemcpy(existColData.data(), deviceBuffers_[4], bufferSizes_[4], cudaMemcpyDeviceToHost);

    std::vector<std::vector<cv::Point>> coords;
    pred2coords(locRowData, locColData, existRowData, existColData, coords);
    return coords;
}

void UFLDDetector::pred2coords(const std::vector<float> &locRow, const std::vector<float> &locCol,
                               const std::vector<float> &existRow, const std::vector<float> &existCol,
                               std::vector<std::vector<cv::Point>> &coords) {
    // Implementation of prediction to coordinates conversion
    // This would mirror the Python implementation's logic
    // ... (Implement the conversion logic here)
}

} // namespace ufld_ros