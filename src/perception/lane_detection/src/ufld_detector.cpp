#include "lane_detection/ufld_detector.h"

namespace ufld_ros {

UFLDDetector::UFLDDetector(const std::string &enginePath, const Config &config) : config_(config) {
    if (!loadEngine(enginePath)) {
        throw std::runtime_error("Failed to load TensorRT engine");
    }
    initializeBuffers();
}

// TODO(Yangsh):模型效果待验证,其他关于模型的处理逻辑尚未修改
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
    // 获取维度信息
    const int numGridRow = config_.inputWidth; // 对应原始Python代码中的num_grid_row
    const int numGridCol = config_.inputWidth; // 对应原始Python代码中的num_grid_col
    const int numClsRow = config_.numRow;      // 对应原始Python代码中的num_cls_row
    const int numClsCol = config_.numCol;      // 对应原始Python代码中的num_cls_col
    const int numLanes = 4;                    // 固定4条车道线

    coords.clear();

    // 处理水平方向的车道线（对应原Python代码中的row_lane_idx = [1, 2]）
    std::vector<int> rowLaneIdx = {1, 2};
    for (int i : rowLaneIdx) {
        std::vector<cv::Point> lane;

        // 检查该车道线是否存在
        int validCount = 0;
        for (int k = 0; k < numClsRow; ++k) {
            if (existRow[k * numLanes + i] > 0.5f) { // 使用0.5作为阈值
                validCount++;
            }
        }

        // 如果有足够多的点被检测到
        if (validCount > numClsRow / 2) {
            for (int k = 0; k < numClsRow; ++k) {
                if (existRow[k * numLanes + i] > 0.5f) {
                    // 找到最大响应位置
                    float maxVal = -1;
                    int maxIdx = 0;
                    for (int grid = 0; grid < numGridRow; ++grid) {
                        float val = locRow[(grid * numClsRow + k) * numLanes + i];
                        if (val > maxVal) {
                            maxVal = val;
                            maxIdx = grid;
                        }
                    }

                    // 计算加权平均位置（软化后的位置）
                    const int windowSize = config_.inputWidth / 8; // 搜索窗口大小
                    float weightedSum = 0;
                    float weightSum = 0;

                    for (int grid = std::max(0, maxIdx - windowSize);
                         grid <= std::min(numGridRow - 1, maxIdx + windowSize); ++grid) {
                        float val = locRow[(grid * numClsRow + k) * numLanes + i];
                        float weight = std::exp(val); // softmax的指数部分
                        weightedSum += grid * weight;
                        weightSum += weight;
                    }

                    float position = weightedSum / weightSum;

                    // 转换到原始图像坐标系
                    int x = static_cast<int>((position / (numGridRow - 1)) * config_.inputWidth);
                    int y = static_cast<int>(config_.rowAnchor[k] * config_.inputHeight);

                    lane.emplace_back(x, y);
                }
            }
            if (!lane.empty()) {
                coords.push_back(lane);
            }
        }
    }

    // 处理垂直方向的车道线（对应原Python代码中的col_lane_idx = [0, 3]）
    std::vector<int> colLaneIdx = {0, 3};
    for (int i : colLaneIdx) {
        std::vector<cv::Point> lane;

        // 检查该车道线是否存在
        int validCount = 0;
        for (int k = 0; k < numClsCol; ++k) {
            if (existCol[k * numLanes + i] > 0.5f) {
                validCount++;
            }
        }

        // 如果有足够多的点被检测到
        if (validCount > numClsCol / 4) { // 注意这里是/4而不是/2
            for (int k = 0; k < numClsCol; ++k) {
                if (existCol[k * numLanes + i] > 0.5f) {
                    // 找到最大响应位置
                    float maxVal = -1;
                    int maxIdx = 0;
                    for (int grid = 0; grid < numGridCol; ++grid) {
                        float val = locCol[(grid * numClsCol + k) * numLanes + i];
                        if (val > maxVal) {
                            maxVal = val;
                            maxIdx = grid;
                        }
                    }

                    // 计算加权平均位置
                    const int windowSize = config_.inputHeight / 8;
                    float weightedSum = 0;
                    float weightSum = 0;

                    for (int grid = std::max(0, maxIdx - windowSize);
                         grid <= std::min(numGridCol - 1, maxIdx + windowSize); ++grid) {
                        float val = locCol[(grid * numClsCol + k) * numLanes + i];
                        float weight = std::exp(val);
                        weightedSum += grid * weight;
                        weightSum += weight;
                    }

                    float position = weightedSum / weightSum;

                    // 转换到原始图像坐标系
                    int x = static_cast<int>(config_.colAnchor[k] * config_.inputWidth);
                    int y = static_cast<int>((position / (numGridCol - 1)) * config_.inputHeight);

                    lane.emplace_back(x, y);
                }
            }
            if (!lane.empty()) {
                coords.push_back(lane);
            }
        }
    }

    // 对每条车道线的点进行排序和平滑处理
    for (auto &lane : coords) {
        if (lane.size() > 1) {
            // 根据y坐标排序
            std::sort(lane.begin(), lane.end(), [](const cv::Point &a, const cv::Point &b) { return a.y < b.y; });

            // 可选：添加点的平滑处理
            if (lane.size() > 2) {
                std::vector<cv::Point> smoothed;
                smoothed.push_back(lane.front());

                for (size_t i = 1; i < lane.size() - 1; ++i) {
                    cv::Point p;
                    p.x = (lane[i - 1].x + 2 * lane[i].x + lane[i + 1].x) / 4;
                    p.y = lane[i].y; // 保持y坐标不变
                    smoothed.push_back(p);
                }

                smoothed.push_back(lane.back());
                lane = smoothed;
            }
        }
    }
}

} // namespace ufld_ros