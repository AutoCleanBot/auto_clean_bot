#pragma once

#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <fstream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace tensorrt {

class Logger : public nvinfer1::ILogger {
    void log(Severity severity, const char *msg) noexcept override {
        if (severity != Severity::kINFO) {
            RCLCPP_INFO(rclcpp::get_logger("tensorrt"), "%s", msg);
        }
    }
} static gLogger;

class TensorRTBase {
  protected:
    std::unique_ptr<nvinfer1::IRuntime> runtime_;
    std::unique_ptr<nvinfer1::ICudaEngine> engine_;
    std::unique_ptr<nvinfer1::IExecutionContext> context_;
    std::vector<void *> deviceBuffers_;
    std::vector<size_t> bufferSizes_;

    bool loadEngine(const std::string &enginePath) {
        std::ifstream file(enginePath, std::ios::binary);
        if (!file.good()) {
            RCLCPP_ERROR(rclcpp::get_logger("tensorrt"), "Failed to open engine file");
            return false;
        }

        file.seekg(0, std::ios::end);
        auto size = file.tellg();
        file.seekg(0, std::ios::beg);

        std::vector<char> engineData(size);
        file.read(engineData.data(), size);

        runtime_.reset(nvinfer1::createInferRuntime(gLogger));
        engine_.reset(runtime_->deserializeCudaEngine(engineData.data(), size));
        context_.reset(engine_->createExecutionContext());

        return true;
    }

    void initializeBuffers() {
        int numBindings = engine_->getNbBindings();
        deviceBuffers_.resize(numBindings);
        bufferSizes_.resize(numBindings);

        for (int i = 0; i < numBindings; i++) {
            nvinfer1::Dims dims = engine_->getBindingDimensions(i);
            size_t size = 1;
            for (int j = 0; j < dims.nbDims; j++) {
                size *= dims.d[j];
            }
            bufferSizes_[i] = size * sizeof(float);
            cudaMalloc(&deviceBuffers_[i], bufferSizes_[i]);
        }
    }

  public:
    virtual ~TensorRTBase() {
        for (void *buffer : deviceBuffers_) {
            cudaFree(buffer);
        }
    }
};

} // namespace tensorrt