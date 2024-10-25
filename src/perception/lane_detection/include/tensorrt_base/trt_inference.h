#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <fstream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

// Logger for TensorRT
class Logger : public nvinfer1::ILogger {
    void log(Severity severity, const char *msg) noexcept override {
        if (severity != Severity::kINFO)
            std::cout << msg << std::endl;
    }
} gLogger;

class UFLDv2 {
  private:
    nvinfer1::ICudaEngine *engine;
    nvinfer1::IExecutionContext *context;
    std::vector<void *> buffers;

    // Configuration parameters
    int oriImgWidth;
    int oriImgHeight;
    int cutHeight;
    int inputWidth;
    int inputHeight;
    int numRow;
    int numCol;
    std::vector<float> rowAnchor;
    std::vector<float> colAnchor;

    // Helper functions
    void loadEngine(const std::string &enginePath) {
        std::ifstream file(enginePath, std::ios::binary);
        if (!file) {
            throw std::runtime_error("Failed to open engine file");
        }

        file.seekg(0, file.end);
        long size = file.tellg();
        file.seekg(0, file.beg);

        std::vector<char> engineData(size);
        file.read(engineData.data(), size);

        nvinfer1::IRuntime *runtime = nvinfer1::createInferRuntime(gLogger);
        engine = runtime->deserializeCudaEngine(engineData.data(), size);
        runtime->destroy();

        if (!engine) {
            throw std::runtime_error("Failed to create CUDA engine");
        }

        context = engine->createExecutionContext();
        if (!context) {
            throw std::runtime_error("Failed to create execution context");
        }

        // Allocate buffers
        int nbBindings = engine->getNbBindings();
        buffers.resize(nbBindings);
        for (int i = 0; i < nbBindings; i++) {
            nvinfer1::Dims dims = engine->getBindingDimensions(i);
            size_t size = 1;
            for (int j = 0; j < dims.nbDims; j++) {
                size *= dims.d[j];
            }
            cudaMalloc(&buffers[i], size * sizeof(float));
        }
    }

  public:
    UFLDv2(const std::string &enginePath, const std::string &configPath, std::pair<int, int> oriSize)
        : oriImgWidth(oriSize.first), oriImgHeight(oriSize.second) {
        // Initialize parameters (these should be loaded from config in practice)
        inputWidth = 800;                                // Example value
        inputHeight = 320;                               // Example value
        numRow = 72;                                     // Example value
        numCol = 81;                                     // Example value
        cutHeight = static_cast<int>(inputHeight * 0.3); // Example crop ratio

        // Initialize anchors
        rowAnchor.resize(numRow);
        for (int i = 0; i < numRow; i++) {
            rowAnchor[i] = 0.42 + (1.0 - 0.42) * i / (numRow - 1);
        }

        colAnchor.resize(numCol);
        for (int i = 0; i < numCol; i++) {
            colAnchor[i] = static_cast<float>(i) / (numCol - 1);
        }

        loadEngine(enginePath);
    }

    ~UFLDv2() {
        for (void *buffer : buffers) {
            cudaFree(buffer);
        }
        if (context) {
            context->destroy();
        }
        if (engine) {
            engine->destroy();
        }
    }

    std::vector<std::vector<cv::Point>> pred2coords(const std::vector<std::vector<float>> &predLocRow,
                                                    const std::vector<std::vector<float>> &predLocCol,
                                                    const std::vector<std::vector<float>> &existRow,
                                                    const std::vector<std::vector<float>> &existCol) {
        std::vector<std::vector<cv::Point>> coords;
        // Implementation of prediction to coordinates conversion
        // This would need to mirror the Python implementation's logic
        // Would need to implement softmax and other operations
        return coords;
    }

    void forward(cv::Mat &img) {
        cv::Mat im0 = img.clone();

        // Crop and resize
        cv::Mat cropped = img(cv::Range(cutHeight, img.rows), cv::Range(0, img.cols));
        cv::Mat resized;
        cv::resize(cropped, resized, cv::Size(inputWidth, inputHeight));

        // Preprocess
        resized.convertTo(resized, CV_32F, 1.0 / 255.0);

        // CUDA operations for inference
        float *input = static_cast<float *>(buffers[0]);
        cudaMemcpy(input, resized.data, inputWidth * inputHeight * 3 * sizeof(float), cudaMemcpyHostToDevice);

        context->executeV2(buffers.data());

        // Get outputs and process
        // This part would need to be implemented based on the specific output format
        // of your model

        // Draw results
        for (const auto &lane : coords) {
            for (const auto &point : lane) {
                cv::circle(im0, point, 2, cv::Scalar(0, 255, 0), -1);
            }
        }

        cv::imshow("result", im0);
    }
};

int main(int argc, char *argv[]) {
    if (argc != 5) {
        std::cout << "Usage: " << argv[0] << " <config_path> <engine_path> <video_path> <width> <height>" << std::endl;
        return 1;
    }

    std::string configPath = argv[1];
    std::string enginePath = argv[2];
    std::string videoPath = argv[3];
    int width = std::stoi(argv[4]);
    int height = std::stoi(argv[5]);

    cv::VideoCapture cap(videoPath);
    if (!cap.isOpened()) {
        std::cout << "Error opening video file" << std::endl;
        return -1;
    }

    UFLDv2 isnet(enginePath, configPath, std::make_pair(width, height));

    cv::Mat frame;
    while (cap.read(frame)) {
        cv::resize(frame, frame, cv::Size(1600, 903));
        cv::Mat cropped = frame(cv::Range(380, 700), cv::Range(0, frame.cols));
        isnet.forward(cropped);

        if (cv::waitKey(25) == 'q') {
            break;
        }
    }

    return 0;
}