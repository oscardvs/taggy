/**
 * TensorRT YOLOv8 Engine Wrapper
 * 
 * Provides YOLOv8 object detection using NVIDIA TensorRT for optimized
 * inference on Jetson platforms. Supports loading pre-built engines or
 * building from ONNX models.
 * 
 * EDTH Hackathon - Unitree Go2 Perception
 */

#ifndef GO2_PERCEPTION__TENSORRT_YOLO_HPP_
#define GO2_PERCEPTION__TENSORRT_YOLO_HPP_

#include <string>
#include <vector>
#include <memory>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <numeric>

#include <opencv2/opencv.hpp>

#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>

namespace go2_perception
{

/**
 * Detection result from YOLOv8 inference
 */
struct Detection {
  int class_id;           // COCO class ID (0-79)
  std::string class_name; // Human-readable class name
  float confidence;       // Detection confidence [0, 1]
  cv::Rect bbox;          // Bounding box in image coordinates
  float cx, cy;           // Bounding box center
};

/**
 * TensorRT Logger for build/runtime messages
 */
class TrtLogger : public nvinfer1::ILogger {
 public:
  void log(Severity severity, const char* msg) noexcept override {
    if (severity <= Severity::kWARNING) {
      std::cout << "[TensorRT] " << msg << std::endl;
    }
  }
};

/**
 * YOLOv8 TensorRT Engine Wrapper
 * 
 * Handles model loading, preprocessing, inference, and post-processing
 * for YOLOv8 object detection.
 */
class TensorRTYolo {
 public:
  /**
   * Constructor
   * @param onnx_path Path to ONNX model file
   * @param engine_path Path to save/load TensorRT engine
   * @param input_width Model input width (default 640)
   * @param input_height Model input height (default 640)
   * @param conf_threshold Confidence threshold for detections
   * @param nms_threshold NMS IoU threshold
   * @param use_fp16 Enable FP16 inference (recommended for Jetson)
   */
  TensorRTYolo(
    const std::string& onnx_path,
    const std::string& engine_path,
    int input_width = 640,
    int input_height = 640,
    float conf_threshold = 0.5f,
    float nms_threshold = 0.45f,
    bool use_fp16 = true)
    : onnx_path_(onnx_path),
      engine_path_(engine_path),
      input_width_(input_width),
      input_height_(input_height),
      conf_threshold_(conf_threshold),
      nms_threshold_(nms_threshold),
      use_fp16_(use_fp16),
      initialized_(false) {
    initClassNames();
  }

  ~TensorRTYolo() {
    // Free CUDA memory
    if (input_device_) cudaFree(input_device_);
    if (output_device_) cudaFree(output_device_);
  }

  /**
   * Initialize the TensorRT engine
   * Attempts to load existing engine, builds from ONNX if not found
   * @return true if initialization successful
   */
  bool initialize() {
    // Try to load existing engine first
    if (loadEngine()) {
      std::cout << "[TensorRT] Loaded engine from: " << engine_path_ << std::endl;
    } else {
      // Build engine from ONNX
      std::cout << "[TensorRT] Building engine from ONNX: " << onnx_path_ << std::endl;
      if (!buildEngine()) {
        std::cerr << "[TensorRT] Failed to build engine" << std::endl;
        return false;
      }
      // Save engine for future use
      saveEngine();
    }

    // Create execution context
    context_.reset(engine_->createExecutionContext());
    if (!context_) {
      std::cerr << "[TensorRT] Failed to create execution context" << std::endl;
      return false;
    }

    // Allocate GPU memory for input/output
    if (!allocateBuffers()) {
      std::cerr << "[TensorRT] Failed to allocate GPU buffers" << std::endl;
      return false;
    }

    initialized_ = true;
    std::cout << "[TensorRT] Engine initialized successfully" << std::endl;
    std::cout << "[TensorRT]   Input size: " << input_width_ << "x" << input_height_ << std::endl;
    std::cout << "[TensorRT]   Confidence threshold: " << conf_threshold_ << std::endl;
    std::cout << "[TensorRT]   NMS threshold: " << nms_threshold_ << std::endl;
    return true;
  }

  /**
   * Run inference on an image
   * @param image Input BGR image (any size, will be resized)
   * @return Vector of detections
   */
  std::vector<Detection> detect(const cv::Mat& image) {
    std::vector<Detection> detections;
    
    if (!initialized_) {
      std::cerr << "[TensorRT] Engine not initialized" << std::endl;
      return detections;
    }

    if (image.empty()) {
      std::cerr << "[TensorRT] Empty input image" << std::endl;
      return detections;
    }

    // Store original dimensions for scaling boxes back
    original_width_ = image.cols;
    original_height_ = image.rows;

    // Preprocess: resize, normalize, HWC->CHW
    cv::Mat preprocessed;
    preprocess(image, preprocessed);

    // Copy to GPU
    cudaMemcpy(input_device_, preprocessed.data, 
               input_width_ * input_height_ * 3 * sizeof(float),
               cudaMemcpyHostToDevice);

    // Run inference
    void* bindings[] = {input_device_, output_device_};
    if (!context_->executeV2(bindings)) {
      std::cerr << "[TensorRT] Inference failed" << std::endl;
      return detections;
    }

    // Copy output from GPU
    std::vector<float> output(output_size_);
    cudaMemcpy(output.data(), output_device_, 
               output_size_ * sizeof(float),
               cudaMemcpyDeviceToHost);

    // Post-process: parse boxes, apply NMS
    detections = postprocess(output);

    return detections;
  }

  /**
   * Check if engine is initialized
   */
  bool isInitialized() const { return initialized_; }

  /**
   * Get class name from ID
   */
  std::string getClassName(int class_id) const {
    if (class_id >= 0 && class_id < static_cast<int>(class_names_.size())) {
      return class_names_[class_id];
    }
    return "unknown";
  }

  /**
   * Set confidence threshold
   */
  void setConfidenceThreshold(float threshold) {
    conf_threshold_ = threshold;
  }

  /**
   * Set NMS threshold
   */
  void setNmsThreshold(float threshold) {
    nms_threshold_ = threshold;
  }

 private:
  // Configuration
  std::string onnx_path_;
  std::string engine_path_;
  int input_width_;
  int input_height_;
  float conf_threshold_;
  float nms_threshold_;
  bool use_fp16_;
  bool initialized_;

  // Original image dimensions (for scaling boxes)
  int original_width_;
  int original_height_;

  // TensorRT objects
  TrtLogger logger_;
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;

  // GPU memory
  float* input_device_ = nullptr;
  float* output_device_ = nullptr;
  size_t output_size_ = 0;

  // COCO class names
  std::vector<std::string> class_names_;

  /**
   * Initialize COCO class names (80 classes)
   * Order must match YOLOv8 COCO training data indices
   */
  void initClassNames() {
    class_names_ = {
      "person", "bicycle", "car", "motorcycle", "airplane",
      "bus", "train", "truck", "boat", "traffic light",
      "fire hydrant", "stop sign", "parking meter", "bench", "bird",
      "cat", "dog", "horse", "sheep", "cow",
      "elephant", "bear", "zebra", "giraffe", "backpack",
      "umbrella", "handbag", "tie", "suitcase", "frisbee",
      "skis", "snowboard", "sports ball", "kite", "baseball bat",
      "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
      "wine glass", "cup", "fork", "knife", "spoon",
      "bowl", "banana", "apple", "sandwich", "orange",
      "broccoli", "carrot", "hot dog", "pizza", "donut",
      "cake", "chair", "couch", "potted plant", "bed",
      "dining table", "toilet", "tv", "laptop", "mouse",
      "remote", "keyboard", "cell phone", "microwave", "oven",
      "toaster", "sink", "refrigerator", "book", "clock",
      "vase", "scissors", "teddy bear", "hair dryer", "toothbrush"
    };
  }

  /**
   * Load serialized TensorRT engine from file
   */
  bool loadEngine() {
    std::ifstream file(engine_path_, std::ios::binary);
    if (!file.good()) {
      return false;
    }

    file.seekg(0, std::ios::end);
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<char> buffer(size);
    file.read(buffer.data(), size);
    file.close();

    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    if (!runtime_) {
      return false;
    }

    engine_.reset(runtime_->deserializeCudaEngine(buffer.data(), size));
    return engine_ != nullptr;
  }

  /**
   * Build TensorRT engine from ONNX model
   */
  bool buildEngine() {
    auto builder = std::unique_ptr<nvinfer1::IBuilder>(
      nvinfer1::createInferBuilder(logger_));
    if (!builder) {
      return false;
    }

    auto network = std::unique_ptr<nvinfer1::INetworkDefinition>(
      builder->createNetworkV2(
        1U << static_cast<uint32_t>(
          nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH)));
    if (!network) {
      return false;
    }

    auto parser = std::unique_ptr<nvonnxparser::IParser>(
      nvonnxparser::createParser(*network, logger_));
    if (!parser) {
      return false;
    }

    // Parse ONNX model
    if (!parser->parseFromFile(onnx_path_.c_str(),
        static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
      std::cerr << "[TensorRT] Failed to parse ONNX file: " << onnx_path_ << std::endl;
      return false;
    }

    // Configure builder
    auto config = std::unique_ptr<nvinfer1::IBuilderConfig>(
      builder->createBuilderConfig());
    if (!config) {
      return false;
    }

    // Set memory pool limit (1GB)
    config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, 1ULL << 30);

    // Enable FP16 if requested and supported
    if (use_fp16_ && builder->platformHasFastFp16()) {
      config->setFlag(nvinfer1::BuilderFlag::kFP16);
      std::cout << "[TensorRT] FP16 mode enabled" << std::endl;
    }

    // Build engine (this takes several minutes on first run)
    std::cout << "[TensorRT] Building engine... (this may take several minutes)" << std::endl;
    auto serialized = std::unique_ptr<nvinfer1::IHostMemory>(
      builder->buildSerializedNetwork(*network, *config));
    if (!serialized) {
      return false;
    }

    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    if (!runtime_) {
      return false;
    }

    engine_.reset(runtime_->deserializeCudaEngine(
      serialized->data(), serialized->size()));
    
    return engine_ != nullptr;
  }

  /**
   * Save engine to file for future use
   */
  void saveEngine() {
    if (!engine_) return;

    auto serialized = std::unique_ptr<nvinfer1::IHostMemory>(
      engine_->serialize());
    if (!serialized) return;

    std::ofstream file(engine_path_, std::ios::binary);
    if (file.good()) {
      file.write(static_cast<const char*>(serialized->data()), 
                 serialized->size());
      file.close();
      std::cout << "[TensorRT] Engine saved to: " << engine_path_ << std::endl;
    }
  }

  /**
   * Allocate GPU memory for input and output tensors
   */
  bool allocateBuffers() {
    // Get input tensor info
    auto input_name = engine_->getIOTensorName(0);
    auto input_dims = engine_->getTensorShape(input_name);
    size_t input_size = input_width_ * input_height_ * 3 * sizeof(float);

    // Get output tensor info
    auto output_name = engine_->getIOTensorName(1);
    auto output_dims = engine_->getTensorShape(output_name);
    
    // YOLOv8 output shape: [1, 84, 8400] for 80 classes
    // 84 = 4 (bbox) + 80 (class scores)
    // 8400 = number of predictions
    output_size_ = 1;
    for (int i = 0; i < output_dims.nbDims; i++) {
      output_size_ *= output_dims.d[i];
    }

    std::cout << "[TensorRT] Output tensor size: " << output_size_ << std::endl;

    // Allocate GPU memory
    if (cudaMalloc(&input_device_, input_size) != cudaSuccess) {
      return false;
    }
    if (cudaMalloc(&output_device_, output_size_ * sizeof(float)) != cudaSuccess) {
      cudaFree(input_device_);
      return false;
    }

    return true;
  }

  /**
   * Preprocess image for inference
   * - Resize to model input size
   * - Convert BGR to RGB
   * - Normalize to [0, 1]
   * - Convert HWC to CHW format
   */
  void preprocess(const cv::Mat& input, cv::Mat& output) {
    cv::Mat resized;
    cv::resize(input, resized, cv::Size(input_width_, input_height_));

    // Convert BGR to RGB
    cv::Mat rgb;
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);

    // Convert to float and normalize to [0, 1]
    rgb.convertTo(rgb, CV_32FC3, 1.0 / 255.0);

    // Convert HWC to CHW (planar format)
    std::vector<cv::Mat> channels(3);
    cv::split(rgb, channels);

    // Create output in CHW format
    output = cv::Mat(3 * input_height_, input_width_, CV_32F);
    
    int channel_size = input_height_ * input_width_;
    for (int c = 0; c < 3; c++) {
      memcpy(output.data + c * channel_size * sizeof(float),
             channels[c].data, channel_size * sizeof(float));
    }
  }

  /**
   * Post-process network output
   * - Parse bounding boxes and class scores
   * - Filter by confidence threshold
   * - Apply Non-Maximum Suppression
   * - Scale boxes to original image size
   */
  std::vector<Detection> postprocess(const std::vector<float>& output) {
    std::vector<Detection> detections;
    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> class_ids;

    // YOLOv8 output format: [1, 84, 8400]
    // Transposed: for each of 8400 predictions, we have 84 values
    // [x, y, w, h, class_0_score, class_1_score, ..., class_79_score]
    const int num_classes = 80;
    const int num_predictions = 8400;
    const int stride = num_classes + 4;

    // Scale factors for converting from model input to original image
    float scale_x = static_cast<float>(original_width_) / input_width_;
    float scale_y = static_cast<float>(original_height_) / input_height_;

    for (int i = 0; i < num_predictions; i++) {
      // Get pointer to this prediction (output is transposed in YOLOv8)
      // Output shape is [1, 84, 8400], so stride between predictions
      // is 1, and stride between features is 8400
      
      // Extract bbox center, width, height
      float cx = output[0 * num_predictions + i];
      float cy = output[1 * num_predictions + i];
      float w = output[2 * num_predictions + i];
      float h = output[3 * num_predictions + i];

      // Find best class
      int best_class = 0;
      float best_score = 0.0f;
      for (int c = 0; c < num_classes; c++) {
        float score = output[(4 + c) * num_predictions + i];
        if (score > best_score) {
          best_score = score;
          best_class = c;
        }
      }

      // Filter by confidence
      if (best_score < conf_threshold_) {
        continue;
      }

      // Convert from center format to corner format and scale
      int x = static_cast<int>((cx - w / 2) * scale_x);
      int y = static_cast<int>((cy - h / 2) * scale_y);
      int width = static_cast<int>(w * scale_x);
      int height = static_cast<int>(h * scale_y);

      // Clamp to image bounds
      x = std::max(0, std::min(x, original_width_ - 1));
      y = std::max(0, std::min(y, original_height_ - 1));
      width = std::min(width, original_width_ - x);
      height = std::min(height, original_height_ - y);

      boxes.push_back(cv::Rect(x, y, width, height));
      confidences.push_back(best_score);
      class_ids.push_back(best_class);
    }

    // Apply NMS
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, conf_threshold_, nms_threshold_, indices);

    // Build final detections
    for (int idx : indices) {
      Detection det;
      det.class_id = class_ids[idx];
      det.class_name = getClassName(class_ids[idx]);
      det.confidence = confidences[idx];
      det.bbox = boxes[idx];
      det.cx = boxes[idx].x + boxes[idx].width / 2.0f;
      det.cy = boxes[idx].y + boxes[idx].height / 2.0f;
      detections.push_back(det);
    }

    return detections;
  }
};

}  // namespace go2_perception

#endif  // GO2_PERCEPTION__TENSORRT_YOLO_HPP_

