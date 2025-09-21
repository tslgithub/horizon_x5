
//
// Created by acer on 25-9-7.
//

#ifndef KEYPOINTS_MATCH_H
#define KEYPOINTS_MATCH_H

#include <hb_dnn.h>
#include <iostream>
#include <vector>
#include "opencv2/opencv.hpp"
#include "logging.h"
#include "model.h"
#include "utils_rgbd.h"
#include "unistd.h"

#define HB_CHECK_SUCCUSS(value,errormsg)                            \
do {                                                                \
    auto ret_code = value;                                          \
    if (ret_code != 0){                                             \
        LOGE("%s, error code: %d",errormsg,ret_code);               \
        }                                                           \
    }while (0);

//struct Shape {
//    int num=0;
//    int channel=0;
//    int height=0;
//    int width=0;
//};
//
//struct ModelConfig {
//    std::string model_keypoints_bin_path = "/root/T-Robot/keypoints_match/model/dfeat.bin";
//    // std::string model_keypoints_bin_path = "/root/T-Robot/stereo_depth/models/depth_exp692_yuv_50_352_640_nchw.bin";
//    std::vector<hbDNNTensor> model_input_tensor;
//    std::vector<hbDNNTensor> model_output_tensor;
//    int32_t model_input_count = 0;
//    int32_t model_output_count = 0;
//    // Shape shape0;
//    // Shape shape1;
//    std::vector<Shape> input_shape_vector;
//    std::vector<Shape> output_shape_vector;
//};


struct KeyPoint {
    int x, y;
    float score;
};

class KEYPOINTS_MATCH {
public:
    void KeypointsMatchRun();

    void ModelPrepareProcess(cv::Mat left_gray);

    void KeyFeatureInference();

    void ModelPostProcess();

    void FreeModel();

    void LoadModel();

    void PrepareInputTensor(hbDNNTensor *input_tensor, hbDNNHandle_t dnn_handle);

    void PrepareOutputTensor(hbDNNTensor *output_tensor, hbDNNHandle_t dnn_handle);

    void IinitImage();

private:
    hbPackedDNNHandle_t dnn_packed_handle_;
    hbDNNTaskHandle_t dnn_task_handle_;
    hbDNNHandle_t dnn_handle_;
    hbDNNInferCtrlParam  infer_ctrl_param_;
    std::unique_ptr<BaseModelConfig> model_config_ptr_ = std::make_unique<BaseModelConfig>();
    std::unique_ptr<UtilsRGBD> utils_rgbd_ptr_ = std::make_unique<UtilsRGBD>();
    cv::Mat semi_mat_;
};


#endif //KEYPOINTS_MATCH_H
