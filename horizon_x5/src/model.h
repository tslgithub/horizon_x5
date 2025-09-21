//
// Created by acer on 25-9-13.
//

#ifndef KEYPOINTS_MATCH_MODEL_H
#define KEYPOINTS_MATCH_MODEL_H


#include <hb_dnn.h>
#include <iostream>
#include <vector>
#include "opencv2/opencv.hpp"
#include "logging.h"

#define HB_CHECK_SUCCUSS(value, errormsg)                            \
do {                                                                \
    auto ret_code = value;                                          \
    if (ret_code != 0){                                             \
        LOGE("%s, error code: %d",errormsg,ret_code);               \
        }                                                           \
    }while (0);

struct Shape {
    int num = 0;
    int channels = 0;
    int height = 0;
    int width = 0;
};

struct BaseModelConfig {
    std::string model_keypoints_bin_path = "/root/T-Robot/keypoints_match/model/dfeat.bin";
    // std::string model_keypoints_bin_path = "/root/T-Robot/stereo_depth/models/depth_exp692_yuv_50_352_640_nchw.bin";
    std::vector<hbDNNTensor> model_input_tensor;
    std::vector<hbDNNTensor> model_output_tensor;
    int32_t model_input_count = 0;
    int32_t model_output_count = 0;
    // Shape shape0;
    // Shape shape1;
    std::vector<Shape> input_shape_vector;
    std::vector<Shape> output_shape_vector;
    bool need_yuv=true;

};

class Model {

public:
    void PrepareInputTensor(hbDNNTensor *input_tensor, hbDNNHandle_t dnn_handle);

    void PrepareOutputTensor(hbDNNTensor *output_tensor, hbDNNHandle_t dnn_handle);

    void LoadModel(std::string model_file_path);

    void FreeModel();

    std::shared_ptr<BaseModelConfig> base_model_config_ptr_;

private:
    hbPackedDNNHandle_t dnn_packed_handle_;
    hbDNNTaskHandle_t dnn_task_handle_;
    hbDNNHandle_t dnn_handle_;

};


#endif //KEYPOINTS_MATCH_MODEL_H
