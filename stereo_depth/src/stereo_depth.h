//
// Created by acer on 25-7-26.
//

#ifndef STEREO_DEPTH_H
#define STEREO_DEPTH_H

#include "hb_dnn.h"
#include <memory>
#include <iostream>
#include <vector>
#include <opencv4/opencv2/opencv.hpp>
#include <stdio.h>
#include <assert.h>
#include "logging.h"
#include <unistd.h>

struct ModelShape {
    int n;
    int channels;
    int height;
    int width;
};

struct ConfigParams {
    std::string model_bin_path = "/root/T-Robot/stereo_depth/models/depth_exp692_yuv_50_352_640_nchw.bin";
    std::string model_bin_id = "da39c34cb75aac3ad54fc4e855dea094"; // md5sum生成模型id
    int model_input_count = 2; // 模型输入的数量,会通过hbdnn被更新
    int model_output_count = 1; // 模型输出的数量,会通过hbdnn被更新
    ModelShape model_shape_input; // 每个输入都一样
    ModelShape model_shape_output; // 每个输出都一样，不考虑不一样的情况
    cv::Mat left_input_mat;
    cv::Mat right_input_mat;
    std::vector<hbDNNTensor> input_tensors;
    std::vector<hbDNNTensor> output_tensors;
    float maxdisp = 192.0;
    float min_distance_threash = 150; // 最小100mm
    float max_distance_threash = 2500; // 最大2500mm
    float max_pix_threash = 234.375; // 60000/256
    float bxf = 17480.443359;
    bool center_crop = false;
    bool save_img = false;
    std::string left_path = "/userdata/data/20250403/left1";
    std::string right_path = "/userdata/data/20250403/right1";
    bool keep_open_stereo_depth = false;
    bool local_test = false;
    int camera_img_height = 544;
    int camera_img_width = 640;
    bool need_yuv = true;
    // cv::Rect roi = cv::Rect(0, 120, 640, 352);
    cv::Rect roi = cv::Rect(0, 120, 640, 472);
    int sleep_time = 100;
    bool send_image = false;
};


class STEREO_DEPTH {
public:
    bool ModelLoad();

    void ModelPreprocess(cv::Mat left_image, cv::Mat right_image);

    bool ModelInferProcess();

    bool ModelFree();

    bool ModelProstprocess();

    bool StereDepthRun();

    int PrepareInputOutputTensor(std::vector<hbDNNTensor> &input_tensors, std::vector<hbDNNTensor> &output_tensors);

    int GetInputProperties(const int &input_count, std::vector<hbDNNTensor> &input_tensors);

    int GetOutputProperties(const int &output_count, std::vector<hbDNNTensor> &output_tensors);

    void ImageCvToTensor(hbDNNTensor *input_tensor, cv::Mat rgb_Mat);

    cv::Mat BgrToNv12(const cv::Mat &bgr);

    void IinitImage();

private:
    // 相关配置
    std::unique_ptr<ConfigParams> config_params_ptr_ = std::make_unique<ConfigParams>();
    // dnn句柄
    std::unique_ptr<hbDNNHandle_t> dnn_handle_ptr_ = std::make_unique<hbDNNHandle_t>();
    // dnn推理句柄
    std::unique_ptr<hbDNNInferCtrlParam> dnn_infer_ctrl_param_ = std::make_unique<hbDNNInferCtrlParam>();
    //     dnn任务句柄
    hbDNNTaskHandle_t task_handle_ = nullptr;
    hbPackedDNNHandle_t packed_dnn_handle_;
    hbDNNTensor *left_input_tensor_, *right_input_tensor_;
    cv::Mat rgb_image_, disparity_image_32FC1_, disparity_image_16UC1_, mask_valid_;
};


#endif //STEREO_DEPTH_H
