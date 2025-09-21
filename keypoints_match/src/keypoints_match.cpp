//
// Created by acer on 25-9-7.
//

#include "keypoints_match.h"

void KEYPOINTS_MATCH::IinitImage() {
    semi_mat_ = cv::Mat::zeros(
        cv::Size(model_config_ptr_->output_shape_vector.at(0).width,
                 model_config_ptr_->output_shape_vector.at(0).height),
        CV_32FC1);
}

void KEYPOINTS_MATCH::PrepareInputTensor(hbDNNTensor *input_tensor, hbDNNHandle_t dnn_handle) {
    printf("\n");
    LOGI("model_input_count -> %d", model_config_ptr_->model_input_count);
    for (int idx = 0; idx < model_config_ptr_->model_input_count; idx++) {
        HB_CHECK_SUCCUSS(
                hbDNNGetInputTensorProperties(&(input_tensor[idx].properties), dnn_handle, idx),
                "hbDNNGetInputTensorProperties fail");
        int input_memory_size = input_tensor[idx].properties.alignedByteSize;
        HB_CHECK_SUCCUSS(
                hbSysAllocCachedMem(&input_tensor[idx].sysMem[0], input_memory_size),
                "hbSysAllocCachedMem fail");

        // 输入对齐，如果用padding，这一步可以不用
        input_tensor[idx].properties.alignedShape = input_tensor[idx].properties.validShape;

        // 获取输入的名称
        const char *input_name;
        HB_CHECK_SUCCUSS(
                hbDNNGetInputName(&input_name, dnn_handle, idx),
                "hbDNNGetInputName fail");
        LOGI("input_name[%d] -> %s", idx, input_name);


        int n, c, h, w;
        LOGI("input_tensor[%d] numDimensions -> %d", idx, input_tensor[idx].properties.alignedShape.numDimensions);
        if (input_tensor[idx].properties.tensorLayout == hbDNNTensorLayout::HB_DNN_LAYOUT_NCHW) {
            n = input_tensor[idx].properties.alignedShape.dimensionSize[0];
            c = input_tensor[idx].properties.alignedShape.dimensionSize[1];
            h = input_tensor[idx].properties.alignedShape.dimensionSize[2];
            w = input_tensor[idx].properties.alignedShape.dimensionSize[3];
            LOGI("input_layout[%d] -> NCHW, shape -> %d,%d,%d,%d", idx, n, c, h, w);
        } else if (input_tensor[idx].properties.tensorLayout == hbDNNTensorLayout::HB_DNN_LAYOUT_NHWC) {
            n = input_tensor[idx].properties.alignedShape.dimensionSize[0];
            h = input_tensor[idx].properties.alignedShape.dimensionSize[1];
            w = input_tensor[idx].properties.alignedShape.dimensionSize[2];
            c = input_tensor[idx].properties.alignedShape.dimensionSize[3];
            LOGI("input_layout[%d] -> NHWC,shape -> %d,%d,%d,%d", idx, n, h, w, c);

        } else {
            LOGE("do not support hbDNNTensorLayout");
            return;
        }
        Shape input_shape;
        input_shape.num = n;
        input_shape.channels = c;
        input_shape.height = h;
        input_shape.width = w;
        model_config_ptr_->input_shape_vector.at(idx) = input_shape;
    }

}

void KEYPOINTS_MATCH::PrepareOutputTensor(hbDNNTensor *output_tensor, hbDNNHandle_t dnn_handle) {
    printf("\n");
    LOGI("model_output_count -> %d", model_config_ptr_->model_output_count);
    for (int idx = 0; idx < model_config_ptr_->model_output_count; idx++) {
        HB_CHECK_SUCCUSS(
                hbDNNGetOutputTensorProperties(&output_tensor[idx].properties, dnn_handle, idx),
                "hbDNNGetOutputTensorProperties fail");
        int output_memory_size = output_tensor[idx].properties.alignedByteSize;
        HB_CHECK_SUCCUSS(hbSysAllocCachedMem(&output_tensor[idx].sysMem[0], output_memory_size),
                         "hbSysAllocCachedMem fail");

        const char *output_name;
        HB_CHECK_SUCCUSS(hbDNNGetOutputName(&output_name, dnn_handle, idx),
                         "hbDNNGetOutputName fail");
        LOGI("output_name[%d] -> %s", idx, output_name);

        int n, c, h, w;
        LOGI("output_tensor[%d] numDimensions -> %d", idx, output_tensor[idx].properties.validShape.numDimensions);
        if (output_tensor[idx].properties.tensorLayout == hbDNNTensorLayout::HB_DNN_LAYOUT_NCHW) {
            n = output_tensor[idx].properties.validShape.dimensionSize[0];
            c = output_tensor[idx].properties.validShape.dimensionSize[1];
            h = output_tensor[idx].properties.validShape.dimensionSize[2];
            w = output_tensor[idx].properties.validShape.dimensionSize[3];
            LOGI("output_layout[%d] -> NCHW, shape -> %d,%d,%d,%d", idx, n, c, h, w);
        } else if (output_tensor[idx].properties.tensorLayout == hbDNNTensorLayout::HB_DNN_LAYOUT_NHWC) {
            n = output_tensor[idx].properties.validShape.dimensionSize[0];
            h = output_tensor[idx].properties.validShape.dimensionSize[1];
            w = output_tensor[idx].properties.validShape.dimensionSize[2];
            c = output_tensor[idx].properties.validShape.dimensionSize[3];
            LOGI("output_layout[%d] -> NHWC, shape -> %d,%d,%d,%d", idx, n, h, w, c);
        }

        Shape shape;
        shape.num = n;
        shape.channels = c;
        shape.width = w;
        shape.height = h;
        model_config_ptr_->output_shape_vector.at(idx)=shape;
    }
    IinitImage();
}

void KEYPOINTS_MATCH::LoadModel() {
    // Step1: 获取句柄
    {
        char const **model_file_list;
        int model_count = 0;
        auto model_file_name = model_config_ptr_->model_keypoints_bin_path.c_str();

        HB_CHECK_SUCCUSS(
                hbDNNInitializeFromFiles(&dnn_packed_handle_, &model_file_name, 1),
                "hbDNNInitializeFromFiles fail");

        HB_CHECK_SUCCUSS(
                hbDNNGetModelNameList(&model_file_list, &model_count, dnn_packed_handle_),
                "hbDNNGetModelNameList fail");

        // 打印模型名称
        for (int i = 0; model_file_list[i] != NULL; i++) {
            LOGI("model_name_list[%d] -> %s", i, model_file_list[i]);
        }

        //推理句柄
        HB_CHECK_SUCCUSS(
                hbDNNGetModelHandle(&dnn_handle_, dnn_packed_handle_, model_file_list[0]),
                "hbDNNGetModelHandle fail");

        LOGI("hbDNN runtime version -> %s", hbDNNGetVersion());
    }

    // Step2: 准备输入数据
    {
        HB_CHECK_SUCCUSS(
                hbDNNGetInputCount(&model_config_ptr_->model_input_count, dnn_handle_),
                "hbDNNGetInputCount fail"
        );

        HB_CHECK_SUCCUSS(
                hbDNNGetOutputCount(&model_config_ptr_->model_output_count, dnn_handle_),
                "hbDNNGetOutputCount fail"
        );
        model_config_ptr_->model_input_tensor.resize( model_config_ptr_->model_input_count);
        model_config_ptr_->model_output_tensor.resize(model_config_ptr_->model_output_count);
        model_config_ptr_->input_shape_vector.resize( model_config_ptr_->model_input_count);
        model_config_ptr_->output_shape_vector.resize(model_config_ptr_->model_output_count);

        // LOGI("model_input_count  -> %d",model_config_ptr_->model_input_count);
        // LOGI("model_output_count -> %d",model_config_ptr_->model_output_count);

        PrepareInputTensor(model_config_ptr_->model_input_tensor.data(), dnn_handle_);
        PrepareOutputTensor(model_config_ptr_->model_output_tensor.data(), dnn_handle_);

    }

}

void KEYPOINTS_MATCH::FreeModel() {

    // for (int i = 0; i < input_count; i++) {
    //     HB_CHECK_SUCCESS(hbSysFreeMem(&(input_tensors[i].sysMem[0])),"hbSysFreeMem failed");
    // }
    // // free output mem
    // for (int i = 0; i < output_count; i++) {
    //     HB_CHECK_SUCCESS(hbSysFreeMem(&(output_tensors[i].sysMem[0])),"hbSysFreeMem failed");
    // }

    // hbDNNReleaseTask(dnn_task_handle_);
    hbDNNRelease(dnn_packed_handle_);
}

void KEYPOINTS_MATCH::ModelPrepareProcess(cv::Mat gray_mat){

    //step1, get model handdle
    //加载模型，已经加载

    //step2,准备输入输出向量
    //已经准备

    // step3,加载数据
    int h = gray_mat.rows;
    int w = gray_mat.cols;
//    cv::Rect roi = cv::Rect(0,0,640,480);
//    gray_mat = gray_mat(roi).clone();


    if(gray_mat.empty()){
        LOGE("input img is impty");
        return;
    }


    assert(model_config_ptr_->model_input_count==1);
    for(int idx=0;idx<model_config_ptr_->model_input_count;idx++){
        Shape input_shape = model_config_ptr_->input_shape_vector.at(idx);
        int32_t input_size = input_shape.width * input_shape.height * input_shape.channels;
        memcpy(model_config_ptr_->model_input_tensor.at(idx).sysMem[0].virAddr, gray_mat.data,input_size);
    }

    for(int idx=0;idx<model_config_ptr_->model_input_count;idx++){
        hbSysFlushMem(&model_config_ptr_->model_input_tensor[idx].sysMem[0],HB_SYS_MEM_CACHE_CLEAN);
    }


}

void KEYPOINTS_MATCH::KeyFeatureInference() {

    // step1,运行推理
    {
        hbDNNTensor *output_tensor = model_config_ptr_->model_output_tensor.data();
        HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param_);
        hbDNNInfer(&dnn_task_handle_,
                   &output_tensor,
                   model_config_ptr_->model_input_tensor.data(),
                   dnn_handle_,
                   &infer_ctrl_param_
                   );
        // 等待任务结束
        hbDNNWaitTaskDone(dnn_task_handle_, 0);
        // 释放task句柄
        hbDNNReleaseTask(dnn_task_handle_);
        // 任务句柄初始化指针
        dnn_task_handle_ = nullptr;
    }
}

void KEYPOINTS_MATCH::ModelPostProcess(){
    // hbDNNTensor *output = model_config_ptr_->model_output_tensor.data();
    // for(int idx=0;idx<model_config_ptr_->model_output_count;idx++){
    //     hbSysFlushMem(&model_config_ptr_->model_output_tensor[idx].sysMem[0],HB_SYS_MEM_CACHE_INVALIDATE);
    // }


    assert(model_config_ptr_->model_output_count==2);
    static  int output0_size = model_config_ptr_->output_shape_vector.at(0).width * model_config_ptr_->output_shape_vector.at(0).height * model_config_ptr_->output_shape_vector.at(0).channels * model_config_ptr_->output_shape_vector.at(0).num;
    static  int output1_size = model_config_ptr_->output_shape_vector.at(1).width * model_config_ptr_->output_shape_vector.at(1).height * model_config_ptr_->output_shape_vector.at(1).channels * model_config_ptr_->output_shape_vector.at(1).num;

    int8_t *output0    = reinterpret_cast<int8_t *>(model_config_ptr_->model_output_tensor[0].sysMem[0].virAddr);
    int8_t *output1 = reinterpret_cast<int8_t *>(model_config_ptr_->model_output_tensor[1].sysMem[0].virAddr);

    // int res_count_high = 0;
    // int res_count_low  = 0;
    // std::vector<KeyPoint> keypoints_all;
    // std::vector<KeyPoint> keypoints_all_high;
    // std::vector<KeyPoint> keypoints_all_low;
    static float scale = *(model_config_ptr_->model_output_tensor.at(0).properties.scale.scaleData);


    float *dst = semi_mat_.ptr<float>(0);
    float value;
    double t0 = utils_rgbd_ptr_->WhatTimeIsItNow();
    for(int idx=0;idx<output0_size;idx++){
        value = ((*(output0 + idx)) )* scale;
        // int x = idx % shape0.width;
        // int y = idx / shape0.width;
        dst[idx]=value;
    }
    double t1 = utils_rgbd_ptr_->WhatTimeIsItNow();
    LOGD("time1 -> %.0f.ms",t1-t0);

    // cv::imwrite("./semi.png",semi_mat*100*100)
    return;


    {
        hbDNNTensor *output = model_config_ptr_->model_output_tensor.data();
        // make sure CPU read data from DDR before using output tensor data
        for (int i = 0; i < model_config_ptr_->model_output_count; i++) {
            hbSysFlushMem(&model_config_ptr_->model_output_tensor[i].sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE);
        }
        std::cout << "----------output_count-----------" << model_config_ptr_->model_output_count << "\n";

        int *shape = output->properties.validShape.dimensionSize;
        int tensor_len = shape[0] * shape[1] * shape[2] * shape[3];
        std::cout << "---tensor_len---0---  " << tensor_len << " " << shape[0] << " * " << shape[1] << " *  "
                  << shape[2] << " *  " << shape[3] << "\n";



//        int *shape_1 = output_tensors[1].properties.validShape.dimensionSize;
//        int tensor_len_1 = shape_1[0] * shape_1[1] * shape_1[2] * shape_1[3];
//        std::cout<<"---tensor_len---1---  "<<tensor_len_1<< " " <<  shape_1[0] << " * " << shape_1[1] << " *  " << shape_1[2] << " *  " << shape_1[3] << "\n";


//        auto semi = reinterpret_cast<int8_t *>(output_tensors[0].sysMem[0].virAddr);
//        auto desc = reinterpret_cast<int8_t *>(output_tensors[1].sysMem[0].virAddr);

        auto semi = reinterpret_cast<int8_t *>(model_config_ptr_->model_output_tensor[0].sysMem[0].virAddr);

        for(int i = 0; i < tensor_len; i++) {
            if(i %64 == 0)printf("\n");
            float scale = 0.00025672250194475055;
            float tmp_dequant = static_cast<int8_t>(semi[i]);
            std::cout<<tmp_dequant<<",";
        }
    }
}

void KEYPOINTS_MATCH::KeypointsMatchRun() {
//    cv::Mat left = cv::imread("/root/T-Robot/stereo_depth/models/1084514_disparity_left_input_data.png", cv::IMREAD_UNCHANGED);
//    cv::Mat right = cv::imread("/root/T-Robot/stereo_depth/models/1084514_disparity_right_input_data.png", cv::IMREAD_UNCHANGED);
//    LOGI("left.shape  -> %d, %d, %d", left.channels(), left.cols,  left.rows);
//    LOGI("right.shape -> %d, %d, %d", left.channels(), right.cols, right.rows);

    cv::Rect roi = cv::Rect(0,0,640,480);

    cv::Mat left  = cv::imread("/root/T-Robot/stereo_depth/models/1084514_disparity_left_input_data.png", cv::IMREAD_GRAYSCALE);
    cv::Mat right = cv::imread("/root/T-Robot/stereo_depth/models/1084514_disparity_right_input_data.png", cv::IMREAD_GRAYSCALE);
    left  = left(roi);
    right = right(roi);
    LOGI("left.shape  -> %d, %d, %d", left.channels(), left.cols,  left.rows);
    LOGI("right.shape -> %d, %d, %d", left.channels(), right.cols, right.rows);
    LoadModel();

    cv::Mat left_gray,right_gray;
    int sleep_time = 0;
    while ( !(right.empty() || right.empty()) ) {
        double t0 = utils_rgbd_ptr_->WhatTimeIsItNow();
        ModelPrepareProcess(left);
        double t1 = utils_rgbd_ptr_->WhatTimeIsItNow();
        KeyFeatureInference();
        double t2 = utils_rgbd_ptr_->WhatTimeIsItNow();
        ModelPostProcess();

        double t3 = utils_rgbd_ptr_->WhatTimeIsItNow();

        LOGD("t2-t1 -> %.0f.ms, t3-t2 -> %.0f.ms",t2-t1,t3-t2);
        sleep_time = 100 - static_cast<int>((t3-t0));
        usleep(sleep_time*1000 );
        double t4 = utils_rgbd_ptr_->WhatTimeIsItNow();
        LOGD("total time -> %.0f.ms",t4-t0);
    }
    FreeModel();

}