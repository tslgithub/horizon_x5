//
// Created by acer on 25-7-26.
//

#include "stereo_depth.h"

int STEREO_DEPTH::GetInputProperties(const int &input_count,std::vector<hbDNNTensor>&input_tensors) {

    input_tensors.clear();
    for(int idx=0;idx<input_count;idx++) {
        LOGI("------------------------------------------------------------>");
        int ret = 0;
        hbDNNTensor input_tensor;
        ret = hbDNNGetInputTensorProperties(&input_tensor.properties,*dnn_handle_ptr_,idx);
        if(ret != 0) {
            printf("hbDNNGetInputTensorProperties[i] failed", idx);
        }

        int input_mem_size = input_tensor.properties.alignedByteSize;
        // LOGI("input_mem_size -> %d",input_mem_size);

        //npu缓存
        ret = hbSysAllocCachedMem(&input_tensor.sysMem[0], input_mem_size);
        if (ret != 0) {
            LOGE("hbSysAllocCachedMem[%s] failed", idx);
            return -1;
        }

        input_tensor.properties.alignedShape = input_tensor.properties.validShape;
        input_tensors.push_back(input_tensor);

        // 获取输入层的名称
        const char *input_name;
        ret = hbDNNGetInputName(&input_name,*dnn_handle_ptr_,idx);
        if (ret != 0) {
            LOGE("hbDNNGetInputName[%s] failed", idx);
            return -1;
        }
        LOGI("input[%d] name -> %s",idx,input_name);


        // 获取输入shape
        hbDNNTensorShape input_tensor_shape = input_tensor.properties.alignedShape;
        // LOGI("input[%d] numDimensions -> %d", i, input_tensor_shape.numDimensions);
        LOGI("input[%d] numDimensions -> %d",idx,input_tensor_shape.numDimensions);
        int input_dimensions = input_tensor_shape.numDimensions;
        int offset = 0;
        char buff[1024];
        for(int32_t shape_idx = 0;shape_idx<input_dimensions;shape_idx++) {
            offset += sprintf(buff+offset,"%d",input_tensor_shape.dimensionSize[shape_idx]);
            if(shape_idx != input_dimensions -1) {
                offset += sprintf(buff+offset,",");
            }
        }

        if(input_tensor.properties.tensorLayout == hbDNNTensorLayout::HB_DNN_LAYOUT_NCHW) {
            LOGI("input[%d] tensorLayout -> NCHW", idx);
            config_params_ptr_->model_shape_input.n         = input_tensor_shape.dimensionSize[0];
            config_params_ptr_->model_shape_input.channels  = input_tensor_shape.dimensionSize[1];
            config_params_ptr_->model_shape_input.height    = input_tensor_shape.dimensionSize[2];
            config_params_ptr_->model_shape_input.width     = input_tensor_shape.dimensionSize[3];
        }else if (input_tensor.properties.tensorLayout == hbDNNTensorLayout::HB_DNN_LAYOUT_NHWC) {
            LOGI("input[%d] tensorLayout -> NHWC", idx);
            config_params_ptr_->model_shape_input.n = input_tensor_shape.dimensionSize[0];
            config_params_ptr_->model_shape_input.height = input_tensor_shape.dimensionSize[1];
            config_params_ptr_->model_shape_input.width = input_tensor_shape.dimensionSize[2];
            config_params_ptr_->model_shape_input.channels = input_tensor_shape.dimensionSize[3];
        } else {
            LOGE("input shape is not supported");
        }
        LOGI("input[%d] shape -> [%s]", idx, buff);
    }

    return 0;

}

int STEREO_DEPTH::GetOutputProperties(const int &output_count, std::vector<hbDNNTensor> &output_tensors) {
    int ret=0;
    output_tensors.clear();
    for (int idx = 0; idx < output_count; idx++) {
        LOGI("------------------------------------------------------------>");
        hbDNNTensor output_tensor;
        ret = hbDNNGetOutputTensorProperties(&output_tensor.properties, *dnn_handle_ptr_, idx);
        if (ret != 0) {
            LOGE("hbDNNGetOutputTensorProperties[%d] failed", idx);
        }
        int output_mem_size = output_tensor.properties.alignedByteSize;
        ret = hbSysAllocCachedMem(&output_tensor.sysMem[0], output_mem_size);
        if (ret != 0) {
            LOGI("hbSysAllocCachedMem[%d] failed", idx);
            return -1;
        }

        output_tensors.push_back(output_tensor);

        const char *output_names;
        hbDNNGetOutputName(&output_names, *dnn_handle_ptr_, idx);
        LOGI("output[%d] name -> %s", idx, output_names);

        hbDNNTensorShape output_tensor_shape = output_tensor.properties.alignedShape;
        LOGI("output[%d] numDimensions -> %d", idx, output_tensor_shape.numDimensions);
        int output_dimensions = output_tensor_shape.numDimensions;
        char buff[1024];
        int offset = 0;

        int output_shape[output_dimensions];
        for (int32_t shape_idx = 0; shape_idx < output_dimensions; shape_idx++) {
            offset += sprintf(buff + offset, "%d", output_tensor_shape.dimensionSize[shape_idx]);
            if (shape_idx != output_dimensions - 1) {
                offset += sprintf(buff + offset, ",");
            }
            output_shape[shape_idx] = output_tensor_shape.dimensionSize[shape_idx];
        }

        if (output_tensor.properties.tensorLayout == hbDNNTensorLayout::HB_DNN_LAYOUT_NHWC) {
            LOGI("output[%d] tensorLayout -> NHWC", idx);
            config_params_ptr_->model_shape_output.n = output_shape[0];
            config_params_ptr_->model_shape_output.height = output_shape[1];
            config_params_ptr_->model_shape_output.width = output_shape[2];
            config_params_ptr_->model_shape_output.channels = output_shape[3];
        } else if (output_tensor.properties.tensorLayout == hbDNNTensorLayout::HB_DNN_LAYOUT_NCHW) {
            LOGI("output[%d] tensorLayout -> NCHW", idx);
            config_params_ptr_->model_shape_output.n = output_shape[0];
            config_params_ptr_->model_shape_output.channels = output_shape[1];
            config_params_ptr_->model_shape_output.height = output_shape[2];
            config_params_ptr_->model_shape_output.width = output_shape[3];
        } else {
            LOGE("output shape is not supported");
        }
        LOGI("output[%d] shape -> [%s]", idx, buff);
        // LOGI("output shape(w,h) -> [%d,%d,%d,%d,]",config_params_ptr_->model_shape_output.n,config_params_ptr_->model_shape_output.width, config_params_ptr_->model_shape_output.height,config_params_ptr_->model_shape_output.channels);
    }
    return 0;

}

int STEREO_DEPTH::PrepareInputOutputTensor(std::vector<hbDNNTensor> &input_tensors,std::vector<hbDNNTensor>&output_tensors) {
    int ret = 0;

    //输入层的数量
    int input_count = 0;

    ret = hbDNNGetInputCount(&input_count,*dnn_handle_ptr_);
    if(ret != 0) {
        LOGE("hbDNNGetInputCount fail");
        return -1;
    }
    LOGI("model input count -> %d",input_count);

    config_params_ptr_->model_input_count = input_count;

    // 获取输出层的数量
    int output_count = 0;
    ret = hbDNNGetOutputCount(&output_count,*dnn_handle_ptr_);
    if(ret != 0) {
        LOGE("hbDNNGetOutputCount fail");
        return -1;
    }
    config_params_ptr_->model_output_count = output_count;
    LOGI("hbDNNGetOutputCount -> %d", output_count);

    // 获取输入属性
    ret = GetInputProperties(input_count, input_tensors);
    if(ret != 0) {
        LOGE("GetINpuptProperties faile");
        return -1;
    }

    // 获取输出属性
    ret = GetOutputProperties(output_count, output_tensors);
    if(ret != 0) {
        LOGE("GetOutputProperties faile");
        return -1;
    }

    return 0;
}

bool STEREO_DEPTH::ModelLoad() {
    std::string version = hbDNNGetVersion();
    LOGI("version -> %s",version.c_str());

    char const *model_name = ((config_params_ptr_->model_bin_path).c_str());
    int ret = hbDNNInitializeFromFiles(&packed_dnn_handle_, &model_name, 1);
    if (ret != 0) {
        LOGE("hbDNNInitializeFromFiles failed");
        return false;
    }

    const char **model_name_list;  // 模型名称列表
    int model_count = 0;           // 模型名称个数
    ret =hbDNNGetModelNameList(&model_name_list,&model_count,packed_dnn_handle_);
    if (ret != 0) {
        LOGE("hbDNNGetModelNameList failed");
        return false;
    }
    LOGI("model_count -> %d", model_count);

    // 打印模型名称
    for (int i = 0; model_name_list[i] != NULL; i++) {
        LOGI("model_name_list[%d]-> %s", i, model_name_list[i]);
    }

    // 推理接口
    ret = hbDNNGetModelHandle(&(*dnn_handle_ptr_),packed_dnn_handle_,model_name_list[0]);
    if(ret != 0) {
        LOGE("hbDNNGetModelHandle failed");
        return false;
    }

    // step2, 获取输入输出
    ret = PrepareInputOutputTensor(config_params_ptr_->input_tensors,config_params_ptr_->output_tensors);
    if(ret != 0) {
        LOGE("PrepareInputOutputTensor failed");
        return false;
    }
    config_params_ptr_->input_tensors.resize(config_params_ptr_->model_input_count);
    config_params_ptr_->output_tensors.resize(config_params_ptr_->model_output_count);
    LOGI("LoadModel done");

    IinitImage();

    return true;
}

void STEREO_DEPTH::ImageCvToTensor(hbDNNTensor *input_tensor, cv::Mat rgb_Mat) {
    // 获取属性
    hbDNNTensorProperties input_properties = input_tensor->properties;
    // 获取地址
    auto input_data = input_tensor->sysMem[0].virAddr;

    //    memcpy(input_data,rgb_data,input_properties.alignedByteSize);
    int input_data_len = 0;
    if (config_params_ptr_->need_yuv) {
        input_data_len = config_params_ptr_->model_shape_input.width * config_params_ptr_->model_shape_input.height *
            config_params_ptr_->model_shape_input.channels * 1.5;
        assert(input_data_len == rgb_Mat.total() * 3 );
    } else {
        input_data_len = config_params_ptr_->model_shape_input.width * config_params_ptr_->model_shape_input.height *
            config_params_ptr_->model_shape_input.channels;
        assert(rgb_Mat.total() * 1 == rgb_Mat.total());
    }

    // LOGI("rgb_Mat.total() -> %d",rgb_Mat.total());
    // LOGI("input_data_len -> %d",input_data_len);


    if (config_params_ptr_->need_yuv) {
        //        if(times_ % 50 == 0){
        //            LOGD("tsl,model_shape_input(width, height) -> %d, %d,
        //            ",config_params_ptr_->model_shape_input.width,config_params_ptr_->model_shape_input.height);
        //            LOGD("tsl,rgb_Mat.shape -> %d,%d",rgb_Mat.cols,rgb_Mat.rows);
        //            LOGD("tsl,rgb_Mat.total() -> %d",rgb_Mat.total());
        //            assert(rgb_Mat.total()==rgb_Mat.cols*rgb_Mat.rows);
        //        }
        memcpy(input_data, rgb_Mat.data, rgb_Mat.total());
    }
}

void STEREO_DEPTH::ModelPreprocess(cv::Mat left_image,cv::Mat right_image) {
    int h = left_image.rows;
    int w = left_image.cols;
    if (h != right_image.rows) {
        LOGE("image hight has error");
    }
    if (w != right_image.cols) {
        LOGE("image width has eroor");
    }

    assert(left_image.channels()*3  == config_params_ptr_->model_shape_input.channels);
    assert(right_image.channels()*3 == config_params_ptr_->model_shape_input.channels);
    if (! config_params_ptr_->need_yuv) {
        // left_image  = BgrToNv12(left_image);
        // right_image = BgrToNv12(right_image);
    }

    left_input_tensor_  = &(config_params_ptr_->input_tensors[0]);
    right_input_tensor_ = &(config_params_ptr_->input_tensors[1]);

    rgb_image_ = left_image.clone();
    ImageCvToTensor(left_input_tensor_, rgb_image_.clone());

    rgb_image_ = right_image.clone();
    ImageCvToTensor(right_input_tensor_, rgb_image_.clone());

    // 清空BPU缓存
    for (int i = 0; i < config_params_ptr_->model_input_count; i++) {
        hbSysFlushMem(&config_params_ptr_->input_tensors[i].sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
    }

}

bool STEREO_DEPTH::ModelInferProcess() {
    HB_DNN_INITIALIZE_INFER_CTRL_PARAM(dnn_infer_ctrl_param_);

    hbDNNTensor *output_tensor = config_params_ptr_->output_tensors.data();

    hbDNNInfer(&task_handle_, &output_tensor, (config_params_ptr_->input_tensors).data(), *dnn_handle_ptr_, &(*dnn_infer_ctrl_param_));

    // 等待任务结束
    hbDNNWaitTaskDone(task_handle_, 0);
    // 释放task句柄
    hbDNNReleaseTask(task_handle_);
    // 任务句柄初始化指针
    task_handle_ = nullptr;
    return true;
}

void STEREO_DEPTH::IinitImage() {
    disparity_image_32FC1_ = cv::Mat::zeros(cv::Size(config_params_ptr_->model_shape_output.width,config_params_ptr_->model_shape_output.height),CV_32FC1);
}

bool STEREO_DEPTH::ModelProstprocess() {
    // 默认output size 是 1
    hbDNNTensor *output_tensor = config_params_ptr_->output_tensors.data();
    if(output_tensor->properties.quantiType == hbDNNQuantiType::NONE) {
        int output_data_len = config_params_ptr_->model_shape_output.width * config_params_ptr_->model_shape_output.height * config_params_ptr_->model_shape_output.channels * config_params_ptr_->model_shape_output.n;
        auto disparity_map_data = reinterpret_cast<float *>(output_tensor->sysMem[0].virAddr);
        memcpy(disparity_image_32FC1_.data,disparity_map_data,output_data_len*sizeof(float));
        mask_valid_ = (disparity_image_32FC1_>0) & (disparity_image_32FC1_<config_params_ptr_->maxdisp);
        disparity_image_32FC1_.setTo(1,~mask_valid_);
        cv::divide(config_params_ptr_->bxf,disparity_image_32FC1_,disparity_image_16UC1_, 1.0, CV_16UC1);
        mask_valid_ = disparity_image_16UC1_>config_params_ptr_->min_distance_threash & disparity_image_16UC1_<config_params_ptr_->max_distance_threash;
        disparity_image_16UC1_.setTo(0,~mask_valid_);

    }else {
        LOGE("has error");
    }


    // hbSysFlushMem(&(output_tensor->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
    return true;
}

bool STEREO_DEPTH::ModelFree() {
    for (int i = 0; i < config_params_ptr_->model_input_count; i++) {
        hbSysFreeMem(&config_params_ptr_->input_tensors[i].sysMem[0]);
    }
    // 释放模型输出的内存
    for (int i = 0; i < config_params_ptr_->model_input_count; i++) {
        hbSysFreeMem(&config_params_ptr_->output_tensors[i].sysMem[0]);
    }
    // 释放dnn句柄
    hbDNNRelease(packed_dnn_handle_);
    return true;
}

bool STEREO_DEPTH::StereDepthRun() {
    bool ret = false;
    ret = ModelLoad();
    if(! ret) {
        LOGE("fail to load model");
    }else {
        LOGI("load model");
    }

    cv::Mat left  = cv::imread("../stereo_depth/models/1084514_disparity_left_input_data.png",cv::IMREAD_UNCHANGED);
    cv::Mat right = cv::imread("../stereo_depth/models/1084514_disparity_right_input_data.png",cv::IMREAD_UNCHANGED);
    LOGI("left.shape  -> %d, %d, %d",left.channels(),left.cols,left.rows);
    LOGI("right.shape -> %d, %d, %d",left.channels(),right.cols,right.rows);

    int16_t times = 0;
    while (true) {
        // LOGI("times -> %d", times);
        ModelPreprocess(left,right);
        ModelInferProcess();
        ModelProstprocess();
        // usleep(1000*70);
        // times++;
    }

    ModelFree();
    return true;
}