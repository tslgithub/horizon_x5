//
// Created by acer on 25-9-13.
//

#include "model.h"

void Model::PrepareInputTensor(hbDNNTensor *input_tensor, hbDNNHandle_t dnn_handle) {
    printf("\n");
    LOGI("model_input_count -> %d",base_model_config_ptr_->model_input_count);
    for(int idx=0;idx<base_model_config_ptr_->model_input_count;idx++) {
        HB_CHECK_SUCCUSS(
                hbDNNGetInputTensorProperties( &(input_tensor[idx].properties),dnn_handle,idx),
                "hbDNNGetInputTensorProperties fail");
        int input_memory_size = input_tensor[idx].properties.alignedByteSize;
        HB_CHECK_SUCCUSS(
                hbSysAllocCachedMem(&input_tensor[idx].sysMem[0],input_memory_size),
                "hbSysAllocCachedMem fail");

        // 输入对齐，如果用padding，这一步可以不用
        input_tensor[idx].properties.alignedShape = input_tensor[idx].properties.validShape;

        // 获取输入的名称
        const char *input_name;
        HB_CHECK_SUCCUSS(
                hbDNNGetInputName(&input_name,dnn_handle,idx),
                "hbDNNGetInputName fail");
        LOGI("input_name[%d] -> %s",idx,input_name);


        int n,c,h,w;
        LOGI("input_tensor[%d] numDimensions -> %d",idx,input_tensor[idx].properties.alignedShape.numDimensions);
        if(input_tensor[idx].properties.tensorLayout==hbDNNTensorLayout::HB_DNN_LAYOUT_NCHW) {
            n = input_tensor[idx].properties.alignedShape.dimensionSize[0];
            c = input_tensor[idx].properties.alignedShape.dimensionSize[1];
            h = input_tensor[idx].properties.alignedShape.dimensionSize[2];
            w = input_tensor[idx].properties.alignedShape.dimensionSize[3];
            LOGI("input_layout[%d] -> NCHW, shape -> %d,%d,%d,%d",idx,n,c,h,w);
        }else if(input_tensor[idx].properties.tensorLayout == hbDNNTensorLayout::HB_DNN_LAYOUT_NHWC) {
            n = input_tensor[idx].properties.alignedShape.dimensionSize[0];
            h = input_tensor[idx].properties.alignedShape.dimensionSize[1];
            w = input_tensor[idx].properties.alignedShape.dimensionSize[2];
            c = input_tensor[idx].properties.alignedShape.dimensionSize[3];
            LOGI("input_layout[%d] -> NHWC,shape -> %d,%d,%d,%d",idx,n,h,w,c);

        }else {
            LOGE("do not support hbDNNTensorLayout");
            return;
        }
        Shape input_shape;
        input_shape.num = n;
        input_shape.channels = c;
        input_shape.height = h;
        input_shape.width = w;
        base_model_config_ptr_->input_shape_vector.push_back(input_shape);
    }
}

void Model::PrepareOutputTensor(hbDNNTensor *output_tensor, hbDNNHandle_t dnn_handle) {
    printf("\n");
    LOGI("model_output_count -> %d",base_model_config_ptr_->model_output_count);
    for(int idx=0;idx<base_model_config_ptr_->model_output_count;idx++) {
        HB_CHECK_SUCCUSS(
                hbDNNGetOutputTensorProperties(&output_tensor[idx].properties,dnn_handle,idx),
                "hbDNNGetOutputTensorProperties fail");
        int output_memory_size = output_tensor[idx].properties.alignedByteSize;
        HB_CHECK_SUCCUSS(hbSysAllocCachedMem(&output_tensor[idx].sysMem[0],output_memory_size ),
                         "hbSysAllocCachedMem fail");

        const char *output_name;
        HB_CHECK_SUCCUSS(hbDNNGetOutputName(&output_name,dnn_handle,idx),
                         "hbDNNGetOutputName fail");
        LOGI("output_name[%d] -> %s",idx,output_name);

        int n,c,h,w;
        LOGI("output_tensor[%d] numDimensions -> %d", idx,output_tensor[idx].properties.validShape.numDimensions );
        if(output_tensor[idx].properties.tensorLayout==hbDNNTensorLayout::HB_DNN_LAYOUT_NCHW) {
            n = output_tensor[idx].properties.validShape.dimensionSize[0];
            c = output_tensor[idx].properties.validShape.dimensionSize[1];
            h = output_tensor[idx].properties.validShape.dimensionSize[2];
            w = output_tensor[idx].properties.validShape.dimensionSize[3];
            LOGI("output_layout[%d] -> NCHW, shape -> %d,%d,%d,%d",idx,n,c,h,w);
        }else if(output_tensor[idx].properties.tensorLayout == hbDNNTensorLayout::HB_DNN_LAYOUT_NHWC) {
            n = output_tensor[idx].properties.validShape.dimensionSize[0];
            h = output_tensor[idx].properties.validShape.dimensionSize[1];
            w = output_tensor[idx].properties.validShape.dimensionSize[2];
            c = output_tensor[idx].properties.validShape.dimensionSize[3];
            LOGI("output_layout[%d] -> NHWC, shape -> %d,%d,%d,%d",idx,n,h,w,c);
        }

        Shape shape;
        shape.num = n;
        shape.channels = c;
        shape.width = w;
        shape.height = h;
        base_model_config_ptr_->output_shape_vector.push_back(shape);

    }
}

void Model::LoadModel(std::string model_file_path) {
    // Step1: 获取句柄
    {
        char const **model_file_list;
        int model_count = 0;
        base_model_config_ptr_->model_keypoints_bin_path = model_file_path;
        auto model_file_name = base_model_config_ptr_->model_keypoints_bin_path.c_str();

        HB_CHECK_SUCCUSS(
                hbDNNInitializeFromFiles(&dnn_packed_handle_,&model_file_name,1),
                "hbDNNInitializeFromFiles fail");

        HB_CHECK_SUCCUSS(
                hbDNNGetModelNameList(&model_file_list,&model_count,dnn_packed_handle_),
                "hbDNNGetModelNameList fail");

        // 打印模型名称
        for (int i = 0; model_file_list[i] != NULL; i++) {
            LOGI("model_name_list[%d] -> %s", i, model_file_list[i] );
        }

        //推理句柄
        HB_CHECK_SUCCUSS(
                hbDNNGetModelHandle(&dnn_handle_,dnn_packed_handle_,model_file_list[0]),
                "hbDNNGetModelHandle fail");

        LOGI("hbDNN runtime version -> %s",hbDNNGetVersion());
    }

    // Step2: 准备输入数据
    {
        HB_CHECK_SUCCUSS(
                hbDNNGetInputCount(&base_model_config_ptr_->model_input_count,dnn_handle_),
                "hbDNNGetInputCount fail"
        );

        HB_CHECK_SUCCUSS(
                hbDNNGetOutputCount(&base_model_config_ptr_->model_output_count,dnn_handle_),
                "hbDNNGetOutputCount fail"
        );
        base_model_config_ptr_->model_input_tensor.resize(base_model_config_ptr_->model_input_count);
        base_model_config_ptr_->model_output_tensor.resize(base_model_config_ptr_->model_output_count);
        base_model_config_ptr_->input_shape_vector.resize(base_model_config_ptr_->model_input_count);
        base_model_config_ptr_->output_shape_vector.resize(base_model_config_ptr_->model_output_count);

        // LOGI("model_input_count  -> %d",base_model_config_ptr_->model_input_count);
        // LOGI("model_output_count -> %d",base_model_config_ptr_->model_output_count);

        PrepareInputTensor(base_model_config_ptr_->model_input_tensor.data(),dnn_handle_);
        PrepareOutputTensor(base_model_config_ptr_->model_output_tensor.data(),dnn_handle_);

    }

}

void Model::FreeModel() {

    // for (int i = 0; i < input_count; i++) {
    //     HB_CHECK_SUCCESS(hbSysFreeMem(&(input_tensors[i].sysMem[0])),"hbSysFreeMem failed");
    // }
    // // free output mem
    // for (int i = 0; i < output_count; i++) {
    //     HB_CHECK_SUCCESS(hbSysFreeMem(&(output_tensors[i].sysMem[0])),"hbSysFreeMem failed");
    // }

    for (int i = 0; i < base_model_config_ptr_->model_input_count; i++) {
        hbSysFreeMem(&base_model_config_ptr_->model_input_tensor[i].sysMem[0]);
    }
    // 释放模型输出的内存
    for (int i = 0; i < base_model_config_ptr_->model_input_count; i++) {
        hbSysFreeMem(&base_model_config_ptr_->model_output_tensor[i].sysMem[0]);
    }
    // 释放dnn句柄
//    hbDNNRelease(packed_dnn_handle_);

    // hbDNNReleaseTask(dnn_task_handle_);
    hbDNNRelease(dnn_packed_handle_);
}