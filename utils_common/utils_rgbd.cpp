//
// Created by l on 2024/10/21.
//

#include "utils_rgbd.h"

void UtilsRGBD::ResizeImage(cv::Mat &image, const cv::Size &size) {
    int h, w;
    h = image.rows;
    w = image.cols;
    if (h != size.height || w != size.width) {
        cv::resize(image, image, size);
    }
}

void UtilsRGBD::ShowImage(const cv::Mat &image, int wait_key) {
    cv::imshow("frame", image);
    cv::waitKey(wait_key);
}

void UtilsRGBD::ThreadImage(cv::Mat &image, const float &min_thread_value, const float max_thread_value, bool rev) {
    if (!rev) {
        cv::threshold(image, image, min_thread_value, max_thread_value, cv::THRESH_BINARY);
    } else {
        cv::threshold(image, image, min_thread_value, max_thread_value, cv::THRESH_BINARY_INV);
    }

}

void UtilsRGBD::SaveData(const std::vector<float> &data_vector, const std::string &save_file_name) {

    char buff[256];
    sprintf(buff, "%s/%s_arm.txt", logs_file_path_.c_str(), save_file_name.c_str());
    FILE *fp = fopen(buff, "w");
    memset(buff, 0, sizeof(buff));

    std::string data;
    int total_len = 0; // debug使用
    for (int i = 0; i < data_vector.size(); i++) {
        // sprintf(buff,"%6d->%.4f\n",i,data_vector[i]);
        sprintf(buff, "%.3f\n", data_vector[i]);
        data += buff;
        total_len = i;
    }

    fprintf(fp, data.c_str());
    fclose(fp);
}

double UtilsRGBD::WhatTimeIsItNow() {
    struct timeval time;
    if (gettimeofday(&time, NULL)) {
        return 0;
    }
    return ((double) time.tv_sec + (double) time.tv_usec * .000001) * 1000;
}

cv::Mat UtilsRGBD::Erode(cv::Mat src, const int &kernel_size) {
    cv::Mat element = GetKernal(kernel_size);
    cv::erode(src, src, element);
    return src;
}

cv::Mat UtilsRGBD::Erode(cv::Mat src, const int &kernel_size_x,const int &kernel_size_y) {
    cv::Mat kernelX = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size_x, 1));
    cv::Mat kernelY = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, kernel_size_y));
    cv::Mat temp;
    cv::erode(src, temp, kernelX);
    cv::erode(temp, src, kernelY);
    return src;
}



cv::Mat UtilsRGBD::Dilation(cv::Mat src, int kernel_size_x,int kernel_size_y) {
    cv::Mat kernelX = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size_x, 1));
    cv::Mat kernelY = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, kernel_size_y));

    cv::Mat temp;
    cv::dilate(src, temp, kernelX);
    cv::dilate(temp, src, kernelY);

    return src;
}

cv::Mat UtilsRGBD::AppliDepthColorMap(const cv::Mat &depth_16u, float alpha) {
    cv::Mat disp_scaled;
    depth_16u.convertTo(disp_scaled, CV_8UC1, alpha);
    cv::Mat depth_colormap;
    cv::applyColorMap(disp_scaled, depth_colormap, cv::COLORMAP_JET);
    return depth_colormap;

}

// void
// UtilsRGBD::LoadPngFile(std::vector<std::pair<std::string, std::string>> &file_path_name, const std::string &png_path) {
//     // 排序
//     std::vector<std::tuple<std::string, std::string, std::string>> temp_files;
//
//     for (const auto &entry : std::filesystem::directory_iterator(png_path)) {
//         if (entry.is_regular_file() && entry.path().extension() == ".png") {
//             std::string abs_path = std::filesystem::absolute(entry.path()).string();
//             std::string file_name = entry.path().stem().string(); //不要后缀
//             std::string num = utils_commont_ptr_->GetNumber(file_name);
// //            printf("num -> %s\n", num.c_str());
//             temp_files.emplace_back(num, abs_path, file_name);
// //            std::pair<std::string,std::string> path_map;
// //            path_map[abs_path]=file_name;
//         }
//     }
//
//     std::sort(temp_files.begin(), temp_files.end(),
//               [](auto &a, auto &b) {
//                 return std::get<0>(a) < std::get<0>(b);
//               });
//
//     // 存入结果
//     for (const auto &[num, path, name] : temp_files) {
//         file_path_name.emplace_back(path, num);
//     }
// }

void UtilsRGBD::Yuv2bgr(const cv::Mat &img_yuv,cv::Mat &img_bgr){
    cv::cvtColor(img_yuv,img_bgr,cv::COLOR_YUV2RGB_NV12);
}

// bool UtilsRGBD::FindSameTimeStamp(const std::deque<std::pair<ImgHeader,cv::Mat>>&left_deque,
//                                   const std::deque<std::pair<ImgHeader,cv::Mat>>&right_deque,
//                                   std::pair<ImgHeader,cv::Mat> &left_img_header,
//                                   std::pair<ImgHeader,cv::Mat> &right_img_header,
//                                   uint64_t  ai_sense_timestamp){
//     uint64_t max_time_diff = 5; // 最大允许时间差（单位：ms）
//     uint64_t min_time_diff_limit = 2;//  小于1ms，则直接认为相同
//
//     for(const auto & left : left_deque){
//         uint64_t left_timestamp = left.first.timestamp;
//         // 在right_deque中找最近的时间戳
//         double min_diff = std::numeric_limits<double>::max();
//         auto best_match = right_deque.end();
//         for(auto it = right_deque.begin();it!=right_deque.end();it++){
//             double right_timestamp = it->first.timestamp;
//             double diff = std::abs(left_timestamp - right_timestamp);
//             uint64_t  ai_sense_timestamp_diff = std::abs(static_cast<int64_t>(right_timestamp)-static_cast<int64_t>(ai_sense_timestamp));
//             if(diff<min_time_diff_limit){
//                 // 时间戳在一定范围内，直接返回
//                 left_img_header = left;
//                 right_img_header = *it;
//
//                 if( ai_sense_timestamp_diff  < max_time_diff ){
//                     return true;
//                 }
//
//             }else if(diff < min_diff){
//                 min_diff = diff;
//                 best_match = it;
//             }
//             if(min_diff <=max_time_diff && best_match != right_deque.end()){
//                 left_img_header = left;
//                 right_img_header = *best_match;
//                 if( ai_sense_timestamp_diff  < max_time_diff ){
//                     return true;
//                 }
//             }
//         }
//     }
//     return false; // 没有找到满足条件的时间戳对
// }


void UtilsRGBD::CenterCrop(cv::Mat &image,const cv::Size &input_shape,int crop_type){
    int h,w;
    h = image.rows;
    w = image.cols;
    cv::Rect roi;
    assert(h>input_shape.height);
    if(crop_type==0){
        roi =cv::Rect(0,0,w,input_shape.height);
    }else if(crop_type==1){
        roi=cv::Rect(0,(h-input_shape.height)/2,w,input_shape.height);
    } else if(crop_type==2){
        roi=cv::Rect(0,h-input_shape.height,w,input_shape.height);
    }
    image  = image(roi);
}

cv::Mat UtilsRGBD::BGR2YUV_NV12(const cv::Mat& src) {
    auto src_h = src.rows;
    auto src_w = src.cols;
    cv::Mat dst(src_h * 1.5, src_w, CV_8UC1);
    cv::cvtColor(src, dst, cv::COLOR_BGR2YUV_I420);  // I420: YYYY...UU...VV...
    auto n_y  = src_h * src_w;
    auto n_uv = n_y / 2;
    auto n_u  = n_y / 4;
    std::vector<uint8_t> uv(n_uv);
    std::copy(dst.data + n_y, dst.data + n_y + n_uv, uv.data());
    for (auto i = 0; i < n_u; i++) {
        dst.data[n_y + 2 * i]     = uv[i];        // U
        dst.data[n_y + 2 * i + 1] = uv[n_u + i];  // V
    }
    return dst;
}

cv::Mat UtilsRGBD::RGB2YUV_NV12(const cv::Mat& src) {
    auto src_h = src.rows;
    auto src_w = src.cols;
    cv::Mat dst(src_h * 1.5, src_w, CV_8UC1);
    cv::cvtColor(src, dst, cv::COLOR_RGB2YUV_I420);  // I420: YYYY...UU...VV...
    auto n_y  = src_h * src_w;
    auto n_uv = n_y / 2;
    auto n_u  = n_y / 4;
    std::vector<uint8_t> uv(n_uv);
    std::copy(dst.data + n_y, dst.data + n_y + n_uv, uv.data());
    for (auto i = 0; i < n_u; i++) {
        dst.data[n_y + 2 * i]     = uv[i];        // U
        dst.data[n_y + 2 * i + 1] = uv[n_u + i];  // V
    }
    return dst;
}

cv::Mat UtilsRGBD::YuvNV12ToI420(char* nv12_data, int w, int h) {
    cv::Mat i420_mat(h * 1.5, w, CV_8UC1);
    char* i420_data = (char*)i420_mat.data;
    memcpy(i420_mat.data, nv12_data, w * h);  // y分量
    for (int i = 0, j = 0; i < w * h / 4; i++, j += 2) {
        memcpy(i420_data + w * h + i, nv12_data + w * h + j, 1);  // u分量
        memcpy(i420_data + w * h + w * h / 4 + i, nv12_data + w * h + j + 1, 1);  // v分量
    }
    return i420_mat;
}

// 寻找最大轮廓的索引
int UtilsRGBD::FindMaxContour(const std::vector<std::vector<cv::Point>> & contours) {
    double max_area = 0;
    int max_idx     = -1;
    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > max_area) {
            max_area = area;
            max_idx  = i;
        }
    }
    return max_idx;
}

int UtilsRGBD::FindMaxContour(const std::vector<std::vector<cv::Point>> &contours, double &max_area){
    int max_idx = -1;
    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > max_area) {
            max_area = area;
            max_idx  = i;
        }
    }
    return max_idx;
}

void UtilsRGBD::FindContours(const cv::Mat &binary_image,std::vector<std::vector<cv::Point>> &contours){
    std::vector<cv::Vec4i> hierachy;
    cv::findContours(binary_image,contours,hierachy,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_NONE);
}

std::vector<std::vector<cv::Point>> UtilsRGBD::CleanSmallContour(const std::vector<std::vector<cv::Point>> &contours,
                                                                 const double &min_area,cv::Mat &binary_image){
    std::vector<std::vector<cv::Point>> new_contours;
    std::vector<std::vector<cv::Point>> small_contours;
    for(size_t i=0;i<contours.size();i++){
        double area = cv::contourArea(contours[i]);
        if(area>min_area){
            new_contours.emplace_back(contours[i]);
        }else{
            small_contours.emplace_back(contours[i]);
        }
    }

    cv::drawContours(binary_image,small_contours,-1,cv::Scalar(0),-1);

    return new_contours;
}

cv::Mat UtilsRGBD::GetRoi(const cv::Rect& rect, const cv::Mat& img) {
    int left = rect.tl().x;
    int top  = rect.tl().y;

    int right       = rect.br().x;
    int bottom      = rect.br().y;
    cv::Mat roi_img = img(cv::Range(top, bottom), cv::Range(left, right));
    return roi_img;
}

cv::Mat UtilsRGBD::MakeRoi(const int &input_w,const int &input_h,
                       const cv::Point &point1, const cv::Point &point2,
                       const cv::Point &point3, const cv::Point &point4){
    std::vector<cv::Point>polygon_points_vector;
//    polygon_points_vector.push_back(cv::Point(30, 280));
//    polygon_points_vector.push_back(cv::Point(100, 70));
//    polygon_points_vector.push_back(cv::Point(413, 70));
//    polygon_points_vector.push_back(cv::Point(480, 280));
    polygon_points_vector.push_back(point1);
    polygon_points_vector.push_back(point2);
    polygon_points_vector.push_back(point3);
    polygon_points_vector.push_back(point4 );
    cv::Mat back = cv::Mat::zeros(cv::Size(input_w,input_h),CV_16UC1);
    std::vector<std::vector<cv::Point>> polygons;
    polygons.push_back(polygon_points_vector);
    cv::fillPoly(back,polygons,cv::Scalar(1));
    return back;
}

cv::Mat UtilsRGBD::MakeBackImage(const int &height, const int &width){
    // 创建CV_16UC1类型的图像
    cv::Mat image(height, width, CV_16UC1);

    // 填充每一行的像素值为对应的高度值
    for (int h = 0; h < height; ++h) {
        image.row(h).setTo(h + 1);
    }
    return image;
}

void UtilsRGBD::ShowShape(const cv::Mat &image,const int idx,std::string function_name){
    LOGD("tsl,%s(%d), image.shape(w,h) -> (%3d, %3d)",function_name.c_str(),idx,image.cols,image.rows);
}

cv::Mat UtilsRGBD::GetKernal(const int &kernel_size) {
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
    return element;
}

cv::Mat UtilsRGBD::Dilation(cv::Mat src, const int &kernel_size) {
    cv::Mat element = GetKernal(kernel_size);
    cv::dilate(src, src, element);
    return src;
}

void UtilsRGBD::SaveImage(const cv::Mat &img, char buffer[]){
    cv::imwrite(buffer, img);
}

void UtilsRGBD::SaveImage(const cv::Mat &img, const int &times) {
    char buff[128];
    sprintf(buff, "%s/%d.jpg", logs_file_path_.c_str(), times);
    cv::imwrite(buff, img);
}

void UtilsRGBD::BgrToGray(cv::Mat &image) {
    if (image.type() != CV_8UC1) {
        cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    }
}

cv::Mat UtilsRGBD::ApplyCustomColorMap(cv::Mat im_gray) {
    // 创建RGB格式的颜色对应表
    unsigned char rgb[256][3] = {{0, 0, 0},      {255, 0, 255}, {0, 153, 0},   {0, 0, 255},
                                 {155, 48, 255}, {255, 255, 0}, {129, 35, 35}, {255, 0, 0}};
    cv::Mat lut               = cv::Mat(256, 1, CV_8UC3, rgb);  // 创建查找表
    cv::cvtColor(lut, lut, cv::COLOR_RGB2BGR);  // 将RGB格式的颜色查找表转换为BGR格式

    // 将传入的灰度图转换为彩色图
    cv::Mat im_p_color;
    cv::cvtColor(im_gray.clone(), im_p_color, cv::COLOR_GRAY2BGR);

    // 为im_p_color设置颜色
    cv::Mat im_color;
    cv::LUT(im_p_color, lut, im_color);
    return im_color;
}

void UtilsRGBD::FindLine(cv::Mat edge_image,std::vector<cv::Vec2f> &lines){
//    double rho=std::max(edge_image.cols,edge_image.rows)/2; //距离精度，以像素为单位
    double rho=1; //距离精度，以像素为单位
    double theta = CV_PI/180;
    int threshold = 100;// 累加器阈值（只有累加器值高于该阈值的点才被认为是直线）
    double srn = 0;           // rho 的除数，用于多尺度变换（通常为 0）
    double stn = 0;           // theta 的除数，用于多尺度变换（通常为 0）
    double min_theta = 0;     // 最小角度（默认 0）
    double max_theta = CV_PI; // 最大角度（默认 π）
    cv::HoughLines(edge_image,lines,rho,theta,threshold,srn,stn,min_theta,max_theta);
}

void UtilsRGBD::FindLineP(cv::Mat edge_image,std::vector<cv::Vec4i> &lines,int threshold,double minLineLength,double maxLineGap){
//    threshold=50	越高越严格，越低越宽松，建议调试
//    minLineLength=30	过滤小碎线
//    maxLineGap=10	适合处理线条有断裂的图像
//    int threshold=50;
//    double minLineLength=30;
//    double maxLineGap=10;
    cv::HoughLinesP(edge_image,lines,1,CV_PI/180,threshold,minLineLength,maxLineGap);
}

void UtilsRGBD::AdaptiveThreshold(cv::Mat thresh_image,cv::Mat &output_image,double max_value,int thresh_type ,int block_size,double c){
//    double max_value = 255;     // 阈值化后大于 threshold 的像素赋的最大值（通常为 255）
//    int thresh_type = 0 ;       // 自适应阈值计算方法（MEAN 或 GAUSSIAN），cv::ADAPTIVE_THRESH_MEAN_C - 使用邻域均值，cv::ADAPTIVE_THRESH_GAUSSIAN_C - 使用邻域加权高斯均值
//    int block_size=17;           // 邻域大小（奇数，3、5、7、9...），用于计算局部阈值。越大越平滑，越小越敏感。
//    double c =3;                // 偏移常数：阈值 = 局部值 - C，从局部阈值中减去的常数，用于微调结果。
    cv::adaptiveThreshold(thresh_image,output_image,max_value,cv::ADAPTIVE_THRESH_MEAN_C,cv::THRESH_BINARY_INV,block_size,c);
}

void UtilsRGBD::DrawLines(std::vector<cv::Vec4i> lines,cv::Mat &color_image){
    // 画出所有线段
    for (const auto& l : lines) {
        cv::line(color_image, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 2);
    }
}

void UtilsRGBD::GrayToBgr(const cv::Mat &gray,cv::Mat &bgr){
    cv::cvtColor(gray,bgr,cv::COLOR_GRAY2BGR);
}

cv::Mat UtilsRGBD::Fft(const cv::Mat &depth_mat_u16,cv::Mat &fft_image_f32){
//    cv::Mat depth_mat_f32;
    cv::Mat fft_image_u8;
    depth_mat_u16.convertTo(fft_image_f32,CV_32FC1);
    // 归一化
    cv::normalize(fft_image_f32,fft_image_f32,0,1,cv::NORM_MINMAX);
    // 转换为复数，即实部和虚部
    cv::Mat planes[] = {fft_image_f32.clone(),cv::Mat::zeros(fft_image_f32.size(),CV_32F)};
    cv::Mat complex_img;
    cv::merge(planes,2,complex_img);
    // 傅里叶变换
    cv::dft(complex_img,complex_img);

    cv::Mat mask = cv::Mat::zeros(complex_img.size(),complex_img.type());
    int cx = mask.cols/2;
    int cy = mask.rows/2;
    float radius = 20;
    //高通滤波器
    for(int y=0;y<mask.rows;y++){
        for(int x = 0; x<mask.cols;x++){
            float dist = std::sqrt(std::pow(x - cx,2) + std::pow(y-cy,2) );
            if(dist > radius){
                mask.at<cv::Vec2f>(y,x) = cv::Vec2f(1,1);
            }
        }
    }
    // 应用大片频域掩膜
    cv::mulSpectrums(complex_img,mask,complex_img,0);

    cv::Mat inv_dft;
    cv::idft(complex_img, inv_dft, cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);  // 实数输出

    // 归一化结果图
//    cv::normalize(inv_dft, inv_dft, 0, 1, cv::NORM_MINMAX);

//    std::cout<<inv_dft<<std::endl;
    inv_dft.convertTo(  fft_image_u8,CV_8UC1,255);
    return fft_image_u8;
    // 可视化
    cv::imshow("Inverse DFT Result", fft_image_u8);
//    cv::waitKey(0);



    //计算频谱
    cv::split(complex_img,planes);
    cv::magnitude(planes[0],planes[1],planes[0]);
    cv::Mat magnitude_img = planes[0];


//    std::cout<<magnitude_img<<std::endl;
    fft_image_f32 = magnitude_img.clone();

    fft_image_f32.setTo(255,fft_image_f32>100);
    fft_image_f32.convertTo(fft_image_u8,CV_8UC1,1,0);
    return fft_image_u8;
}

void UtilsRGBD::MorphologyEx(cv::Mat binary_image_clean,cv::Mat &dst,const int &kernel_size_x,const int &kernel_size_y,cv::MorphTypes morph_types){
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size_x, kernel_size_y));
    cv::morphologyEx(binary_image_clean.clone(),dst,morph_types,kernel);
}

void UtilsRGBD::GetMeanStd(const cv::Mat &depth_mat,float &mean_value,float &std_velue){
    cv::Scalar mean,stddev;
    cv::meanStdDev(depth_mat,mean,stddev);
    mean_value = mean[0];
    std_velue = stddev[0];
}

cv::Mat UtilsRGBD::BgrToNv12(const cv::Mat &bgr) {
    CV_Assert(bgr.type() == CV_8UC3);

    int width = bgr.cols;
    int height = bgr.rows;

    // Step1: 先转为 YUV420p (I420) 格式
    cv::Mat yuv420p;
    cv::cvtColor(bgr, yuv420p, cv::COLOR_BGR2YUV_I420);

    // Step2: 分离Y、U、V平面
    int y_size = width * height;
    int uv_size = y_size / 4;

    unsigned char *y_ptr = yuv420p.data;
    unsigned char *u_ptr = y_ptr + y_size;
    unsigned char *v_ptr = u_ptr + uv_size;

    // Step3: 构建NV12格式Mat
    cv::Mat nv12(height * 3 / 2, width, CV_8UC1);
    unsigned char *nv12_ptr = nv12.data;

    // 复制Y平面
    memcpy(nv12_ptr, y_ptr, y_size);

    // UV交错存储
    unsigned char *uv_ptr = nv12_ptr + y_size;
    for (int i = 0; i < uv_size; i++)
    {
        uv_ptr[2 * i] = u_ptr[i];     // U
        uv_ptr[2 * i + 1] = v_ptr[i]; // V
    }

    return nv12;
}