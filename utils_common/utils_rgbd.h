//
// Created by l on 2024/10/21.
//

#ifndef NAVISEEKER_UTILS_RGBD_H
#define NAVISEEKER_UTILS_RGBD_H

#include <opencv2/opencv.hpp>
#include <filesystem>
#include <sys/time.h>

#include <regex>
//#include "camera_msg.h"
//#include "utils_common.h"
#include <cmath>
#include "logging.h"

class UtilsRGBD {
public:
    //  UtilsRGBD();

    void ResizeImage(cv::Mat &image, const cv::Size &size);

    void BgrToGray(cv::Mat &image);

    void ShowImage(const cv::Mat &image, int wait_key = 1);

    void SaveData(const std::vector<float> &data, const std::string &save_file_name);

    double WhatTimeIsItNow();

    cv::Mat Dilation(cv::Mat src, int kernel_size_x, int kernel_size_y);

    cv::Mat Dilation(cv::Mat src, const int &kernel_size);

    cv::Mat Erode(cv::Mat src, const int &kernel_size);

    cv::Mat Erode(cv::Mat src, const int &kernel_size_x, const int &kernel_size_y);

    cv::Mat GetKernal(const int &kernel_size);

    void ThreadImage(cv::Mat &image, const float &min_thread_value, const float max_thread_value, bool rev = false);

    cv::Mat AppliDepthColorMap(const cv::Mat &disp, float alpha = 0.01f);

    void LoadPngFile(std::vector<std::pair<std::string, std::string> > &file_path_name, const std::string &png_path);

    // //时间戳对齐
    // bool FindSameTimeStamp(const std::deque<std::pair<ImgHeader, cv::Mat>> &left_deque,
    //                        const std::deque<std::pair<ImgHeader, cv::Mat>> &right_deque,
    //                        std::pair<ImgHeader, cv::Mat> &left_img_header,
    //                        std::pair<ImgHeader, cv::Mat> &right_img_header,
    //                        uint64_t ai_sense_timestamp
    // );

    void CenterCrop(cv::Mat &image, const cv::Size &input_shape, int crop_type = 0);

    cv::Mat BGR2YUV_NV12(const cv::Mat &src);

    cv::Mat RGB2YUV_NV12(const cv::Mat &src);

    cv::Mat YuvNV12ToI420(char *nv12_data, int w, int h);

    int FindMaxContour(const std::vector<std::vector<cv::Point> > &contours);

    int FindMaxContour(const std::vector<std::vector<cv::Point> > &contours, double &max_area);

    cv::Mat GetRoi(const cv::Rect &rect, const cv::Mat &img);

    cv::Mat MakeRoi(const int &input_w, const int &input_h,
                    const cv::Point &point1, const cv::Point &point2,
                    const cv::Point &point3, const cv::Point &point4);

    cv::Mat MakeBackImage(const int &height, const int &width);

    void ShowShape(const cv::Mat &image, const int idx, std::string function_name);

    void SaveImage(const cv::Mat &img, const int &times);

    void SaveImage(const cv::Mat &img, char buffer[]);

    cv::Mat ApplyCustomColorMap(cv::Mat im_gray);

    void Yuv2bgr(const cv::Mat &img_yuv, cv::Mat &img_bgr);

    std::string logs_file_path_ = "/userdata/depth_data/";

    std::vector<std::vector<cv::Point> > CleanSmallContour(const std::vector<std::vector<cv::Point> > &contours,
                                                           const double &min_area, cv::Mat &binary_image);

    //  std::vector<std::vector<cv::Point>> contours
    void FindContours(const cv::Mat &binary_image, std::vector<std::vector<cv::Point> > &contours);

    void FindLine(cv::Mat edge_image, std::vector<cv::Vec2f> &lines);

    void FindLineP(cv::Mat edge_image, std::vector<cv::Vec4i> &lines, int threshold = 50, double minLineLength = 30,
                   double maxLineGap = 10);

    void AdaptiveThreshold(cv::Mat thresh_image, cv::Mat &output_image, double max_value = 255, int thresh_type = 0,
                           int block_size = 17, double c = 3);

    void DrawLines(std::vector<cv::Vec4i> lines, cv::Mat &color_image);

    cv::Mat Fft(const cv::Mat &depth_mat_u16, cv::Mat &fft_image_f32);

    void GrayToBgr(const cv::Mat &gray, cv::Mat &bgr);

    // 形态学操作
    void MorphologyEx(cv::Mat binary_image_clean, cv::Mat &dst, const int &kernel_size_x, const int &kernel_size_y,
                      cv::MorphTypes morph_types = cv::MORPH_OPEN);

    //计算方差和均值
    void GetMeanStd(const cv::Mat &depth_mat, float &mean_value, float &std_velue);

    cv::Mat BgrToNv12(const cv::Mat &bgr);

private:
    bool hasLogsFile_ = false;

    // std::unique_ptr<UtilsCommon> utils_commont_ptr_ = std::make_unique<UtilsCommon>();
};

#endif  // NAVISEEKER_UTILS_RGBD_H
