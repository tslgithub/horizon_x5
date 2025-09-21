/**
 * @file init_config.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-08-12
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "cloud_perception.h"
#include "edges_perception.h"
#include "process_ai_sense.h"
#include "thermal_camera.h"
#if defined(PLATFORM_X3M)
    #include "mono_depth.h"
#else if defined(PLATFORM_X5)
    #include "stereo_depth.h"
#endif

template <typename T>
static void JsonGet(const json_t &json, const char *key, T &var) {
    if (json.contains(key)) {
        json[key].get_to(var);
        if constexpr (std::is_floating_point_v<T>) {
            LOGD("param[%s]:% .4f", key, var);
        } else if constexpr (std::is_integral_v<T>) {
            LOGD("param[%s]: %d", key, var);
        } else {
            LOGD("param.contains(%s)", key);
        }
    }
}

/**
 * @brief 获取run config的工具函数
 * @param key
 * @param default_value
 * @return value
 */
template <typename T>
T RunConfigGet(const char* key, const T& default_value) {
    const auto [ret, val] = RUN_CONFIG->GetJsonValue<T>(key);
    if (ret) {
        if constexpr (std::is_floating_point_v<T>) {
            LOGD("run_config[%s]:% .4f", key, val);
        } else if constexpr (std::is_integral_v<T>) {
            LOGD("run_config[%s]: %d", key, val);
        } else {
            LOGD("run_config.contains(%s)", key);
        }
        return val;
    } else {
        LOGW("run_config[%s] failed", key);
        return default_value;
    }
}

/**
 * @brief 解析json配置的bounding box
 *
 * @param config_json
 * @param bounding_box
 * @return true
 * @return false
 */
bool PraseBoundingBox(json_t& config_json, pcl::PointCloud<pcl::PointXYZ>& bounding_box) {
    // 解析点云数据刷新区域角点
    if (config_json.contains("points_num") && config_json["points_num"].is_number_unsigned() &&
        config_json.contains("points")) {
        uint16_t points_num = config_json["points_num"];
        json_t j_points     = config_json["points"];
        if (points_num == j_points.size()) {
            pcl::PointXYZ point_tmp;
            std::string idx_str;
            pcl::PointCloud<pcl::PointXYZ> boundingbox_tmp;
            for (size_t i = 0; i < points_num; i++) {
                idx_str = std::to_string(i);
                if (j_points.contains(idx_str) &&
                    (j_points[idx_str].contains("x") && j_points[idx_str]["x"].is_number() &&
                     j_points[idx_str].contains("y") && j_points[idx_str]["y"].is_number() &&
                     j_points[idx_str].contains("z") && j_points[idx_str]["z"].is_number())) {
                    point_tmp.x = j_points[idx_str]["x"];
                    point_tmp.y = j_points[idx_str]["y"];
                    point_tmp.z = j_points[idx_str]["z"];
                    boundingbox_tmp.push_back(point_tmp);
                } else {
                    break;
                }
            }

            if (boundingbox_tmp.size() == points_num) {
                bounding_box = boundingbox_tmp;
                return true;
            }
        }
    }
    return false;
}

namespace cv {

/**
 * @brief 解析json roi到cv::Rect
 */
void from_json(const json_t& j, cv::Rect& rect) {
    cv::Point start, end;
    j.at("start_x").get_to(start.x);
    j.at("start_y").get_to(start.y);
    j.at("end_x").get_to(end.x);
    j.at("end_y").get_to(end.y);
    rect = cv::Rect(start, end);
}

}  // namespace cv

/**
 * @brief
 *
 */
bool CloudPerception::InitConfig(json_t& json_in) {
    LOGD("CloudPerception::InitConfig");
    json_t config_json;
    if (json_in.contains("perception")) {
        json_t perception_json = json_in["perception"];
        if (perception_json.contains("save_cloud_file") && perception_json["save_cloud_file"].is_boolean()) {
            save_cloud_file_ = perception_json["save_cloud_file"];
        }
        FLOGI("here save_cloud_file is %d", save_cloud_file_);
        if (perception_json.contains("keep_open_pcp") && perception_json["keep_open_pcp"].is_boolean()) {
            keep_open_pcp_ = perception_json["keep_open_pcp"];
        }
        FLOGI("here keep open %d", keep_open_pcp_);
        if (perception_json.contains("print_cloud_size") && perception_json["print_cloud_size"].is_boolean()) {
            print_cloud_size_ = perception_json["print_cloud_size"];
        }
        FLOGI("here print_cloud_size %d", print_cloud_size_);
        if (perception_json.contains("is_single_camera_mode") &&
            perception_json["is_single_camera_mode"].is_boolean()) {
            is_single_camera_mode_ = perception_json["is_single_camera_mode"];
        }
        LOGI("is_single_camera_mode: %d", is_single_camera_mode_);

        if (perception_json.contains("sensor_source") && perception_json["sensor_source"].is_number_unsigned()) {
            uint8_t sensor_source = perception_json["sensor_source"];
            LOGI("sensor_source: %u", sensor_source);
            sensor_source_.SetSensor(sensor_source);
            LOGD("PCP_SENSOR_DEPTH_CAMERA:%d", sensor_source_.HasSensor(PCP_SENSOR_DEPTH_CAMERA));
            LOGD("PCP_SENSOR_MONOCULAR_DEPTH_CAMERA:%d", sensor_source_.HasSensor(PCP_SENSOR_MONOCULAR_DEPTH_CAMERA));
            LOGD("PCP_SENSOR_LIDAR:%d", sensor_source_.HasSensor(PCP_SENSOR_LIDAR));
            point_collecter_->SetSensorSource(sensor_source_);
        }

        // 石板路相关功能
        if (perception_json.contains("func_cross_stone_road") && perception_json["func_cross_stone_road"].is_boolean()) {
            func_cross_stone_road_ = perception_json["func_cross_stone_road"];
        }
        LOGI("func_cross_stone_road: %d", func_cross_stone_road_);

        if (perception_json.contains("is_open_semantic") && perception_json["is_open_semantic"].is_boolean()) {
            is_open_semantic_ = perception_json["is_open_semantic"];
        }
        LOGI("is_open_semantic: %d", is_open_semantic_);

        if (perception_json.contains("func_filt_fov_blind") && perception_json["func_filt_fov_blind"].is_boolean()) {
            func_filt_fov_blind_ = perception_json["func_filt_fov_blind"];
        }
        LOGI("func_filt_fov_blind: %d", func_filt_fov_blind_);
        
        if (perception_json.contains("cloud_perception")) {
            config_json = perception_json["cloud_perception"];
            LOGI("cloud_perception_config");
        }
    }

    // 解析点云数据刷新区域角点
    std::string key_str = "refresh_boundingbox";
    if (config_json.contains(key_str)) {
        json_t j_boundingbox = config_json[key_str];
        PraseBoundingBox(j_boundingbox, update_boundingbox_);
        FLOGI("update_boundingbox_ num: %d", update_boundingbox_.size());
        for (size_t i = 0; i < update_boundingbox_.size(); i++) {
            FLOGI("update_boundingbox[%d]: x:%f  y:%f  z:%f", i, update_boundingbox_[i].x, update_boundingbox_[i].y,
                  update_boundingbox_[i].z);
        }
    }

    key_str = "fov_blind_boundingbox";
    if (config_json.contains(key_str)) {
        json_t j_boundingbox = config_json[key_str];
        PraseBoundingBox(j_boundingbox, blind_fov_boundingbox_);
        FLOGI("blind_fov_boundingbox_ num: %d", blind_fov_boundingbox_.size());
        for (size_t i = 0; i < blind_fov_boundingbox_.size(); i++) {
            FLOGI("update_boundingbox[%d]: x:%f  y:%f  z:%f", i, blind_fov_boundingbox_[i].x, blind_fov_boundingbox_[i].y,
                blind_fov_boundingbox_[i].z);
        }
    }

    // 解析maybe obs 高度异常点云检查角点
    key_str = "mayobs_abnormal_height_boundingbox";
    if (config_json.contains(key_str)) {
        json_t j_boundingbox = config_json[key_str];
        PraseBoundingBox(j_boundingbox, mayobs_abnormal_height_boundingbox_);
        FLOGI("mayobs_abnormal_height_boundingbox num: %d", mayobs_abnormal_height_boundingbox_.size());
        for (size_t i = 0; i < mayobs_abnormal_height_boundingbox_.size(); i++) {
            FLOGI("mayobs_abnormal_height_boundingbox[%d]: x:%f  y:%f  z:%f", i,
                  mayobs_abnormal_height_boundingbox_[i].x, mayobs_abnormal_height_boundingbox_[i].y,
                  mayobs_abnormal_height_boundingbox_[i].z);
        }
    }

    key_str = "front_unnormal_boundingbox";
    if (config_json.contains(key_str)) {
        json_t j_boundingbox = config_json[key_str];
        PraseBoundingBox(j_boundingbox, car_detect_front_unnormal_boundingbox_);
        FLOGI("car_detect_front_unnormal_boundingbox_ num: %d", car_detect_front_unnormal_boundingbox_.size());
        for (size_t i = 0; i < car_detect_front_unnormal_boundingbox_.size(); i++) {
            FLOGI("car_detect_front_unnormal_boundingbox_[%d]: x:%f  y:%f  z:%f", i,
                  car_detect_front_unnormal_boundingbox_[i].x, car_detect_front_unnormal_boundingbox_[i].y,
                  car_detect_front_unnormal_boundingbox_[i].z);
        }
    }

    // 车身轮廓
    std::vector<Point2D> car_contours;
    CARRIER_GET_CARRIER_CONTOURS(car_contours);
    pcl::PointXYZ point;
    FLOGI("car_contours_boundingbox_ num: %d", car_contours.size());
    for (auto&& contour : car_contours) {
        point.x = contour.x;
        point.y = contour.y;
        point.z = 0.0;
        car_contours_boundingbox_.push_back(point);
        FLOGI("car_contours_boundingbox_: x:%f  y:%f  z:%f", point.x, point.y, point.z);
    }

    if (config_json.contains("seg_plane_iter_num") && config_json["seg_plane_iter_num"].is_number()) {
        seg_plane_iter_num_ = config_json["seg_plane_iter_num"];
    }
    FLOGI("seg_plane_iter_num_ : %d", seg_plane_iter_num_);

    if (config_json.contains("seg_plane_dis_threshold") && config_json["seg_plane_dis_threshold"].is_number()) {
        seg_plane_dis_threshold_ = config_json["seg_plane_dis_threshold"];
    }
    FLOGI("seg_plane_dis_threshold_ : %f", seg_plane_dis_threshold_);

    if (config_json.contains("front_unnormal_plane_dis_threshold") &&
        config_json["front_unnormal_plane_dis_threshold"].is_number()) {
        front_unnormal_plane_dis_threshold_ = config_json["front_unnormal_plane_dis_threshold"];
    }
    FLOGI("front_unnormal_plane_dis_threshold_ : %f", front_unnormal_plane_dis_threshold_);
    if (config_json.contains("front_unnormal_plane_epsangle") &&
        config_json["front_unnormal_plane_epsangle"].is_number()) {
        front_unnormal_plane_epsangle_ = config_json["front_unnormal_plane_epsangle"];
    }
    FLOGI("front_unnormal_plane_epsangle_ : %f", front_unnormal_plane_epsangle_);
    if (config_json.contains("front_unnormal_plane_legal_height") &&
        config_json["front_unnormal_plane_legal_height"].is_number()) {
        front_unnormal_plane_legal_height_ = config_json["front_unnormal_plane_legal_height"];
    }
    FLOGI("front_unnormal_plane_legal_height_ : %f", front_unnormal_plane_legal_height_);
    if (config_json.contains("front_unnormal_max_height") && config_json["front_unnormal_max_height"].is_number()) {
        front_unnormal_max_height_ = config_json["front_unnormal_max_height"];
    }
    FLOGI("front_unnormal_max_height_ : %f", front_unnormal_max_height_);
    if (config_json.contains("front_unnormal_cliff_height") && config_json["front_unnormal_cliff_height"].is_number()) {
        front_unnormal_cliff_height_ = config_json["front_unnormal_cliff_height"];
    }
    FLOGI("front_unnormal_cliff_height_ : %f", front_unnormal_cliff_height_);
    if (config_json.contains("unnormal_obs_height") && config_json["unnormal_obs_height"].is_number()) {
        unnormal_obs_height_ = config_json["unnormal_obs_height"];
    }
    FLOGI("unnormal_obs_height_ : %f", unnormal_obs_height_);

    if (config_json.contains("radius_filter_r") && config_json["radius_filter_r"].is_number()) {
        radius_filter_r_ = config_json["radius_filter_r"];
    }
    FLOGI("radius_filter_r_ : %f", radius_filter_r_);

    if (config_json.contains("radius_filter_min_neighbors_num") &&
        config_json["radius_filter_min_neighbors_num"].is_number()) {
        radius_filter_min_neighbors_num_ = config_json["radius_filter_min_neighbors_num"];
    }
    FLOGI("radius_filter_min_neighbors_num_ : %d", radius_filter_min_neighbors_num_);

    if (config_json.contains("ai_sward_radius_filter_min_neighbors_num_building") &&
        config_json["ai_sward_radius_filter_min_neighbors_num_building"].is_number()) {
        ai_sward_radius_filter_min_neighbors_num_building_ =
            config_json["ai_sward_radius_filter_min_neighbors_num_building"];
    }
    FLOGI("ai_sward_radius_filter_min_neighbors_num_building_ : %d",
          ai_sward_radius_filter_min_neighbors_num_building_);

    if (config_json.contains("ai_sward_radius_filter_min_neighbors_num") &&
        config_json["ai_sward_radius_filter_min_neighbors_num"].is_number()) {
        ai_sward_radius_filter_min_neighbors_num_ = config_json["ai_sward_radius_filter_min_neighbors_num"];
    }
    FLOGI("ai_sward_radius_filter_min_neighbors_num_ : %d", ai_sward_radius_filter_min_neighbors_num_);

    if (config_json.contains("refresh_record_frame_num") && config_json["refresh_record_frame_num"].is_number()) {
        refresh_record_frame_num_ = config_json["refresh_record_frame_num"];
    }
    FLOGI("refresh_record_frame_num_ : %d", refresh_record_frame_num_);

    if (config_json.contains("camera_height") && config_json["camera_height"].is_number()) {
        camera_height_ = config_json["camera_height"];
    }
    octomap_processor_->SetCamHeight(camera_height_);
    FLOGI("here init camera height %f", camera_height_);

    if (config_json.contains("camera_blind_y") && config_json["camera_blind_y"].is_number()) {
        camera_y_blind_ = config_json["camera_blind_y"];
    }
    FLOGI("here init camera blind y dis %f", camera_y_blind_);

    point_collecter_->SetCamBlindDisY(camera_y_blind_);

    if (config_json.contains("obs_height") && config_json["obs_height"].is_number()) {
        obs_height_ = config_json["obs_height"];
    }
    FLOGI("here init obs_height %f", obs_height_);

    if (config_json.contains("ai_obs_height") && config_json["ai_obs_height"].is_number()) {
        ai_obs_height_ = config_json["ai_obs_height"];
    }
    FLOGI("here init ai_obs_height %f", ai_obs_height_);

    if (config_json.contains("robot_height") && config_json["robot_height"].is_number()) {
        robot_height_ = config_json["robot_height"];
    }
    FLOGI("here init robot_height %f", robot_height_);

    if (config_json.contains("may_obs_top_height") && config_json["may_obs_top_height"].is_number()) {
        may_obs_top_height_ = config_json["may_obs_top_height"];
    }
    FLOGI("here init may_obs_top_height %f", may_obs_top_height_);

    if (config_json.contains("mayobs_abnormal_height") && config_json["mayobs_abnormal_height"].is_number()) {
        mayobs_abnormal_height_ = config_json["mayobs_abnormal_height"];
    }
    FLOGI("here init mayobs_abnormal_height %f", mayobs_abnormal_height_);

    if (config_json.contains("extend_convex_dis") && config_json["extend_convex_dis"].is_number()) {
        extend_convex_dis_ = config_json["extend_convex_dis"];
    }
    FLOGI("here init extend_convex_dis %f", extend_convex_dis_);

    if (config_json.contains("mayobs_abnormal_height_to_obs_top") && config_json["mayobs_abnormal_height_to_obs_top"].is_number()) {
        mayobs_abnormal_height_to_obs_top_ = config_json["mayobs_abnormal_height_to_obs_top"];
    }
    FLOGI("here init mayobs_abnormal_height_to_obs_top %f", mayobs_abnormal_height_to_obs_top_);

    if (config_json.contains("using_fitting_plane") && config_json["using_fitting_plane"].is_boolean()) {
        using_fitting_plane_ = config_json["using_fitting_plane"];
    }
    FLOGI("here using plane fitting is %d", using_fitting_plane_);

    if (config_json.contains("use_ai_result") && config_json["use_ai_result"].is_boolean()) {
        use_ai_res_ = config_json["use_ai_result"];
    }
    FLOGI(" use_ai_result %d", use_ai_res_);

    if (config_json.contains("ai_res_ignore_acc") && config_json["ai_res_ignore_acc"].is_number()) {
        ai_res_ignore_acc_ = config_json["ai_res_ignore_acc"];
    }
    FLOGI(" ai_res_ignore_acc %f", ai_res_ignore_acc_);

    if (config_json.contains("ai_res_ignore_gyro_z") && config_json["ai_res_ignore_gyro_z"].is_number()) {
        ai_res_ignore_gyro_z_ = config_json["ai_res_ignore_gyro_z"];
    }
    FLOGI(" ai_res_ignore_gyro_z %f", ai_res_ignore_gyro_z_);

    if (config_json.contains("fiter_bottom_near_channel_or_blanks") &&
        config_json["fiter_bottom_near_channel_or_blanks"].is_number()) {
        fiter_bottom_near_channel_or_blanks_ = config_json["fiter_bottom_near_channel_or_blanks"];
    }
    FLOGI(" fiter_bottom_near_channel_or_blanks %f", fiter_bottom_near_channel_or_blanks_);

    if (config_json.contains("ai_may_obs_use_fit_plane") && config_json["ai_may_obs_use_fit_plane"].is_boolean()) {
        ai_may_obs_use_fit_plane_ = config_json["ai_may_obs_use_fit_plane"];
    }
    FLOGI(" ai_may_obs_use_fit_plane %d", ai_may_obs_use_fit_plane_);

    if (config_json.contains("ai_may_obs_fit_plane_dis_threshold") &&
        config_json["ai_may_obs_fit_plane_dis_threshold"].is_number()) {
        ai_may_obs_fit_plane_dis_threshold_ = config_json["ai_may_obs_fit_plane_dis_threshold"];
    }
    FLOGI(" ai_may_obs_fit_plane_dis_threshold %f", ai_may_obs_fit_plane_dis_threshold_);

    if (config_json.contains("ai_may_obs_fit_plane_ang_threshold") &&
        config_json["ai_may_obs_fit_plane_ang_threshold"].is_number()) {
        ai_may_obs_fit_plane_ang_threshold_ = config_json["ai_may_obs_fit_plane_ang_threshold"];
    }
    FLOGI(" ai_may_obs_fit_plane_ang_threshold %f deg", ai_may_obs_fit_plane_ang_threshold_);

    if (config_json.contains("ai_may_obs_cloud_AABB_area_min") &&
        config_json["ai_may_obs_cloud_AABB_area_min"].is_number()) {
        ai_may_obs_cloud_AABB_area_min_ = config_json["ai_may_obs_cloud_AABB_area_min"];
    }
    FLOGI(" ai_may_obs_cloud_AABB_area_min %f ", ai_may_obs_cloud_AABB_area_min_);

    if (config_json.contains("ai_may_obs_cloud_density_min") &&
        config_json["ai_may_obs_cloud_density_min"].is_number()) {
        ai_may_obs_cloud_density_min_ = config_json["ai_may_obs_cloud_density_min"];
    }
    FLOGI(" ai_may_obs_cloud_density_min %f ", ai_may_obs_cloud_density_min_);

    if (config_json.contains("history_height_cloud_reserve") &&
        config_json["history_height_cloud_reserve"].is_boolean()) {
        history_height_cloud_reserve_ = config_json["history_height_cloud_reserve"];
    }
    FLOGI(" history_height_cloud_reserve %d ", history_height_cloud_reserve_);

    if (config_json.contains("filter_maybe_mag_tag_width") && config_json["filter_maybe_mag_tag_width"].is_number()) {
        filter_mag_tag_width_ = config_json["filter_maybe_mag_tag_width"];
    }
    FLOGI("filter_maybe_mag_tag_width: %f ", filter_mag_tag_width_);

    if (config_json.contains("use_mag_tag_filt_sward") && config_json["use_mag_tag_filt_sward"].is_boolean()) {
        need_filt_mag_tag_cloud_ = config_json["use_mag_tag_filt_sward"];
    }
    mag_tag_.SetLifeTime(3 * 1000);  // 可能的磁标位姿
    mag_tag_.Inactive();
    FLOGI(" use_mag_tag_filt_sward %d ", need_filt_mag_tag_cloud_);

    if (config_json.contains("generate_charger_high_cloud") &&
        config_json["generate_charger_high_cloud"].is_boolean()) {
        generate_charger_high_cloud_ = config_json["generate_charger_high_cloud"];
    }
    FLOGI(" generate_charger_high_cloud %d ", generate_charger_high_cloud_);

    if (config_json.contains("pub_charger_high_cloud_dis_thresold") &&
        config_json["pub_charger_high_cloud_dis_thresold"].is_number()) {
        pub_charger_high_cloud_dis_thresold_ = config_json["pub_charger_high_cloud_dis_thresold"];
    }
    FLOGI(" pub_charger_high_cloud_dis_thresold_ %f ", pub_charger_high_cloud_dis_thresold_);

    key_str = "charger_high_cloud_boundingbox";
    if (config_json.contains(key_str)) {
        json_t j_boundingbox = config_json[key_str];
        PraseBoundingBox(j_boundingbox, charger_high_cloud_boundingbox_);
        FLOGI("charger_high_cloud_boundingbox num: %d", charger_high_cloud_boundingbox_.size());
        for (size_t i = 0; i < charger_high_cloud_boundingbox_.size(); i++) {
            FLOGI("charger_high_cloud_boundingbox[%d]: x:%f  y:%f  z:%f", i, charger_high_cloud_boundingbox_[i].x,
                  charger_high_cloud_boundingbox_[i].y, charger_high_cloud_boundingbox_[i].z);
        }
    }
    // 获取自动切换虚拟碰撞的配置
    if (config_json.contains("auto_switch_virtual_collision")) {
        auto auto_switch_virtual_colli_       = config_json["auto_switch_virtual_collision"];
        enable_auto_switch_virtual_collision_ = auto_switch_virtual_colli_["enable"];
        fitline_cloud_dis_thres_              = auto_switch_virtual_colli_["fitline_cloud_dis_thres"];
        fitline_min_nighbor_dis_thres_        = auto_switch_virtual_colli_["fitline_min_nighbor_dis_thres"];
        fitline_dis_2_line_thres_             = auto_switch_virtual_colli_["fitline_dis_2_line_thres"];
        fitline_line_dis_thres_               = auto_switch_virtual_colli_["fitline_line_dis_thres"];
        over_pitch_disable_switch_vcolli_holdtime_ = auto_switch_virtual_colli_["over_pitch_disable_switch_vcolli_holdtime"];
        fitline_max_pitch_angle_ = auto_switch_virtual_colli_["fitline_max_pitch_angle"];

        LOGD("auto_switch_virtual_collision: %d", enable_auto_switch_virtual_collision_);
        LOGD("fitline_cloud_dis_thres_: %f", fitline_cloud_dis_thres_);
        LOGD("fitline_min_nighbor_dis_thres_: %f", fitline_min_nighbor_dis_thres_);
        LOGD("fitline_dis_2_line_thres_: %f", fitline_dis_2_line_thres_);
        LOGD("fitline_line_dis_thres_: %f", fitline_line_dis_thres_);
        LOGD("over_pitch_disable_switch_vcolli_holdtime_: %ld ms", over_pitch_disable_switch_vcolli_holdtime_);
        LOGD("fitline_max_pitch_angle_: %f deg", fitline_max_pitch_angle_);
    }
    FLOGD("init end");
    GenerateOriginalChargerHighCloud();

    // 获取跨越石板路功能相关配置
    if (auto [ret, value] = RUN_CONFIG->GetJsonValue<bool>("cross_stone_road"); ret) {
        if (value) {
            SetFuncCrossStoneRoad(true);
            MSG_MANAGER->SetFuncCrossStoneRoad(true);
            LOGD("Perception: SetFuncCrossStoneRoad: true");
        } else {
            SetFuncCrossStoneRoad(false);
            MSG_MANAGER->SetFuncCrossStoneRoad(false);
            LOGD("Perception: SetFuncCrossStoneRoad: false");
        }
    }

    return true;
}

bool EdgesPerception::InitEdgesCloudConfig(json_t config_json) {
    json_t perception_json = config_json["perception"];
    if (perception_json.contains("keep_open_pcp") && perception_json["keep_open_pcp"].is_boolean()) {
        keep_open_pcp_ = perception_json["keep_open_pcp"];
    }
    FLOGI("here keep open %d", keep_open_pcp_);
    if (perception_json.contains("print_cloud_size") && perception_json["print_cloud_size"].is_boolean()) {
        print_cloud_size_ = perception_json["print_cloud_size"];
    }
    FLOGI("here print_cloud_size %d", print_cloud_size_);
    config_json = config_json["perception"]["cloud_perception"];

    if (perception_json.contains("is_open_semantic") && perception_json["is_open_semantic"].is_boolean()) {
        is_open_semantic_ = perception_json["is_open_semantic"];
    }
    FLOGI("here is_open_semantic %d", is_open_semantic_);

    std::string key_str = "publish_update_boundingbox";
    if (config_json.contains(key_str)) {
        json_t j_boundingbox = config_json[key_str];
        PraseBoundingBox(j_boundingbox, pub_update_boundingbox_);
        FLOGI("pub_update_boundingbox_ num: %d", pub_update_boundingbox_.size());
        for (size_t i = 0; i < pub_update_boundingbox_.size(); i++) {
            FLOGI("pub_update_boundingbox_[%d]: x:%f  y:%f  z:%f", i, pub_update_boundingbox_[i].x,
                  pub_update_boundingbox_[i].y, pub_update_boundingbox_[i].z);
        }
    }

    if (config_json.contains("morph_rad_above_edge") && config_json["morph_rad_above_edge"].is_boolean()) {
        morph_rad_above_edge_ = config_json["morph_rad_above_edge"];
    }
    FLOGI("here morph_rad_above_edge %d", morph_rad_above_edge_);

    if (config_json.contains("use_charger_forbid_area") && config_json["use_charger_forbid_area"].is_boolean()) {
        use_charger_forbid_area_ = config_json["use_charger_forbid_area"];
    }
    LOGD("use_charger_forbid_area %d", use_charger_forbid_area_);

    if (config_json.contains("generate_back_colli_cloud") && config_json["generate_back_colli_cloud"].is_boolean()) {
        generate_back_colli_cloud_ = config_json["generate_back_colli_cloud"];
    }
    LOGD("generate_back_colli_cloud %d", generate_back_colli_cloud_);

    if (config_json.contains("use_colli_life") && config_json["use_colli_life"].is_boolean()) {
        use_colli_life_ = config_json["use_colli_life"];
    }
    LOGD("use_colli_life %d", use_colli_life_);

    if (config_json.contains("front_colli_life") && config_json["front_colli_life"].is_number()) {
        front_colli_life_ = config_json["front_colli_life"];
    }
    LOGD("front_colli_life %ld ms", front_colli_life_);

    if (config_json.contains("back_colli_life") && config_json["back_colli_life"].is_number()) {
        back_colli_life_ = config_json["back_colli_life"];
    }
    LOGD("back_colli_life %ld ms", back_colli_life_);

    // 车身轮廓
    std::vector<Point2D> car_contours;
    CARRIER_GET_CARRIER_CONTOURS(car_contours);
    pcl::PointXYZ point;
    FLOGI("car_contours_boundingbox_ num: %d", car_contours.size());
    for (auto&& contour : car_contours) {
        point.x = contour.x;
        point.y = contour.y;
        point.z = 0.0;
        car_contours_boundingbox_.push_back(point);
        FLOGI("car_contours_boundingbox_: x:%f  y:%f  z:%f", point.x, point.y, point.z);
    }

    // 侧方碰撞配置
    if (config_json.contains("generate_side_colli_cloud") && config_json["generate_side_colli_cloud"].is_boolean()) {
        generate_side_colli_cloud_ = config_json["generate_side_colli_cloud"];
    }
    LOGD("generate_side_colli_cloud: %d", generate_side_colli_cloud_);

    if (config_json.contains("side_collision")) {
        side_colli_physi_cfgs_.clear();
        for (auto item : config_json["side_collision"].items()) {
            json_t it_value = item.value();
            if (it_value.contains("start_point") && it_value.contains("end_point") &&
                it_value.contains("trigger_value") && it_value["trigger_value"].is_number()) {
                SideCollisionPhysicalCfg side_colli_physi_cfg;
                if (it_value["start_point"].contains("x") && it_value["start_point"].contains("y") &&
                    it_value["start_point"].contains("z") && it_value["start_point"]["x"].is_number() &&
                    it_value["start_point"]["y"].is_number() && it_value["start_point"]["z"].is_number()) {
                    side_colli_physi_cfg.start_point.x = it_value["start_point"]["x"];
                    side_colli_physi_cfg.start_point.y = it_value["start_point"]["y"];
                    side_colli_physi_cfg.start_point.z = it_value["start_point"]["z"];
                    LOGD("start_point: %f %f %f", side_colli_physi_cfg.start_point.x,
                         side_colli_physi_cfg.start_point.y, side_colli_physi_cfg.start_point.z);
                }
                if (it_value["end_point"].contains("x") && it_value["end_point"].contains("y") &&
                    it_value["end_point"].contains("z") && it_value["end_point"]["x"].is_number() &&
                    it_value["end_point"]["y"].is_number() && it_value["end_point"]["z"].is_number()) {
                    side_colli_physi_cfg.end_point.x = it_value["end_point"]["x"];
                    side_colli_physi_cfg.end_point.y = it_value["end_point"]["y"];
                    side_colli_physi_cfg.end_point.z = it_value["end_point"]["z"];
                    LOGD("end_point: %f %f %f", side_colli_physi_cfg.end_point.x, side_colli_physi_cfg.end_point.y,
                         side_colli_physi_cfg.end_point.z);
                }
                side_colli_physi_cfg.trigger_value = it_value["trigger_value"];
                LOGD("trigger_value: %d", side_colli_physi_cfg.trigger_value);

                side_colli_physi_cfgs_.push_back(side_colli_physi_cfg);
            }
        }
    }

    GenerateAllCollisionOriginalArcCloud();
    GenerateAllCollisionOriginalLineCloud();
    FLOGD("init end");
    return true;
}


/**
 * @brief 解析虚拟碰撞相关配置
 *
 */
void ProcessAISense::VirtualCollisionConfig(json_t &config_json) {
    if (config_json.contains("virtual_collisions") && config_json["virtual_collisions"].is_array()) {
        virtual_colli_.clear();
        // 初始化
        color_detect_roi_start_ = cv::Point2i(std::numeric_limits<int>::max(), std::numeric_limits<int>::max());
        color_detect_roi_end_   = cv::Point2i(0, 0);
        for (auto &colli : config_json["virtual_collisions"]) {
            VirtualColliInfo colli_info;
            // 默认生存周期
            uint64_t life_time = life_time_virtual_colli_;
            // 解析 roi 相关参数
            if (colli.contains("roi")) {
                json_t roi = colli["roi"];
                if (roi.contains("start_x") && roi.contains("start_y") && roi.contains("end_x") &&
                    roi.contains("end_y") && roi["start_x"].is_number() && roi["start_y"].is_number() &&
                    roi["end_x"].is_number() && roi["end_y"].is_number()) {
                    uint16_t start_x     = roi["start_x"];
                    uint16_t start_y     = roi["start_y"];
                    uint16_t end_x       = roi["end_x"];
                    uint16_t end_y       = roi["end_y"];
                    colli_info.roi_start = cv::Point2i(start_x, start_y);
                    colli_info.roi_end   = cv::Point2i(end_x, end_y);
                    colli_info.roi_key   = ((start_x << 48) | (start_y << 32) | (end_x << 16) | (end_y << 0));
                } else {
                    continue;
                }
            } else {
                continue;
            }

            if (virtual_colli_.find(colli_info.roi_key) != virtual_colli_.end()) {
                continue;
            }

            if (colli.contains("trigger_value") && colli["trigger_value"].is_number()) {
                colli_info.value = colli["trigger_value"];
            } else {
                continue;
            }

            // 解析虚拟碰撞车身坐标系点碰撞点云
            pcl::PointXYZ point_start;
            pcl::PointXYZ point_end;
            if (colli.contains("dis") && colli["dis"].is_number() && colli.contains("start_angle") &&
                colli["start_angle"].is_number() && colli.contains("end_angle") && colli["end_angle"].is_number()) {
                colli_info.dis         = colli["dis"];
                colli_info.start_angle = colli["start_angle"];
                colli_info.start_angle = DEG2RAD(colli_info.start_angle);
                colli_info.end_angle   = colli["end_angle"];
                colli_info.end_angle   = DEG2RAD(colli_info.end_angle);
                point_start.x          = -colli_info.dis * sin(colli_info.start_angle);
                point_start.y          = colli_info.dis * cos(colli_info.start_angle);
                point_start.z          = 0.0;

                point_end.x = -colli_info.dis * sin(colli_info.end_angle);
                point_end.y = colli_info.dis * cos(colli_info.end_angle);
                point_end.z = 0.0;
            } else {
                continue;
            }

            // 解析是否存在特殊的生存周期
            if (colli.contains("life_time") && colli["life_time"].is_number()) {
                life_time = colli["life_time"];
            } else {
                life_time = life_time_virtual_colli_;
            }

            // 生成相机坐标系坐标系下虚拟碰撞点云
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr =
                p_depth_collect_->cloud_util_.CreatLineSegment(point_start, point_end, 0.025);
            colli_info.cloud = *cloud_ptr;

            // for debug
            if (save_cloud_file_) {
                std::string file_str = "ai_virtual_collision_before_transf_" + to_string(colli_info.value) + "_";
                p_depth_collect_->cloud_util_.SavePcd(colli_info.cloud, cloud_save_dir_, file_str);
            }

            LOGI("virtual colli(%d)::roi:(%d,%d)<->(%d,%d),roi key:%ld,dis:%f,angle: %f<->%f,line:(%f, %f)<->(%f, %f)",
                 colli_info.value, colli_info.roi_start.x, colli_info.roi_start.y, colli_info.roi_end.x,
                 colli_info.roi_end.y, colli_info.roi_key, colli_info.dis, RAD2DEG(colli_info.start_angle),
                 RAD2DEG(colli_info.end_angle), point_start.x, point_start.y, point_end.x, point_end.y);

            //
            VirtualColli life_colli;
            life_colli.pose.SetLifeTime(life_time);
            life_colli.pose.Inactive();
            life_colli.info                    = colli_info;
            virtual_colli_[colli_info.roi_key] = life_colli;

            if (colli_info.roi_start.x < color_detect_roi_start_.x) {
                color_detect_roi_start_.x = colli_info.roi_start.x;
            }
            if (colli_info.roi_start.y < color_detect_roi_start_.y) {
                color_detect_roi_start_.y = colli_info.roi_start.y;
            }
            if (colli_info.roi_end.x > color_detect_roi_end_.x) {
                color_detect_roi_end_.x = colli_info.roi_end.x;
            }
            if (colli_info.roi_end.y > color_detect_roi_end_.y) {
                color_detect_roi_end_.y = colli_info.roi_end.y;
            }
        }
    }
    // 再与sward_edge_roi_end_比较约束下边界，防止检测到车头
    color_detect_roi_end_.y = std::min(sward_edge_roi_end_.y, color_detect_roi_end_.y);
    LOGD("color_detect_roi_start(%d %d) color_detect_roi_end(%d %d)", color_detect_roi_start_.x,
         color_detect_roi_start_.y, color_detect_roi_end_.x, color_detect_roi_end_.y);
    // 得知颜色检测ROI后预分配内存
    color_detect_roi_ = cv::Rect(color_detect_roi_start_.x, color_detect_roi_start_.y,
                                 color_detect_roi_end_.x - color_detect_roi_start_.x,
                                 color_detect_roi_end_.y - color_detect_roi_start_.y);
    color_detect_roi_rgb_img_.create(color_detect_roi_.size(), CV_8UC3);
    color_detect_roi_hsv_img_.create(color_detect_roi_.size(), CV_8UC3);
    color_detect_roi_lab_img_.create(color_detect_roi_.size(), CV_8UC3);
    roi_sward_mask_is_sward_.create(color_detect_roi_.size(), CV_8UC1);
    roi_sward_mask_non_sward_.create(color_detect_roi_.size(), CV_8UC1);
    color_detect_result_mask_.create(color_detect_roi_.size(), CV_8UC1);

    LOGI("Virtual collision size: %d", virtual_colli_.size());
}

/**
 * @brief ai sense配置应用
 *
 */
void ProcessAISense::AiSenseConfig(json_t json_in) {
    json_t config_json;
    if (json_in.contains("perception")) {
        json_t perception_json = json_in["perception"];
        if (perception_json.contains("save_cloud_file") && perception_json["save_cloud_file"].is_boolean()) {
            save_cloud_file_ = perception_json["save_cloud_file"];
        }
        FLOGI("here save_cloud_file is %d", save_cloud_file_);
        if (perception_json.contains("is_single_camera_mode") &&
            perception_json["is_single_camera_mode"].is_boolean()) {
            is_single_camera_mode_ = perception_json["is_single_camera_mode"];
            p_depth_collect_->SetSingleCameraMode(is_single_camera_mode_);
        }
        LOGI("is_single_camera_mode: %d", is_single_camera_mode_);
        // 石板路相关功能
        if (perception_json.contains("func_cross_stone_road") && perception_json["func_cross_stone_road"].is_boolean()) {
            func_cross_stone_road_ = perception_json["func_cross_stone_road"];
        }
        LOGI("func_cross_stone_road: %d", func_cross_stone_road_);

        if (perception_json.contains("sensor_source") && perception_json["sensor_source"].is_number_unsigned()) {
            uint8_t sensor_source = perception_json["sensor_source"];
            LOGI("sensor_source: %u", sensor_source);
            PerceptionSensorSource sensor_source_tmp;
            sensor_source_tmp.SetSensor(sensor_source);
            LOGD("PCP_SENSOR_DEPTH_CAMERA:%d", sensor_source_tmp.HasSensor(PCP_SENSOR_DEPTH_CAMERA));
            LOGD("PCP_SENSOR_MONOCULAR_DEPTH_CAMERA:%d", sensor_source_tmp.HasSensor(PCP_SENSOR_MONOCULAR_DEPTH_CAMERA));
            LOGD("PCP_SENSOR_LIDAR:%d", sensor_source_tmp.HasSensor(PCP_SENSOR_LIDAR));
        }
        if (perception_json.contains("is_open_semantic") && perception_json["is_open_semantic"].is_boolean()) {
            is_open_semantic_ = perception_json["is_open_semantic"];
        }
        LOGI("is_open_semantic: %d", is_open_semantic_);

        // //////////////////////////////////////
        if (perception_json.contains("ai_sense")) {
            config_json = perception_json["ai_sense"];
            LOGI("ai_sense_config");
        }
    }
    // std::cout << "config_json :" << std::endl << config_json << std::endl;
    if (config_json.contains("obs_contours_is_rectangle") && config_json["obs_contours_is_rectangle"].is_boolean()) {
        obs_contours_is_rectangle_ = config_json["obs_contours_is_rectangle"];
    }
    FLOGI(" obs_contours_is_rectangle: %d", obs_contours_is_rectangle_);

    if (config_json.contains("need_exp_sward") && config_json["need_exp_sward"].is_boolean()) {
        need_exp_sward_ = config_json["need_exp_sward"];
    }
    FLOGI(" need_exp_sward: %d", need_exp_sward_);

    if (config_json.contains("use_rgb_img_type") && config_json["use_rgb_img_type"].is_string()) {
        std::string img_type_str = config_json["use_rgb_img_type"];
        if ("CAM_IMG_RGB" == img_type_str) {
            use_rgb_img_type_ = CAM_IMG_RGB;
        } else if ("CAM_IMG_RGB_L" == img_type_str) {
            use_rgb_img_type_ = CAM_IMG_RGB_L;
        } else if ("CAM_IMG_RGB_R" == img_type_str) {
            use_rgb_img_type_ = CAM_IMG_RGB_R;
        }
    }
    LOGD("use_rgb_img_type:%s", ENUM_STR(use_rgb_img_type_).c_str());

    if (config_json.contains("ignore_edge_pix_num") && config_json["ignore_edge_pix_num"].is_number()) {
        ignore_edge_pix_num_ = config_json["ignore_edge_pix_num"];
    }
    FLOGI(" ignore_edge_pix_num: %d", ignore_edge_pix_num_);

    if (config_json.contains("ai_edge_ignore_pix_dis") && config_json["ai_edge_ignore_pix_dis"].is_number()) {
        ai_edge_ignore_pix_dis_ = config_json["ai_edge_ignore_pix_dis"];
    }
    FLOGI(" ai_edge_ignore_pix_dis: %f", ai_edge_ignore_pix_dis_);

    if (config_json.contains("ai_edge_ignore_pix_angle") && config_json["ai_edge_ignore_pix_angle"].is_number()) {
        ai_edge_ignore_pix_angle_ = config_json["ai_edge_ignore_pix_angle"];
    }
    FLOGI(" ai_edge_ignore_pix_angle: %f deg", ai_edge_ignore_pix_angle_);

    double camera_y_blind = 0.17;
    if (config_json.contains("camera_blind_y") && config_json["camera_blind_y"].is_number()) {
        camera_y_blind = config_json["camera_blind_y"];
    }
    FLOGI("here init camera blind y dis %f", camera_y_blind);
    p_depth_collect_->SetCamBlindDisY(camera_y_blind);

    if (config_json.contains("dis_y_limit") && config_json["dis_y_limit"].is_number()) {
        double dis_y_limit = config_json["dis_y_limit"];
        p_depth_collect_->SetDisYLimit(dis_y_limit);
        FLOGI("here init dis_y_limit %f", dis_y_limit);
    }

    if (config_json.contains("dis_y_ai_limit") && config_json["dis_y_ai_limit"].is_number()) {
        double dis_y_ai_limit = config_json["dis_y_ai_limit"];
        p_depth_collect_->SetDisYAiLimit(dis_y_ai_limit);
        FLOGI("here init dis_y_ai_limit %f", dis_y_ai_limit);
    }

    if (config_json.contains("save_ai_contour") && config_json["save_ai_contour"].is_boolean()) {
        save_ai_contour_ = config_json["save_ai_contour"];
    }
    FLOGI(" save ai contour: %d", save_ai_contour_);

    if (config_json.contains("use_mag_tag_filt_sward") && config_json["use_mag_tag_filt_sward"].is_boolean()) {
        use_mag_tag_filt_sward_ = config_json["use_mag_tag_filt_sward"];
    }
    FLOGI("  use_mag_tag_filt_sward_: %d", use_mag_tag_filt_sward_);

    if (config_json.contains("cam_dark_ignore_ai_edge") && config_json["cam_dark_ignore_ai_edge"].is_boolean()) {
        cam_dark_ignore_ai_edge_ = config_json["cam_dark_ignore_ai_edge"];
    }
    FLOGI("  cam_dark_ignore_ai_edge_: %d", cam_dark_ignore_ai_edge_);

    if (config_json.contains("sward_wide_roi")) {
        json_t j_roi = config_json["sward_wide_roi"];
        if (j_roi.contains("start_x") && j_roi.contains("start_y") && j_roi.contains("end_x") &&
            j_roi.contains("end_y") && j_roi["start_x"].is_number() && j_roi["start_y"].is_number() &&
            j_roi["end_x"].is_number() && j_roi["end_y"].is_number()) {
            int start_x                 = j_roi["start_x"];
            int start_y                 = j_roi["start_y"];
            int end_x                   = j_roi["end_x"];
            int end_y                   = j_roi["end_y"];
            sward_check_wide_roi_start_ = cv::Point2i(start_x, start_y);
            sward_check_wide_roi_end_   = cv::Point2i(end_x, end_y);
        }
    }
    FLOGI("  wide_roi: (%d, %d) <-> (%d, %d)", sward_check_wide_roi_start_.x, sward_check_wide_roi_start_.y,
          sward_check_wide_roi_end_.x, sward_check_wide_roi_end_.y);

    if (config_json.contains("sward_narrow_roi")) {
        json_t j_roi = config_json["sward_narrow_roi"];
        if (j_roi.contains("start_x") && j_roi.contains("start_y") && j_roi.contains("end_x") &&
            j_roi.contains("end_y") && j_roi["start_x"].is_number() && j_roi["start_y"].is_number() &&
            j_roi["end_x"].is_number() && j_roi["end_y"].is_number()) {
            int start_x                         = j_roi["start_x"];
            int start_y                         = j_roi["start_y"];
            int end_x                           = j_roi["end_x"];
            int end_y                           = j_roi["end_y"];
            sward_check_narrow_roi_start_       = cv::Point2i(start_x, start_y);
            sward_check_narrow_roi_end_         = cv::Point2i(end_x, end_y);
        }
    }
    FLOGI("  narrow_roi: (%d, %d) <-> (%d, %d)", sward_check_narrow_roi_start_.x, sward_check_narrow_roi_start_.y,
          sward_check_narrow_roi_end_.x, sward_check_narrow_roi_end_.y);

    if (config_json.contains("use_sward_edge_smoother") && config_json["use_sward_edge_smoother"].is_boolean()) {
        use_sward_edge_smoother_ = config_json["use_sward_edge_smoother"];
    }
    FLOGI("use_sward_edge_smoother_: %d", use_sward_edge_smoother_);

    if (config_json.contains("func_check_roi_far_and_near") && config_json["func_check_roi_far_and_near"].is_boolean()) {
        func_check_roi_far_and_near_ = config_json["func_check_roi_far_and_near"];
    }
    FLOGI("func_check_roi_far_and_near: %d", func_check_roi_far_and_near_);

    if (func_check_roi_far_and_near_ && config_json.contains("check_roi_far_and_near")) {
        for (auto& j_roi : config_json["check_roi_far_and_near"]) {
            if (j_roi.contains("start_x") && j_roi.contains("start_y") && j_roi.contains("end_x") &&
                j_roi.contains("end_y") && j_roi["start_x"].is_number() && j_roi["start_y"].is_number() &&
                j_roi["end_x"].is_number() && j_roi["end_y"].is_number() && j_roi["id"].is_number() &&
                j_roi["id"].is_number()) {
                int start_x = j_roi["start_x"];
                int start_y = j_roi["start_y"];
                int end_x   = j_roi["end_x"];
                int end_y   = j_roi["end_y"];

                SwardCheckROIInfo roi;
                roi.id        = j_roi["id"];
                roi.roi_start = cv::Point2i(start_x, start_y);
                roi.roi_end   = cv::Point2i(end_x, end_y);
                check_roi_buff_.emplace_back(roi);

                FLOGI("check_roi_far_and_near(%d): (%d, %d) <-> (%d, %d)", roi.id, roi.roi_start.x, roi.roi_start.y,
                      roi.roi_end.x, roi.roi_end.y);
            }
        }
    }

    if (config_json.contains("sward_smoother_kernel_size") && config_json["sward_smoother_kernel_size"].is_number()) {
        sward_smoother_kernel_size_ = config_json["sward_smoother_kernel_size"];
    }
    FLOGI("sward_smoother_kernel_size: %d", sward_smoother_kernel_size_);

    if (config_json.contains("sward_edge_roi")) {
        json_t j_roi = config_json["sward_edge_roi"];
        if (j_roi.contains("start_x") && j_roi.contains("start_y") && j_roi.contains("end_x") &&
            j_roi.contains("end_y") && j_roi["start_x"].is_number() && j_roi["start_y"].is_number() &&
            j_roi["end_x"].is_number() && j_roi["end_y"].is_number()) {
            int start_x           = j_roi["start_x"];
            int start_y           = j_roi["start_y"];
            int end_x             = j_roi["end_x"];
            int end_y             = j_roi["end_y"];
            sward_edge_roi_start_ = cv::Point2i(start_x, start_y);
            sward_edge_roi_end_   = cv::Point2i(end_x, end_y);
            sward_edge_roi_       = cv::Rect(sward_edge_roi_start_, sward_edge_roi_end_);
        }
    }
    FLOGI("sward_edge_roi: (%d, %d) <-> (%d, %d)", sward_edge_roi_start_.x, sward_edge_roi_start_.y,
          sward_edge_roi_end_.x, sward_edge_roi_end_.y);

    if (config_json.contains("need_filt_inner_edge_ai_sward_edge") &&
        config_json["need_filt_inner_edge_ai_sward_edge"].is_boolean()) {
        need_filt_inner_edge_ai_sward_edge_ = config_json["need_filt_inner_edge_ai_sward_edge"];
    }
    FLOGI("need_filt_inner_edge_ai_sward_edge: %d", need_filt_inner_edge_ai_sward_edge_);

    if (config_json.contains("ai_sward_inner_edge_filter_bottom") &&
        config_json["ai_sward_inner_edge_filter_bottom"].is_number()) {
        ai_sward_inner_edge_filter_bot_ = config_json["ai_sward_inner_edge_filter_bottom"];
    }
    FLOGI("ai_sward_inner_edge_filter_bottom: %f", ai_sward_inner_edge_filter_bot_);

    if (config_json.contains("filter_surface_area_min") && config_json["filter_surface_area_min"].is_number()) {
        filter_surface_area_min_ = config_json["filter_surface_area_min"];
    }
    FLOGI("filter_surface_area_min: %f", filter_surface_area_min_);

    if (config_json.contains("pose_maybe_mag_tag_lifetime") && config_json["pose_maybe_mag_tag_lifetime"].is_number()) {
        pose_maybe_mag_tag_lifetime_ = config_json["pose_maybe_mag_tag_lifetime"];
    }
    if (pose_maybe_mag_tag_lifetime_ > 15 * 1000) {
        pose_maybe_mag_tag_lifetime_ = 15 * 1000;
    }
    FLOGI("pose_maybe_mag_tag_lifetime: %ld ms", pose_maybe_mag_tag_lifetime_);
    pose_maybe_mag_tag_.SetLifeTime(pose_maybe_mag_tag_lifetime_);

    if (config_json.contains("filter_maybe_mag_tag_width") && config_json["filter_maybe_mag_tag_width"].is_number()) {
        filter_maybe_mag_tag_width_ = config_json["filter_maybe_mag_tag_width"];
    }
    FLOGI("filter_maybe_mag_tag_width: %f ", filter_maybe_mag_tag_width_);

    if (config_json.contains("filter_maybe_mag_tag_height") && config_json["filter_maybe_mag_tag_height"].is_number()) {
        filter_maybe_mag_tag_height_ = config_json["filter_maybe_mag_tag_height"];
    }
    FLOGI("filter_maybe_mag_tag_height: %f ", filter_maybe_mag_tag_height_);

    // 解析虚拟碰撞配置
    if (config_json.contains("use_virtual_collision") && config_json["use_virtual_collision"].is_boolean()) {
        use_virtual_collision_ = config_json["use_virtual_collision"];
    }
    FLOGI("ai use_virtual_collision is %d", use_virtual_collision_);

    if (config_json.contains("virtual_collision_keep_enable") && config_json["virtual_collision_keep_enable"].is_boolean()) {
        virtual_collision_keep_enable_ = config_json["virtual_collision_keep_enable"];
    }
    FLOGI("ai virtual_collision_keep_enable is %d", virtual_collision_keep_enable_);

    if (config_json.contains("virtual_collision_life_time") && config_json["virtual_collision_life_time"].is_number()) {
        life_time_virtual_colli_ = config_json["virtual_collision_life_time"];
    }
    FLOGI("virtual_collision_life_time: %ld ", life_time_virtual_colli_);

    if (config_json.contains("virtual_colli_w_limit") && config_json["virtual_colli_w_limit"].is_number()) {
        virtual_colli_w_limit_ = config_json["virtual_colli_w_limit"];
    }
    FLOGI("virtual_colli_w_limit: %f ", virtual_colli_w_limit_);
    VirtualCollisionConfig(config_json);
    // 获取虚拟碰撞功能用户配置
    if (auto [ret, value] = RUN_CONFIG->GetJsonValue<uint8_t>("ai_sensitivity"); ret) {
        LOGI("ai_sensitivity: %d ", value);
        if (value) {
            AppEnableFuncVirtualCollision();
            LOGI("User config AppEnableFuncVirtualCollision ");
        } else {
            AppDisableFuncVirtualCollision();
            LOGI("User config AppDisableFuncVirtualCollision ");
        }
    }

    if (config_json.contains("erode_sward_rad") && config_json["erode_sward_rad"].is_number()) {
        erode_sward_rad_ = config_json["erode_sward_rad"];
    }
    FLOGI("  erode_sward_rad: %d", erode_sward_rad_);

    if (config_json.contains("inner_erode_sward_rad") && config_json["inner_erode_sward_rad"].is_number()) {
        inner_erode_sward_rad_ = config_json["inner_erode_sward_rad"];
    }
    FLOGI("  inner_erode_sward_rad: %d", inner_erode_sward_rad_);


    JsonGet(config_json, "depth_offset", depth_offset_);

    JsonGet(config_json, "obs_depth_interval", obs_depth_interval_);

    // 获取跨越石板路功能相关配置
    if (auto [ret, value] = RUN_CONFIG->GetJsonValue<bool>("cross_stone_road"); ret) {
        if (value) {
            SetFuncCrossStoneRoad(true);
            LOGD("Perception: SetFuncCrossStoneRoad: true");
        } else {
            SetFuncCrossStoneRoad(false);
            LOGD("Perception: SetFuncCrossStoneRoad: false");
        }
    }

    JsonGet(config_json, "ai_ignore_roi_array", ai_ignore_roi_array_);
    for (const auto& roi : ai_ignore_roi_array_) {
        LOGI("ai_ignore_roi: [%d, %d, %d, %d] ", roi.x, roi.y, roi.width, roi.height);
    }

    // 是否开启颜色检测
    enable_color_detect_ = false;
    if (config_json.contains("enable_color_detect") && config_json["enable_color_detect"].is_boolean()) {
        enable_color_detect_ = config_json["enable_color_detect"];
        // 读取HSV LAB颜色范围，然后调用AddColorParams增加颜色参数
        if (enable_color_detect_ && config_json.contains("color_check") && config_json["color_check"].is_array()) {
            for (auto& color : config_json["color_check"]) {
                if (color.contains("color_name") && color["color_name"].is_string()) {
                    std::string color_name = color["color_name"];
                    if (
                        color.contains("hsv_h_low") && color["hsv_h_low"].is_number() && 
                        color.contains("hsv_h_high") && color["hsv_h_high"].is_number() && 
                        color.contains("hsv_s_low") && color["hsv_s_low"].is_number() && 
                        color.contains("hsv_s_high") && color["hsv_s_high"].is_number() && 
                        color.contains("hsv_v_low") && color["hsv_v_low"].is_number() && 
                        color.contains("hsv_v_high") && color["hsv_v_high"].is_number() && 
                        color.contains("lab_l_low") && color["lab_l_low"].is_number() && 
                        color.contains("lab_l_high") && color["lab_l_high"].is_number() && 
                        color.contains("lab_a_low") && color["lab_a_low"].is_number() && 
                        color.contains("lab_a_high") && color["lab_a_high"].is_number() && 
                        color.contains("lab_b_low") && color["lab_b_low"].is_number() && 
                        color.contains("lab_b_high") && color["lab_b_high"].is_number()) {
                        ColorParams color_params;
                        color_params.color_hsv.HSV_H_L = color["hsv_h_low"];
                        color_params.color_hsv.HSV_H_H = color["hsv_h_high"];
                        color_params.color_hsv.HSV_S_L = color["hsv_s_low"];
                        color_params.color_hsv.HSV_S_H = color["hsv_s_high"];
                        color_params.color_hsv.HSV_V_L = color["hsv_v_low"];
                        color_params.color_hsv.HSV_V_H = color["hsv_v_high"];
                        color_params.color_lab.LAB_L_L = color["lab_l_low"];
                        color_params.color_lab.LAB_L_H = color["lab_l_high"];
                        color_params.color_lab.LAB_A_L = color["lab_a_low"];
                        color_params.color_lab.LAB_A_H = color["lab_a_high"];
                        color_params.color_lab.LAB_B_L = color["lab_b_low"];
                        color_params.color_lab.LAB_B_H = color["lab_b_high"];
                        AddColorParams(color_name, color_params);
                        LOGD("AddColorParams: %s HSV(%d %d %d)-(%d %d %d)  LAB(%d %d %d)-(%d %d %d)", color_name.c_str(), 
                             color_params.color_hsv.HSV_H_L, color_params.color_hsv.HSV_S_L, color_params.color_hsv.HSV_V_L,
                             color_params.color_hsv.HSV_H_H, color_params.color_hsv.HSV_S_H, color_params.color_hsv.HSV_V_H,
                             color_params.color_lab.LAB_L_L, color_params.color_lab.LAB_A_L, color_params.color_lab.LAB_B_L,
                             color_params.color_lab.LAB_L_H, color_params.color_lab.LAB_A_H, color_params.color_lab.LAB_B_H);
                    }
                }
            }
        }
    }
    LOGD("enable_color_detect: %d", enable_color_detect_);

//    // 配置文件加载完才开启线程
//    StartProcessAIRes();
}

void MsgManager::InitConfig(const json_t& json) {
    auto j_pcp = json.at("perception");

    // 骑边总开关
    JsonGet(j_pcp, "enable_edge_cutting", enable_edge_cutting_);
}

void ThermalCamera::InitConfig(json_t& cfg_json) {
    FLOGD("here init config thermal_camera");
    if (cfg_json.contains("thermal_camera")) {
        json_t para = cfg_json["thermal_camera"];

        if (para.contains("has_thermal") && para["has_thermal"].is_boolean()) {
            has_thermal_ = para["has_thermal"];
            LOGD("json config has_thermal %d", has_thermal_);
        }

        if (para.contains("print_cloud_to_movable") && para["print_cloud_to_movable"].is_boolean()) {
            print_cloud_to_movable_ = para["print_cloud_to_movable"];
            LOGD("json config print_cloud_to_movable %d", print_cloud_to_movable_);
        }

        if (para.contains("print_depth_img") && para["print_depth_img"].is_boolean()) {
            print_depth_img_ = para["print_depth_img"];
            LOGD("json config print_depth_img %d", print_depth_img_);
        }

        if (para.contains("print_temp_img") && para["print_temp_img"].is_boolean()) {
            print_temp_img_ = para["print_temp_img"];
            LOGD("json config print_temp_img %d", print_temp_img_);
        }

        if (para.contains("fx") && para["fx"].is_number()) {
            fx_ = para["fx"];
            LOGD("json config fx %f", fx_);
        }

        if (para.contains("fy") && para["fy"].is_number()) {
            fy_ = para["fy"];
            LOGD("json config fy %f", fy_);
        }

        if (para.contains("cx") && para["cx"].is_number()) {
            cx_ = para["cx"];
            LOGD("json config cx %f", cx_);
        }

        if (para.contains("cy") && para["cy"].is_number()) {
            cy_ = para["cy"];
            LOGD("json config cy %f", cy_);
        }

        if (para.contains("rx") && para["rx"].is_number()) {
            rx_ = para["rx"];
            LOGD("json config rx %f", rx_);
        }

        if (para.contains("tz") && para["tz"].is_number()) {
            tz_ = para["tz"];
            LOGD("json config tz %f", tz_);
        }

        if (para.contains("tx") && para["tx"].is_number()) {
            tx_ = para["tx"];
            LOGD("json config tx %f", tx_);
        }

        if (para.contains("ty") && para["ty"].is_number()) {
            ty_ = para["ty"];
            LOGD("json config ty %f", ty_);
        }

        if (para.contains("temp_show_min") && para["temp_show_min"].is_number()) {
            temp_show_min_ = para["temp_show_min"];
            LOGD("json config temp_show_min %f", temp_show_min_);
        }

        if (para.contains("temp_show_max") && para["temp_show_max"].is_number()) {
            temp_show_max_ = para["temp_show_max"];
            LOGD("json config temp_show_max %f", temp_show_max_);
        }

        if (para.contains("cont_ratio") && para["cont_ratio"].is_number()) {
            cont_ratio_ = para["cont_ratio"];
            LOGD("json config cont_ratio %f", cont_ratio_);
        }


        if (para.contains("thresh_value") && para["thresh_value"].is_number()) {
            thresh_value_ = para["thresh_value"];
            LOGD("json config thresh_value_ %d", thresh_value_);
        }

        if (para.contains("erode_temp_rad") && para["erode_temp_rad"].is_number()) {
            erode_temp_rad_ = para["erode_temp_rad"];
            LOGD("json config erode_temp_rad_ %d", erode_temp_rad_);
        }

        if (para.contains("area_thresh") && para["area_thresh"].is_number()) {
            area_thresh_ = para["area_thresh"];
            LOGD("json config area_thresh %f", area_thresh_);
        }

        if (para.contains("min_temp") && para["min_temp"].is_number()) {
            min_thermal_temp_ = para["min_temp"];
            LOGD("json config min_thermal_temp_ %f", min_thermal_temp_);
        }

    }

    update_boundingbox_.push_back(pcl::PointXYZ(-0.16, 0.18, -0.117));
    update_boundingbox_.push_back(pcl::PointXYZ(0.16, 0.18, -0.117));
    update_boundingbox_.push_back(pcl::PointXYZ(1.65, 1.92, 0.65));
    update_boundingbox_.push_back(pcl::PointXYZ(-1.65, 1.92, 0.65));

    IdealDepthTableInit();
}

#if defined(PLATFORM_X5)
void StereoDepth::InitStereoDepthConfig(json_t &stereo_depth_cfg){
    if(stereo_depth_cfg.contains("keep_open_stereo_depth")){
        if(stereo_depth_cfg["keep_open_stereo_depth"].is_boolean()){
            config_params_ptr_->keep_open_stereo_depth = stereo_depth_cfg["keep_open_stereo_depth"];
            if(config_params_ptr_->keep_open_stereo_depth){
                LOGI("tsl,keep_open_stereo_depth is True");
            }else{
                LOGW("tsl,stereo is closed.");
                return;
            }
        }
    }
    if(!config_params_ptr_->keep_open_stereo_depth) {
        LOGW("tsl,stereo is closed.");
        return;
    }

    if(stereo_depth_cfg.contains("model_bin_path") ){
        if(stereo_depth_cfg["model_bin_path"].is_string()){
            config_params_ptr_->model_bin_path = stereo_depth_cfg["model_bin_path"];
            LOGI("tsl,model_bin_path -> %s",config_params_ptr_->model_bin_path.c_str());
        }
    }
    if(stereo_depth_cfg.contains("maxdisp")){
        if(stereo_depth_cfg["maxdisp"].is_number()){
            config_params_ptr_->maxdisp = stereo_depth_cfg["maxdisp"];
        }
    }
    if(stereo_depth_cfg.contains("min_distance_threash")){
        if(stereo_depth_cfg["min_distance_threash"].is_number()){
            config_params_ptr_->min_distance_threash = stereo_depth_cfg["min_distance_threash"];
        }
    }
    if(stereo_depth_cfg.contains("max_distance_threash")){
        if(stereo_depth_cfg["max_distance_threash"].is_number()){
            config_params_ptr_->max_distance_threash = stereo_depth_cfg["max_distance_threash"];
        }
    }
    if(stereo_depth_cfg.contains("max_pix_threash")){
        if(stereo_depth_cfg["max_pix_threash"].is_number()){
            config_params_ptr_->max_pix_threash = stereo_depth_cfg["max_pix_threash"];
        }
    }
    if(stereo_depth_cfg.contains("bxf")){
        if(stereo_depth_cfg["bxf"].is_number()){
            config_params_ptr_->bxf = stereo_depth_cfg["bxf"];
        }
    }
    if(stereo_depth_cfg.contains("center_crop")){
        if(stereo_depth_cfg["center_crop"].is_boolean()){
            config_params_ptr_->center_crop = stereo_depth_cfg["center_crop"];
        }
    }
    if(stereo_depth_cfg.contains("save_img")){
        if(stereo_depth_cfg["save_img"].is_boolean()){
            config_params_ptr_->save_img = stereo_depth_cfg["save_img"];
        }
    }
    if(stereo_depth_cfg.contains("test_left_path") ){
        if(stereo_depth_cfg["test_left_path"].is_string()){
            config_params_ptr_->left_path = stereo_depth_cfg["test_left_path"];
        }
    }
    if(stereo_depth_cfg.contains("test_right_path") ){
        if(stereo_depth_cfg["test_right_path"].is_string()){
            config_params_ptr_->right_path = stereo_depth_cfg["test_right_path"];
        }
    }
    if(stereo_depth_cfg.contains("model_bin_id") ){
        if(stereo_depth_cfg["model_bin_id"].is_string()){
            config_params_ptr_->model_bin_id = stereo_depth_cfg["model_bin_id"];
        }
    }
    if(stereo_depth_cfg.contains("sleep_time") ){
        if(stereo_depth_cfg["sleep_time"].is_number()){
            config_params_ptr_->sleep_time = stereo_depth_cfg["sleep_time"];
        }
    }
    if(stereo_depth_cfg.contains("roi")){
        if(stereo_depth_cfg["roi"].contains("x1")){
            if(stereo_depth_cfg["roi"]["x1"].is_number()){
                config_params_ptr_->roi.x = stereo_depth_cfg["roi"]["x1"];
            }
            if(stereo_depth_cfg["roi"]["y1"].is_number()){
                config_params_ptr_->roi.y = stereo_depth_cfg["roi"]["y1"];
            }
            if(stereo_depth_cfg["roi"]["x2"].is_number()){
                config_params_ptr_->roi.width = stereo_depth_cfg["roi"]["x2"];// - config_params_ptr_->roi.x;
                config_params_ptr_->roi.width  -= config_params_ptr_->roi.x;
            }
            if(stereo_depth_cfg["roi"]["y2"].is_number()){
                config_params_ptr_->roi.height = stereo_depth_cfg["roi"]["y2"];// -  config_params_ptr_->roi.y;
                config_params_ptr_->roi.height -= config_params_ptr_->roi.y;
            }
        }
    }

    if(stereo_depth_cfg.contains("send_image")){
        if(stereo_depth_cfg["send_image"].is_boolean()){
            config_params_ptr_->send_image=stereo_depth_cfg["send_image"];
        }
    }
}
#endif

#if defined(PLATFORM_X3M)
void DepthProcess::LoadConfig(json_t &config_json){
    LOGD("DepthProcess::LoadConfig");

    if(config_json.contains("perception")){
        json_t perception_json = config_json["perception"];
        if (perception_json.contains("keep_open_pcp") &&
            perception_json["keep_open_pcp"].is_boolean()) {
            config_params_.keep_open_pcp = perception_json["keep_open_pcp"];
        }
        if(perception_json.contains("depth")){
            if(perception_json["depth"].contains("enable_mono_depth") && perception_json["depth"]["enable_mono_depth"].is_boolean()){
                config_params_.enable_mono_depth = perception_json["depth"]["enable_mono_depth"];
            }
            if(perception_json["depth"].contains("logging_enabled") && perception_json["depth"]["logging_enabled"].is_boolean()){
                config_params_.logging_enabled = perception_json["depth"]["logging_enabled"];
            }
            if(perception_json["depth"].contains("enable_save_images") && perception_json["depth"]["enable_save_images"].is_boolean()){
                config_params_.enable_save_images = perception_json["depth"]["enable_save_images"];
            }
            if(perception_json["depth"].contains("enable_online_fitting") && perception_json["depth"]["enable_online_fitting"].is_boolean()){
                config_params_.enable_online_fitting = perception_json["depth"]["enable_online_fitting"];
            }
            if(perception_json["depth"].contains("model_bin_path") ){
                config_params_.model_bin_path = perception_json["depth"]["model_bin_path"];
            }
            if(perception_json["depth"].contains("model_bin_id") ){
                config_params_.model_bin_id = perception_json["depth"]["model_bin_id"];
            }
            if(perception_json["depth"].contains("pub_depth_h") && perception_json["depth"]["pub_depth_h"].is_number()){
                config_params_.pub_depth_h = perception_json["depth"]["pub_depth_h"];
            }
            if(perception_json["depth"].contains("pub_depth_w") && perception_json["depth"]["depth_depth_w"].is_number() ){
                config_params_.pub_depth_w = perception_json["depth"]["pub_depth_w"];
            }
            if(perception_json["depth"].contains("discard_depth") && perception_json["depth"]["discard_depth"].is_number()){
                config_params_.discard_depth = perception_json["depth"]["discard_depth"];
            }
            if(perception_json["depth"].contains("robot_head_distance_from_camera") && perception_json["depth"]["robot_head_distance_from_camera"].is_number()){
                config_params_.robot_head_distance_from_camera = perception_json["depth"]["robot_head_distance_from_camera"];
            }
            if(perception_json["depth"].contains("x1") && perception_json["depth"]["x1"].is_number()){
                config_params_.point1.x = perception_json["depth"]["x1"];
            }
            if(perception_json["depth"].contains("y1") && perception_json["depth"]["y1"].is_number()){
                config_params_.point1.y = perception_json["depth"]["y1"];
            }
            if(perception_json["depth"].contains("x2") && perception_json["depth"]["x2"].is_number()){
                config_params_.point2.x = perception_json["depth"]["x2"];
            }
            if(perception_json["depth"].contains("y2") && perception_json["depth"]["y2"].is_number()){
                config_params_.point2.y = perception_json["depth"]["y2"];
            }
            if(perception_json["depth"].contains("x3") && perception_json["depth"]["x3"].is_number()){
                config_params_.point3.x = perception_json["depth"]["x3"];
            }
            if(perception_json["depth"].contains("y3") && perception_json["depth"]["y3"].is_number()){
                config_params_.point3.y = perception_json["depth"]["y3"];
            }
            if(perception_json["depth"].contains("x4") && perception_json["depth"]["x4"].is_number()){
                config_params_.point4.x = perception_json["depth"]["x4"];
            }
            if(perception_json["depth"].contains("y4") && perception_json["depth"]["y4"].is_number()){
                config_params_.point4.y = perception_json["depth"]["y4"];
            }

            if(perception_json["depth"].contains("use_img_type") && perception_json["depth"]["use_img_type"].is_string() ){
                std::string img_type_str = perception_json["depth"]["use_img_type"];
                if ("CAM_IMG_DEPTH" == img_type_str) {
                    config_params_.use_img_type = CAM_IMG_DEPTH;
                } else if ("CAM_IMG_RGB" == img_type_str) {
                    config_params_.use_img_type = CAM_IMG_RGB;
                } else if ("CAM_IMG_RGB_L" == img_type_str) {
                    config_params_.use_img_type = CAM_IMG_RGB_L;
                } else if ("CAM_IMG_RGB_R" == img_type_str) {
                    config_params_.use_img_type = CAM_IMG_RGB_R;
                }
            }
            if(perception_json["depth"].contains("offline_fitcoef")){
                if(perception_json["depth"]["offline_fitcoef"].contains("a") && perception_json["depth"]["offline_fitcoef"]["a"].is_number()){
                    config_params_.offline_fitcoef.a = perception_json["depth"]["offline_fitcoef"]["a"];
                }
                if(perception_json["depth"]["offline_fitcoef"].contains("b") && perception_json["depth"]["offline_fitcoef"]["b"].is_number()){
                    config_params_.offline_fitcoef.b = perception_json["depth"]["offline_fitcoef"]["b"];
                }
                if(perception_json["depth"]["offline_fitcoef"].contains("c") && perception_json["depth"]["offline_fitcoef"]["c"].is_number()){
                    config_params_.offline_fitcoef.c = perception_json["depth"]["offline_fitcoef"]["c"];
                }
                if(perception_json["depth"]["offline_fitcoef"].contains("a2") && perception_json["depth"]["offline_fitcoef"]["a2"].is_number()){
                    config_params_.offline_fitcoef.a2 = perception_json["depth"]["offline_fitcoef"]["a2"];
                }
                if(perception_json["depth"]["offline_fitcoef"].contains("b2") && perception_json["depth"]["offline_fitcoef"]["b2"].is_number()){
                    config_params_.offline_fitcoef.b2 = perception_json["depth"]["offline_fitcoef"]["b2"];
                }
                if(perception_json["depth"]["offline_fitcoef"].contains("c2") && perception_json["depth"]["offline_fitcoef"]["c2"].is_number()){
                    config_params_.offline_fitcoef.c2 = perception_json["depth"]["offline_fitcoef"]["c2"];
                }
                if(perception_json["depth"]["offline_fitcoef"].contains("slice_distance") && perception_json["depth"]["offline_fitcoef"]["slice_distance"].is_number()){
                    config_params_.offline_fitcoef.slice_distance = perception_json["depth"]["offline_fitcoef"]["slice_distance"];
                }

                if(perception_json["depth"]["offline_fitcoef"].contains("rate") && perception_json["depth"]["offline_fitcoef"]["rate"].is_number()){
                    config_params_.offline_fitcoef.rate = perception_json["depth"]["offline_fitcoef"]["rate"];
                }
                if(perception_json["depth"]["offline_fitcoef"].contains("intercept") && perception_json["depth"]["offline_fitcoef"]["intercept"].is_number()){
                    config_params_.offline_fitcoef.intercept = perception_json["depth"]["offline_fitcoef"]["intercept"];
                }
                // pivot
                if(perception_json["depth"]["offline_fitcoef"].contains("pivot") && perception_json["depth"]["offline_fitcoef"]["pivot"].is_number()){
                    config_params_.offline_fitcoef.pivot = perception_json["depth"]["offline_fitcoef"]["pivot"];
                }

            }
            if(perception_json["depth"].contains("local_test") && perception_json["depth"]["local_test"].is_boolean()){
                config_params_.local_test = perception_json["depth"]["local_test"];
            }

        }
    } else{
        LOGD("tsl,do not find depth config, so using default");
    }
}
#endif //
