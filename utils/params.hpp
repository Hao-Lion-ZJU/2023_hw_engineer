//
// Created by liming on 12/26/17.
// Co-modified by dangwi & Julian Lin on 2022-03-07 22:01:31
// modified by HaoLion on 2022-12-08 12:01:31

#ifndef __PARAMS_H__
#define __PARAMS_H__

#include <opencv2/opencv.hpp>
#include <string>

struct Robot
{
    float yaw;
    float pitch;
    float roll;
};
extern Robot g_robot;

struct Params
{
    Params();
    ~Params() = default;

    void loadCamera(const std::string &camera_path);

    void loadConfig(const std::string &config);

    void loadMode(const std::string &mode_path);

    void print();

    std::string robot_name; // 当前机器人的名字，用于加载不同参数

    int anti_top_mode = 0; // 0：不开启反陀螺，1：开启反陀螺
    int mode;              // 0：not working； 1：auto-shooooot
    int target_color;      // 0: all 1: blue 2: red

    bool enable_predict;
    bool enable_auto_change;
    bool enable_auto_shoot;
    bool debug;

    int camera_type;
    std::string mv_camera_name; // 'auto' for anynoe
    std::string video_path;

    bool enable_serial;
    bool enable_camera;
    std::string engine_name;

    bool show_source_img;
    bool show_debug;
    bool show_pose;
    bool show_predict;
    bool show_digitx;
    bool save_video;

    std::string big_template_path;
    std::string small_template_path;
    std::string small_net_path;
    std::string big_net_path;

    struct Cam
    {
        cv::Mat CI_MAT; // 内参矩阵
        cv::Mat D_MAT;  // 畸变矩阵
        float width, height;
        float cx, cy, fx, fy;
        float k1, k2, p1, p2, k3;
        /* MVCamera */
        int exposure;
        int rGain, gGain, bGain;
        int analogGain;
        int gamma;
        int contrast;
    } camera;
    // multi_cameras用于哨兵底盘、双目等多相机场景，单目用不到
    Cam multi_cameras[2];

   /**
    * @brief 描述角标参数句柄
     */
    struct {
        //预处理
        int brightness_threshold;
        float template_extend_ratio;
        float Gama;
        //箭头
        float rect_min_area;
        float arrow_min_size;
        float rect_max_ratio;
        float mtach_step;		//旋转步距角
        float score_threshold;

        //大小箭头
        float arrow_max_lenth;
    }arrow_para;

    struct
    {
        int hmin, smin, vmin;
        int hmax, smax, vmax;
        int blue_hmin, blue_hmax;
        int red_hmin, red_hmax;
    } hsv;


    struct
    {
        float x;
        float y;
        float z;
    } distance_of_cam_fric, distance_of_cam_spindle;

    struct {
        float roll;
        float pitch;
        float yaw;
    } imu_compensate;

};

extern Params param;

#endif //__PARAMS_H__
