//
// Created by helloworld on 2021/3/7.
// Co-modified by dangwi & Julian Lin on 2022-03-07 22:01:31
//
#include <Eigen/Dense>
#include <opencv2/core//eigen.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <glog/logging.h>
#include <iostream>

#include "params.hpp"

using namespace std;
using namespace cv;

Robot g_robot;
Params param;

namespace cv {
    // Define a new bool reader in order to accept "true/false"-like values.
    void read_bool(const cv::FileNode &node, bool &value, const bool &default_value) {
        std::string s(static_cast<std::string>(node));
        if(s=="y"||s=="Y"||s=="yes"||s=="Yes"||s=="YES"||s=="true"||s=="True"||s=="TRUE"||s=="on"||s=="On"||s=="ON")
            {value=true; return;}
        if(s=="n"||s=="N"||s=="no"||s=="No"||s=="NO"||s=="false"||s=="False"||s=="FALSE"||s=="off"||s=="Off"||s=="OFF")
            {value=false; return;}
        value= static_cast<int>(node);
    }
    // Specialize cv::operator>> for bool.
    template<> inline void operator >> (const cv::FileNode& n, bool& value) {
        read_bool(n, value, false);
    }
} // cv

Params::Params()
{
    loadMode("../config/mode.yml");
    loadConfig("../config/marker_config.yml");
    loadCamera("../config/camera.yml");

    print();
}

void Params::loadMode(const string &mode_path)
{
    FileStorage f(mode_path, FileStorage::READ);



    f["camera_type"]            >> camera_type;
    f["enable_serial"]          >> enable_serial;
    f["enable_camera"]          >> enable_camera;
    f["debug"]                  >> debug;
    f["engine_name"]            >> engine_name;
    f["mv_camera_name"]         >> mv_camera_name;
    f["video_path"]             >> video_path;
    f["big_template_path"]      >> big_template_path;
    f["small_template_path"]    >> small_template_path;
    f["big_net_path"]           >> big_net_path;
    f["small_net_path"]         >> small_net_path;

    f["save_video"]      >> save_video;
    f["show_source_img"] >> show_source_img;
    f["show_debug"]      >>  show_debug;
    f["show_pose"]       >> show_pose;
    f["show_predict"]    >> show_predict;
    f["show_digitx"]     >> show_digitx;
    f["target_color"]    >> target_color;

    f["predict"]         >> enable_predict;
    f["auto_change"]     >> enable_auto_change;
    f["auto_shoot"]      >> enable_auto_shoot;

    f["robot_name"]      >> robot_name;
}

void Params::loadCamera(const string &camera_path) {
    // read calibration parameters
    cv::FileStorage fCamera(camera_path, cv::FileStorage::READ);
    // 小彩石与众不同
    if ( robot_name == "小彩石底盘" ) {
        for (size_t i=0; i<2; i++) {
            fCamera["小彩石底盘USB相机"+to_string(i+1)]["CI"] >> multi_cameras[i].CI_MAT;
            fCamera["小彩石底盘USB相机"+to_string(i+1)]["D"]  >> multi_cameras[i].D_MAT;
            LOG_IF(WARNING, multi_cameras[i].CI_MAT.empty()) << "CONFIG : CI MAT empty";
            LOG_IF(WARNING, multi_cameras[i].D_MAT.empty())  << "CONFIG : D MAT empty";
            multi_cameras[i].fx = multi_cameras[i].CI_MAT.at<double>(0, 0);
            multi_cameras[i].fy = multi_cameras[i].CI_MAT.at<double>(1, 1);
            multi_cameras[i].cx = multi_cameras[i].CI_MAT.at<double>(0, 2);
            multi_cameras[i].cy = multi_cameras[i].CI_MAT.at<double>(1, 2);
        }
    }
    else {
        fCamera[robot_name]["CI"] >> camera.CI_MAT;
        fCamera[robot_name]["D"] >> camera.D_MAT;
        LOG_IF(WARNING, camera.CI_MAT.empty()) << "CONFIG : CI MAT empty";
        LOG_IF(WARNING, camera.D_MAT.empty())  << "CONFIG : D MAT empty";
        camera.fx = camera.CI_MAT.at<double>(0, 0);
        camera.fy = camera.CI_MAT.at<double>(1, 1);
        camera.cx = camera.CI_MAT.at<double>(0, 2);
        camera.cy = camera.CI_MAT.at<double>(1, 2);
        camera.k1 = camera.D_MAT.at<double>(0,0);
        camera.k2 = camera.D_MAT.at<double>(0,1);
        camera.p1 = camera.D_MAT.at<double>(0,2);
        camera.p2 = camera.D_MAT.at<double>(0,3);
        camera.k3 = camera.D_MAT.at<double>(0,4);
        fCamera["exposure"]   >> camera.exposure;
        fCamera["rGain"]      >> camera.rGain;
        fCamera["gGain"]      >> camera.gGain;
        fCamera["bGain"]      >> camera.bGain;
        fCamera["analogGain"] >> camera.analogGain;
        fCamera["gamma"]      >> camera.gamma;
        fCamera["contrast"]   >> camera.contrast;
    }
}

void Params::loadConfig(const string &config) {
    // read config parameters
    cv::FileStorage fConfig(config, cv::FileStorage::READ);
    //pretreatment
    hsv.hmin = fConfig["pretreatment.hmin"];
    hsv.hmax = fConfig["pretreatment.hmax"];
    hsv.blue_hmin = fConfig["pretreatment.blue_hmin"];
    hsv.blue_hmax = fConfig["pretreatment.blue_hmax"];
    hsv.red_hmin = fConfig["pretreatment.red_hmin"];
    hsv.red_hmax = fConfig["pretreatment.red_hmax"];
    hsv.smin = fConfig["pretreatment.smin"];
    hsv.smax = fConfig["pretreatment.smax"];
    hsv.vmin = fConfig["pretreatment.vmin"];
    hsv.vmax = fConfig["pretreatment.vmax"];
    // Match
    arrow_para.arrow_max_lenth = fConfig["Arrow.arrow_max_lenth"];
    arrow_para.arrow_min_size = fConfig["Arrow.arrow_min_size"];
    arrow_para.brightness_threshold = fConfig["Arrow.brightness_threshold"];
    arrow_para.Gama = fConfig["Arrow.Gama"];
    arrow_para.mtach_step = fConfig["Arrow.mtach_step"];
    arrow_para.rect_max_ratio = fConfig["Arrow.rect_max_ratio"];
    arrow_para.rect_min_area = fConfig["Arrow.rect_min_area"];
    arrow_para.score_threshold = fConfig["Arrow.score_threshold"];
    arrow_para.template_extend_ratio = fConfig["Arrow.template_extend_ratio"];
    // target
    target_color = fConfig["Target.color"];

    //枪口补偿
    distance_of_cam_fric.x = fConfig[robot_name]["distance_of_cam_fric.x"];
    distance_of_cam_fric.y = fConfig[robot_name]["distance_of_cam_fric.y"];
    distance_of_cam_fric.z = fConfig[robot_name]["distance_of_cam_fric.z"];
    distance_of_cam_spindle.x = fConfig[robot_name]["distance_of_cam_spindle.x"];
    distance_of_cam_spindle.y = fConfig[robot_name]["distance_of_cam_spindle.y"];
    distance_of_cam_spindle.z = fConfig[robot_name]["distance_of_cam_spindle.z"];
    imu_compensate.pitch = fConfig[robot_name]["imu_compensate.pitch"];
}

void Params::print() {
    char s[][100] ={
    " ╔=============================================================================╗",
    " ║                         H E L L O   W O R L D       🚀🚀🚀                  ║",
    " ╠======╦================================================╦=====================╣",
    " ║      ║                                                ║    📺 图片显示      ║",
    " ║      ║  ⚪ 串口 :             🎥 保存视频 :           ╟- - - - - - - - - - -╢",
    " ║  配  ║                                                ║  显示原图 :         ║",
    " ║      ║  📷 相机 :                                     ║  显示调试 :         ║",
    " ║  置  ║     详情 :                                     ║  显示位姿 :         ║",
    " ║      ║                                                ║  显示预测 :         ║",
    " ║      ║                                                ║                     ║",
    " ╠======╩================================================╩=====================╣",
    " ║  设备名称:                                                                   ║",
    " ║                                                                             ║",
    " ╚=============================================================================╝"
    };
    strncpy(s[4]+29, enable_serial ? "TRUE " : "FALSE", 5);
    strncpy(s[4]+61, save_video ? "TRUE " : "FALSE", 5);
    char const *camera_str[] = {"MVCamera", "USBcamera", "Video"};
    strncpy(s[6]+30, camera_str[camera_type], strlen(camera_str[camera_type]));
    if ( camera_type == 0 ) strncpy(s[7]+29, mv_camera_name.c_str(), mv_camera_name.size());
    else if ( camera_type == 2 ) strncpy(s[7]+29, video_path.c_str(), video_path.size());
    strncpy(s[11]+24, robot_name.c_str(), robot_name.size());

    strncpy(s[5]+82, show_source_img ? "TRUE " : "FALSE", 5);
    strncpy(s[6]+85, show_debug ? "TRUE " : "FALSE", 5);
    strncpy(s[7]+84, show_pose ? "TRUE " : "FALSE", 5);
    strncpy(s[8]+81, show_predict ? "TRUE " : "FALSE", 5);

    for (int i=0; i<14; i++) {
        cout << s[i] << endl;
    }
}

