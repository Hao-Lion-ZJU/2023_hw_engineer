/*
 * @Description: camera
 * @Version: 1.0
 * @Autor: dangwi
 * @Date: 2022-04-06
 */

#ifndef __CAMERA_HPP__
#define __CAMERA_HPP__

#include <opencv2/opencv.hpp>

#include "CameraApi.h"


class Camera
{
protected:
    // 初始化param的分辨率，在构造函数内使用
    void setParamResolution();
public:
    virtual bool open(){ return true;};
    virtual bool getFrame(cv::Mat &img) = 0;
    virtual bool close() {return true;}
    virtual ~Camera() ;
};

class MVCamera: public Camera
{
private:
    std::string camera_name;
    int hCamera;
    tSdkCameraCapbility tCapability;  //设备描述信息

public:

    MVCamera(const std::string &_camera_name);
    bool open();
    bool close() override;
    bool getFrame(cv::Mat &img) override;

};

class USBCamera: public Camera
{
private:
    cv::VideoCapture cap;
    void sentryChassisSet();
public:
    USBCamera(int index);
    USBCamera(const char *cam_name);
    bool getFrame(cv::Mat &img) override;

};

class VideoCamera: public Camera
{
private:
    std::string path;
    // 读取floder 待实现
    int type; // 0: mp4  1: jpg/png  2: folder

    cv::VideoCapture cap;
    std::vector<std::string> files;
    std::vector<cv::String> fn;
public:
    VideoCamera(const std::string &_path);
    bool getFrame(cv::Mat &img) override;
    bool ReadImages(cv::String pattern);
};


#endif
