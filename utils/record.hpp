/*
 * @Description: record videos
 * @Version: 1.0
 * @Autor: dangwi
 * @Date: 2022-03-10
 */

#ifndef __HELLOWORLD_RECORD_HPP__
#define __HELLOWORLD_RECORD_HPP__

#include <opencv2/opencv.hpp>
#include <deque>

// 用法：初始化的size默认为相机图片大小，后续图片
//       必须与size一样大，否则可能会出错，可以通
//       过传入size自定义视频尺寸
// 建议：原图保存1280x1080一分钟约80MB，硬盘不足可以resize后再保存
//       push函数不耗时，该类有自带线程保存图片
class Record
{
private:
    cv::VideoWriter writer;
    std::deque<cv::Mat> q;
    void writeThread();
    std::string dir;
    cv::Size size;

public:
    Record(bool _enable, cv::Size _size={0,0});

    ~Record();

    void push(cv::Mat &img);
};





#endif
