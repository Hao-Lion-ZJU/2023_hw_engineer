/*
 * @Description: utils
 * @Version: 1.0
 * @Autor: dangwi
 * @Date: 2022-03-10
 */

#ifndef __UTILS_HPP__
#define __UTILS_HPP__

#include <opencv2/opencv.hpp>
#include <chrono>

#define TIME_BEGIN()                               \
    auto start = std::chrono::steady_clock::now();

#define TIME_END(TAG)                                                  \
    auto end = std::chrono::steady_clock::now();                       \
    float durt = std::chrono::duration_cast<std::chrono::microseconds>(\
        end - start).count();                                          \
    LOG(INFO) << "*** << " << TAG << " >> cost  " << durt/1000 << " ms ***";
    // std::cout << "*** << " << TAG << " >> cost" << durt/1000 << " ms ***" << std::endl;

#define MSG_SHOW(PREFIX)       {__msg_box.__print(PREFIX);}
// 用法: MSG_ADD(本地变量1，2...) 最多支持同时传递五个变量
#define MSG_ADD(V...)          _MSG_GLUE(MSG_ADD, _MSG_COUNT_ARGS(V))(V)
#define MSG_UPDATE(key, value) {__msg_box.__add(key, value);}

// msg implement begin
#define _MSG_GLUE(M,n)         _MSG_GLUE_IMPL(M, n)
#define _MSG_GLUE_IMPL(M,n)    M##n
#define _MSG_COUNT_ARGS(V...)  _MSG_COUNT_ARGS_IMPL(V,5,4,3,2,1,0)
#define _MSG_COUNT_ARGS_IMPL(_1,_2,_3,_4,_5, N, ...) N

#define MSG_ADD1(V1)             \
    {__msg_box.__add(#V1, float(V1));}
#define MSG_ADD2(V1,V2)          \
    {__msg_box.__add(#V1, float(V1));__msg_box.__add(#V2, float(V2));}
#define MSG_ADD3(V1,V2,V3)       \
    {__msg_box.__add(#V1, float(V1));__msg_box.__add(#V2, float(V2));__msg_box.__add(#V3, float(V3));}
#define MSG_ADD4(V1,V2,V3,V4)    \
    {__msg_box.__add(#V1, float(V1));__msg_box.__add(#V2, float(V2));__msg_box.__add(#V3, float(V3));__msg_box.__add(#V4, float(V4));}
#define MSG_ADD5(V1,V2,V3,V4,V5) \
    {__msg_box.__add(#V1, float(V1));__msg_box.__add(#V2, float(V2));__msg_box.__add(#V3, float(V3));__msg_box.__add(#V4, float(V4));__msg_box.__add(#V5, float(V5));}
// msg implement end


struct MsgBox {
    static constexpr size_t capacity = 20;
    static const char *keys[capacity];
    static float  values[capacity];
    static size_t idx;
    void __add(const char *key, const float value);
    void __print(const char *prefix);
};

extern MsgBox __msg_box;


namespace util {
    float get_fps();

    void checkInRange(cv::Point2f &pt, float max_x, float max_y);

    void showImg(const std::string &_name, const cv::Mat &img);

    void showXYZ(float x, float y, float depth);

    void showImu(const float &yaw, const float &pitch);
    

    void showPredict(const float &yaw, const float &pitch, const float &real_yaw, const float &real_pitch);
}

#endif
