/*
 * @Description: utils
 * @Version: 1.0
 * @Autor: dangwi
 * @Date: 2022-03-10
 */

#include <deque>
#include <unordered_map>
#include <glog/logging.h>
#include <iostream>
#include <iomanip>
#include <string.h>

#include "utils.hpp"


using namespace std;
using namespace chrono;
using namespace cv;

const char *MsgBox::keys[MsgBox::capacity];
float       MsgBox::values[MsgBox::capacity];
size_t      MsgBox::idx;

MsgBox __msg_box;

void MsgBox::__add(const char *key, const float value)
{
    if ( idx == capacity ) {
        LOG(ERROR) << "util.cpp: 参数个数超过" << capacity << "个";
        exit(-1);
    }
    keys[idx]   = key;
    // 控制精度，方便输出
    if ( value < 0 )
        values[idx] = int(value*1e4) / 1e4;
    else
        values[idx] = int(value*1e5) / 1e5;
    idx ++;
}

void MsgBox::__print(const char *prefix)
{
    cout << "| " << prefix << " | ";
    for (size_t i=0; i<idx; i++) {
        size_t l = std::max<size_t>(strlen(keys[i]), 5);
        cout << setw(l) << setiosflags(ios::right) << setfill(' ') << keys[i] << " | ";
    }
    cout << endl;
    cout << "| " << setw(strlen(prefix)+2) << setfill('-') << setiosflags(ios::right) << " |";
    for (size_t i=0; i<idx; i++) {
        size_t l = std::max<size_t>(strlen(keys[i])+2, 7);
        cout << setw(l) << setiosflags(ios::right) << setfill(' ') << setprecision(5) << values[i] << "|";
    }
    cout << endl;
    cout << setw(120) << setfill('=') << " " << endl;
    idx = 0;
}

namespace util {
    
    static auto last_time = steady_clock::now();
    static float fps = 100;
    float get_fps()
    {
        auto now_time = steady_clock::now();
        auto durt = duration_cast<microseconds>(now_time - last_time).count();

        fps = 5e6 / (durt + 4e6 / fps);
        last_time = now_time;
        return fps;
    }

    void checkInRange(Point2f &pt, float max_x, float max_y)
    {
        pt.x = max<float>(pt.x, 0);
        pt.x = min<float>(pt.x, max_x-1);
        pt.y = max<float>(pt.y, 0);
        pt.y = min<float>(pt.y, max_y-1);
    }

    void showImg(const string &_name, const Mat &img)
    {
        static unordered_map<string, steady_clock::time_point> m_time;
        auto it = m_time.find(_name);
        auto now_time = steady_clock::now();
        if ( it!=m_time.end() && duration_cast<milliseconds>(now_time-it->second).count()<40 )
            return ;
        imshow(_name, img);
        m_time.insert({_name, now_time});
    }


    void showXYZ(float x, float y,float depth)
    {
        Mat figure(400, 400, CV_8UC3, Scalar(255, 255, 255));
        struct xyz{int x; int y; int z;};
        static deque<xyz> list;
        
        list.push_back({int(x/2), int(y/2), int(depth*50)});
        if ( list.size() > 200 ) list.pop_front();

        line(figure, {0,200}, {399,200}, {128,128,128,0.5});
        int i = 0;
        for (auto it=list.begin()+1; it!=list.end(); it++) {
            circle(figure, {i*2,it->x}, 1, {255,0,0});
            circle(figure, {i*2,it->y}, 1, {0,255,0});
            circle(figure, {i*2,it->z}, 1, {0,0,255});
            if ( (it-1)->x && it->x ) 
                line(figure, {(i-1)*2,(it-1)->x}, {i*2,it->x}, {255,0,0});
            if ( (it-1)->y && it->y )
                line(figure, {(i-1)*2,(it-1)->y}, {i*2,it->y}, {0,255,0});
            if ( (it-1)->z && it->z )
                line(figure, {(i-1)*2,(it-1)->z}, {i*2,it->z}, {0,0,255});
            i ++;
        }
        util::showImg("XYZ", figure);
    }

    void showImu(const float &yaw, const float &pitch)
    {
        Mat figure(720, 400, CV_8UC3, Scalar(255, 255, 255));
        struct yaw_pitch{int y; int p;};
        static deque<yaw_pitch> list;
        
        list.push_back({int(yaw*2+360), int(pitch*8+360)});
        if ( list.size() > 200 ) list.pop_front();

        line(figure, {0,360}, {399,360}, {128,128,128,0.5});
        int i = 0;
        for (auto it=list.begin()+1; it!=list.end(); it++) {
            circle(figure, {i*2,it->y}, 1, {255,0,0});
            circle(figure, {i*2,it->p}, 1, {0,0,255});
            if ( (it-1)->y && it->y ) 
                line(figure, {(i-1)*2,(it-1)->y}, {i*2,it->y}, {255,0,0});
            if ( (it-1)->p && it->p )
                line(figure, {(i-1)*2,(it-1)->p}, {i*2,it->p}, {0,255,0});
            i ++;
        }
        util::showImg("imu", figure);
    }
    
    void showPredict(const float &yaw, const float &pitch, const float &real_yaw, const float &real_pitch)
    {
        Mat figure(720, 400, CV_8UC3, Scalar(255, 255, 255));
        struct yaw_pitch{int y; int p;};
        static deque<yaw_pitch> list_predict;

        static deque<yaw_pitch> list_real;
        list_predict.push_back({int(yaw*2+360), int(pitch*8+360)});
        list_real.push_back({int(real_yaw*2+360), int(real_pitch*8+360)});
        if ( list_predict.size() > 200 ) list_predict.pop_front();
        if ( list_real.size() > 200 ) list_real.pop_front();

        line(figure, {0,360}, {399,360}, {128,128,128,0.5});
        int i = 0;
        for (int i=1; i < list_real.size(); i++) {
            circle(figure, {i*2,list_predict[i].y}, 1, {255,0,0});
            circle(figure, {i*2,list_predict[i].p}, 1, {0,0,255});
            circle(figure, {i*2,list_real[i].y}, 1, {255,255,0});
            circle(figure, {i*2,list_real[i].p}, 1, {0,255,255});
            if ( fabs(list_predict[i-1].y-list_predict[i].y) < 100 )
                line(figure, {(i-1)*2,list_predict[i-1].y}, {i*2,list_predict[i].y}, {255,0,0});
            if ( fabs(list_predict[i-1].p-list_predict[i].p) < 100 )
                line(figure, {(i-1)*2,list_predict[i-1].p}, {i*2,list_predict[i].p}, {0,255,0});
            if ( fabs(list_real[i-1].y-list_real[i].y) < 100 )
                line(figure, {(i-1)*2,list_real[i-1].y}, {i*2,list_real[i].y}, {255,255,0});
            if ( fabs(list_real[i-1].p-list_real[i].p) < 100 )
                line(figure, {(i-1)*2,list_real[i-1].p}, {i*2,list_real[i].p}, {0,255,255});
        }
        
        util::showImg("predict", figure);
    }

}
