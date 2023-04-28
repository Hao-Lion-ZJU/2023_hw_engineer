/*
 * @Description: record videos
 * @Version: 1.0
 * @Autor: dangwi
 * @Date: 2022-03-10
 */

#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <sys/stat.h> 
#include <sys/types.h> 
#include <glog/logging.h>

#include "record.hpp"
#include "params.hpp"

using namespace std;
using namespace cv;


static mutex mtx;
static condition_variable cond;

Record::Record(bool _enable, cv::Size _size)
{
    if ( false == _enable ) return;

    // 创建文件夹
    auto now = chrono::system_clock::now();
    time_t t_c = chrono::system_clock::to_time_t(now);
    auto lc = localtime(&t_c);

    dir = "../saves/" + to_string(lc->tm_mon+1) + "_" + to_string(lc->tm_mday) + "_" \
          + to_string(lc->tm_hour) + "_" + to_string(lc->tm_min) ;
    mkdir("../saves", S_IRWXU | S_IRWXG | S_IRWXO );
    mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IRWXO );

    // 初始化videowriter
    int coder = VideoWriter::fourcc('M', 'P', '4', '2'); // 编码器
    if ( _size == cv::Size{0,0} )
        size = cv::Size{param.camera.width, param.camera.height};
    else
        size = _size;

    writer.open(dir+"/video.avi", coder, 25, size, true);

    thread t(&Record::writeThread, this);
    t.detach();

}

Record::~Record()
{
}


void Record::writeThread()
{
    while ( true ) {
        unique_lock<mutex> lk(mtx);
        cond.wait(lk, [this]{ return !q.empty(); });

        lk.unlock();

        writer << q.front();
        q.pop_front();
    }
}

void Record::push(cv::Mat &img)
{
    static auto last_time = chrono::steady_clock::now();
    auto this_time = chrono::steady_clock::now();
    if ( q.size() > 5 \
     || chrono::duration_cast<chrono::milliseconds>(this_time-last_time).count() < 35 ) 
        return;
    
    if ( img.cols != size.width || img.rows != size.height ) {
        LOG(WARNING) << "[record]  图片尺寸不符，无法保存";
        return;
    }
    unique_lock<mutex> lk(mtx);
    q.push_back(img);
    lk.unlock();
    cond.notify_one();
    last_time = this_time;
}