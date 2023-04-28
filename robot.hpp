#pragma once
#include <memory>
#include <iostream>
#include <mutex>

#include "params.hpp"
#include "./Match/Match.hpp"
#include "./alogrithm/alogrithm.hpp"
#include "./pose/slove.hpp"
#include "./device/camera.hpp"
#include "./device/communicator.hpp"
#include "record.hpp"

using namespace std;
using namespace cv;






class Rmversion{
public:
    /**
     * @brief Construct a new Rmversion object
     */
    Rmversion();
    /**
     * @brief Destroy the Rmversion object
     */
    ~Rmversion();
    /**
     * @brief 图像处理函数
     */
    void Imgprocess();
    /**
     * @brief 读取图像到缓冲区
     */
    bool Graber(cv::Mat& src);
    /**
     * @brief 
     */
    [[noreturn]] void Receiver();
    /**
     * @brief 初始化
     */
    void Init();
    /**
 	* @brief 重新设置窗口位置
 	*/
	void resize_window();
/**********************data**********************/
private:
    uint8_t flags;
    std::unique_ptr<arrow_Imgprocess> arrow_Imgprocess_Ptr;
    std::unique_ptr<PoseSlover> PoseSlover_Ptr;
    std::unique_ptr<Camera> Camera_Ptr;
    std::unique_ptr<Communicator> Communicator_Ptr;
    std::unique_ptr<Record>Record_Ptr;
};



