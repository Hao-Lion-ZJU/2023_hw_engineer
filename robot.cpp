/**
 * ***************************@copyright (C) Copyright  2022  ZJU***************************
 * @file   robot.cpp
 * @brief 线程函数
 * @author Hao Lion(郝亮亮)    Email:(haolion_zju@foxmail.com)
 * @version 1.0
 * @date 2022-10-01
 * 
 * @verbatim:
 * ==============================================================================
 *                                                                               
 * ==============================================================================
 * @endverbatim
 * ***************************@copyright (C) Copyright  2022  ZJU***************************
 */

#include "robot.hpp"
#include <glog/logging.h>
#include "params.hpp"
#include <unistd.h>

#define picture
// #define Video
// #define camera_test


bool RUNNING = true;


/**
 * @brief Construct a new Rmversion object
 */
Rmversion::Rmversion()
{
    arrow_Imgprocess_Ptr = make_unique<arrow_Imgprocess>();
    PoseSlover_Ptr = make_unique<PoseSlover>();
    Communicator_Ptr = make_unique<Communicator>(param.enable_serial);
    #ifdef camera_test
    if (param.enable_camera)
        Camera_Ptr = make_unique<MVCamera>(param.mv_camera_name);
        Record_Ptr = make_unique<Record>(param.save_video);
    #endif // camera_test
    
    
}

/**
 * @brief 释放智能指针
 */
Rmversion::~Rmversion()
{
    arrow_Imgprocess_Ptr.release();
    PoseSlover_Ptr.release();
    Communicator_Ptr.release();
    Camera_Ptr.release();
    Record_Ptr.release();
}

/**
 * @brief 初始化一些标志位
 */
void Rmversion::Init()
{
    flags = CANNOT_SOLVE;
    /*调整下窗口位置，方便DEBUG*/
    this->resize_window();
    /**/
    #ifdef camera_test
    if (param.enable_camera)
        this->Camera_Ptr->open();
    #endif // camera_test
    
}

/**
 * @brief 图像处理线程
 */
void Rmversion::Imgprocess()
{
    Mat src_img;
    Robot robot;
    chrono::time_point<chrono::system_clock> img_time;
    uint8_t match_flag = 0;
    std::vector<Point2f> Points;
    while(RUNNING)
    {
        #ifdef camera_test
        {
            if ( !RUNNING )
            {
                usleep(10000);//10ms
                continue;
            } 
            if(!Graber(src_img))continue;
            // imshow("tmp",src_img);
            // waitKey(0);
            // imwrite("/home/lion/rm/engineer_version/Image/0.jpg",src_img);
            
        }
        #endif //
        #ifdef picture
        {
            src_img = cv::imread("/home/lion/rm/engineer_version/Image/2.jpg");
            RUNNING = 0;
        }
        #endif // picture
        {
            // imshow("src",src_img);
            // waitKey(1);
            if(src_img.empty())
                continue;
            // imwrite("/home/haolion/test/rm/engineer_version/Image/0.jpg",src_img);
            arrow_Imgprocess_Ptr->pretreatment(src_img);
            match_flag = arrow_Imgprocess_Ptr->Arrow_detector(); 
            if(match_flag == PNP_SOLVE)
            {
                Points = arrow_Imgprocess_Ptr->get_PointOfPnp();
                PoseSlover_Ptr->setTarget(Points,PNP_SOLVE);
                PoseSlover_Ptr->solve();
             
            }
            else if (match_flag == ANGLE_SOLVE)
            {
                Points = arrow_Imgprocess_Ptr->get_PointOfAngle();
                PoseSlover_Ptr->setTarget(Points,ANGLE_SOLVE);
                PoseSlover_Ptr->solve();
                
            }
            if(param.show_debug)
            {
                
                arrow_Imgprocess_Ptr->Draw_bound();
                arrow_Imgprocess_Ptr->debug_show();
            }

            if(param.show_pose)
            {
                PoseSlover_Ptr->showPoseInfo();
            }
            #ifdef camera_test
            {
                waitKey(1);
            }
            #endif //
            #ifdef picture
            {
                waitKey(0);
            }
            #endif // picture
            
        }
    }
}


/**
 * @brief 读取图像到缓冲区
 */
bool  Rmversion::Graber(cv::Mat& src)
{

    // TIME_BEGIN()
    bool status = Camera_Ptr->getFrame(src);
    // TIME_END("get Frame")
    if ( false == status ) {
        LOG(WARNING) << "Get Image Failed !";
        usleep(10000);//10ms
        return false;
    }
    if ( param.save_video )
        Record_Ptr->push(src);
    return true;

}


/**
 * @brief 
 */
[[noreturn]] void Rmversion::Receiver()
{
    uint8_t buffer[12] = {0};
    while ( RUNNING ) {
        /// receive
        int num_bytes = Communicator_Ptr->receive(buffer, 7);
        if (num_bytes <= 0) {
            continue;
        }

        if(buffer[0] == 0x7f && buffer[1] == 0x7f && buffer[2] == 0x7f && buffer[3] == 0x7f && buffer[4] == 0x7f && buffer[5] == 0x7f)
        {
            LOG(FATAL) << "-------------------------------------shutdown!!!";
            system("sudo shutdown now");
        }
        else{
            int16_t pitch_angle = buffer[0] << 8 | buffer[1];
            int16_t yaw_angle   = buffer[2] << 8 | buffer[3];
            int16_t move_speed  = buffer[4] << 8 | buffer[5];
            int     mode        = buffer[6];

            g_robot.yaw         = yaw_angle / 100.f;
            g_robot.pitch       = pitch_angle / 100.f;

            // 串口中的比特位（敌方颜色） 蓝0 红1
            param.target_color  = (mode & 0x80) ? COLOR::RED : COLOR::BLUE;

        }
    }
}

/**
 * @brief 重新设置窗口位置
 */
#define SHOW_HEIGHT  450
#define SHOW_WIDTH	600
void Rmversion::resize_window()
{
	cv::namedWindow("gray", cv::WINDOW_NORMAL);
	cv::namedWindow("Pre_bin", cv::WINDOW_NORMAL);
	cv::namedWindow("thin", cv::WINDOW_NORMAL);
	cv::namedWindow("corner", cv::WINDOW_NORMAL);
	cv::namedWindow("PoseSlover", cv::WINDOW_NORMAL);
	cv::namedWindow("result", cv::WINDOW_NORMAL);

    cv::resizeWindow("gray", SHOW_WIDTH,SHOW_HEIGHT);
	cv::resizeWindow("Pre_bin", SHOW_WIDTH,SHOW_HEIGHT);
	cv::resizeWindow("thin", SHOW_WIDTH,SHOW_HEIGHT);
	cv::resizeWindow("corner", SHOW_WIDTH,SHOW_HEIGHT);
	cv::resizeWindow("PoseSlover", SHOW_WIDTH,SHOW_HEIGHT);
	cv::resizeWindow("result", SHOW_WIDTH,SHOW_HEIGHT);

    cv::moveWindow("gray", 0, 0);
    cv::moveWindow("Pre_bin", SHOW_WIDTH,0);
    cv::moveWindow("thin", 2*SHOW_WIDTH, 0);
    cv::moveWindow("corner", 0, SHOW_HEIGHT);
    cv::moveWindow("PoseSlover", SHOW_WIDTH, SHOW_HEIGHT);
    cv::moveWindow("result", 2*SHOW_WIDTH,SHOW_HEIGHT);
}
