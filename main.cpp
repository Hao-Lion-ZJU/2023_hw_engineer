/**
 * ***************************@copyright (C) Copyright  2022  ZJU***************************
 * @File   main.cpp
 * @brief  
 * @Author : Hao Lion(郝亮亮)    Email : haolion_zju@foxmail.com
 * @Version  1.0
 * @Creat Date : 2022-11-17
 * 
 * @verbatim
 * ==============================================================================
 *                                                                               
 * ==============================================================================
 * @endverbatim
 * ***************************@copyright (C) Copyright  2022  ZJU***************************
 */

#include "robot.hpp"
#include <glog/logging.h>
#include <thread>

int main(int argc, char **argv) {

    //-------------------------------------------------------
	// init glog 
	//-------------------------------------------------------
	FLAGS_logtostderr = true;
	google::InitGoogleLogging(argv[0]);

    Rmversion rm;
    rm.Init();
    rm.Imgprocess();

    
    LOG(INFO) << "Hello, World!";
    return 0;
}