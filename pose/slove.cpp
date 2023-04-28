/**
 * ***************************@copyright (C) Copyright  2022  ZJU***************************
 * @file   slove.cpp
 * @brief  解算角度
 * @author Hao Lion(郝亮亮)    Email:(haolion_zju@foxmail.com)
 * @version 1.0
 * @date 2022-12-08
 * 
 * @verbatim:
 * ==============================================================================
 *                                                                               
 * ==============================================================================
 * @endverbatim
 * ***************************@copyright (C) Copyright  2022  ZJU***************************
 */


#include "slove.hpp"
#include "opencv2/opencv.hpp"
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include "math.h"
#include "params.hpp"
#include "../Match/Match.hpp"
#include "../alogrithm/alogrithm.hpp"
using namespace cv;
using namespace std;



/*bool solvePnP(InputArray objectPoints, InputArray imagePoints, InputArray cameraMatrix, InputArray distCoeffs,
 OutputArray rvec, OutputArray tvec, bool useExtrinsicGuess=false, int flags=ITERATIVE )

　　objectPoints：特征点的世界坐标，坐标值需为float型，不能为double型，可以为mat类型，也可以直接输入vector
　　imagePoints：特征点在图像中的像素坐标，可以输入mat类型，也可以直接输入vector，
	注意输入点的顺序要与前面的特征点的世界坐标一一对应
　　cameraMatrix：相机内参矩阵
　　distCoeffs：相机的畸变参数
　　rvec：输出的旋转向量
　　tvec：输出的平移向量
　　最后的输入参数有三个可选项：
　　CV_ITERATIVE，默认值，它通过迭代求出重投影误差最小的解作为问题的最优解。
　　CV_P3P则是使用非常经典的Gao的P3P问题求解算法。
　　CV_EPNP使用文章《EPnP: Efficient Perspective-n-Point Camera Pose Estimation》中的方法求解。*/
  //由于装甲板是一个平板，四个顶点的Z轴坐标可以设为0，x轴和y轴坐标可以分别设置为正负二分之一的长和宽，装甲板中心即为原点

#define LengthOfSlot 275

static const vector<Point3d> POINT_3D_OF_SLOT = { // 单位：mm
    {LengthOfSlot/2, LengthOfSlot/2, 0.},
    {-LengthOfSlot/2, LengthOfSlot/2, 0.},
    {-LengthOfSlot/2, -LengthOfSlot/2, 0.},
    {LengthOfSlot/2, -LengthOfSlot/2, 0.}};

static const vector<Point3d> POINT_3D_OF_ANGLE_RIGHT = { // 单位：mm
    {288/2, 0., -45.5},                 //角点
    {288/2, 100, -145.5},              //上端点
    {288/2, -100, -145.5},            //下端点
    {288/2, 0, -55.5}                //内角点
    };             
    

static const vector<Point3d> POINT_3D_OF_ANGLE_LEFT = { // 单位：mm
    {-288/2, 0., -45.5},                 //角点
    {-288/2, 100, -145.5},              //上端点
    {-288/2, 0, -55.5},                 //构造点
    {-288/2, -100, -145.5}             //下端点
    };             
    

PoseSlover::PoseSlover() 
{
	//将四个像素点坐标初始化为(0, 0)
	for(int ll = 0; ll <= 3; ll++)
		point_2d_of_slot.push_back(cv::Point2f(0.0, 0.0));
    alpha = 0.2;
    pose = last_pose = {0};
}




void PoseSlover::setTarget(const std::vector<cv::Point2f> objectPoints, int method)
{
    if(method == PNP_SOLVE)
    {
        point_2d_of_slot = objectPoints;
        ITERATIVE_solve();
    }
    else if(method == ANGLE_SOLVE)
    {
        point_2d_of_angle = objectPoints;
        ANGLE_solve();
    }
	else
    {
        return;
    }   
}



PoseSlover::AngleFlag PoseSlover::solve()
//类外定义PoseSlover类的成员函数solve，返回PoseSlover::AngleFlag类型的数据
{
    //使用solvePNP求解相机姿态和位置信息  rvec：输出的旋转向量，tvec：输出的平移相量
    //平移向量就是以当前摄像头中心为原点时物体原点所在的位置。
    if(angle_solver_algorithm)
    {
        //判断左右角标
        if(point_2d_of_angle.at(0).x < point_2d_of_angle.at(2).x)
        {
            solvePnP(POINT_3D_OF_ANGLE_RIGHT, point_2d_of_angle, param.camera.CI_MAT, param.camera.D_MAT, _rVec, _tVec, false, SOLVEPNP_AP3P);
        }
        else
        {
            solvePnP(POINT_3D_OF_ANGLE_LEFT, point_2d_of_angle, param.camera.CI_MAT, param.camera.D_MAT, _rVec, _tVec, false, SOLVEPNP_AP3P);
        }
        
    }
    else
    {
        solvePnP(POINT_3D_OF_SLOT, point_2d_of_slot, param.camera.CI_MAT, param.camera.D_MAT, _rVec, _tVec, false, SOLVEPNP_IPPE);
    }
    
    cv::Mat _RVec = cv::Mat::zeros(3, 1, CV_64FC1);
    Rodrigues(_rVec, _RVec);
    // 转换格式
    _RVec.convertTo(_RVec, CV_64FC1);
    // 转成Eigen下的矩阵
    Eigen::Matrix3f Rotated_matrix;
    cv2eigen(_RVec, Rotated_matrix);
    Eigen::Vector3f euler_angles = Rotated_matrix.eulerAngles(0, 1, 2);
    

    pose.pitch = (1-alpha)*(euler_angles[0] * 180 / CV_PI) + alpha*last_pose.pitch;
    pose.yaw = (1-alpha)*(euler_angles[1] * 180 / CV_PI) + alpha*last_pose.yaw;
    pose.roll = (1-alpha)*(euler_angles[2] * 180 / CV_PI) + alpha*last_pose.roll;

    pose.x = (1-alpha)*(_tVec.at<double>(0, 0)) + alpha*last_pose.x;
    pose.y = (1-alpha)*(-_tVec.at<double>(1, 0)) + alpha*last_pose.y;
    pose.z = (1-alpha)*(_tVec.at<double>(2, 0)) + alpha*last_pose.z;
    // pose.roll = atan2(_rVec.at<double>(2, 1),_rVec.at<double>(2, 2))/CV_PI*180;
    // pose.pitch = atan2(-_rVec.at<double>(2,0),
    // sqrt(_rVec.at<double>(2,0)*_rVec.at<double>(2,0)+_rVec.at<double>(2,2)*_rVec.at<double>(2,2)))/CV_PI*180;
    // pose.yaw =  atan2(_rVec.at<double>(1, 0),_rVec.at<double>(0, 0))/CV_PI*180;
    last_pose = pose;
    
    return ANGLES_AND_DISTANCE;
    // else
    // {
    //     // cv::Vec4f line1 = Vec4f(point_2d_of_angle.at(0).x,point_2d_of_angle.at(0).y,point_2d_of_angle.at(1).x,point_2d_of_angle.at(1).y);
    //     // cv::Vec4f line2 = Vec4f(point_2d_of_angle.at(0).x,point_2d_of_angle.at(0).y,point_2d_of_angle.at(2).x,point_2d_of_angle.at(2).y);
    //     // cv::Vec4f line3 = Vec4f(point_2d_of_angle.at(1).x,point_2d_of_angle.at(1).y,point_2d_of_angle.at(2).x,point_2d_of_angle.at(2).y);
    //     // double a = angleLineOf(line1,line2);
    //     // double b = angleLineOf(line1,line3);
    //     // double c = 180-a-b;
        

        
    // }
    return ANGLE_ERROR;
}





void PoseSlover::ITERATIVE_solve()
{
    slover_algorithm="ITERATIVE_Solve";
    angle_solver_algorithm = 0;
}

void PoseSlover::ANGLE_solve()
{
    slover_algorithm="ANGLE_Solve";
    angle_solver_algorithm = 1;
}



//获取偏转的角度向量
const cv::Vec3f PoseSlover::getPosition()
{
    return cv::Vec3f(pose.x, pose.y, pose.z);
}

const cv::Vec2f PoseSlover::getDev()
{
    return cv::Vec2f(_tVec.at<float>(0, 0), _tVec.at<float>(1, 0));
}



void PoseSlover::showPoseInfo()
{
    Mat angleImage = Mat::zeros(350,600,CV_8UC3);
    putText(angleImage, "Roll: " + to_string(pose.roll), Point(100, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 1, 8, false);
    putText(angleImage, "Pitch: " + to_string(pose.pitch), Point(100, 100), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 1, 8, false);
    putText(angleImage, "Yaw: " + to_string(pose.yaw), Point(100, 150), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 1, 8, false);
    putText(angleImage, "X:" + formatDoubleValue((pose.x),1), Point(50, 200), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 1, 8, false);
    putText(angleImage, "Y:" + formatDoubleValue((pose.y),1), Point(200, 200), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 1, 8, false);
    putText(angleImage, "Z:" + formatDoubleValue(pose.z,1), Point(350, 200), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 1, 8, false);
    putText(angleImage, ("Algorithm:" + slover_algorithm),Point(100,250), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 255), 1, 8, false);
    std::cout << "X:"<< pose.x << "Y:" << pose.y << "Z:" << pose.z << std::endl;
    imshow("PoseSlover",angleImage);
}



