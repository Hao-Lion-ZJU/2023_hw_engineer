#ifndef __SLOVE_HPP__
#define __SLOVE_HPP__

#pragma once
#include "opencv2/core/core.hpp"
#include<iostream>
#include<string>
#include <opencv2/opencv.hpp>
#include<math.h>

using namespace cv;


struct Pose_t
{
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
};


/**
*	solve by PNP, that is, using four points to detect the angle and distance.
*	It's not very useful if the armor is far.If it' far try solve by one point
*/
class PoseSlover
{
public:
    PoseSlover();

    ~PoseSlover() = default;


	enum AngleFlag
	{
		ANGLE_ERROR = 0,                //an error appeared in angle solution
		ONLY_ANGLES = 1,		//only angles is avilable
		TOO_FAR = 2,			//the distance is too far, only angles is avilable
		ANGLES_AND_DISTANCE = 3		//distance and angles are all avilable and correct
	};

	
	/*
	* @brief set the 2D center point or corners of armor, or the center of buff as target 
	* @param Input armorPoints/centerPoint
	*/
    void setTarget(const std::vector<cv::Point2f> objectPoints, int method);  //set corner points and centerpoints
															

	/*
	* @brief slove the angle by selected algorithm
    */
    AngleFlag solve();


	/*
	* @brief get the angle solved
	*/
	const cv::Vec3f getPosition();

    /*
    * @brief get the tvec
    */
    const cv::Vec2f getDev();


    void ITERATIVE_solve();


    void ANGLE_solve();


    void showPoseInfo();


private:
    cv::Mat measurement;
	cv::Mat _rVec = cv::Mat::zeros(3, 1, CV_64FC1);//init rvec
	cv::Mat _tVec = cv::Mat::zeros(3, 1, CV_64FC1);//init tvec
	std::vector<cv::Point2f> point_2d_of_slot;
    std::vector<cv::Point2f> point_2d_of_angle;
    std::string slover_algorithm="ITERATIVE_Solve";
	int angle_solver_algorithm = 1;
    Pose_t pose;
    float alpha;    //惯性滤波参数
    Pose_t last_pose;
};


#endif /* __SLOVE_HPP__ */
