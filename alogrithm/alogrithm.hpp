/**
 * ***************************@copyright (C) Copyright  2022  ZJU***************************
 * @file   alogrithm.hpp
 * @brief  此文件定义了一些自己实现的算法
 * @author Hao Lion(郝亮亮)    Email:(haolion_zju@foxmail.com)
 * @version 1.0
 * @date 2022-10-02
 * 
 * @verbatim:
 * ==============================================================================
 *                                                                               
 * ==============================================================================
 * @endverbatim
 * ***************************@copyright (C) Copyright  2022  ZJU***************************
 */
#pragma once
#ifndef _ALOGRITHM_
#define _ALOGRITHM_

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

#define CV_COLOR_RED cv::Scalar(0,0,255)       //纯红
#define CV_COLOR_GREEN cv::Scalar(0,255,0)        //纯绿
#define CV_COLOR_BLUE cv::Scalar(255,0,0)       //纯蓝

#define CV_COLOR_DARKGRAY cv::Scalar(169,169,169) //深灰色
#define CV_COLOR_DARKRED cv::Scalar(0,0,139) //深红色
#define CV_COLOR_ORANGERED cv::Scalar(0,69,255)     //橙红色

#define CV_COLOR_CHOCOLATE cv::Scalar(30,105,210) //巧克力
#define CV_COLOR_GOLD cv::Scalar(10,215,255) //金色
#define CV_COLOR_YELLOW cv::Scalar(0,255,255)     //纯黄色

#define CV_COLOR_OLIVE cv::Scalar(0,128,128) //橄榄色
#define CV_COLOR_LIGHTGREEN cv::Scalar(144,238,144) //浅绿色
#define CV_COLOR_DARKCYAN cv::Scalar(139,139,0)     //深青色


#define CV_COLOR_SKYBLUE cv::Scalar(230,216,173) //天蓝色
#define CV_COLOR_INDIGO cv::Scalar(130,0,75) //藏青色
#define CV_COLOR_PURPLE cv::Scalar(128,0,128)     //紫色

#define CV_COLOR_PINK cv::Scalar(203,192,255) //粉色
#define CV_COLOR_DEEPPINK cv::Scalar(147,20,255) //深粉色
#define CV_COLOR_VIOLET cv::Scalar(238,130,238)     //紫罗兰

extern void GammaTransform(cv::Mat &gray_image,cv::Mat &dst,double gama);
extern std::vector<cv::Point> getPoints(const cv::Mat &thinSrc, unsigned int raudis = 4, unsigned int thresholdMax = 6, unsigned int thresholdMin = 4);
extern void filterOver(cv::Mat & thinSrc);
extern cv::Mat thinImage(const cv::Mat & src);
extern std::string formatDoubleValue(double val , int fixed);
extern const double angleLineOf(const cv::Vec4f& line1, const cv::Vec4f& line2);
extern std::vector<cv::Point> getPointsInContours(std::vector<cv::Point> contour,const cv::Mat& pre_img);

extern const double angleLineOf(const cv::Point2f& pt1, const cv::Point2f& pt2, const cv::Point2f& pt0);

// /**
//  * @brief C++11 中make_unique的实现，避免使用new多次分配内存
//  * @tparam T 
//  * @tparam Ts 
//  * @param  params           My Param doc
//  * @return std::unique_ptr<T> 
//  */
// template<typename T, typename... Ts>
// std::unique_ptr<T> make_unique(Ts&&... params)
// {
//     return std::unique_ptr<T>(new T(std::forward<Ts>(params)...));
// }



/**
 * @brief 求解两条直线的交点坐标
 * @tparam ValType 
 * @param  line1            My Param doc
 * @param  line2            My Param doc
 * @return const cv::Point2f 
 */
template<typename ValType>
const cv::Point2f crossPointOf(const std::array<cv::Point_<ValType>, 2>& line1, const std::array<cv::Point_<ValType>, 2>& line2)
{
	ValType a1 = line1[0].y - line1[1].y;
	ValType b1 = line1[1].x - line1[0].x;
	ValType c1 = line1[0].x*line1[1].y - line1[1].x*line1[0].y;

	ValType a2 = line2[0].y - line2[1].y;
	ValType b2 = line2[1].x - line2[0].x;
	ValType c2 = line2[0].x*line2[1].y - line2[1].x*line2[0].y;

	ValType d = a1 * b2 - a2 * b1;

	if(d == 0.0)
	{
		return cv::Point2f(FLT_MAX, FLT_MAX);
	}
	else
	{
		return cv::Point2f(float(b1*c2 - b2 * c1) / d, float(c1*a2 - c2 * a1) / d);
	}
}
/**
 * @brief 内联函数
 * @param  line1            My Param doc
 * @param  line2            My Param doc
 * @return const cv::Point2f 
 */
inline const cv::Point2f crossPointOf(const cv::Vec4f& line1, const cv::Vec4f& line2)
{
	const std::array<cv::Point2f, 2> line1_{cv::Point2f(line1[2],line1[3]),cv::Point2f(line1[2] + line1[0],line1[3] + line1[1])};
	const std::array<cv::Point2f, 2> line2_{cv::Point2f(line2[2],line2[3]),cv::Point2f(line2[2] + line2[0],line2[3] + line2[1])};
	return crossPointOf(line1_, line2_);
}
#endif