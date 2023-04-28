/**
 * ***************************@copyright (C) Copyright  2022  ZJU***************************
 * @file   Match.cpp
 * @brief 
 * @author Hao Lion(郝亮亮)    Email:(haolion_zju@foxmail.com)
 * @version 1.0
 * @date 2022-12-09
 * 
 * @verbatim:
 * ==============================================================================
 *                                                                               
 * ==============================================================================
 * @endverbatim
 * ***************************@copyright (C) Copyright  2022  ZJU***************************
 */

#include "Match.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/dnn.hpp>
#include "../alogrithm/alogrithm.hpp"
#include "params.hpp"
#include <glog/logging.h>
using namespace cv;
using namespace std;


/************************************箭头匹配************************************/
/**
 * @brief 载入模板图片
 * @param  template_path    模板路径
 * @param  arrowImgSize     模板图片尺寸
 */
void arrow_classifier::Load_Template(const char* big_template_path,const char* small_template_path,Size arrowImgSize)
{
    this->arrowImgSize = arrowImgSize;
    Big_arrow_template = imread(big_template_path);
    Small_arrow_template = imread(small_template_path);
    if(Big_arrow_template.empty() || Small_arrow_template.empty())
    {
        LOG(ERROR) << "模板读取失败";
        exit(0);
    }
    cvtColor(Big_arrow_template, Big_arrow_template, COLOR_RGB2GRAY);
    threshold(Big_arrow_template, Big_arrow_template,50, 255, cv::THRESH_BINARY);
    cvtColor(Small_arrow_template, Small_arrow_template, COLOR_RGB2GRAY);
    threshold(Small_arrow_template, Small_arrow_template,50, 255, cv::THRESH_BINARY);
    //set dstPoints (the same to arrowImgSize, as it can avoid resize arrowImg)
    dstPoints[0] = Point2f(0, 0);
    dstPoints[1] = Point2f(arrowImgSize.width, 0);
    dstPoints[2] = Point2f(arrowImgSize.width, arrowImgSize.height);
    dstPoints[3] = Point2f(0, arrowImgSize.height);
}


/**
 * @brief 载入角标识别CNN模型
 * @param  model_patch    模板路径
 * @param  arrowImgSize     模板图片尺寸
 */
void arrow_classifier::Load_Model(const char* model_patch,Size arrowImgSize)
{
    this->arrowImgSize = arrowImgSize;
    small_arrow_net = cv::dnn::readNetFromTensorflow(model_patch);
    //set dstPoints (the same to arrowImgSize, as it can avoid resize arrowImg)
    dstPoints[0] = Point2f(0, 0);
    dstPoints[1] = Point2f(arrowImgSize.width, 0);
    dstPoints[2] = Point2f(arrowImgSize.width, arrowImgSize.height);
    dstPoints[3] = Point2f(0, arrowImgSize.height);

}

/**
 * @brief	将图像旋转一定角度
 * @param  src              原图像
 * @param  angle            旋转角度
 * @return Mat 				返回旋转后的图像
 */
Mat arrow_classifier::ImageRotate(Mat src,float angle)
{
    Mat newImg;
    Point2f pt = Point2f((float)src.cols / 2, (float)src.rows / 2);
    Mat M = getRotationMatrix2D(pt, angle, param.arrow_para.template_extend_ratio);
    warpAffine(src, newImg, M, src.size());
    return newImg;
}

/**
 * @brief 多角度模板匹配得到箭头图像置信度
 * @param  arrow         	箭头
 * @return const float 		置信度分数
 */
const float arrow_classifier::Arrow_confidence_Template(arrow_info& arrow)
{

    //set the arrow vertex as srcPoints
    for (int i = 0; i < 4; i++)
        srcPoints[i] = arrow.Points[i];

    //get the arrow image using warpPerspective
    warpPerspective_mat = getPerspectiveTransform(srcPoints, dstPoints);  // get perspective transform matrix  透射变换矩阵
    warpPerspective(warpPerspective_src, warpPerspective_dst, warpPerspective_mat, arrowImgSize, INTER_NEAREST, BORDER_CONSTANT, Scalar(0)); //warpPerspective to get arrowImage
    warpPerspective_dst.copyTo(arrow.warpPerspective_img); //copyto arrowImg

    // imshow("warp",warpPerspective_dst);
    // waitKey(0);
    
    char datapath[100];
    Mat Template;
    if(arrow.arrow_type == Big)
    {
        Template = Big_arrow_template;
        //imwrite("/home/lion/rm/big_arrow_dataset/big.jpg",warpPerspective_dst);
    }
    else 
    {
        Template = Small_arrow_template;
    }
    auto step = param.arrow_para.mtach_step;
    double value = 0;
    Mat newImg;
    Mat result;
    for (int i = 0; i <= 360 / step; i++)
      {
        newImg = ImageRotate(Template, step * i);
        matchTemplate(warpPerspective_dst, newImg, result, TM_CCOEFF_NORMED);
        double minval, maxval;
        Point minloc, maxloc;
        minMaxLoc(result, &minval, &maxval, &minloc, &maxloc);
        if(maxval>value)
        {
            arrow.angle = i;
            value = maxval;
        }
      }
      return value;
}


/**
 * @brief CNN得到箭头图像置信度
 * @param  arrow         	箭头
 * @return const float 		置信度分数
 */
const float arrow_classifier::Arrow_confidence_Cnn(arrow_info& arrow)
{
    float value = 0;
    Mat result;
    //set the arrow vertex as srcPoints
    for (int i = 0; i < 4; i++)
        srcPoints[i] = arrow.Points[i];

    //get the arrow image using warpPerspective
    warpPerspective_mat = getPerspectiveTransform(srcPoints, dstPoints);  // get perspective transform matrix  透射变换矩阵
    warpPerspective(warpPerspective_src, warpPerspective_dst, warpPerspective_mat, arrowImgSize, INTER_NEAREST, BORDER_CONSTANT, Scalar(0)); //warpPerspective to get arrowImage
    warpPerspective_dst.copyTo(arrow.warpPerspective_img); //copyto arrowImg


    Mat blob = cv::dnn::blobFromImage(warpPerspective_dst,1/255.0,Size(64,64));
    small_arrow_net.setInput(blob);
    //推理
    result = small_arrow_net.forward();
    value = 1.0 - result.at<float>(0,1);
    return value;

}

/************************************************寻找箭头角点**********************************************/

/**
 * @brief Construct a new arrow Imgprocess::arrow Imgprocess object
 */
arrow_Imgprocess::arrow_Imgprocess()
{
    Init();
}

/**
 * @brief Init object
 */
void arrow_Imgprocess::Init()
{
    Arrow_Flag = CANNOT_SOLVE;
    classifier.Load_Template(param.big_template_path.data(), param.small_template_path.data());
    classifier.Load_Model(param.small_net_path.data());
}

/**
 * @brief 载入原图像并对其进行预处理
 * @param  src              原图像
 */
void arrow_Imgprocess::pretreatment(const Mat &src)
{
    Mat Hsv_Img = Mat::zeros(Src_Img.size(), Src_Img.type());
    Mat Filter_Img = Mat::zeros(Src_Img.size(), Src_Img.type());
    src.copyTo(Src_Img);
    ImgCenter = Point2f(Src_Img.cols/2, Src_Img.rows/2);
    
    GaussianBlur(Src_Img,Filter_Img,Size(3,3),3);

    cvtColor(Filter_Img, Gray_Img, COLOR_RGB2GRAY);
    //转换HSV颜色体系
    cvtColor(Filter_Img, Hsv_Img, COLOR_RGB2HSV);

    // inRange(
    //         Hsv_Img, Scalar(param.hsv.hmin, param.hsv.smin, param.hsv.vmin),
    //         Scalar(param.hsv.hmax, param.hsv.smax, param.hsv.vmax), Bin_Img);
    
	// GammaTransform(Gray_Img, Gray_Img, param.arrow_para.Gama);
    
	//阈值化
	threshold(Gray_Img, Bin_Img,0, 255, cv::THRESH_OTSU);

    

	//定义椭圆形结构元素，锚点为元素中心点，用于膨胀操作，结构大小为内切3×3矩形的椭圆
	cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
	//开运算，去除白色噪点
	morphologyEx(Bin_Img, Pre_Img, MORPH_OPEN, element);
	// element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
	//morphologyEx(Bin_Img, Pre_Img, MORPH_CLOSE, element);
    classifier.PreLoad(Gray_Img);
    /*八邻域算法骨架提取*/
    Thin_Img = thinImage(Pre_Img);
       
}

/**
 * @brief 识别角标主函数
 */
Arrow_Flag_e arrow_Imgprocess::Arrow_detector()
{
    arrows.clear();
    {
        std::vector<std::vector<Point>> arrowContours;
		cv::findContours(Pre_Img.clone(), arrowContours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
		for (const auto& contour:arrowContours)
		{
            //计算整个轮廓的面积
			float arrowContourArea = contourArea(contour);
            if(contour.size()<=param.arrow_para.arrow_min_size || arrowContourArea<= param.arrow_para.rect_min_area)
                continue;
            cv::RotatedRect rect = minAreaRect(contour);
            float height = MAX(rect.size.height,rect.size.width);
            float width = MIN(rect.size.height,rect.size.width);
            if((height/width)>param.arrow_para.rect_max_ratio)
                continue;
            arrow_info arrow(rect);
            if(height>=param.arrow_para.arrow_max_lenth)
            {
                arrow.arrow_type = Big;
            }
            else
            {
                arrow.arrow_type = Small;
            }
            arrow.score = classifier.Arrow_confidence_Cnn(arrow);
            // Corner_detector_PCA(contour,arrow);
            // this->arrows.emplace_back(arrow);
            if(arrow.score>param.arrow_para.score_threshold)
            {
                std::vector<cv::Point> points_In_contour = getPointsInContours(contour,Pre_Img);
                Arrow_PCA(points_In_contour,arrow);
                if(arrow.arrow_type == Small)
                {
                    Corner_detector_Hull(contour,arrow);
                }
                else
                {
                    Corner_detector_PCA(contour,arrow);
                }
                this->arrows.emplace_back(arrow);
            }  
        }
    }
    size_t big_arrow_count,small_arrow_count=0;
    for(auto arrow:arrows)
    {
        if(arrow.arrow_type == Small)
        {
            small_arrow_count++;
        }
        else
        {
            big_arrow_count++;
        }
    }
    if(small_arrow_count<4 && !big_arrow_count)
    {
        Arrow_Flag = CANNOT_SOLVE;
    }
    else if(small_arrow_count<4 && big_arrow_count)
    {
        Arrow_Flag = ANGLE_SOLVE;
    }
    else if(small_arrow_count ==4)
    {
        Arrow_Flag = PNP_SOLVE;
    }
    return Arrow_Flag;
}


/**
 * @brief 对角标进行主成分分析
 * @param  pts              轮廓内所有点的合集
 * @param  arrow            角标
 */
void arrow_Imgprocess::Arrow_PCA(const std::vector<Point>& pts, arrow_info& arrow)
{
    arrow.eigen_vecs.clear();
    arrow.eigen_vecs.resize(2);
    arrow.eigen_val.clear();
    arrow.eigen_val.resize(2);
    Mat pac_img = Src_Img.clone();
    //构建pca数据。这里做的是将轮廓点的x和y作为两个维压到data_pts中去。
    Mat data_pts = Mat(pts.size(), 2, CV_64FC1);//使用mat来保存数据，也是为了后面pca处理需要
    for (int i = 0; i < data_pts.rows; ++i)
    {
        data_pts.at<double>(i, 0) = pts[i].x;
        data_pts.at<double>(i, 1) = pts[i].y;
    }
    //执行PCA分析
    PCA pca_analysis(data_pts, Mat(), 0);
    //获得最主要分量（均值），在本例中，对应的就是轮廓中点，也是图像中点
    arrow.HeartOfShape = Point(pca_analysis.mean.at<double>(0, 0), pca_analysis.mean.at<double>(0, 1));
    
    for (int i = 0; i < 2; ++i)
    {
        arrow.eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0), pca_analysis.eigenvectors.at<double>(i, 1));
        arrow.eigen_val[i] = pca_analysis.eigenvalues.at<double>(i, 0);//在轮廓/图像中点绘制小圆
    }
    circle(pac_img, arrow.HeartOfShape, 3, CV_RGB(255, 0, 255), 1);
    //计算出直线，在主要方向上绘制直线（每个特征向量乘以其特征值并转换为平均位置。有一个 0.02 的缩放系数，它只是为了确保矢量适合图像并且没有 10000 像素的长度）
    line(pac_img, arrow.HeartOfShape, arrow.HeartOfShape + 0.5 * Point(arrow.eigen_vecs[0].x * arrow.eigen_val[0], arrow.eigen_vecs[0].y * arrow.eigen_val[0]), CV_COLOR_RED,3);
    line(pac_img, arrow.HeartOfShape, arrow.HeartOfShape + 0.5 * Point(arrow.eigen_vecs[1].x * arrow.eigen_val[1], arrow.eigen_vecs[1].y * arrow.eigen_val[1]), CV_COLOR_RED,3);
    imshow("show_pca",pac_img);
    waitKey(0);
}

/**
 * @brief 寻找角标最外面的角点
 * @param  arrow            My Param doc
 * @return cv::Point2f 角点坐标
 */
cv::Point2f arrow_Imgprocess::Corner_detector(arrow_info& arrow)
{
    vector<Point2f> corners;
    Rect imgBound = Rect(cv::Point(0, 0), Src_Img.size());
    Rect bbox = (arrow.rec().boundingRect()) & imgBound;
    Mat mask = Mat::zeros(Src_Img.size(), CV_8UC1); 
    mask(bbox).setTo(255);
    int maxcorners = 1;
    double qualityLevel = 0.1;  //角点检测可接受的最小特征值
	double minDistance = 10;	//角点之间最小距离
	int blockSize = 7;//计算导数自相关矩阵时指定的领域范围
	double  k = 0.04; //权重系数
    goodFeaturesToTrack(Thin_Img,corners,maxcorners,qualityLevel,minDistance,mask,blockSize,false,k);
    return corners[0];
}


/**
 * @brief 采用凸包方法寻找最外角点(Hull)
 * @param  pts              最外部轮廓点
 * @param  arrow            角标类
 */
void arrow_Imgprocess::Corner_detector_Hull(const std::vector<Point>& pts, arrow_info& arrow)
{
    //找出最小包围三角形
    vector<Point> triangle;
    minEnclosingTriangle(pts, triangle);
    Mat hull_img = Src_Img.clone();
    //绘制最小包围三角形
    for(size_t i = 0; i < 3; i++ )
        line(hull_img, triangle[i], triangle[(i+1)%3], Scalar(255, 255, 0), 1, LINE_AA);
    
    // cv::Vec4f line1(triangle[0].x,triangle[0].y,triangle[1].x,triangle[1].y);
    // cv::Vec4f line2(triangle[0].x,triangle[0].y,triangle[2].x,triangle[2].y);
    // cv::Vec4f line3(triangle[1].x,triangle[1].y,triangle[2].x,triangle[2].y);
    // double angle_1 = angleLineOf(line1,line2);
    // double angle_2 = angleLineOf(line1,line3);
    // double angle_3 = angleLineOf(line2,line3);
    double angle_1 = angleLineOf(triangle[1],triangle[2],triangle[0]);
    double angle_2 = angleLineOf(triangle[0],triangle[2],triangle[1]);
    double angle_3 = angleLineOf(triangle[0],triangle[1],triangle[2]);
    cout<<"angle_1: "<<angle_1<<endl;
    cout<<"angle_2: "<<angle_2<<endl;
    cout<<"angle_3: "<<angle_3<<endl;
    cv::Point vertex;
    if(angle_1>angle_2)
    {
        if(angle_1>angle_3)
        {
            vertex = triangle[0];
        }
        else
        {
            vertex = triangle[2];
        }
    }
    else
    {
        if(angle_2>angle_3)
        {
            vertex = triangle[1];
        }
        else
        {
            vertex = triangle[2];
        }
    }

    // // /*寻找角点*/
    // double corner_max_proj,corner_min_proj = 0;
    // double corner_max_pos,corner_min_pos = 0;
    // for(int i=0; i<pts.size();i++)
    // {
    //     //相对形心的坐标
    //     Point lpt = pts.at(i) - vertex;
    //     //对水平特征方向上的投影
    //     double corner_projection = arrow.eigen_vecs[1].dot(lpt);
    //     //对竖直特征方向上的投影
    //     double endpoint_projection = arrow.eigen_vecs[0].dot(lpt);
    //     if(corner_projection > corner_max_proj)
    //     {
    //         corner_max_proj = corner_projection;
    //         corner_max_pos = i;
    //     }
    //     if(corner_projection < corner_min_proj)
    //     {
    //         corner_min_proj = corner_projection;
    //         corner_min_pos = i;
    //     }
    // }


    // /**竖直方向投影绝对值*/
    // double tmp1 = abs(arrow.eigen_vecs[0].dot(pts.at(corner_min_pos)-vertex));
    // double tmp2 = abs(arrow.eigen_vecs[0].dot(pts.at(corner_max_pos)-vertex));

    // if(tmp1 <= tmp2)
    // {
    //     arrow.corner = pts.at(corner_min_pos);
    // }
    // else
    // {
    //     arrow.corner = pts.at(corner_max_pos);
    // }
    double corner_min_dis = std::sqrt(std::pow((pts[0].x - vertex.x), 2) + std::pow((pts[0].y - vertex.y), 2));;
    for(const auto& pt: pts)
    {
        //对顶点的距离
        double distance = std::sqrt(std::pow((pt.x - vertex.x), 2) + std::pow((pt.y - vertex.y), 2));
        if(distance < corner_min_dis)
        {
            corner_min_dis = distance;
            arrow.corner = pt;
        }
    }


    circle(hull_img,vertex,5,CV_COLOR_RED,2);
    circle(hull_img,arrow.corner,5,CV_COLOR_RED,2);
    imshow("show_hull",hull_img);
}

/**
 * @brief 寻找角标最外面的角点和端点(PCA)
 * @param  arrow            My Param doc
 * @param  corners            My Param doc
 * @return cv::Point2f
 */
void arrow_Imgprocess::Corner_detector_PCA(const std::vector<Point>& pts, arrow_info& arrow)
{
    

   
    /*寻找角点*/
    double corner_max_proj,corner_min_proj = 0;
    double corner_max_pos,corner_min_pos = 0;
    /*寻找端点*/
    double endpoint_max_proj,endpoint_min_proj = 0;
    double endpoint_max_pos,endpoint_min_pos = 0;
    for(int i=0; i<pts.size();i++)
    {
        //相对形心的坐标
        Point lpt = pts.at(i) - arrow.HeartOfShape;
        //对水平特征方向上的投影
        double corner_projection = arrow.eigen_vecs[1].dot(lpt);
        //对竖直特征方向上的投影
        double endpoint_projection = arrow.eigen_vecs[0].dot(lpt);
        if(corner_projection > corner_max_proj)
        {
            corner_max_proj = corner_projection;
            corner_max_pos = i;
        }
        if(corner_projection < corner_min_proj)
        {
            corner_min_proj = corner_projection;
            corner_min_pos = i;
        }
        if(endpoint_projection > endpoint_max_proj)
        {
            endpoint_max_proj = endpoint_projection;
            endpoint_max_pos = i;
        }
        if(endpoint_projection < endpoint_min_proj)
        {
            endpoint_min_proj = endpoint_projection;
            endpoint_min_pos = i;
        }
    }


    /**竖直方向投影绝对值*/
    double tmp1 = abs(arrow.eigen_vecs[0].dot(pts.at(corner_min_pos)-arrow.HeartOfShape));
    double tmp2 = abs(arrow.eigen_vecs[0].dot(pts.at(corner_max_pos)-arrow.HeartOfShape));

    if(tmp1 <= tmp2)
    {
        arrow.corner = pts.at(corner_min_pos);
    }
    else
    {
        arrow.corner = pts.at(corner_max_pos);
    }

    arrow.endpoint[0] = pts.at(endpoint_min_pos);
    arrow.endpoint[1] = pts.at(endpoint_max_pos);

    if(arrow.endpoint[0].y >= arrow.endpoint[1].y)
    {
        std::swap(arrow.endpoint[0],arrow.endpoint[1]);
    }
    
    //大角标还需要找到内角点的坐标  
    if(arrow.arrow_type == Big)
    {
        //求出主方向直线和外接矩形的交点，
        Point2f pt1 = arrow.corner;
        Point2f pt2 = pt1+0.08 * Point2f(arrow.eigen_vecs[1].x * arrow.eigen_val[1], arrow.eigen_vecs[1].y * arrow.eigen_val[1]);

        //遍历两个交点之间的线段，得出和轮廓的交点
        LineIterator it(Src_Img.clone(), pt1, pt2, 8);
        for(int i = 0; i < it.count; i++, ++it)
        {
            Point pt(it.pos());//获得线段上的点
            if (abs(pointPolygonTest(pts,pt,true)) < 1)
            {
                arrow.inner = pt;
            }         
        }
    }
    else
    {
        arrow.inner = Point2f(0,0);
    }
    

    
    
}


/**
 * @brief 返回角点坐标
 * @return const vector<Point2f> 
 */
std::vector<Point2f> arrow_Imgprocess::get_PointOfPnp()
{

    PointOfSlot.clear();
    //存储小角标
    std::vector<arrow_info> tmp;
    tmp.clear();
    for(auto arrow:arrows)
    {
        if(arrow.arrow_type == Small)
        {
            tmp.emplace_back(arrow);
        }
    }
    if(tmp.size()==4)
    {
        std::sort(tmp.begin(), tmp.end(), [](const arrow_info & a, const arrow_info & b)
	    {
		    return  a.area < b.area;
	    });
        std::sort(tmp.begin()+1, tmp.end(), [](const arrow_info & a, const arrow_info & b)
	    {
		    return  a.center.x < b.center.x;
	    });
        if(tmp.at(1).center.y > tmp.at(2).center.y)
        {
            std::swap(tmp.at(2), tmp.at(1));
        }
        PointOfSlot.resize(4);
        for(int i = 0; i < 4; i++)
        {
            PointOfSlot.at(i) = tmp.at(i).corner;
        }
    }
    
        
    return PointOfSlot;
}


/**
 * @brief 返回大角标的角点和端点便于角度解算
 * @return const vector<Point2f> 
 */
std::vector<Point2f> arrow_Imgprocess::get_PointOfAngle()
{
    std::vector<Point2f> PointOfAngle;
    //存储大角标
    arrow_info tmp;
    for(auto arrow:arrows)
    {
        if(arrow.arrow_type == Big)
        {
            tmp = arrow;
        }
    }

    PointOfAngle.resize(4);
    //考虑大角标的顶点方向问题
    PointOfAngle.at(0) = tmp.corner;
    PointOfAngle.at(1) = tmp.endpoint[0];
    PointOfAngle.at(3) = tmp.inner;
    PointOfAngle.at(2) = tmp.endpoint[1];
    
    return PointOfAngle;
}

/**
 * @brief 可视化检测结果
 */
void arrow_Imgprocess::Draw_bound()
{
    arrowDisplay = Src_Img.clone();
    if(!arrows.empty())
    {
        
        string tmp_str;
        cv::Scalar color = CV_COLOR_YELLOW;
        putText(arrowDisplay, "FOUND"+std::to_string(arrows.size())+"ARROW", Point(100, 50), FONT_HERSHEY_SIMPLEX, 1, CV_COLOR_PINK, 2, 8, false);
	    for(auto arrow:arrows)
	    {
            if(arrow.arrow_type == Small)
            {
                tmp_str = "Small_Arrow";
                color = CV_COLOR_YELLOW;
            }
            else
            {
                tmp_str = "Big_Arrow";
                color = CV_COLOR_GREEN;
            }
            for (size_t i = 0; i < 4; i++)
            {
                line(arrowDisplay, arrow.get_vertex()[i], arrow.get_vertex()[(i + 1) % 4], color, 2, 8, 0);
            }
            putText(arrowDisplay, tmp_str+to_string(float(arrow.score)), arrow.rec().boundingRect().tl() + Point2i(0, 30), FONT_HERSHEY_PLAIN, 1, Scalar(255, 255, 255), 1, 8, false);
            
            if(arrow.arrow_type == Big)
                circle(arrowDisplay,arrow.inner,2,Scalar(0,0,255));
            circle(arrowDisplay,arrow.corner,2,CV_COLOR_RED);
            circle(arrowDisplay,arrow.endpoint[0],2,CV_COLOR_GREEN);
            circle(arrowDisplay,arrow.endpoint[1],2,CV_COLOR_RED);
	    }
    }
    if(!PointOfSlot.empty())
    {
        for(const auto& pt : PointOfSlot)
        {
            //display its center point x,y value 显示中点坐标
            putText(arrowDisplay, to_string(int(pt.x)), pt, FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 255), 1, 8, false);
            putText(arrowDisplay, to_string(int(pt.y)), pt+ Point2f(0, 15), FONT_HERSHEY_PLAIN, 1, Scalar(255, 0, 255), 1, 8, false);
            circle(arrowDisplay, pt, 2, CV_COLOR_RED, 2);
        }

    }
    
}

/**
 * @brief 展示过程图像
 */
void arrow_Imgprocess::debug_show()
{
    imshow("gray",Gray_Img);
    imshow("Pre_bin",Pre_Img);
    imshow("thin",Thin_Img);
    imshow("result",arrowDisplay);

    //waitKey(1);
}