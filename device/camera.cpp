/*
 * @Description: camera
 * @Version: 1.0
 * @Autor: dangwi
 * @Date: 2022-04-06
 */

#include <thread>
#include <chrono>
#include <glog/logging.h>

#include "camera.hpp"
#include "params.hpp"

using namespace std;
using namespace cv;

void Camera::setParamResolution()
{
    Mat firstFrame;
    while ( firstFrame.empty() ) getFrame(firstFrame);
    param.camera.width = firstFrame.cols;
    param.camera.height= firstFrame.rows;
}

Camera::~Camera()
{
    LOG(INFO) << "相机关闭中 ... ";
}

/*********************************************************/
/******************* * *  MVCamera  * * ******************/
/*********************************************************/

#define MV_CHECK_API_ERROR(expr) do {                    \
    auto status = (expr);                                \
    if ( status != CAMERA_STATUS_SUCCESS ) {             \
        LOG(ERROR) << #expr << " -> " << status << " "   \
            << string(CameraGetErrorString(status));     \
        return false;                                    \
    }                                                    \
} while ( 0 );

#define MV_CHECK_API_WARNING(expr) do {                  \
    auto status = (expr);                                \
    if ( status != CAMERA_STATUS_SUCCESS ) {             \
        LOG(WARNING) << #expr << " -> " << status << " " \
            << string(CameraGetErrorString(status));     \
        break;                                           \
    }                                                    \
} while ( 0 );

MVCamera::MVCamera(const string &_camera_name)
:camera_name(_camera_name)
{
    
    while ( false == open() ) {
        LOG(ERROR) << "迈德相机打开失败...";
        this_thread::sleep_for(chrono::milliseconds(1000));
    }
}

bool MVCamera::open()
{
    MV_CHECK_API_ERROR(CameraSdkInit(1))

    // 寻找相机
    tSdkCameraDevInfo tCameraEnumList[4];
    int camera_idx = -1;
    while (1) {
        int camera_nums = 4;
        LOG(INFO) << "looking for camera <" << camera_name << ">";
        MV_CHECK_API_ERROR(CameraEnumerateDevice(tCameraEnumList, &camera_nums))
        for (int i=0; i<camera_nums; i++) {
            if ( tCameraEnumList[i].acFriendlyName == camera_name \
                 || camera_name == "auto" ) {
                camera_idx = i;
                break;
            }
            LOG(INFO) << "camera <" << tCameraEnumList[i].acFriendlyName << "> detected";
        }
        if ( camera_idx != -1 ) break;

        LOG(WARNING) << camera_nums << " cameras detected " \
          << "but camera <" << camera_name << "> not found";
        this_thread::sleep_for(chrono::milliseconds(1000));
        // MV_CHECK_API_ERROR(CameraSdkInit(1))
    }
    LOG(INFO) << "found camera <" << tCameraEnumList[camera_idx].acFriendlyName << ">";

    //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    MV_CHECK_API_ERROR(CameraInit(&(tCameraEnumList[camera_idx]), -1, -1, &hCamera))

    //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    MV_CHECK_API_WARNING(CameraGetCapability(hCamera, &tCapability))

    //设置触发模式, 0: 连续采集 1: 软触发 2: 硬触发
    MV_CHECK_API_WARNING(CameraSetTriggerMode(hCamera, 0))

    //设置输出为彩色
    MV_CHECK_API_WARNING(CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8))

    //设置曝光
    if ( -1 != param.camera.exposure ) {
        MV_CHECK_API_WARNING(CameraSetAeState(hCamera, false))  // 取消自动曝光
        MV_CHECK_API_WARNING(CameraSetExposureTime(hCamera, param.camera.exposure))
    }

    // 设置增益 
    if ( -1 != param.camera.rGain && -1 != param.camera.gGain && -1 != param.camera.bGain)
        MV_CHECK_API_WARNING(CameraSetGain(hCamera, param.camera.rGain, param.camera.gGain, param.camera.bGain))
    if ( -1 != param.camera.analogGain )
        MV_CHECK_API_WARNING(CameraSetAnalogGain(hCamera, param.camera.analogGain))
    if ( -1 != param.camera.gamma )
        MV_CHECK_API_WARNING(CameraSetGamma(hCamera, param.camera.gamma))

    // 分辨率设置
    tSdkImageResolution RoiResolution;
    RoiResolution.iIndex = 0xff;
    RoiResolution.iWidthFOV = 1280;
    RoiResolution.iHeightFOV = 1024;
    RoiResolution.iWidth = 1280;
    RoiResolution.iHeight = 1024;
    // 04/19: iWidth 和 iHeight 要和相机匹配，否则会出错
    // 04/30: 换了MVSDK_V2.1.0.31又坏了，不得不说这个包真的垃圾
    // MV_CHECK_API_WARNING(CameraSetImageResolution(hCamera, &RoiResolution))
    
    // 畸变矫正设置
    // double cameraMatrix[] = {param.camera.fx, param.camera.fy, param.camera.cx, param.camera.cy};
    // double distCoeff[ ] = {param.camera.k1, param.camera.k2, param.camera.p1, param.camera.p2, param.camera.k3};
    // MV_CHECK_API_WARNING(CameraSetUndistortParams(hCamera, 1280, 1024, cameraMatrix, distCoeff));

    MV_CHECK_API_WARNING(CameraPlay(hCamera))

    this->setParamResolution();
    this_thread::sleep_for(chrono::milliseconds(5));

    return true;
}

bool MVCamera::getFrame(cv::Mat &img)
{
    tSdkFrameHead sFrameInfo;
    BYTE *pbyBuffer;
    while(CameraConnectTest(hCamera) != CAMERA_STATUS_SUCCESS){
        // CameraReConnect(hCamera);    MVAPI默认有开自动重连，所以这里不要开手动重连，开着反而会失败
        LOG(ERROR) << "MV相机: 我寄了救救我 救救我我寄了 完蛋了完蛋了完蛋了完蛋了";
        this_thread::sleep_for(chrono::milliseconds(500));
    }
    MV_CHECK_API_ERROR(CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 100))
    if ( img.cols != sFrameInfo.iWidth, img.rows != sFrameInfo.iHeight ) {
        img.create(sFrameInfo.iHeight, sFrameInfo.iWidth, CV_8UC3);
    }
    MV_CHECK_API_ERROR(CameraImageProcess(hCamera, pbyBuffer, img.data, &sFrameInfo))
    //在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
    //否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
    MV_CHECK_API_ERROR(CameraReleaseImageBuffer(hCamera, pbyBuffer))

    return true;
}

bool MVCamera::close()
{
    MV_CHECK_API_ERROR(CameraSaveParameterToFile(hCamera, "mvcamera.config"))

    MV_CHECK_API_ERROR(CameraUnInit(hCamera));
}

/*********************************************************/
/****************** * *  USBCamera  * * ******************/
/*********************************************************/

USBCamera::USBCamera(int index)
{
    while ( cap.open(index, CAP_V4L2) == false ) {
        LOG(ERROR) << "can't find usb camera " << index;
        this_thread::sleep_for(chrono::milliseconds(1000));
    }
    sentryChassisSet();
    this->setParamResolution();
}

USBCamera::USBCamera(const char *cam_name)
{
    while ( cap.open(cam_name, CAP_V4L2) == false ) {
        LOG(ERROR) << "can't find usb camera " << cam_name;
        this_thread::sleep_for(chrono::milliseconds(1000));
    }
    sentryChassisSet();
    this->setParamResolution();
}

// 针对哨兵底盘的设置，不然跑不到120帧和全像素
void USBCamera::sentryChassisSet()
{
    cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CAP_PROP_FRAME_HEIGHT, 720);
}

bool USBCamera::getFrame(Mat &img)
{
    return cap.read(img);
}


/*********************************************************/
/***************** * *  VideoCamera  * * *****************/
/*********************************************************/
bool VideoCamera::ReadImages(cv::String pattern){
    LOG(INFO) << "dataset preparing";
    cv::glob(pattern, fn, false);
    if( fn.empty() ) return false;

    sort(fn.rbegin(), fn.rend(), 
        [](const string &s1, const string &s2) {
            size_t i1 = s1.rfind('/'), i2 = s2.rfind('/');

            if ( i1 != string::npos && i2 != string::npos ) {
                size_t d1 = s1.rfind('.'), d2 = s2.rfind('.');
                string sub1 = s1.substr(i1+1, d1-i1-1),   
                       sub2 = s2.substr(i2+1, d2-i2-1);   
                if ( sub1.length() == sub2.length() )
                    return sub1 < sub2;
                else
                    return sub1.length() < sub2.length();
            }
            else {
                return s1 < s2;
            }
        });
    return true;
}
VideoCamera::VideoCamera(const string &_path)
:path(_path)
{
    if ( path.find(".mp4") != string::npos ) {
        // todo : 当 _path 位置不存在时会卡死 
        cap.open(path);
        cap.set(CAP_PROP_POS_FRAMES, 200);
        type = 0;
    }
    else if ( path.find(".jpg") != string::npos \
           || path.find(".png") != string::npos ) {
        type = 1;
    }
    else {
        type = 2;
        vector<string> patterns = {"/*.jpg","/*.png"};
        int i = 0;
        while(i < patterns.size() && !ReadImages(_path + patterns[i])) i++;
        if(i >= patterns.size()) LOG(INFO) << "ERROR PATH, PLZ CHECK IT !!!";
    }
    this->setParamResolution();
}

bool VideoCamera::getFrame(Mat &img)
{
    if ( type == 0 ) {
        return cap.read(img);
    }
    else if ( type == 1 ) {
        img = imread(path);
    }
    if ( type == 2 ) {
        if ( fn.empty() ) return false;
        string img_path = fn.back();
        img = imread(img_path);
        fn.pop_back();
    }
    
    return true;
}
