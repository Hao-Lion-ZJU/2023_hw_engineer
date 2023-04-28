/*
 * @Description: gpio
 * @Version: 1.0
 * @Autor: Julian Lin
 * @Date: 2022-11-19 22:49:48
 * @LastEditors: Julian Lin
 * @LastEditTime: 2022-12-03 17:17:40
 */
#ifndef GPIO_HPP
#define GPIO_HPP

#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <string>
#define ON "1"                      //开启
#define OFF "0"                     //关闭
#define BUZZER "436"                //控制引脚
#define DIRECTION "out"             //方向为输出
#define GPIO_SUCCESS 0              //成功
#define GPIO_OPEN_ERROR -1          //打开失败
#define GPIO_CLOSE_ERROR -2         //关闭失败
#define GPIO_SET_VALUE_ERROR -3     //  设置电平失败
#define GPIO_SET_DIRECTION_ERROR -4 //设置方向失败
#define GPIO_SET_EDGE_ERROR -5      //设置边沿失败
#define GPIO_READ_VALUE_ERROR -6    //读取电平失败

class Gpio
{
private:
    /* data */
    char *port;
    int OpenGpio();                              //导入gpio
    int CloseGpio();                             //导出gpio
    int ReadGPIOValue(int level);                //读取gpio值
    int SetGpioDirection(const char *direction); //设置gpio方向
    int SetGpioValue(const char *level);         //设置gpio值
    int SetGpioEdge(const char *edge);           //设置gpio输入模式
public:
    Gpio(/* args */);
    Gpio(char *port);
    ~Gpio();

    void PWM(); // PWN
    inline std::string GpioGetErrorString(int status)
    {
        switch (status)
        {
        case GPIO_OPEN_ERROR:
            return "GPIO_OPEN_ERROR";
            break;
        case GPIO_CLOSE_ERROR:
            return "GPIO_CLOSE_ERROR";
            break;
        case GPIO_SET_VALUE_ERROR:
            return "GPIO_SET_VALUE_ERROR";
            break;
        case GPIO_SET_DIRECTION_ERROR:
            return "GPIO_SET_DIRECTION_ERROR";
            break;
        case GPIO_SET_EDGE_ERROR:
            return "GPIO_SET_EDGE_ERROR";
            break;
        case GPIO_READ_VALUE_ERROR:
            return "GPIO_READ_VALUE_ERROR";
            break;
        default:
            break;
        }
    };
};

#endif
