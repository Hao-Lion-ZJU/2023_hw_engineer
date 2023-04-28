/*
 * @Description: gpio
 * @Version: 1.0
 * @Autor: Julian Lin
 * @Date: 2022-11-19 22:50:04
 * @LastEditors: Julian Lin
 * @LastEditTime: 2022-12-03 18:30:41
 */
#include "gpio.hpp"
#include <glog/logging.h>
#include "utils.hpp"
#include "params.hpp"
#include "string"
#ifndef GPIO_CHECK_ERROR
#define GPIO_CHECK_ERROR(expr)                                \
    do                                                        \
    {                                                         \
        auto status = (expr);                                 \
        if (status != GPIO_SUCCESS)                           \
        {                                                     \
            LOG(ERROR) << #expr << " -> " << status << " "    \
                       << GpioGetErrorString(status); \
            assert(0);                                        \
        }                                                     \
    } while (0);
#endif

Gpio::Gpio(/* args */)
{
}
Gpio::Gpio(char *port)
{
    this->port = (char *)port;
}
Gpio::~Gpio()
{
}

int Gpio::OpenGpio()
{
    int fd;
    const char *path = "/sys/class/gpio/export";

    fd = open(path, O_WRONLY);
    if (fd == -1)
    {
        perror("Failed to open gpio! ");
        return -1;
    }

    write(fd, port, sizeof(port));
    close(fd);
    return 0;
}

int Gpio::CloseGpio()
{
    int fd;
    const char *path = "/sys/class/gpio/unexport";

    fd = open(path, O_WRONLY);
    if (fd == -1)
    {
        perror("Failed to open gpio! ");
        return -1;
    }

    write(fd, port, sizeof(port));
    close(fd);
    return 0;
}

int Gpio::ReadGPIOValue(int level)
{
    int fd;
    char path[40];
    sprintf(path, "/sys/class/gpio/gpio%s/value", port);
    fd = open(path, O_RDONLY);
    if (fd == -1)
    {
        perror("Failed to read GPIO value!");
        return -1;
    }
    char value[2];
    read(fd, value, 1);
    level = atoi(value);

    return 0;
}

int Gpio::SetGpioDirection(const char *direction)
{
    int fd;
    char path[40];
    sprintf(path, "/sys/class/gpio/gpio%s/direction", port);
    fd = open(path, O_WRONLY);

    if (fd == -1)
    {
        perror("Failed to set GPIO direction. ");

        return -1;
    }

    write(fd, direction, sizeof(direction));
    close(fd);
    return 0;
}

int Gpio::SetGpioEdge(const char *edge)
{
    int fd;
    char path[40];
    sprintf(path, "/sys/class/gpio/gpio%s/edge", port);
    fd = open(path, O_WRONLY);

    if (fd == -1)
    {
        perror("Failed to set GPIO edge. ");

        return -1;
    }

    write(fd, edge, sizeof(edge));
    close(fd);
    return 0;
}

int Gpio::SetGpioValue(const char *level)
{
    int fd;
    char path[40];
    sprintf(path, "/sys/class/gpio/gpio%s/value", port);
    fd = open(path, O_RDWR);
    if (fd == -1)
    {
        perror("Failed to set GPIO value! ");
        return -1;
    }

    write(fd, level, sizeof(level));
    close(fd);
    return 0;
}
void Gpio::PWM()
{
    GPIO_CHECK_ERROR(SetGpioValue(ON))
    GPIO_CHECK_ERROR(SetGpioValue(OFF))
}
