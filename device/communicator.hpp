/*
 * @Description: serial encapsulation
 * @Version: 1.0
 * @Autor: dangwi
 * @Date: 2022-02-27
 * @LastEditors: Julian Lin
 * @LastEditTime: 2022-04-08 09:02:33
 */

#ifndef __COMMUNICATOR_HPP__
#define __COMMUNICATOR_HPP__

#include <stdint.h>
#include <glog/logging.h>
#include "serial/serial.h"

struct SerialPkg
{
    uint8_t frame[12];
    uint8_t size;

    // 自定义发送包
    // 发送自瞄角度
    SerialPkg(float predict_pitch_angle, float predict_yaw_angle, int16_t shoot_flag, int16_t target_num, int16_t target_color = 0){
		frame[0] = 0xaa, frame[1] = 0xbb, frame[2] = 0xcc;
		int16_t pitch_angle = predict_pitch_angle*100 ;
		int16_t yaw_angle = predict_yaw_angle*100 ; 

		frame[3] = (uint8_t)(pitch_angle >> 8);
		frame[4] = (uint8_t)(pitch_angle);
		frame[5] = (uint8_t)(yaw_angle >> 8);
		frame[6] = (uint8_t)(yaw_angle);
        frame[7] = (uint8_t)(shoot_flag | target_color << 2  | target_num << 4);
	LOG(INFO) << "color: " << target_color << " num : " << target_num << " send: " << (shoot_flag + target_color * 4 + target_num * 16);
        frame[8] = 0xFF;
		size = 9;
	}

    // 哨兵底盘的NX
    SerialPkg(float top_pitch, float top_yaw, float down_pitch, float down_yaw){
		frame[0] = 0xaa, frame[1] = 0xbb, frame[2] = 0xcc;

		int16_t top_pitch100  = top_pitch  * 100;
		int16_t top_yaw100    = top_yaw    * 100 ; 
		int16_t down_pitch100 = down_pitch * 100;
		int16_t down_yaw100   = down_yaw   * 100 ; 

		frame[3]  = (uint8_t)(top_pitch100 >> 8);
		frame[4]  = (uint8_t)(top_pitch100);
		frame[5]  = (uint8_t)(top_yaw100 >> 8);
		frame[6]  = (uint8_t)(top_yaw100);
		frame[7]  = (uint8_t)(down_pitch100 >> 8);
		frame[8]  = (uint8_t)(down_pitch100);
		frame[9]  = (uint8_t)(down_yaw100 >> 8);
		frame[10] = (uint8_t)(down_yaw100);

		size = 11;
	}
};


class Communicator: public serial::Serial 
{
private:
    bool enable;
    bool connected;
public:
    // 不上车调试时，设enable为false
    Communicator(bool _enable);
    ~Communicator();

    void connect();

    // usage: send({...})
    // 隐式调用SerialPkg构造
    void send(const SerialPkg &pkg) ;

    int receive(uint8_t *buffer, size_t n);
};



#endif
