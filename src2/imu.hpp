#pragma once

#include <stdint.h>
namespace breeze::imu
{
struct imu_data_t
{
    float raw_acc[3];
    float raw_gyr[3];
    float body_acc[3];//去除重力加速度后的机体加速度

    float yaw;
    float pitch;
    float roll;
    
    //经过低通滤波的角速度
    float rate_yaw;
    float rate_pitch;
    float rate_roll;
};
}
