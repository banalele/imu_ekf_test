#pragma once

#include <algorithm>
#include <cmath>
#include <optional>

#include "robotpilots/algorithm/ekf_filter.hpp"
#include "robotpilots/algorithm/math.hpp"
#include "imu.hpp"

namespace breeze::imu
{

using robotpilots::algorithm::Ekf;
using robotpilots::algorithm::Eye;
using robotpilots::algorithm::LowPassFilter;
using robotpilots::algorithm::Matrix;
using robotpilots::algorithm::Zeros;

/// @brief 6 状态 IMU 姿态 EKF，移植自 bmi_stream.cpp 的 IMU_QuaternionEKF。
///        状态 x = [q0, q1, q2, q3, bias_x, bias_y]，
///        观测 z = 加速度计归一化后的重力方向（3x1）。
class ImuEkf final
{
public:
    ImuEkf()  
    {
        Ekf::Config config_;

        config_.InitialState = Zeros<float>(6, 1);
        config_.InitialState[0][0] = 1.0f; // 单位四元数

        // 对应 IMU_QuaternionEKF_P：四元数 100000，零偏 100
        config_.InitialCovariance = Eye<float>(6);
        config_.InitialCovariance *= 100000.0f;
        config_.InitialCovariance[4][4] = 100.0f;
        config_.InitialCovariance[5][5] = 100.0f;

        // 对应 Set_Q()：Q = diag(Q1*dt ×4, Q2*dt ×2)
        // Q1 设置四元数的过程噪声协方差矩阵，越小解算数据越平滑，越大系统对快速变化的反应越快   10
        //Q2 设置陀螺仪零偏估计过程噪声协方差矩阵     0.001
        constexpr float nominalDt = 0.001f;
        config_.ProcessNoise = Zeros<float>(6, 6);
        for (int i = 0; i < 4; ++i)
        {
            config_.ProcessNoise[i][i] = 10 * nominalDt;
        }
        config_.ProcessNoise[4][4] = 0.001f * nominalDt;
        config_.ProcessNoise[5][5] = 0.001f * nominalDt;

        // 对应 Set_R()：R = diag(R, R, R)
        //R  设置加速度计测量噪声协方差矩阵，越小对加速度越信任，系统对快速变化的反应越快       1000000
        config_.MeasurementNoise = Eye<float>(3);
        config_.MeasurementNoise *= 1000000.0f; // 加速度计量测噪声
        config_.InnovationThreshold = 3.5e-8f;

        // 状态转移：使用陀螺角速度的四元数指数映射进行非线性传播
        config_.StateTransition = [](const Matrix<float> &state, const Matrix<float> &control,
                                    float deltaTime)
        {
            
            float gx = control[0][0];
            float gy = control[1][0];
            float gz = control[2][0];

            gx -= state[4][0];
            gy -= state[5][0];

            // angularRate：𝜔 角速度向量的模长
            const float angularRate = std::sqrt(gx * gx + gy * gy + gz * gz);
            const float halfAngle = 0.5f * angularRate * deltaTime;
            float vectorScale = 0.5f * deltaTime;
            // 小角度泰勒展开，避免除零,只有在角速度大于 1e-6 rad/s 时才使用 sin(θ/2)/|ω|
            if (angularRate > 1e-6f)
            {
                vectorScale = std::sin(halfAngle) / angularRate;
            }
            const float dq0 = std::cos(halfAngle);
            const float dq1 = gx * vectorScale;
            const float dq2 = gy * vectorScale;
            const float dq3 = gz * vectorScale;

            const float q0 = state[0][0];
            const float q1 = state[1][0];
            const float q2 = state[2][0];
            const float q3 = state[3][0];

            // 指数映射：q⊗[cos⁡(θ/2), ω^sin⁡(θ/2)]，其中 θ=∣ω∣Δt
            Matrix<float> next = Zeros<float>(6, 1);
            next[0][0] = q0 * dq0 - q1 * dq1 - q2 * dq2 - q3 * dq3;
            next[1][0] = q0 * dq1 + q1 * dq0 + q2 * dq3 - q3 * dq2;
            next[2][0] = q0 * dq2 - q1 * dq3 + q2 * dq0 + q3 * dq1;
            next[3][0] = q0 * dq3 + q1 * dq2 - q2 * dq1 + q3 * dq0;
            next[4][0] = state[4][0];
            next[5][0] = state[5][0];

            NormalizeQuaternion(next);
            return next;
        };

        // 观测模型：由四元数推算重力方向（对应 EKF_H_Update）
        config_.MeasurementModel = [](const Matrix<float> &state)
        {
            /* H
            0     1     2     3     4     5
            6     7     8     9    10    11
            12    13    14    15    16    17
            last two cols are zero
            */
            const float q0 = state[0][0];
            const float q1 = state[1][0];
            const float q2 = state[2][0];
            const float q3 = state[3][0];

            Matrix<float> h = Zeros<float>(3, 1);
            h[0][0] = 2.0f * (q1 * q3 - q0 * q2);
            h[1][0] = 2.0f * (q0 * q1 + q2 * q3);
            h[2][0] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
            return h;
        };

        // 状态雅可比 F（对应 EKF_F_XhatMinus_Update + EKF_jacF_Update）
        config_.StateJacobian = [](const Matrix<float> &state, const Matrix<float> &control,
                                  float deltaTime)
        {
            float gx = control[0][0];
            float gy = control[1][0];
            float gz = control[2][0];
        
            gx -= state[4][0];
            gy -= state[5][0];

            const float hx = 0.5f * gx * deltaTime;
            const float hy = 0.5f * gy * deltaTime;
            const float hz = 0.5f * gz * deltaTime;
            const float halfDt = 0.5f * deltaTime;

            const float q0 = state[0][0];
            const float q1 = state[1][0];
            const float q2 = state[2][0];
            const float q3 = state[3][0];

            Matrix<float> F = Eye<float>(6);
            F[0][1] = -hx;
            F[0][2] = -hy;
            F[0][3] = -hz;
            F[1][0] = hx;
            F[1][2] = hz;
            F[1][3] = -hy;
            F[2][0] = hy;
            F[2][1] = -hz;
            F[2][3] = hx;
            F[3][0] = hz;
            F[3][1] = hy;
            F[3][2] = -hx;

            // 四元数对零偏的偏导（F 右上 4x2 分块）
            F[0][4] = q1 * halfDt;
            F[0][5] = q2 * halfDt;
            F[1][4] = -q0 * halfDt;
            F[1][5] = q3 * halfDt;
            F[2][4] = -q3 * halfDt;
            F[2][5] = -q0 * halfDt;
            F[3][4] = q2 * halfDt;
            F[3][5] = -q1 * halfDt;

            return F;
        };

        // 观测雅可比 H（对应 EKF_jacH_Update），后两列（零偏）为 0
        config_.MeasurementJacobian = [](const Matrix<float> &state)
        {
            const float dq0 = 2.0f * state[0][0];
            const float dq1 = 2.0f * state[1][0];
            const float dq2 = 2.0f * state[2][0];
            const float dq3 = 2.0f * state[3][0];

            Matrix<float> H = Zeros<float>(3, 6);
            H[0][0] = -dq2;
            H[0][1] = dq3;
            H[0][2] = -dq0;
            H[0][3] = dq1;
            H[1][0] = dq1;
            H[1][1] = dq0;
            H[1][2] = dq3;
            H[1][3] = dq2;
            H[2][0] = dq0;
            H[2][1] = -dq1;
            H[2][2] = -dq2;
            H[2][3] = -dq3;
            return H;
        };

        // 状态校正：零偏修正限幅 + 四元数单位化（对应 EKF_Xhat_Update）
        config_.ApplyCorrection = [](const Matrix<float> &state, const Matrix<float> &correction)
        {
            // 原实现为 1e-2 * dt；当前更新周期为 1 ms。
            constexpr float kMaxBiasStep = 1.0e-5f;

            Matrix<float> fix = correction;
            for (int i = 4; i < 6; ++i)
            {
                if (fix[i][0] > kMaxBiasStep)
                {
                    fix[i][0] = kMaxBiasStep;
                }
                else if (fix[i][0] < -kMaxBiasStep)
                {
                    fix[i][0] = -kMaxBiasStep;
                }
            }

            Matrix<float> next = state + fix;
            NormalizeQuaternion(next);
            return next;
        };

        ekf_.emplace(config_);
    }

    //z轴的零偏要先处理
    /// @brief 每 1 ms 更新一次 EKF；acc 单位 m/s^2，gyo 单位 rad/s,更新周期dt
    [[nodiscard]] imu_data_t Update(const float acc[3], const float gyo[3], float dt)
    {
        imu_data_t output{};
        auto gyro = Zeros<float>(3, 1);
        auto accel = Zeros<float>(3, 1);
        for (int i = 0; i < 3; ++i)
        {
            output.raw_acc[i] = acc[i];
            output.raw_gyr[i] = gyo[i];
            gyro[i][0] = gyo[i];
            accel[i][0] = acc[i];
        }

        ekf_->Predict(dt, gyro);

        const float accNorm = std::sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
        if (std::isfinite(accNorm) && accNorm > 1e-6f)
        {
            for (int i = 0; i < 3; ++i)
            {
                accel[i][0] /= accNorm;
            }
            static_cast<void>(ekf_->Update(accel));
        }

        const auto& state = ekf_->State();
        const float q0 = state[0][0];
        const float q1 = state[1][0];
        const float q2 = state[2][0];
        const float q3 = state[3][0];
        const float rad2Deg = 57.295779513f;

        const float yaw = std::atan2(
            2.0f * (q0 * q3 + q1 * q2),
            2.0f * (q0 * q0 + q1 * q1) - 1.0f);
        const float roll = std::atan2(
            2.0f * (q0 * q1 + q2 * q3),
            2.0f * (q0 * q0 + q3 * q3) - 1.0f);
        const float pitchSin = std::clamp(-2.0f * (q1 * q3 - q0 * q2), -1.0f, 1.0f);
        const float pitch = std::asin(pitchSin);

        output.yaw = yaw * rad2Deg;
        output.pitch = pitch * rad2Deg;
        output.roll = roll * rad2Deg;

        const float correctedGyr[3] = 
        {
            gyo[0] - state[4][0],
            gyo[1] - state[5][0],
            gyo[2]
        };
        
        static float filteredGyr[3] = {0, 0, 0};
        for (int i = 0; i < 3; ++i)
        {
            filteredGyr[i] = LowPassFilter(filteredGyr[i], correctedGyr[i], 0.84f);
        }

        // 直接用机体角速度
        output.rate_roll = filteredGyr[0] * rad2Deg; // 机体 x 轴角速度 
        output.rate_pitch = filteredGyr[1] * rad2Deg; // 机体 y 轴角速度 
        output.rate_yaw = filteredGyr[2] * rad2Deg;   // 机体 z 轴角速度 

        const float g = 9.80665f;
        const float gbx = 2.0f * (q1 * q3 - q0 * q2) * g;
        const float gby = 2.0f * (q2 * q3 + q0 * q1) * g;
        const float gbz = (1.0f - 2.0f * (q1 * q1 + q2 * q2)) * g;

        output.body_acc[0] = acc[0] - gbx;
        output.body_acc[1] = acc[1] - gby;
        output.body_acc[2] = acc[2] - gbz;
        return output;
    }

private:
    /// @brief 四元数单位化
    static void NormalizeQuaternion(Matrix<float>& state)
    {
        const float norm = std::sqrt(state[0][0] * state[0][0] + state[1][0] * state[1][0] +
                                      state[2][0] * state[2][0] + state[3][0] * state[3][0]);
        if (norm > 1e-6f && std::isfinite(norm))
        {
            state[0][0] /= norm;
            state[1][0] /= norm;
            state[2][0] /= norm;
            state[3][0] /= norm;
        }
        else
        {
            state[0][0] = 1.0f;
            state[1][0] = 0.0f;
            state[2][0] = 0.0f;
            state[3][0] = 0.0f;
        }
    }


    /// @brief EKF 配置实例，必须在 ekf_ 之前声明（先构造后使用）。
    std::optional<Ekf> ekf_;
};

} // namespace breeze::imu