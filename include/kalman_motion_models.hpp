#pragma once

#include <opencv2/opencv.hpp>

// Optional Kalman motion/process-noise models for the Task 07 exercises.
// State orders:
//   random walk:            [x, y]
//   constant velocity:      [x, y, vx, vy]
//   constant acceleration:  [x, y, vx, vy, ax, ay]
namespace kalman_models {

inline void init_random_walk_2d(cv::KalmanFilter& kf,
                                float position_noise_variance,
                                float measurement_noise_variance)
{
    kf.init(2, 2, 0, CV_32F);
    kf.transitionMatrix = cv::Mat::eye(2, 2, CV_32F);
    kf.measurementMatrix = cv::Mat::eye(2, 2, CV_32F);
    kf.processNoiseCov = position_noise_variance * cv::Mat::eye(2, 2, CV_32F);
    kf.measurementNoiseCov = measurement_noise_variance * cv::Mat::eye(2, 2, CV_32F);
    kf.statePost = cv::Mat::zeros(2, 1, CV_32F);
    kf.errorCovPost = cv::Mat::eye(2, 2, CV_32F);
}

// Discrete random acceleration per frame. This matches the Q matrix used by
// src/task_07/main.cpp when dt=1 and q is its per-frame acceleration variance.
inline void init_cv_discrete_white_accel(cv::KalmanFilter& kf,
                                         float dt,
                                         float acceleration_variance,
                                         float measurement_noise_variance)
{
    kf.init(4, 2, 0, CV_32F);
    kf.transitionMatrix = (cv::Mat_<float>(4, 4) <<
        1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1,  0,
        0, 0, 0,  1);

    kf.measurementMatrix = (cv::Mat_<float>(2, 4) <<
        1, 0, 0, 0,
        0, 1, 0, 0);

    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    const float dt4 = dt2 * dt2;
    kf.processNoiseCov = acceleration_variance * (cv::Mat_<float>(4, 4) <<
        dt4 / 4, 0,        dt3 / 2, 0,
        0,        dt4 / 4, 0,        dt3 / 2,
        dt3 / 2, 0,        dt2,      0,
        0,        dt3 / 2, 0,        dt2);

    kf.measurementNoiseCov = measurement_noise_variance * cv::Mat::eye(2, 2, CV_32F);
    kf.statePost = cv::Mat::zeros(4, 1, CV_32F);
    kf.errorCovPost = cv::Mat::eye(4, 4, CV_32F);
}

// Continuous-time white acceleration noise integrated over one frame.
// This keeps the same constant-velocity state transition as the model above,
// but uses a different process-noise assumption and therefore a different Q.
inline void init_cv_continuous_white_accel(cv::KalmanFilter& kf,
                                           float dt,
                                           float acceleration_spectral_density,
                                           float measurement_noise_variance)
{
    kf.init(4, 2, 0, CV_32F);
    kf.transitionMatrix = (cv::Mat_<float>(4, 4) <<
        1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1,  0,
        0, 0, 0,  1);

    kf.measurementMatrix = (cv::Mat_<float>(2, 4) <<
        1, 0, 0, 0,
        0, 1, 0, 0);

    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    kf.processNoiseCov = acceleration_spectral_density * (cv::Mat_<float>(4, 4) <<
        dt3 / 3, 0,        dt2 / 2, 0,
        0,        dt3 / 3, 0,        dt2 / 2,
        dt2 / 2, 0,        dt,       0,
        0,        dt2 / 2, 0,        dt);

    kf.measurementNoiseCov = measurement_noise_variance * cv::Mat::eye(2, 2, CV_32F);
    kf.statePost = cv::Mat::zeros(4, 1, CV_32F);
    kf.errorCovPost = cv::Mat::eye(4, 4, CV_32F);
}

// Constant-acceleration state with continuous-time white jerk process noise.
// The independent x/y 1D Q blocks are placed in state indices [x,vx,ax]
// and [y,vy,ay], respectively.
inline void init_ca_white_jerk_2d(cv::KalmanFilter& kf,
                                  float dt,
                                  float jerk_spectral_density,
                                  float measurement_noise_variance)
{
    kf.init(6, 2, 0, CV_32F);
    const float dt2 = dt * dt;
    kf.transitionMatrix = (cv::Mat_<float>(6, 6) <<
        1, 0, dt, 0, dt2 / 2, 0,
        0, 1, 0, dt, 0, dt2 / 2,
        0, 0, 1, 0, dt, 0,
        0, 0, 0, 1, 0, dt,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1);

    kf.measurementMatrix = (cv::Mat_<float>(2, 6) <<
        1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0);

    const float dt3 = dt2 * dt;
    const float dt4 = dt3 * dt;
    const float dt5 = dt4 * dt;
    const float qBlock[3][3] = {
        {dt5 / 20, dt4 / 8,  dt3 / 6},
        {dt4 / 8,  dt3 / 3,  dt2 / 2},
        {dt3 / 6,  dt2 / 2,  dt}
    };
    const int xIndices[3] = {0, 2, 4};
    const int yIndices[3] = {1, 3, 5};
    kf.processNoiseCov = cv::Mat::zeros(6, 6, CV_32F);
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            const float value = jerk_spectral_density * qBlock[row][col];
            kf.processNoiseCov.at<float>(xIndices[row], xIndices[col]) = value;
            kf.processNoiseCov.at<float>(yIndices[row], yIndices[col]) = value;
        }
    }

    kf.measurementNoiseCov = measurement_noise_variance * cv::Mat::eye(2, 2, CV_32F);
    kf.statePost = cv::Mat::zeros(6, 1, CV_32F);
    kf.errorCovPost = cv::Mat::eye(6, 6, CV_32F);
}

} // namespace kalman_models
