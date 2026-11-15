#pragma once

#include <cmath>
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

// Coordinated-turn (CT) EKF state order: [x, y, vx, vy, omega].
// q_acceleration and q_turn_rate are continuous-time noise spectral densities.
// The Q below is a practical small-step approximation: independent Cartesian
// acceleration noise plus a random walk in turn rate.
inline cv::Mat ct_process_noise_2d(float dt,
                                   float acceleration_spectral_density,
                                   float turn_rate_spectral_density)
{
    CV_Assert(dt >= 0.0f);
    CV_Assert(acceleration_spectral_density >= 0.0f);
    CV_Assert(turn_rate_spectral_density >= 0.0f);

    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    cv::Mat q = cv::Mat::zeros(5, 5, CV_32F);

    // Continuous white acceleration blocks at indices (x,vx) and (y,vy).
    const float qPosition = acceleration_spectral_density * dt3 / 3.0f;
    const float qPositionVelocity = acceleration_spectral_density * dt2 / 2.0f;
    const float qVelocity = acceleration_spectral_density * dt;
    q.at<float>(0, 0) = qPosition;
    q.at<float>(0, 2) = qPositionVelocity;
    q.at<float>(2, 0) = qPositionVelocity;
    q.at<float>(2, 2) = qVelocity;
    q.at<float>(1, 1) = qPosition;
    q.at<float>(1, 3) = qPositionVelocity;
    q.at<float>(3, 1) = qPositionVelocity;
    q.at<float>(3, 3) = qVelocity;

    // omega follows a continuous-time random walk: d(omega)=sqrt(qw)dW.
    q.at<float>(4, 4) = turn_rate_spectral_density * dt;
    return q;
}

// State-dependent CT process noise for variable-speed turns. Tangential
// acceleration changes speed; normal acceleration changes the velocity
// direction. The local tangent/normal frame is held fixed over this time step.
inline cv::Mat ct_process_noise_tangent_normal_2d(
    float dt,
    float vx,
    float vy,
    float tangential_acceleration_spectral_density,
    float normal_acceleration_spectral_density,
    float turn_rate_spectral_density)
{
    CV_Assert(dt >= 0.0f);
    CV_Assert(tangential_acceleration_spectral_density >= 0.0f);
    CV_Assert(normal_acceleration_spectral_density >= 0.0f);
    CV_Assert(turn_rate_spectral_density >= 0.0f);

    const float speed = std::sqrt(vx * vx + vy * vy);
    float qxx, qxy, qyy;
    if (speed < 1e-5f) {
        // With no reliable velocity direction, use the isotropic mean intensity.
        const float q = 0.5f * (tangential_acceleration_spectral_density +
                                normal_acceleration_spectral_density);
        qxx = q;
        qxy = 0.0f;
        qyy = q;
    } else {
        const float tx = vx / speed;
        const float ty = vy / speed;
        const float nx = -ty;
        const float ny = tx;
        qxx = tangential_acceleration_spectral_density * tx * tx +
              normal_acceleration_spectral_density * nx * nx;
        qxy = tangential_acceleration_spectral_density * tx * ty +
              normal_acceleration_spectral_density * nx * ny;
        qyy = tangential_acceleration_spectral_density * ty * ty +
              normal_acceleration_spectral_density * ny * ny;
    }

    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    cv::Mat q = cv::Mat::zeros(5, 5, CV_32F);
    const float pp = dt3 / 3.0f;
    const float pv = dt2 / 2.0f;
    const float vv = dt;

    // Q = [[dt^3/3 Q_a, dt^2/2 Q_a], [dt^2/2 Q_a, dt Q_a]]
    // in position/velocity blocks, with state order [x,y,vx,vy,omega].
    q.at<float>(0, 0) = pp * qxx;
    q.at<float>(0, 1) = pp * qxy;
    q.at<float>(1, 0) = pp * qxy;
    q.at<float>(1, 1) = pp * qyy;
    q.at<float>(0, 2) = pv * qxx;
    q.at<float>(0, 3) = pv * qxy;
    q.at<float>(1, 2) = pv * qxy;
    q.at<float>(1, 3) = pv * qyy;
    q.at<float>(2, 0) = pv * qxx;
    q.at<float>(2, 1) = pv * qxy;
    q.at<float>(3, 0) = pv * qxy;
    q.at<float>(3, 1) = pv * qyy;
    q.at<float>(2, 2) = vv * qxx;
    q.at<float>(2, 3) = vv * qxy;
    q.at<float>(3, 2) = vv * qxy;
    q.at<float>(3, 3) = vv * qyy;
    q.at<float>(4, 4) = turn_rate_spectral_density * dt;
    return q;
}

inline void init_ct_ekf_2d(cv::KalmanFilter& kf,
                          float dt,
                          float acceleration_spectral_density,
                          float turn_rate_spectral_density,
                          float measurement_noise_variance)
{
    CV_Assert(dt > 0.0f);
    CV_Assert(measurement_noise_variance >= 0.0f);

    kf.init(5, 2, 0, CV_32F);
    kf.transitionMatrix = cv::Mat::eye(5, 5, CV_32F);
    kf.measurementMatrix = (cv::Mat_<float>(2, 5) <<
        1, 0, 0, 0, 0,
        0, 1, 0, 0, 0);
    kf.processNoiseCov = ct_process_noise_2d(
        dt, acceleration_spectral_density, turn_rate_spectral_density);
    kf.measurementNoiseCov =
        measurement_noise_variance * cv::Mat::eye(2, 2, CV_32F);
    kf.statePost = cv::Mat::zeros(5, 1, CV_32F);
    kf.errorCovPost = cv::Mat::eye(5, 5, CV_32F);
}

inline void init_ct_variable_speed_ekf_2d(
    cv::KalmanFilter& kf,
    float dt,
    float tangential_acceleration_spectral_density,
    float normal_acceleration_spectral_density,
    float turn_rate_spectral_density,
    float measurement_noise_variance)
{
    CV_Assert(tangential_acceleration_spectral_density >= 0.0f);
    CV_Assert(normal_acceleration_spectral_density >= 0.0f);
    const float mean_acceleration_spectral_density = 0.5f *
        (tangential_acceleration_spectral_density +
         normal_acceleration_spectral_density);
    init_ct_ekf_2d(kf, dt, mean_acceleration_spectral_density,
                   turn_rate_spectral_density, measurement_noise_variance);
}

// Nonlinear coordinated-turn state prediction and EKF covariance prediction.
// Call kf.correct(positionMeasurement) after this when a detection is available.
// OpenCV's predict() is linear; this helper computes f(x) and its Jacobian F
// explicitly, then fills statePre/errorCovPre for the standard correct() call.
inline const cv::Mat& predict_ct_ekf_2d_with_noise(
    cv::KalmanFilter& kf, float dt, const cv::Mat& process_noise)
{
    CV_Assert(kf.statePost.rows == 5 && kf.statePost.cols == 1);
    CV_Assert(dt >= 0.0f);
    CV_Assert(process_noise.rows == 5 && process_noise.cols == 5 &&
              process_noise.type() == CV_32F);

    const float x = kf.statePost.at<float>(0);
    const float y = kf.statePost.at<float>(1);
    const float vx = kf.statePost.at<float>(2);
    const float vy = kf.statePost.at<float>(3);
    const float omega = kf.statePost.at<float>(4);
    const float angle = omega * dt;
    const float s = std::sin(angle);
    const float c = std::cos(angle);

    float a, b, da, db;
    if (std::abs(omega) < 1e-4f) {
        const float omega2 = omega * omega;
        const float dt2 = dt * dt;
        const float dt3 = dt2 * dt;
        const float dt4 = dt3 * dt;
        const float dt5 = dt4 * dt;
        const float dt6 = dt5 * dt;
        a = dt - omega2 * dt3 / 6.0f;
        b = omega * dt2 / 2.0f - omega * omega2 * dt4 / 24.0f;
        da = -omega * dt3 / 3.0f + omega * omega2 * dt5 / 30.0f;
        db = dt2 / 2.0f - omega2 * dt4 / 8.0f + omega2 * omega2 * dt6 / 144.0f;
    } else {
        a = s / omega;
        b = (1.0f - c) / omega;
        da = (omega * dt * c - s) / (omega * omega);
        db = (omega * dt * s - (1.0f - c)) / (omega * omega);
    }

    kf.statePre = (cv::Mat_<float>(5, 1) <<
        x + a * vx - b * vy,
        y + b * vx + a * vy,
        c * vx - s * vy,
        s * vx + c * vy,
        omega);

    cv::Mat f = (cv::Mat_<float>(5, 5) <<
        1, 0, a, -b, da * vx - db * vy,
        0, 1, b,  a, db * vx + da * vy,
        0, 0, c, -s, -dt * (s * vx + c * vy),
        0, 0, s,  c,  dt * (c * vx - s * vy),
        0, 0, 0,  0, 1);

    kf.transitionMatrix = f;
    kf.processNoiseCov = process_noise;
    kf.errorCovPre = f * kf.errorCovPost * f.t() + kf.processNoiseCov;
    return kf.statePre;
}

inline const cv::Mat& predict_ct_ekf_2d(cv::KalmanFilter& kf,
                                       float dt,
                                       float acceleration_spectral_density,
                                       float turn_rate_spectral_density)
{
    const cv::Mat q = ct_process_noise_2d(
        dt, acceleration_spectral_density, turn_rate_spectral_density);
    return predict_ct_ekf_2d_with_noise(kf, dt, q);
}

inline const cv::Mat& predict_ct_variable_speed_ekf_2d(
    cv::KalmanFilter& kf,
    float dt,
    float tangential_acceleration_spectral_density,
    float normal_acceleration_spectral_density,
    float turn_rate_spectral_density)
{
    CV_Assert(kf.statePost.rows == 5 && kf.statePost.cols == 1);
    const float vx = kf.statePost.at<float>(2, 0);
    const float vy = kf.statePost.at<float>(3, 0);
    const cv::Mat q = ct_process_noise_tangent_normal_2d(
        dt, vx, vy, tangential_acceleration_spectral_density,
        normal_acceleration_spectral_density, turn_rate_spectral_density);
    return predict_ct_ekf_2d_with_noise(kf, dt, q);
}

// Use when a detection is missing after predict_ct_ekf_2d(). OpenCV's normal
// predict() promotes the prediction to the posterior automatically; the EKF
// helper leaves it in the Pre fields so correct() can consume it if available.
inline void commit_ct_ekf_prediction(cv::KalmanFilter& kf)
{
    kf.statePre.copyTo(kf.statePost);
    kf.errorCovPre.copyTo(kf.errorCovPost);
}

} // namespace kalman_models
