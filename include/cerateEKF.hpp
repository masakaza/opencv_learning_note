
#pragma once

#include <opencv2/opencv.hpp>

#include <cmath>
#include <stdexcept>

/*
参数	            增大	      减小
sigma_position	更相信运动模型	更相信 YOLO
q_accel	更容易适应线速度变化	更依赖原有运动预测
q_alpha	更容易适应角速度变化	更依赖原有转弯预测

总体和kf参数思路差不多，完全依赖模型则会造成严重的祗候，如果完全依赖预测则会造成预测框的抖动严重
*/

//5维ekf x,y,vx,vy,角速度
class CoordinatedTurnEKF5D
{
public:
    // 五维状态：[x, y, vx, vy, omega]
    using State = cv::Vec<double, 5>;

    // 5x5矩阵
    using Matrix5 = cv::Matx<double, 5, 5>;

    // 二维观测：[x, y]
    using Measurement = cv::Vec2d;

    // 2x2矩阵
    using Matrix2 = cv::Matx22d;

    // 2x5观测矩阵
    using Matrix25 = cv::Matx<double, 2, 5>;

    // 5x2卡尔曼增益
    using Matrix52 = cv::Matx<double, 5, 2>;

private:
    // 状态向量
    State state_;

    // 状态协方差
    Matrix5 P_;

    // 测量噪声协方差
    Matrix2 R_;

    // 噪声参数
    double sigma_position_;
    double q_accel_;
    double q_alpha_;

    // 初始化不确定性
    double init_velocity_var_;
    double init_omega_var_;

    bool initialized_ = false;

public:
    // ========================================
    // 构造函数
    // ========================================

    CoordinatedTurnEKF5D(
        double sigma_position = 5.0,
        double q_accel = 0.01,
        double q_alpha = 0.00001,
        double init_velocity_var = 100.0,
        double init_omega_var = 0.25)
        : sigma_position_(sigma_position),
          q_accel_(q_accel),
          q_alpha_(q_alpha),
          init_velocity_var_(init_velocity_var),
          init_omega_var_(init_omega_var)
    {
        if (sigma_position <= 0 ||
            q_accel < 0 ||
            q_alpha < 0 ||
            init_velocity_var <= 0 ||
            init_omega_var <= 0)
        {
            throw std::invalid_argument(
                "Invalid EKF parameters");
        }

        state_ = State::all(0.0);

        P_ = Matrix5::zeros();

        // R矩阵自动生成
        R_ = Matrix2::zeros();

        R_(0, 0) = sigma_position_ * sigma_position_;
        R_(1, 1) = sigma_position_ * sigma_position_;
    }

    // ========================================
    // 初始化
    // 使用连续三次检测中心
    // ========================================

    void initialize(
        const cv::Point2f &p0,
        const cv::Point2f &p1,
        const cv::Point2f &p2,
        double dt01 = 1.0,
        double dt12 = 1.0)
    {
        if (dt01 <= 0 || dt12 <= 0)
            throw std::invalid_argument("Invalid dt");

        // 第一段速度
        double vx01 = (p1.x - p0.x) / dt01;
        double vy01 = (p1.y - p0.y) / dt01;

        // 第二段速度
        double vx12 = (p2.x - p1.x) / dt12;
        double vy12 = (p2.y - p1.y) / dt12;

        // 位置
        state_[0] = p2.x;
        state_[1] = p2.y;

        // 速度
        state_[2] = vx12;
        state_[3] = vy12;

        // 计算角速度
        double cross =
            vx01 * vy12 - vy01 * vx12;

        double dot =
            vx01 * vx12 + vy01 * vy12;

        double speed01 = std::hypot(vx01, vy01);
        double speed12 = std::hypot(vx12, vy12);

        double dt_mid = (dt01 + dt12) / 2.0;

        if (speed01 > 1e-6 && speed12 > 1e-6)
        {
            double dtheta = std::atan2(cross, dot);

            state_[4] = dtheta / dt_mid;
        }
        else
        {
            state_[4] = 0.0;
        }

        // 初始化P矩阵
        P_ = Matrix5::zeros();

        P_(0, 0) = R_(0, 0);
        P_(1, 1) = R_(1, 1);

        P_(2, 2) = init_velocity_var_;
        P_(3, 3) = init_velocity_var_;

        P_(4, 4) = init_omega_var_;

        initialized_ = true;
    }

    // ========================================
    // EKF预测
    // dt单位：Frame
    // ========================================

    void predict(double dt = 1.0)
    {
        if (!initialized_)
            throw std::runtime_error(
                "EKF not initialized");

        if (dt <= 0)
            throw std::invalid_argument("Invalid dt");

        double x = state_[0];
        double y = state_[1];

        double vx = state_[2];
        double vy = state_[3];

        double omega = state_[4];

        double q = omega * dt;

        double s = std::sin(q);
        double c = std::cos(q);

        double A, B, Aw, Bw;

        // 处理角速度接近0的情况
        if (std::abs(q) < 1e-3)
        {
            double q2 = q * q;
            double q3 = q2 * q;
            double q4 = q2 * q2;
            double q5 = q4 * q;

            A = dt * (1.0 - q2 / 6.0 + q4 / 120.0);

            B = dt * (q / 2.0 - q3 / 24.0 + q5 / 720.0);

            Aw = dt * dt * (-q / 3.0 + q3 / 30.0);

            Bw = dt * dt * (0.5 - q2 / 8.0 + q4 / 144.0);
        }
        else
        {
            A = s / omega;

            B = (1.0 - c) / omega;

            Aw = (q * c - s) / (omega * omega);

            Bw = (q * s - (1.0 - c)) / (omega * omega);
        }

        // 1. 非线性状态预测

        State predicted;

        predicted[0] = x + A * vx - B * vy;

        predicted[1] = y + B * vx + A * vy;

        predicted[2] = c * vx - s * vy;

        predicted[3] = s * vx + c * vy;

        predicted[4] = omega;

        // 2. 雅可比矩阵F

        Matrix5 F = Matrix5::zeros();

        F(0, 0) = 1.0;
        F(0, 2) = A;
        F(0, 3) = -B;
        F(0, 4) = Aw * vx - Bw * vy;

        F(1, 1) = 1.0;
        F(1, 2) = B;
        F(1, 3) = A;
        F(1, 4) = Bw * vx + Aw * vy;

        F(2, 2) = c;
        F(2, 3) = -s;
        F(2, 4) = -dt * (s * vx + c * vy);

        F(3, 2) = s;
        F(3, 3) = c;
        F(3, 4) = dt * (c * vx - s * vy);

        F(4, 4) = 1.0;

        // 3. 生成过程噪声矩阵Q

        Matrix5 Q = Matrix5::zeros();

        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double dt4 = dt2 * dt2;

        // X方向加速度噪声
        Q(0, 0) = q_accel_ * dt4 / 4.0;
        Q(0, 2) = q_accel_ * dt3 / 2.0;
        Q(2, 0) = q_accel_ * dt3 / 2.0;
        Q(2, 2) = q_accel_ * dt2;

        // Y方向加速度噪声
        Q(1, 1) = q_accel_ * dt4 / 4.0;
        Q(1, 3) = q_accel_ * dt3 / 2.0;
        Q(3, 1) = q_accel_ * dt3 / 2.0;
        Q(3, 3) = q_accel_ * dt2;

        // 角加速度噪声对角速度的影响
        Q(4, 4) = q_alpha_ * dt;

        // 4. 更新状态

        state_ = predicted;

        // 5. 更新状态协方差

        P_ = F * P_ * F.t() + Q;

        P_ = (P_ + P_.t()) * 0.5;
    }

    // ========================================
    // EKF修正
    // ========================================

    void correct(const Measurement &z)
    {
        if (!initialized_)
            throw std::runtime_error(
                "EKF not initialized");

        // 1. 观测矩阵H
        Matrix25 H = Matrix25::zeros();

        H(0, 0) = 1.0;
        H(1, 1) = 1.0;

        // 2. 计算观测残差
        Measurement innovation = z - H * state_;

        // 3. 残差协方差
        Matrix2 S = H * P_ * H.t() + R_;

        // 4. 卡尔曼增益
        Matrix52 K =
            P_ * H.t() * S.inv(cv::DECOMP_SVD);

        // 5. 修正状态
        state_ = state_ + K * innovation;

        // 6. Joseph形式更新P矩阵
        Matrix5 I = Matrix5::eye();

        Matrix5 M = I - K * H;

        P_ = M * P_ * M.t() + K * R_ * K.t();

        P_ = (P_ + P_.t()) * 0.5;
    }

    // ========================================
    // 获取状态
    // ========================================

    State getState() const
    {
        return state_;
    }

    // 获取中心坐标
    cv::Point2f getPosition() const
    {
        return cv::Point2f(
            static_cast<float>(state_[0]),
            static_cast<float>(state_[1]));
    }

    // 获取线速度大小
    double getSpeed() const
    {
        return std::hypot(state_[2], state_[3]);
    }

    // 获取角速度
    double getOmega() const
    {
        return state_[4];
    }

    // 获取是否初始化
    bool isInitialized() const
    {
        return initialized_;
    }

    // ========================================
    // 获取矩形框
    // ========================================

    cv::Rect getRect(
        double width,
        double height) const
    {
        if (!initialized_ || width <= 0 || height <= 0)
            return cv::Rect();

        int w = cvRound(width);
        int h = cvRound(height);

        int x = cvRound(state_[0] - width / 2.0);
        int y = cvRound(state_[1] - height / 2.0);

        return cv::Rect(x, y, w, h);
    }

    // ========================================
    // 绘制矩形框
    // ========================================

    void drawRect(
        cv::Mat &image,
        double width,
        double height,
        const cv::Scalar &color,
        int thickness = 2) const
    {
        if (!initialized_ || image.empty())
            return;

        cv::Rect rect = getRect(width, height);

        if (rect.empty())
            return;

        // 限制绘制区域
        cv::Rect image_rect(
            0, 0,
            image.cols,
            image.rows);

        cv::Rect clipped = rect & image_rect;

        if (clipped.empty())
            return;

        cv::rectangle(
            image,
            clipped,
            color,
            thickness);
    }

    // ========================================
    // 重置滤波器
    // ========================================

    void reset()
    {
        state_ = State::all(0.0);

        P_ = Matrix5::zeros();

        initialized_ = false;
    }
};


//除了像5维的ekf传入控制参数外，6维ekf还可以控制另外三个参数，具体说明见 构造函数
class CoordinatedTurnEKF6D
{
public:

    // 六维状态：[x, y, vx, vy, omega, alpha]
    using State = cv::Vec<double, 6>;

    // 6x6矩阵
    using Matrix6 = cv::Matx<double, 6, 6>;

    // 二维观测：[x, y]
    using Measurement = cv::Vec2d;

    // 2x2矩阵
    using Matrix2 = cv::Matx22d;

    // 2x6观测矩阵
    using Matrix26 = cv::Matx<double, 2, 6>;

    // 6x2卡尔曼增益
    using Matrix62 = cv::Matx<double, 6, 2>;


private:

    // 状态向量
    State state_;

    // 状态协方差矩阵
    Matrix6 P_;

    // 测量噪声协方差矩阵
    Matrix2 R_;

    // 噪声参数
    double sigma_position_;
    double q_accel_;
    double q_alpha_;

    // 初始化不确定性
    double init_velocity_var_;
    double init_omega_var_;
    double init_alpha_var_;

    bool initialized_ = false;


    // ========================================
    // 非线性状态转移函数
    // ========================================

    State transition(
        const State& state,
        double dt
    ) const
    {
        double x = state[0];
        double y = state[1];

        double vx = state[2];
        double vy = state[3];

        double omega = state[4];
        double alpha = state[5];

        // 使用区间平均角速度近似积分
        double omega_mid = omega + 0.5 * alpha * dt;

        double q = omega_mid * dt;

        double s = std::sin(q);
        double c = std::cos(q);

        double A, B;

        // 处理角速度接近0的情况
        if (std::abs(q) < 1e-3)
        {
            double q2 = q * q;
            double q3 = q2 * q;
            double q4 = q2 * q2;
            double q5 = q4 * q;

            A = dt * (
                1.0 - q2 / 6.0 + q4 / 120.0
            );

            B = dt * (
                q / 2.0 - q3 / 24.0 + q5 / 720.0
            );
        }
        else
        {
            A = s / omega_mid;

            B = (1.0 - c) / omega_mid;
        }

        State predicted;

        // 位置预测
        predicted[0] = x + A * vx - B * vy;

        predicted[1] = y + B * vx + A * vy;

        // 速度方向旋转
        predicted[2] = c * vx - s * vy;

        predicted[3] = s * vx + c * vy;

        // 角速度更新
        predicted[4] = omega + alpha * dt;

        // 角加速度暂时保持不变
        predicted[5] = alpha;

        return predicted;
    }


    // ========================================
    // 数值计算雅可比矩阵
    // 使用中心差分法
    // ========================================

    Matrix6 calculateJacobian(
        const State& state,
        double dt
    ) const
    {
        Matrix6 F = Matrix6::zeros();

        for (int j = 0; j < 6; j++)
        {
            State plus = state;
            State minus = state;

            double epsilon =
                1e-6 * std::max(1.0, std::abs(state[j]));

            plus[j] += epsilon;
            minus[j] -= epsilon;

            State f_plus = transition(plus, dt);
            State f_minus = transition(minus, dt);

            for (int i = 0; i < 6; i++)
            {
                F(i, j) =
                    (f_plus[i] - f_minus[i])
                    / (2.0 * epsilon);
            }
        }

        return F;
    }


    // ========================================
    // 生成过程噪声矩阵Q
    // ========================================

    Matrix6 calculateQ(double dt) const
    {
        Matrix6 Q = Matrix6::zeros();

        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double dt4 = dt2 * dt2;

        // X方向加速度噪声
        Q(0, 0) = q_accel_ * dt4 / 4.0;
        Q(0, 2) = q_accel_ * dt3 / 2.0;
        Q(2, 0) = q_accel_ * dt3 / 2.0;
        Q(2, 2) = q_accel_ * dt2;

        // Y方向加速度噪声
        Q(1, 1) = q_accel_ * dt4 / 4.0;
        Q(1, 3) = q_accel_ * dt3 / 2.0;
        Q(3, 1) = q_accel_ * dt3 / 2.0;
        Q(3, 3) = q_accel_ * dt2;

        // 角加速度扰动
        Q(4, 4) = q_alpha_ * dt4 / 4.0;
        Q(4, 5) = q_alpha_ * dt3 / 2.0;
        Q(5, 4) = q_alpha_ * dt3 / 2.0;
        Q(5, 5) = q_alpha_ * dt2;

        return Q;
    }


public:

    // ========================================
    // 构造函数
    // ========================================

    //6D模型中，初始方差增加都会降低对初始速度的信任
    CoordinatedTurnEKF6D(
        double sigma_position = 5.0,
        double q_accel = 0.01,
        double q_alpha = 0.00001,
        double init_velocity_var = 100.0,     //初始速度方差
        double init_omega_var = 0.35,        //初始角速度方差
        double init_alpha_var = 0.01        //初始角加速度方差
    )
        : sigma_position_(sigma_position),
          q_accel_(q_accel),
          q_alpha_(q_alpha),
          init_velocity_var_(init_velocity_var),
          init_omega_var_(init_omega_var),
          init_alpha_var_(init_alpha_var)
    {
        if (sigma_position <= 0 ||
            q_accel < 0 ||
            q_alpha < 0 ||
            init_velocity_var <= 0 ||
            init_omega_var <= 0 ||
            init_alpha_var <= 0)
        {
            throw std::invalid_argument(
                "Invalid EKF parameters"
            );
        }

        state_ = State::all(0.0);

        P_ = Matrix6::zeros();

        // 自动生成R矩阵
        R_ = Matrix2::zeros();

        R_(0, 0) =
            sigma_position_ * sigma_position_;

        R_(1, 1) =
            sigma_position_ * sigma_position_;
    }


    // ========================================
    // 初始化
    // 使用连续三次检测中心
    // ========================================

    void initialize(
        const cv::Point2f& p0,
        const cv::Point2f& p1,
        const cv::Point2f& p2,
        double dt01 = 1.0,
        double dt12 = 1.0
    )
    {
        if (dt01 <= 0 || dt12 <= 0)
            throw std::invalid_argument("Invalid dt");

        // 第一段速度
        double vx01 = (p1.x - p0.x) / dt01;
        double vy01 = (p1.y - p0.y) / dt01;

        // 第二段速度
        double vx12 = (p2.x - p1.x) / dt12;
        double vy12 = (p2.y - p1.y) / dt12;

        // 初始化位置
        state_[0] = p2.x;
        state_[1] = p2.y;

        // 初始化速度
        state_[2] = vx12;
        state_[3] = vy12;

        // 估计角速度
        double cross =
            vx01 * vy12 - vy01 * vx12;

        double dot =
            vx01 * vx12 + vy01 * vy12;

        double speed01 = std::hypot(vx01, vy01);
        double speed12 = std::hypot(vx12, vy12);

        double dt_mid = (dt01 + dt12) / 2.0;

        if (speed01 > 1e-6 && speed12 > 1e-6)
        {
            double dtheta = std::atan2(cross, dot);

            state_[4] = dtheta / dt_mid;
        }
        else
        {
            state_[4] = 0.0;
        }

        // 初始角加速度暂时设为0
        state_[5] = 0.0;

        // 初始化协方差
        P_ = Matrix6::zeros();

        P_(0, 0) = R_(0, 0);
        P_(1, 1) = R_(1, 1);

        P_(2, 2) = init_velocity_var_;
        P_(3, 3) = init_velocity_var_;

        P_(4, 4) = init_omega_var_;
        P_(5, 5) = init_alpha_var_;

        initialized_ = true;
    }


    // ========================================
    // EKF预测
    // 默认每次前进1帧
    // ========================================

    void predict(double dt = 1.0)
    {
        if (!initialized_)
            throw std::runtime_error(
                "EKF not initialized"
            );

        if (dt <= 0)
            throw std::invalid_argument("Invalid dt");

        // 1. 计算雅可比矩阵
        Matrix6 F = calculateJacobian(state_, dt);

        // 2. 非线性状态预测
        state_ = transition(state_, dt);

        // 3. 生成过程噪声矩阵
        Matrix6 Q = calculateQ(dt);

        // 4. 更新协方差
        P_ = F * P_ * F.t() + Q;

        // 保证矩阵对称
        P_ = (P_ + P_.t()) * 0.5;
    }


    // ========================================
    // EKF修正
    // ========================================

    void correct(const Measurement& z)
    {
        if (!initialized_)
            throw std::runtime_error(
                "EKF not initialized"
            );

        // 1. 观测矩阵H
        Matrix26 H = Matrix26::zeros();

        H(0, 0) = 1.0;
        H(1, 1) = 1.0;

        // 2. 计算观测残差
        Measurement innovation = z - H * state_;

        // 3. 残差协方差
        Matrix2 S = H * P_ * H.t() + R_;

        // 4. 卡尔曼增益
        Matrix62 K =
            P_ * H.t() * S.inv(cv::DECOMP_SVD);

        // 5. 修正状态
        state_ = state_ + K * innovation;

        // 6. Joseph形式更新P
        Matrix6 I = Matrix6::eye();

        Matrix6 M = I - K * H;

        P_ = M * P_ * M.t()
             + K * R_ * K.t();

        P_ = (P_ + P_.t()) * 0.5;
    }


    // ========================================
    // 获取完整状态
    // ========================================

    State getState() const
    {
        return state_;
    }


    // 获取位置
    cv::Point2f getPosition() const
    {
        return cv::Point2f(
            static_cast<float>(state_[0]),
            static_cast<float>(state_[1])
        );
    }


    // 获取线速度大小
    double getSpeed() const
    {
        return std::hypot(state_[2], state_[3]);
    }


    // 获取角速度
    double getOmega() const
    {
        return state_[4];
    }


    // 获取角加速度
    double getAlpha() const
    {
        return state_[5];
    }


    // 判断是否初始化
    bool isInitialized() const
    {
        return initialized_;
    }


    // ========================================
    // 获取矩形框
    // ========================================

    cv::Rect getRect(
        double width,
        double height
    ) const
    {
        if (!initialized_ ||
            width <= 0 ||
            height <= 0)
        {
            return cv::Rect();
        }

        int w = cvRound(width);
        int h = cvRound(height);

        int x = cvRound(state_[0] - width / 2.0);
        int y = cvRound(state_[1] - height / 2.0);

        return cv::Rect(x, y, w, h);
    }


    // ========================================
    // 绘制矩形框
    // ========================================

    void drawRect(
        cv::Mat& image,
        double width,
        double height,
        const cv::Scalar& color,
        int thickness = 2
    ) const
    {
        if (!initialized_ || image.empty())
            return;

        cv::Rect rect = getRect(width, height);

        if (rect.empty())
            return;

        cv::Rect image_rect(
            0, 0,
            image.cols,
            image.rows
        );

        cv::Rect clipped = rect & image_rect;

        if (clipped.empty())
            return;

        cv::rectangle(
            image,
            clipped,
            color,
            thickness
        );
    }


    // ========================================
    // 重置
    // ========================================

    void reset()
    {
        state_ = State::all(0.0);

        P_ = Matrix6::zeros();

        initialized_ = false;
    }

};

//6D通用ekf运动
class GeneralMotionEKF6D
{
public:

    using State = cv::Vec<double, 6>;
    using Measurement = cv::Vec2d;
    using Covariance = cv::Matx<double, 6, 6>;

private:

    // 状态：[x, y, v, theta, a, omega]
    State state_;

    // 状态协方差
    Covariance P_;

    bool initialized_ = false;

    // 测量噪声标准差
    double sigma_position_;

    // 切向加速度过程噪声方差
    double q_accel_;

    // 角加速度过程噪声方差
    double q_omega_;

    // 初始化协方差
    double init_speed_var_;
    double init_heading_var_;
    double init_accel_var_;
    double init_omega_var_;

    // 角度归一化到[-PI, PI]
    static double normalizeAngle(double angle)
    {
        return std::atan2(
            std::sin(angle),
            std::cos(angle)
        );
    }

public:

    // 构造函数
    GeneralMotionEKF6D(
        double sigma_position = 5.0,
        double q_accel = 0.01,
        double q_omega = 0.00001,
        double init_speed_var = 100.0,
        double init_heading_var = 0.25,
        double init_accel_var = 1.0,
        double init_omega_var = 0.25
    )
        : sigma_position_(sigma_position),
          q_accel_(std::max(0.0, q_accel)),
          q_omega_(std::max(0.0, q_omega)),
          init_speed_var_(init_speed_var),
          init_heading_var_(init_heading_var),
          init_accel_var_(init_accel_var),
          init_omega_var_(init_omega_var)
    {
        state_ = State::all(0.0);
        P_ = Covariance::zeros();
    }


    // =================================================
    // 初始化：使用三个连续位置
    // =================================================

    void initialize(
        const cv::Point2f& p0,
        const cv::Point2f& p1,
        const cv::Point2f& p2,
        double dt01 = 1.0,
        double dt12 = 1.0
    )
    {
        if (dt01 <= 0.0 || dt12 <= 0.0)
            return;

        // 第一次位移对应的速度
        double vx1 = (p1.x - p0.x) / dt01;
        double vy1 = (p1.y - p0.y) / dt01;

        // 第二次位移对应的速度
        double vx2 = (p2.x - p1.x) / dt12;
        double vy2 = (p2.y - p1.y) / dt12;

        // 速度大小
        double speed1 = std::hypot(vx1, vy1);
        double speed2 = std::hypot(vx2, vy2);

        // 航向角
        double heading1 =
            speed1 > 1e-6 ? std::atan2(vy1, vx1) : 0.0;

        double heading2 =
            speed2 > 1e-6
            ? std::atan2(vy2, vx2)
            : heading1;

        // 切向加速度
        double accel = (speed2 - speed1) / dt12;

        // 平均时间间隔
        double dt_avg = (dt01 + dt12) / 2.0;

        // 角速度
        double omega =
            normalizeAngle(heading2 - heading1) / dt_avg;

        // 初始化状态
        state_ = State(
            p2.x,
            p2.y,
            speed2,
            heading2,
            accel,
            omega
        );

        // 初始化协方差
        P_ = Covariance::zeros();

        P_(0, 0) = sigma_position_ * sigma_position_;
        P_(1, 1) = sigma_position_ * sigma_position_;

        P_(2, 2) = init_speed_var_;
        P_(3, 3) = init_heading_var_;
        P_(4, 4) = init_accel_var_;
        P_(5, 5) = init_omega_var_;

        initialized_ = true;
    }


    // =================================================
    // EKF预测
    // =================================================

    void predict(double dt = 1.0)
    {
        if (!initialized_ || dt <= 0.0)
            return;

        double x = state_[0];
        double y = state_[1];
        double v = state_[2];
        double theta = state_[3];
        double a = state_[4];
        double omega = state_[5];

        // 中间速度和中间航向角
        double vm = v + 0.5 * a * dt;

        double tm = theta + 0.5 * omega * dt;

        double c = std::cos(tm);
        double s = std::sin(tm);

        double dt2 = dt * dt;

        // -----------------------------------------
        // 1. 非线性状态预测
        // -----------------------------------------

        State next_state;

        next_state[0] = x + vm * c * dt;
        next_state[1] = y + vm * s * dt;

        next_state[2] = v + a * dt;

        next_state[3] =
            normalizeAngle(theta + omega * dt);

        next_state[4] = a;
        next_state[5] = omega;


        // -----------------------------------------
        // 2. 计算雅可比矩阵 F
        // -----------------------------------------

        Covariance F = Covariance::eye();

        F(0, 2) = c * dt;
        F(0, 3) = -vm * s * dt;
        F(0, 4) = 0.5 * c * dt2;
        F(0, 5) = -0.5 * vm * s * dt2;

        F(1, 2) = s * dt;
        F(1, 3) = vm * c * dt;
        F(1, 4) = 0.5 * s * dt2;
        F(1, 5) = 0.5 * vm * c * dt2;

        F(2, 4) = dt;
        F(3, 5) = dt;


        // -----------------------------------------
        // 3. 构造过程噪声矩阵 Q
        // -----------------------------------------

        // 切向加速度扰动的影响向量
        State G_accel(
            0.5 * c * dt2,
            0.5 * s * dt2,
            dt,
            0.0,
            1.0,
            0.0
        );

        // 角加速度扰动的影响向量
        State G_omega(
            -0.5 * vm * s * dt2,
             0.5 * vm * c * dt2,
             0.0,
             0.5 * dt2,
             0.0,
             dt
        );

        Covariance Q =
            q_accel_ * (G_accel * G_accel.t())
            +
            q_omega_ * (G_omega * G_omega.t());


        // -----------------------------------------
        // 4. 更新状态协方差
        // -----------------------------------------

        P_ = F * P_ * F.t() + Q;

        // 保持协方差矩阵对称
        P_ = (P_ + P_.t()) * 0.5;

        // 更新状态
        state_ = next_state;
    }


    // =================================================
    // EKF修正：测量值为二维中心坐标
    // =================================================

    void correct(const Measurement& z)
    {
        if (!initialized_)
            return;

        // -----------------------------------------
        // 1. 计算测量残差
        // -----------------------------------------

        Measurement innovation(
            z[0] - state_[0],
            z[1] - state_[1]
        );

        // -----------------------------------------
        // 2. 计算残差协方差 S
        // H矩阵只提取x、y
        // -----------------------------------------

        double R = sigma_position_ * sigma_position_;

        cv::Matx22d S(
            P_(0, 0) + R, P_(0, 1),
            P_(1, 0), P_(1, 1) + R
        );

        double det = S(0, 0) * S(1, 1)
                   - S(0, 1) * S(1, 0);

        if (std::abs(det) < 1e-12)
            return;

        // S的逆矩阵
        cv::Matx22d S_inv(
             S(1, 1) / det, -S(0, 1) / det,
            -S(1, 0) / det,  S(0, 0) / det
        );

        // -----------------------------------------
        // 3. 计算卡尔曼增益 K
        // K = P * H^T * S^-1
        // -----------------------------------------

        cv::Matx<double, 6, 2> PHt;

        for (int i = 0; i < 6; i++)
        {
            PHt(i, 0) = P_(i, 0);
            PHt(i, 1) = P_(i, 1);
        }

        cv::Matx<double, 6, 2> K = PHt * S_inv;


        // -----------------------------------------
        // 4. 更新状态
        // -----------------------------------------

        state_ += K * innovation;

        state_[3] = normalizeAngle(state_[3]);


        // -----------------------------------------
        // 5. Joseph形式更新协方差
        // -----------------------------------------

        Covariance A = Covariance::eye();

        for (int i = 0; i < 6; i++)
        {
            A(i, 0) -= K(i, 0);
            A(i, 1) -= K(i, 1);
        }

        cv::Matx22d R_mat(
            R, 0.0,
            0.0, R
        );

        P_ =
            A * P_ * A.t()
            +
            K * R_mat * K.t();

        P_ = (P_ + P_.t()) * 0.5;
    }


    // =================================================
    // 获取状态
    // =================================================

    cv::Point2f getPosition() const
    {
        return cv::Point2f(
            static_cast<float>(state_[0]),
            static_cast<float>(state_[1])
        );
    }

    cv::Point2f getVelocity() const
    {
        return cv::Point2f(
            static_cast<float>(state_[2] * std::cos(state_[3])),
            static_cast<float>(state_[2] * std::sin(state_[3]))
        );
    }

    double getSpeed() const
    {
        return state_[2];
    }

    double getHeading() const
    {
        return state_[3];
    }

    double getAcceleration() const
    {
        return state_[4];
    }

    double getOmega() const
    {
        return state_[5];
    }

    const State& getState() const
    {
        return state_;
    }

    const Covariance& getCovariance() const
    {
        return P_;
    }


    // =================================================
    // 初始化状态管理
    // =================================================

    bool isInitialized() const
    {
        return initialized_;
    }

    void reset()
    {
        state_ = State::all(0.0);
        P_ = Covariance::zeros();
        initialized_ = false;
    }
};

//kf的6维预测模型
class ConstantAccelerationKF6D
{
public:

    // 状态：[x, y, vx, vy, ax, ay]
    using State = cv::Vec<double, 6>;

    // 量测：[x, y]
    using Measurement = cv::Vec2d;

    using Covariance = cv::Matx<double, 6, 6>;

private:

    cv::KalmanFilter kf_;

    double q_;
    double r_;

    bool initialized_ = false;

    // 上一次predict是否还没有correct
    bool prediction_pending_ = false;

    // 用于兼容原来的const引用接口
    mutable State state_cache_;
    mutable Covariance covariance_cache_;


    // =================================================
    // 构造过程噪声矩阵Q
    // =================================================

    cv::Mat buildQ(double dt) const
    {
        cv::Mat Q = cv::Mat::zeros(6, 6, CV_64F);

        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double dt4 = dt3 * dt;
        double dt5 = dt4 * dt;

        // 单个方向的位置、速度、加速度噪声矩阵
        double block[3][3] =
        {
            {
                dt5 / 20.0,
                dt4 / 8.0,
                dt3 / 6.0
            },
            {
                dt4 / 8.0,
                dt3 / 3.0,
                dt2 / 2.0
            },
            {
                dt3 / 6.0,
                dt2 / 2.0,
                dt
            }
        };

        // X方向索引
        int x_index[3] = {0, 2, 4};

        // Y方向索引
        int y_index[3] = {1, 3, 5};

        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                Q.at<double>(
                    x_index[i],
                    x_index[j]
                ) = q_ * block[i][j];

                Q.at<double>(
                    y_index[i],
                    y_index[j]
                ) = q_ * block[i][j];
            }
        }

        return Q;
    }


    // =================================================
    // 初始化OpenCV KalmanFilter
    // =================================================

    void configure()
    {
        kf_.init(6, 2, 0, CV_64F);

        // 状态转移矩阵
        kf_.transitionMatrix =
            cv::Mat::eye(6, 6, CV_64F);

        // 测量矩阵H
        kf_.measurementMatrix =
            cv::Mat::zeros(2, 6, CV_64F);

        kf_.measurementMatrix.at<double>(0, 0) = 1.0;
        kf_.measurementMatrix.at<double>(1, 1) = 1.0;

        // 过程噪声
        kf_.processNoiseCov = buildQ(1.0);

        // 测量噪声R
        kf_.measurementNoiseCov =
            cv::Mat::eye(2, 2, CV_64F) * r_;

        // 初始化状态
        kf_.statePost =
            cv::Mat::zeros(6, 1, CV_64F);

        kf_.statePre = kf_.statePost.clone();

        // 初始化协方差
        kf_.errorCovPost =
            cv::Mat::eye(6, 6, CV_64F) * 100.0;

        kf_.errorCovPre = kf_.errorCovPost.clone();
    }


    // =================================================
    // 深拷贝KalmanFilter
    // 保证临时EKF/KF对象不会修改原对象
    // =================================================

    void copyFrom(const ConstantAccelerationKF6D& other)
    {
        q_ = other.q_;
        r_ = other.r_;

        initialized_ = other.initialized_;
        prediction_pending_ = other.prediction_pending_;

        kf_.init(6, 2, 0, CV_64F);

        kf_.statePre = other.kf_.statePre.clone();
        kf_.statePost = other.kf_.statePost.clone();

        kf_.transitionMatrix =
            other.kf_.transitionMatrix.clone();

        kf_.controlMatrix =
            other.kf_.controlMatrix.clone();

        kf_.measurementMatrix =
            other.kf_.measurementMatrix.clone();

        kf_.processNoiseCov =
            other.kf_.processNoiseCov.clone();

        kf_.measurementNoiseCov =
            other.kf_.measurementNoiseCov.clone();

        kf_.errorCovPre =
            other.kf_.errorCovPre.clone();

        kf_.errorCovPost =
            other.kf_.errorCovPost.clone();

        kf_.gain = other.kf_.gain.clone();

        kf_.temp1 = other.kf_.temp1.clone();
        kf_.temp2 = other.kf_.temp2.clone();
        kf_.temp3 = other.kf_.temp3.clone();
        kf_.temp4 = other.kf_.temp4.clone();
        kf_.temp5 = other.kf_.temp5.clone();
    }


    // =================================================
    // 获取当前有效状态矩阵
    // =================================================

    const cv::Mat& currentState() const
    {
        return prediction_pending_
            ? kf_.statePre
            : kf_.statePost;
    }

    const cv::Mat& currentCovariance() const
    {
        return prediction_pending_
            ? kf_.errorCovPre
            : kf_.errorCovPost;
    }


public:

    // =================================================
    // 构造函数
    // =================================================

    ConstantAccelerationKF6D(
        double q = 1.0,
        double r = 25.0
    )
        : q_(std::max(0.0, q)),
          r_(std::max(1e-9, r))
    {
        configure();
    }


    // 拷贝构造函数
    ConstantAccelerationKF6D(
        const ConstantAccelerationKF6D& other
    )
    {
        copyFrom(other);
    }


    // 拷贝赋值运算符
    ConstantAccelerationKF6D& operator=(
        const ConstantAccelerationKF6D& other
    )
    {
        if (this != &other)
        {
            copyFrom(other);
        }

        return *this;
    }


    // =================================================
    // 初始化：使用三个连续位置
    // =================================================

    void initialize(
        const cv::Point2f& p0,
        const cv::Point2f& p1,
        const cv::Point2f& p2,
        double dt01 = 1.0,
        double dt12 = 1.0
    )
    {
        if (dt01 <= 0.0 || dt12 <= 0.0)
            return;

        // -----------------------------------------
        // 1. 计算两个区间的平均速度
        // -----------------------------------------

        double vx01 = (p1.x - p0.x) / dt01;
        double vy01 = (p1.y - p0.y) / dt01;

        double vx12 = (p2.x - p1.x) / dt12;
        double vy12 = (p2.y - p1.y) / dt12;

        // -----------------------------------------
        // 2. 计算加速度
        // -----------------------------------------

        double D = (dt01 + dt12) * 0.5;

        double ax = (vx12 - vx01) / D;
        double ay = (vy12 - vy01) / D;

        // -----------------------------------------
        // 3. 计算p2时刻的速度
        // -----------------------------------------

        double vx = vx12 + 0.5 * ax * dt12;
        double vy = vy12 + 0.5 * ay * dt12;

        // -----------------------------------------
        // 4. 设置初始状态
        // -----------------------------------------

        kf_.statePost.at<double>(0, 0) = p2.x;
        kf_.statePost.at<double>(1, 0) = p2.y;

        kf_.statePost.at<double>(2, 0) = vx;
        kf_.statePost.at<double>(3, 0) = vy;

        kf_.statePost.at<double>(4, 0) = ax;
        kf_.statePost.at<double>(5, 0) = ay;

        // -----------------------------------------
        // 5. 计算初始化协方差
        // 根据三个位置测量的噪声传播
        // -----------------------------------------

        double c[3] =
        {
            1.0 / (dt01 * D),

            -(1.0 / dt01 + 1.0 / dt12) / D,

            1.0 / (dt12 * D)
        };

        double b[3] =
        {
            0.5 * dt12 * c[0],

            -1.0 / dt12 + 0.5 * dt12 * c[1],

            1.0 / dt12 + 0.5 * dt12 * c[2]
        };

        // [p2, v2, a]对[p0,p1,p2]的系数
        double coeff[3][3] =
        {
            {0.0, 0.0, 1.0},
            {b[0], b[1], b[2]},
            {c[0], c[1], c[2]}
        };

        // X方向索引
        int x_index[3] = {0, 2, 4};

        // Y方向索引
        int y_index[3] = {1, 3, 5};

        kf_.errorCovPost =
            cv::Mat::zeros(6, 6, CV_64F);

        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                double covariance = 0.0;

                for (int k = 0; k < 3; k++)
                {
                    covariance +=
                        coeff[i][k] * coeff[j][k];
                }

                covariance *= r_;

                kf_.errorCovPost.at<double>(
                    x_index[i], x_index[j]
                ) = covariance;

                kf_.errorCovPost.at<double>(
                    y_index[i], y_index[j]
                ) = covariance;
            }
        }

        // 同步预测状态与协方差
        kf_.statePre = kf_.statePost.clone();

        kf_.errorCovPre = kf_.errorCovPost.clone();

        initialized_ = true;
        prediction_pending_ = false;
    }


    // =================================================
    // 预测
    // =================================================

    void predict(double dt = 1.0)
    {
        if (!initialized_ || dt <= 0.0)
            return;

        // 如果上一帧只有预测，没有correct，
        // 就将上一次预测结果提交为当前状态。
        if (prediction_pending_)
        {
            kf_.statePre.copyTo(kf_.statePost);

            kf_.errorCovPre.copyTo(kf_.errorCovPost);

            prediction_pending_ = false;
        }

        // -----------------------------------------
        // 1. 设置状态转移矩阵F
        // -----------------------------------------

        kf_.transitionMatrix =
            cv::Mat::eye(6, 6, CV_64F);

        double dt2 = dt * dt;

        // x
        kf_.transitionMatrix.at<double>(0, 2) = dt;
        kf_.transitionMatrix.at<double>(0, 4) = 0.5 * dt2;

        // y
        kf_.transitionMatrix.at<double>(1, 3) = dt;
        kf_.transitionMatrix.at<double>(1, 5) = 0.5 * dt2;

        // vx
        kf_.transitionMatrix.at<double>(2, 4) = dt;

        // vy
        kf_.transitionMatrix.at<double>(3, 5) = dt;

        // -----------------------------------------
        // 2. 更新过程噪声矩阵Q
        // -----------------------------------------

        kf_.processNoiseCov = buildQ(dt);

        // -----------------------------------------
        // 3. 执行OpenCV预测
        // -----------------------------------------

        kf_.predict();

        prediction_pending_ = true;
    }


    // =================================================
    // 修正
    // =================================================

    void correct(const Measurement& z)
    {
        if (!initialized_)
            return;

        // 如果没有执行过predict，
        // 则以当前后验状态作为本次修正的先验状态。
        if (!prediction_pending_)
        {
            kf_.statePost.copyTo(kf_.statePre);

            kf_.errorCovPost.copyTo(kf_.errorCovPre);

            prediction_pending_ = true;
        }

        // 构造测量矩阵
        cv::Mat measurement =
            cv::Mat::zeros(2, 1, CV_64F);

        measurement.at<double>(0, 0) = z[0];
        measurement.at<double>(1, 0) = z[1];

        // 执行OpenCV修正
        kf_.correct(measurement);

        prediction_pending_ = false;
    }


    // =================================================
    // 获取位置
    // =================================================

    cv::Point2f getPosition() const
    {
        const cv::Mat& state = currentState();

        return cv::Point2f(
            static_cast<float>(state.at<double>(0, 0)),
            static_cast<float>(state.at<double>(1, 0))
        );
    }


    // =================================================
    // 获取速度
    // =================================================

    cv::Point2f getVelocity() const
    {
        const cv::Mat& state = currentState();

        return cv::Point2f(
            static_cast<float>(state.at<double>(2, 0)),
            static_cast<float>(state.at<double>(3, 0))
        );
    }


    // =================================================
    // 获取加速度
    // =================================================

    cv::Point2f getAcceleration() const
    {
        const cv::Mat& state = currentState();

        return cv::Point2f(
            static_cast<float>(state.at<double>(4, 0)),
            static_cast<float>(state.at<double>(5, 0))
        );
    }


    // =================================================
    // 获取完整状态
    // =================================================

    const State& getState() const
    {
        const cv::Mat& state = currentState();

        for (int i = 0; i < 6; i++)
        {
            state_cache_[i] =
                state.at<double>(i, 0);
        }

        return state_cache_;
    }


    // =================================================
    // 获取协方差
    // =================================================

    const Covariance& getCovariance() const
    {
        const cv::Mat& P = currentCovariance();

        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                covariance_cache_(i, j) =
                    P.at<double>(i, j);
            }
        }

        return covariance_cache_;
    }


    // =================================================
    // 获取噪声参数
    // =================================================

    double getQ() const
    {
        return q_;
    }

    double getR() const
    {
        return r_;
    }


    // =================================================
    // 初始化状态管理
    // =================================================

    bool isInitialized() const
    {
        return initialized_;
    }

    void reset()
    {
        kf_.statePost.setTo(0);
        kf_.statePre.setTo(0);

        kf_.errorCovPost.setTo(0);
        kf_.errorCovPre.setTo(0);

        initialized_ = false;
        prediction_pending_ = false;
    }
};

namespace KFDetail
{
    // =================================================
    // OpenCV KalmanFilter深拷贝
    // =================================================

    inline void copyFilter(
        cv::KalmanFilter& dst,
        const cv::KalmanFilter& src,
        int state_dim
    )
    {
        dst.init(state_dim, 2, 0, CV_64F);

        dst.statePre = src.statePre.clone();
        dst.statePost = src.statePost.clone();

        dst.transitionMatrix = src.transitionMatrix.clone();
        dst.controlMatrix = src.controlMatrix.clone();
        dst.measurementMatrix = src.measurementMatrix.clone();

        dst.processNoiseCov = src.processNoiseCov.clone();
        dst.measurementNoiseCov = src.measurementNoiseCov.clone();

        dst.errorCovPre = src.errorCovPre.clone();
        dst.errorCovPost = src.errorCovPost.clone();

        dst.gain = src.gain.clone();

        dst.temp1 = src.temp1.clone();
        dst.temp2 = src.temp2.clone();
        dst.temp3 = src.temp3.clone();
        dst.temp4 = src.temp4.clone();
        dst.temp5 = src.temp5.clone();
    }
}


// =====================================================
// 1. 四维恒速度 KF
// State: [x, y, vx, vy]
// =====================================================

class ConstantVelocityKF4D
{
public:

    using State = cv::Vec<double, 4>;

    using Measurement = cv::Vec2d;

    using Covariance = cv::Matx<double, 4, 4>;

private:

    cv::KalmanFilter kf_;

    double q_;
    double r_;

    bool initialized_ = false;

    bool prediction_pending_ = false;

    mutable State state_cache_;

    mutable Covariance covariance_cache_;


    // =================================================
    // 构建过程噪声矩阵Q
    // 白噪声加速度模型
    // =================================================

    cv::Mat buildQ(double dt) const
    {
        cv::Mat Q = cv::Mat::zeros(4, 4, CV_64F);

        double dt2 = dt * dt;
        double dt3 = dt2 * dt;

        double block[2][2] =
        {
            {
                dt3 / 3.0,
                dt2 / 2.0
            },
            {
                dt2 / 2.0,
                dt
            }
        };

        int x_index[2] = {0, 2};
        int y_index[2] = {1, 3};

        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < 2; j++)
            {
                Q.at<double>(
                    x_index[i], x_index[j]
                ) = q_ * block[i][j];

                Q.at<double>(
                    y_index[i], y_index[j]
                ) = q_ * block[i][j];
            }
        }

        return Q;
    }


    // =================================================
    // 初始化OpenCV KF
    // =================================================

    void configure()
    {
        kf_.init(4, 2, 0, CV_64F);

        // 状态转移矩阵
        kf_.transitionMatrix =
            cv::Mat::eye(4, 4, CV_64F);

        // 测量矩阵
        kf_.measurementMatrix =
            cv::Mat::zeros(2, 4, CV_64F);

        kf_.measurementMatrix.at<double>(0, 0) = 1.0;
        kf_.measurementMatrix.at<double>(1, 1) = 1.0;

        // 过程噪声
        kf_.processNoiseCov = buildQ(1.0);

        // 测量噪声
        kf_.measurementNoiseCov =
            cv::Mat::eye(2, 2, CV_64F) * r_;

        // 状态
        kf_.statePost =
            cv::Mat::zeros(4, 1, CV_64F);

        kf_.statePre = kf_.statePost.clone();

        // 协方差
        kf_.errorCovPost =
            cv::Mat::eye(4, 4, CV_64F) * 100.0;

        kf_.errorCovPre =
            kf_.errorCovPost.clone();
    }


    // =================================================
    // 深拷贝
    // =================================================

    void copyFrom(const ConstantVelocityKF4D& other)
    {
        q_ = other.q_;
        r_ = other.r_;

        initialized_ = other.initialized_;
        prediction_pending_ = other.prediction_pending_;

        KFDetail::copyFilter(kf_, other.kf_, 4);
    }


    // =================================================
    // 当前有效状态
    // =================================================

    const cv::Mat& currentState() const
    {
        return prediction_pending_
            ? kf_.statePre
            : kf_.statePost;
    }

    const cv::Mat& currentCovariance() const
    {
        return prediction_pending_
            ? kf_.errorCovPre
            : kf_.errorCovPost;
    }


public:

    // =================================================
    // 构造函数
    // =================================================

    ConstantVelocityKF4D(
        double q = 1.0,
        double r = 25.0
    )
        : q_(std::max(0.0, q)),
          r_(std::max(1e-9, r))
    {
        configure();
    }


    // 拷贝构造
    ConstantVelocityKF4D(
        const ConstantVelocityKF4D& other
    )
    {
        copyFrom(other);
    }


    // 拷贝赋值
    ConstantVelocityKF4D& operator=(
        const ConstantVelocityKF4D& other
    )
    {
        if (this != &other)
        {
            copyFrom(other);
        }

        return *this;
    }


    // =================================================
    // 初始化
    // 使用三个连续检测点进行最小二乘速度估计
    // =================================================

    void initialize(
        const cv::Point2f& p0,
        const cv::Point2f& p1,
        const cv::Point2f& p2,
        double dt01 = 1.0,
        double dt12 = 1.0
    )
    {
        if (dt01 <= 0.0 || dt12 <= 0.0)
            return;

        // 三个点相对于p2的时间
        double t[3] =
        {
            -(dt01 + dt12),
            -dt12,
            0.0
        };

        double S1 = t[0] + t[1] + t[2];

        double S2 =
            t[0] * t[0] +
            t[1] * t[1] +
            t[2] * t[2];

        double det = 3.0 * S2 - S1 * S1;

        if (det <= 1e-12)
            return;

        // X方向最小二乘估计
        double sum_x = p0.x + p1.x + p2.x;

        double sum_tx =
            t[0] * p0.x +
            t[1] * p1.x;

        double x =
            (S2 * sum_x - S1 * sum_tx) / det;

        double vx =
            (3.0 * sum_tx - S1 * sum_x) / det;

        // Y方向最小二乘估计
        double sum_y = p0.y + p1.y + p2.y;

        double sum_ty =
            t[0] * p0.y +
            t[1] * p1.y;

        double y =
            (S2 * sum_y - S1 * sum_ty) / det;

        double vy =
            (3.0 * sum_ty - S1 * sum_y) / det;

        // 设置状态
        kf_.statePost.at<double>(0, 0) = x;
        kf_.statePost.at<double>(1, 0) = y;

        kf_.statePost.at<double>(2, 0) = vx;
        kf_.statePost.at<double>(3, 0) = vy;

        // 初始化协方差
        kf_.errorCovPost =
            cv::Mat::zeros(4, 4, CV_64F);

        int x_index[2] = {0, 2};
        int y_index[2] = {1, 3};

        double P[2][2] =
        {
            {
                r_ * S2 / det,
                -r_ * S1 / det
            },
            {
                -r_ * S1 / det,
                r_ * 3.0 / det
            }
        };

        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < 2; j++)
            {
                kf_.errorCovPost.at<double>(
                    x_index[i], x_index[j]
                ) = P[i][j];

                kf_.errorCovPost.at<double>(
                    y_index[i], y_index[j]
                ) = P[i][j];
            }
        }

        kf_.statePre = kf_.statePost.clone();

        kf_.errorCovPre =
            kf_.errorCovPost.clone();

        initialized_ = true;
        prediction_pending_ = false;
    }


    // =================================================
    // Predict
    // =================================================

    void predict(double dt = 1.0)
    {
        if (!initialized_ || dt <= 0.0)
            return;

        // 提交上一次未修正的预测结果
        if (prediction_pending_)
        {
            kf_.statePre.copyTo(kf_.statePost);

            kf_.errorCovPre.copyTo(
                kf_.errorCovPost
            );

            prediction_pending_ = false;
        }

        // 状态转移矩阵
        kf_.transitionMatrix =
            cv::Mat::eye(4, 4, CV_64F);

        kf_.transitionMatrix.at<double>(0, 2) = dt;
        kf_.transitionMatrix.at<double>(1, 3) = dt;

        // 更新Q
        kf_.processNoiseCov = buildQ(dt);

        // 执行预测
        kf_.predict();

        prediction_pending_ = true;
    }


    // =================================================
    // Correct
    // =================================================

    void correct(const Measurement& z)
    {
        if (!initialized_)
            return;

        if (!prediction_pending_)
        {
            kf_.statePost.copyTo(kf_.statePre);

            kf_.errorCovPost.copyTo(
                kf_.errorCovPre
            );

            prediction_pending_ = true;
        }

        const cv::Mat measurement(z);

        kf_.correct(measurement);

        prediction_pending_ = false;
    }


    // =================================================
    // 获取位置
    // =================================================

    cv::Point2f getPosition() const
    {
        const cv::Mat& state = currentState();

        return cv::Point2f(
            static_cast<float>(state.at<double>(0, 0)),
            static_cast<float>(state.at<double>(1, 0))
        );
    }


    // =================================================
    // 获取速度
    // =================================================

    cv::Point2f getVelocity() const
    {
        const cv::Mat& state = currentState();

        return cv::Point2f(
            static_cast<float>(state.at<double>(2, 0)),
            static_cast<float>(state.at<double>(3, 0))
        );
    }


    // 恒速度模型不估计加速度
    cv::Point2f getAcceleration() const
    {
        return cv::Point2f(0.0f, 0.0f);
    }


    // =================================================
    // 获取完整状态
    // =================================================

    const State& getState() const
    {
        const cv::Mat& state = currentState();

        for (int i = 0; i < 4; i++)
        {
            state_cache_[i] =
                state.at<double>(i, 0);
        }

        return state_cache_;
    }


    // =================================================
    // 获取协方差
    // =================================================

    const Covariance& getCovariance() const
    {
        const cv::Mat& P = currentCovariance();

        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                covariance_cache_(i, j) =
                    P.at<double>(i, j);
            }
        }

        return covariance_cache_;
    }


    // =================================================
    // 参数获取
    // =================================================

    double getQ() const
    {
        return q_;
    }

    double getR() const
    {
        return r_;
    }


    // =================================================
    // 初始化状态管理
    // =================================================

    bool isInitialized() const
    {
        return initialized_;
    }

    void reset()
    {
        kf_.statePost.setTo(0);
        kf_.statePre.setTo(0);

        kf_.errorCovPost.setTo(0);
        kf_.errorCovPre.setTo(0);

        initialized_ = false;
        prediction_pending_ = false;
    }
};


// =====================================================
// 2. Singer 六维 KF
// State: [x, y, vx, vy, ax, ay]
// =====================================================

class SingerKF6D
{
public:

    using State = cv::Vec<double, 6>;

    using Measurement = cv::Vec2d;

    using Covariance = cv::Matx<double, 6, 6>;

private:

    cv::KalmanFilter kf_;

    double q_;
    double r_;

    // 加速度衰减时间常数，单位：frame
    double tau_;

    bool initialized_ = false;

    bool prediction_pending_ = false;

    mutable State state_cache_;

    mutable Covariance covariance_cache_;


    // =================================================
    // 构建Singer状态转移矩阵
    // =================================================

    cv::Mat buildF(double dt) const
    {
        cv::Mat F =
            cv::Mat::eye(6, 6, CV_64F);

        double alpha = 1.0 / tau_;

        double E = std::exp(-alpha * dt);

        double A;
        double B;

        // 避免alpha接近0时的数值不稳定
        if (std::abs(alpha * dt) < 1e-4)
        {
            double dt2 = dt * dt;
            double dt3 = dt2 * dt;
            double dt4 = dt3 * dt;
            double dt5 = dt4 * dt;

            A =
                dt
                - alpha * dt2 / 2.0
                + alpha * alpha * dt3 / 6.0
                - alpha * alpha * alpha * dt4 / 24.0;

            B =
                dt2 / 2.0
                - alpha * dt3 / 6.0
                + alpha * alpha * dt4 / 24.0
                - alpha * alpha * alpha * dt5 / 120.0;
        }
        else
        {
            A = (1.0 - E) / alpha;

            B = (dt - A) / alpha;
        }

        // X方向索引：x, vx, ax
        F.at<double>(0, 2) = dt;
        F.at<double>(0, 4) = B;

        F.at<double>(2, 4) = A;

        F.at<double>(4, 4) = E;

        // Y方向索引：y, vy, ay
        F.at<double>(1, 3) = dt;
        F.at<double>(1, 5) = B;

        F.at<double>(3, 5) = A;

        F.at<double>(5, 5) = E;

        return F;
    }


    // =================================================
    // 构建Singer过程噪声矩阵Q
    //
    // 连续时间模型：
    // da/dt = -alpha*a + w(t)
    //
    // q为白噪声跃度驱动强度
    // 使用8点Gauss-Legendre积分
    // =================================================

    cv::Mat buildQ(double dt) const
    {
        cv::Mat Q =
            cv::Mat::zeros(6, 6, CV_64F);

        const double nodes[4] =
        {
            0.9602898564975363,
            0.7966664774136267,
            0.5255324099163290,
            0.1834346424956498
        };

        const double weights[4] =
        {
            0.1012285362903763,
            0.2223810344533745,
            0.3137066458778873,
            0.3626837833783620
        };

        double alpha = 1.0 / tau_;

        int x_index[3] = {0, 2, 4};
        int y_index[3] = {1, 3, 5};

        // 积分区间：[0, dt]
        for (int n = 0; n < 4; n++)
        {
            for (int sign = -1; sign <= 1; sign += 2)
            {
                double s =
                    0.5 * dt * (1.0 + sign * nodes[n]);

                double weight =
                    0.5 * dt * weights[n];

                double g[3];

                if (std::abs(alpha * s) < 1e-5)
                {
                    // alpha趋近于0时的极限
                    g[0] = 0.5 * s * s;
                    g[1] = s;
                    g[2] = 1.0;
                }
                else
                {
                    double E = std::exp(-alpha * s);

                    g[2] = E;

                    g[1] = (1.0 - E) / alpha;

                    g[0] =
                        (alpha * s - (1.0 - E))
                        / (alpha * alpha);
                }

                for (int i = 0; i < 3; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        double value =
                            q_ * weight * g[i] * g[j];

                        Q.at<double>(
                            x_index[i],
                            x_index[j]
                        ) += value;

                        Q.at<double>(
                            y_index[i],
                            y_index[j]
                        ) += value;
                    }
                }
            }
        }

        return Q;
    }


    // =================================================
    // 初始化OpenCV KF
    // =================================================

    void configure()
    {
        kf_.init(6, 2, 0, CV_64F);

        // 状态转移矩阵
        kf_.transitionMatrix =
            cv::Mat::eye(6, 6, CV_64F);

        // 测量矩阵
        kf_.measurementMatrix =
            cv::Mat::zeros(2, 6, CV_64F);

        kf_.measurementMatrix.at<double>(0, 0) = 1.0;
        kf_.measurementMatrix.at<double>(1, 1) = 1.0;

        // 过程噪声
        kf_.processNoiseCov = buildQ(1.0);

        // 测量噪声
        kf_.measurementNoiseCov =
            cv::Mat::eye(2, 2, CV_64F) * r_;

        // 状态
        kf_.statePost =
            cv::Mat::zeros(6, 1, CV_64F);

        kf_.statePre = kf_.statePost.clone();

        // 协方差
        kf_.errorCovPost =
            cv::Mat::eye(6, 6, CV_64F) * 100.0;

        kf_.errorCovPre =
            kf_.errorCovPost.clone();
    }


    // =================================================
    // 深拷贝
    // =================================================

    void copyFrom(const SingerKF6D& other)
    {
        q_ = other.q_;
        r_ = other.r_;
        tau_ = other.tau_;

        initialized_ = other.initialized_;
        prediction_pending_ = other.prediction_pending_;

        KFDetail::copyFilter(kf_, other.kf_, 6);
    }


    // =================================================
    // 当前有效状态
    // =================================================

    const cv::Mat& currentState() const
    {
        return prediction_pending_
            ? kf_.statePre
            : kf_.statePost;
    }

    const cv::Mat& currentCovariance() const
    {
        return prediction_pending_
            ? kf_.errorCovPre
            : kf_.errorCovPost;
    }


public:

    // =================================================
    // 构造函数
    // =================================================

    SingerKF6D(
        double q = 1.0,
        double r = 25.0,
        double tau = 5.0
    )
        : q_(std::max(0.0, q)),
          r_(std::max(1e-9, r)),
          tau_(std::max(1e-6, tau))
    {
        configure();
    }


    // 拷贝构造
    SingerKF6D(const SingerKF6D& other)
    {
        copyFrom(other);
    }


    // 拷贝赋值
    SingerKF6D& operator=(const SingerKF6D& other)
    {
        if (this != &other)
        {
            copyFrom(other);
        }

        return *this;
    }


    // =================================================
    // 初始化
    // 使用三个连续检测点
    // =================================================

    void initialize(
        const cv::Point2f& p0,
        const cv::Point2f& p1,
        const cv::Point2f& p2,
        double dt01 = 1.0,
        double dt12 = 1.0
    )
    {
        if (dt01 <= 0.0 || dt12 <= 0.0)
            return;

        double vx01 = (p1.x - p0.x) / dt01;
        double vy01 = (p1.y - p0.y) / dt01;

        double vx12 = (p2.x - p1.x) / dt12;
        double vy12 = (p2.y - p1.y) / dt12;

        double D = (dt01 + dt12) * 0.5;

        double ax = (vx12 - vx01) / D;
        double ay = (vy12 - vy01) / D;

        // p2时刻的速度
        double vx = vx12 + 0.5 * ax * dt12;
        double vy = vy12 + 0.5 * ay * dt12;

        // 设置状态
        kf_.statePost.at<double>(0, 0) = p2.x;
        kf_.statePost.at<double>(1, 0) = p2.y;

        kf_.statePost.at<double>(2, 0) = vx;
        kf_.statePost.at<double>(3, 0) = vy;

        kf_.statePost.at<double>(4, 0) = ax;
        kf_.statePost.at<double>(5, 0) = ay;

        // 初始化协方差
        double c[3] =
        {
            1.0 / (dt01 * D),

            -(1.0 / dt01 + 1.0 / dt12) / D,

            1.0 / (dt12 * D)
        };

        double b[3] =
        {
            0.5 * dt12 * c[0],

            -1.0 / dt12 + 0.5 * dt12 * c[1],

            1.0 / dt12 + 0.5 * dt12 * c[2]
        };

        double coeff[3][3] =
        {
            {0.0, 0.0, 1.0},
            {b[0], b[1], b[2]},
            {c[0], c[1], c[2]}
        };

        int x_index[3] = {0, 2, 4};
        int y_index[3] = {1, 3, 5};

        kf_.errorCovPost =
            cv::Mat::zeros(6, 6, CV_64F);

        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                double covariance = 0.0;

                for (int k = 0; k < 3; k++)
                {
                    covariance +=
                        coeff[i][k] * coeff[j][k];
                }

                covariance *= r_;

                kf_.errorCovPost.at<double>(
                    x_index[i],
                    x_index[j]
                ) = covariance;

                kf_.errorCovPost.at<double>(
                    y_index[i],
                    y_index[j]
                ) = covariance;
            }
        }

        kf_.statePre = kf_.statePost.clone();

        kf_.errorCovPre =
            kf_.errorCovPost.clone();

        initialized_ = true;
        prediction_pending_ = false;
    }


    // =================================================
    // Predict
    // =================================================

    void predict(double dt = 1.0)
    {
        if (!initialized_ || dt <= 0.0)
            return;

        // 提交上一次未修正的预测
        if (prediction_pending_)
        {
            kf_.statePre.copyTo(kf_.statePost);

            kf_.errorCovPre.copyTo(
                kf_.errorCovPost
            );

            prediction_pending_ = false;
        }

        // 更新状态转移矩阵
        kf_.transitionMatrix = buildF(dt);

        // 更新过程噪声矩阵
        kf_.processNoiseCov = buildQ(dt);

        // 执行预测
        kf_.predict();

        prediction_pending_ = true;
    }


    // =================================================
    // Correct
    // =================================================

    void correct(const Measurement& z)
    {
        if (!initialized_)
            return;

        if (!prediction_pending_)
        {
            kf_.statePost.copyTo(kf_.statePre);

            kf_.errorCovPost.copyTo(
                kf_.errorCovPre
            );

            prediction_pending_ = true;
        }

        const cv::Mat measurement(z);

        kf_.correct(measurement);

        prediction_pending_ = false;
    }


    // =================================================
    // 获取位置
    // =================================================

    cv::Point2f getPosition() const
    {
        const cv::Mat& state = currentState();

        return cv::Point2f(
            static_cast<float>(state.at<double>(0, 0)),
            static_cast<float>(state.at<double>(1, 0))
        );
    }


    // =================================================
    // 获取速度
    // =================================================

    cv::Point2f getVelocity() const
    {
        const cv::Mat& state = currentState();

        return cv::Point2f(
            static_cast<float>(state.at<double>(2, 0)),
            static_cast<float>(state.at<double>(3, 0))
        );
    }


    // =================================================
    // 获取加速度
    // =================================================

    cv::Point2f getAcceleration() const
    {
        const cv::Mat& state = currentState();

        return cv::Point2f(
            static_cast<float>(state.at<double>(4, 0)),
            static_cast<float>(state.at<double>(5, 0))
        );
    }


    // =================================================
    // 获取完整状态
    // =================================================

    const State& getState() const
    {
        const cv::Mat& state = currentState();

        for (int i = 0; i < 6; i++)
        {
            state_cache_[i] =
                state.at<double>(i, 0);
        }

        return state_cache_;
    }


    // =================================================
    // 获取协方差
    // =================================================

    const Covariance& getCovariance() const
    {
        const cv::Mat& P = currentCovariance();

        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                covariance_cache_(i, j) =
                    P.at<double>(i, j);
            }
        }

        return covariance_cache_;
    }


    // =================================================
    // 参数获取
    // =================================================

    double getQ() const
    {
        return q_;
    }

    double getR() const
    {
        return r_;
    }

    double getTau() const
    {
        return tau_;
    }


    // =================================================
    // 初始化状态管理
    // =================================================

    bool isInitialized() const
    {
        return initialized_;
    }

    void reset()
    {
        kf_.statePost.setTo(0);
        kf_.statePre.setTo(0);

        kf_.errorCovPost.setTo(0);
        kf_.errorCovPre.setTo(0);

        initialized_ = false;
        prediction_pending_ = false;
    }
};
