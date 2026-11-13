# 07｜卡尔曼滤波目标跟踪

本章对应 `src/task_07/main.cpp`。卡尔曼滤波把“运动模型预测”和“图像测量”融合起来，用内部状态估计目标位置和速度。在检测有噪声时，它能平滑轨迹；检测偶尔缺失时，可以短时间外推位置。

## 状态和测量

当前程序使用四维状态与二维测量：

`x_k = [p_x, p_y, v_x, v_y]^T`，`z_k = [p_x, p_y]^T`

其中位置单位是像素，速度单位是像素/帧。常速度转移模型为：

`x_k = A x_(k-1) + w_k`

`z_k = H x_k + v_k`

当帧间隔 `Δt` 固定时：

```text
A = [1 0 Δt 0]
    [0 1 0 Δt]
    [0 0 1  0]
    [0 0 0  1]

H = [1 0 0 0]
    [0 1 0 0]
```

`A` 表示匀速外推位置、保持速度；`H` 表示摄像头只直接观测位置，不直接测速度。

## 预测和校正的原理

滤波器同时维护状态估计 `x` 和误差协方差 `P`。

**预测：**

`x⁻ = A x`，`P⁻ = A P Aᵀ + Q`

**得到测量后校正：**

`K = P⁻ Hᵀ (H P⁻ Hᵀ + R)⁻¹`

`x = x⁻ + K (z - H x⁻)`，`P = (I - K H) P⁻`

`K` 是卡尔曼增益：预测不确定性大或测量噪声小时，算法更信测量；预测可信或测量噪声大时，更信运动模型。预测和测量不是简单平均，权重由协方差按每一维动态计算。

![运行记录中的测量中心和滤波中心轨迹](../assets/figures/task07_kalman_trajectory.png)

该图根据仓库里已有的 `image_output/task07/cap.csv` 和 `filter.csv` 轨迹记录绘制；无效帧间断线，滤波器启动阶段的 `(0,0)` 占位值不参与绘图。

## OpenCV 接口与矩阵

| 接口/成员 | 作用 | 代码中的用途 |
|---|---|---|
| `KalmanFilter::init(dynamParams, measureParams, controlParams, type)` | 分配滤波器矩阵；此处状态 4 维、测量 2 维、无控制输入、32 位浮点。 | `kf.init(4,2,0,CV_32F);` |
| `transitionMatrix` | 状态转移矩阵 `A`。 | 编码 `x+=vx*dt`、`y+=vy*dt`。 |
| `measurementMatrix` | 测量矩阵 `H`。 | 只从状态中取出 x、y。 |
| `processNoiseCov` | 过程噪声协方差 `Q`，描述匀速模型未包含的加速度等变化。 | 使用白噪声加速度模型构造。 |
| `measurementNoiseCov` | 测量噪声协方差 `R`，描述颜色阈值、轮廓边界和圆心估计带来的位置误差。 | 设为 `diag(r,r)`。 |
| `statePost` | 最近一次校正后的后验状态。 | 用前两帧检测中心初始化位置和速度。 |
| `errorCovPost` | 后验状态协方差。 | 表示校正后各状态的不确定性；初始化时也应合理赋值。 |
| `predict()` | 执行时间更新，返回当前测量到来前的状态预测。 | `Mat prediction = kf.predict();` |
| `correct(measurement)` | 把本帧测量融入预测并返回后验状态。 | `Mat corrected = kf.correct(measurement);` |
| `minEnclosingCircle(points, center, radius)` | 求包含轮廓所有点的最小圆。 | 以轮廓中心近似篮球中心，并取得绘图半径。 |

## `Q`、`R` 的调节

- 增大 `Q` 表示实际运动更可能偏离匀速模型，滤波器通常会更快接纳新观测，但预测更不稳定。
- 增大 `R` 表示图像测量更不可靠，滤波器更信预测，轨迹更平滑但更可能滞后。
- 程序中的 `q=0.5` 和 `r=2.3` 是当前分辨率和单位下的经验参数。`R` 的对角值是位置测量方差，量纲为像素平方；实际应通过静止目标/标注数据估计噪声，而不是把数字当成通用“平滑强度”。
- `Q` 必须与状态单位、帧间隔和噪声模型一致。帧率变化时，`Δt` 应用真实秒数重新计算，不能固定为 1 后仍把速度解释为像素/秒。

## 当前实现中的注意点

1. 初始化速度使用前两次可检测中心差，假设两次观测间隔恰为一帧；若中间丢帧，应除以实际帧数或时间间隔。
2. 首次初始化后应设置合理的 `errorCovPost`；矩阵的不确定性会影响初期卡尔曼增益。
3. 检测缺失时本代码只做 `predict()` 并绘制预测中心，不调用 `correct()`。这属于短时纯预测，连续丢帧越多误差通常越大；应设置最大外推帧数并在恢复检测时做关联判断。
4. 当前 `statePost` 初始化取的是最近一次检测的中心，速度是前两中心差；要确保位置、速度都对应同一时刻。
5. 只有一个最大面积轮廓被选作目标。多目标跟踪需要先做检测—轨迹关联，为每个轨迹维护自己的滤波器。

## 按本项目参数初始化模型

下面的初始化矩阵直接对应 `src/task_07/main.cpp::init_kalman_model`：状态顺序为 `(x, y, vx, vy)`，观测只有 `(x, y)`，`dt=1` 表示相邻视频帧间隔一个单位，`q=0.5`，`r=2.3`。

```cpp
void init_kalman_model(KalmanFilter& kf)
{
    kf.init(4, 2, 0, CV_32F); // 状态4维、测量2维、控制0维
    const float dt = 1.0f;
    const float q = 0.5f;
    const float r = 2.3f;

    kf.transitionMatrix = (Mat_<float>(4,4) <<
        1,0,dt,0,
        0,1,0,dt,
        0,0,1, 0,
        0,0,0, 1);

    kf.measurementMatrix = (Mat_<float>(2,4) <<
        1,0,0,0,
        0,1,0,0);

    // 白噪声加速度模型的过程噪声协方差
    kf.processNoiseCov = q * (Mat_<float>(4,4) <<
        dt*dt*dt*dt/4, 0,                  dt*dt*dt/2, 0,
        0,                  dt*dt*dt*dt/4, 0,                  dt*dt*dt/2,
        dt*dt*dt/2, 0,                  dt*dt,      0,
        0,                  dt*dt*dt/2, 0,                  dt*dt);

    kf.measurementNoiseCov = (Mat_<float>(2,2) <<
        r,0,
        0,r);
}
```

这里 `q` 乘上由 `dt` 推导出的矩阵，形成完整的 `Q`；`r` 是测量噪声协方差 `R` 的两个对角元素。`kf.init` 会建立内部矩阵，但运动状态还要用实际检测数据初始化。

## 首次状态与每帧怎么调用

至少拿到两个连续有效检测中心，才能估计初速度。先把两帧位置和速度写进 `statePost`，之后每一帧先预测；若有检测，就再用测量值校正：

```cpp
KalmanFilter kf;
init_kalman_model(kf);

Point2f p0(120.0f, 80.0f);  // 前一帧的检测中心
Point2f p1(126.0f, 84.0f);  // 当前帧的检测中心
float dt = 1.0f;            // 两次检测间的帧数/时间单位
kf.statePost = (Mat_<float>(4,1) <<
    p1.x, p1.y,
    (p1.x - p0.x) / dt,
    (p1.y - p0.y) / dt);
kf.errorCovPost = Mat::eye(4, 4, CV_32F); // 初始化状态协方差 P0

// 后续每帧：先时间预测
Mat predictedState = kf.predict();
Point2f predictedCenter(predictedState.at<float>(0,0),
                        predictedState.at<float>(1,0));

if (detected) {
    // detectedCenter 是本帧视觉检测出的中心点
    Mat measurement = (Mat_<float>(2,1) <<
        detectedCenter.x, detectedCenter.y);
    Mat correctedState = kf.correct(measurement);
    Point2f filteredCenter(correctedState.at<float>(0,0),
                           correctedState.at<float>(1,0));
    // 用 filteredCenter 画滤波后的位置；速度在索引 2、3
} else {
    // 没有测量时不调用 correct，只短时使用 predictedCenter 外推
}
```

每个调用的对象和参数：

- `kf.predict()`：不需要传参，返回 4×1 状态预测 `[x,y,vx,vy]`，并更新预测协方差。
- `errorCovPost`：这里额外设为单位阵作为初始后验协方差；测量或速度初值更不确定时，应按噪声大小设置对角元素。原项目只设置了状态，没有显式初始化该矩阵。
- `kf.correct(measurement)`：传入 2×1、`CV_32F` 的 `(x,y)` 测量矩阵，返回校正后的 4×1 状态。
- `measurement.at<float>(0)`、`at<float>(1)`：取中心坐标；`correctedState.at<float>(2/3)` 是校正后的 x/y 速度。
- `predict()` 要每帧调用一次。若本帧没有检测，保留预测即可；不要把无效的 `(0,0)` 当作真实观测传给 `correct()`。

本项目 `basketball_analyse(...)` 把上述步骤封装起来：参数 `vaild` 表示本帧是否找到有效球，`index` 是最大轮廓索引，`contours_point` 是轮廓集合，`src` 用于画检测/预测/滤波结果，`kf` 是该目标持续复用的滤波器。返回 `pair<Point2f,Point2f>`：`first` 是检测中心或丢检时的预测中心，`second` 是有检测且滤波器已启动后的校正中心。每帧必须传同一个 `kf`，不能在循环里重新构造滤波器。

### 对应 `circle` 画出三种位置

```cpp
circle(frame, detectedPixel, 5, Scalar(0,255,0), 2);  // 绿色：检测位置
circle(frame, predictedPixel, 5, Scalar(0,0,255), 2); // 红色：预测位置
circle(frame, filteredPixel, 5, Scalar(255,0,0), 2);  // 蓝色：校正后滤波位置
```

绘图坐标是整数 `Point`；状态坐标是浮点 `Point2f` 时，用 `Point(cvRound(x),cvRound(y))` 转换后再传给 `circle`。颜色参数是 BGR。

> 当前源码的 `init_kalman_model` 设置了 `A/H/Q/R`，而 `statePost` 在两个检测中心到来后才初始化。上面的代码延续这个设计，并把初速度按实际 `dt` 计算。长时间丢检时预测会发散，应限制连续外推帧数；恢复检测后还需先判断测量是否仍属于同一目标。


### 调用本项目的篮球跟踪函数

主循环中先从轮廓得到有效标志和索引，再把同一个 `kf` 传给分析函数：

```cpp
KalmanFilter kf;
init_kalman_model(kf); // 只初始化一次，不要每帧重置

int index = getmax_area_contours(contours);
bool valid = !contours.empty() && contourArea(contours[index]) > 3000;
auto centers = basketball_analyse(valid, index, contours, frame, kf);

Point2f detectedOrPredicted = centers.first;
Point2f filtered = centers.second;
```

调用前的 `contours` 是 `findContours` 得到的轮廓集合；阈值 3000 是当前练习使用的面积门限。`basketball_analyse` 只有在 `valid == true` 时才会用 `index` 访问轮廓，所以空集合时要保证 `valid` 为 false。函数内部用 `minEnclosingCircle` 取中心和半径，滤波器暖机完成后先 `predict()`，检测有效时再构造 `(x,y)` 测量并 `correct()`。

项目函数返回的两个中心有不同含义：`first` 在检测有效时是实测球心，检测丢失时变成预测球心；`second` 只有滤波器已启动且本帧有效检测时才写入校正后的中心。使用 `second` 前应检查是否已完成初始化和本帧检测有效，避免把未设置的 `Point2f` 当作结果。


## 模型不只一种

本任务的 `init_kalman_model` 是匀速状态 `[x,y,vx,vy]` 配合每帧随机加速度扰动的模型。考核若改变运动方程，必须同时考虑状态维度、`transitionMatrix`、`measurementMatrix` 和 `processNoiseCov`。新增的 [运动模型与过程噪声说明](08-kalman-models.md) 和 `include/kalman_motion_models.hpp` 给出随机游走、连续白加速度匀速及白 jerk 匀加速等可选方案。
