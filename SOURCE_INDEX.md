# 源码索引

## 练习程序

| 文件 | 内容 | 主要 OpenCV 接口 |
|---|---|---|
| `src/image_base/main.cpp` | `Mat` 创建、复制、颜色转换、通道拆分和窗口显示 | `imread`、`clone`、`copyTo`、`Mat::zeros/ones`、`cvtColor`、`split`、`imshow`、`waitKey` |
| `src/task_01/main.cpp` | 读图、灰度化、ROI 与鼠标回调 | `imread`、`cvtColor`、`Rect`、`Mat::operator()`、`setMouseCallback`、`imwrite` |
| `src/task_02/main.cpp` | HSV 颜色阈值和开闭运算 | `inRange`、`bitwise_or`、`getStructuringElement`、`morphologyEx` |
| `src/task_03/main.cpp` | 轮廓属性、旋转矩形和灯条候选筛选 | `findContours`、`contourArea`、`minAreaRect`、`line`、`putText` |
| `src/task_04/main.cpp` | 两灯条几何配对与透视展开 | `minAreaRect`、`getPerspectiveTransform`、`warpPerspective` |
| `src/task_05/main.cpp` | 相机参数读取、PnP 和重投影误差 | `FileStorage`、`solvePnP`、`projectPoints`、`setMouseCallback` |
| `src/task_06/main.cpp` | 视频帧处理、检测结果记录、基础坐标关联 | `VideoCapture`、`VideoWriter`、`TickMeter`、`read`、`write` |
| `src/task_07/main.cpp` | 篮球检测与卡尔曼滤波 | `KalmanFilter::predict/correct`、`findContours`、`minEnclosingCircle` |
| `src/cap_video/main.cpp` | 视频逐帧解码并保存图片 | `VideoCapture::read/release`、`imwrite` |
| `src/test/main.cpp` | CMake 最小构建目标（无 OpenCV 逻辑） | — |

## 辅助头文件

| 文件 | 自定义函数 |
|---|---|
| `include/lighting_cap.hpp` | `get_blue_mask`、`get_red_mask`、`calculate_draw`、`calculate_light`、`draw_doub_light`、`draw_point`、`use_perspectTrans` |
| `include/task_func.hpp` | `calculate_draw`（轮廓框绘制草稿）、`draw_rect`、`use_perspectTrans`、`read_camera_config`、`calculate_rmse`、`pnp_calculate`、`mousecb`、`traker`、`update_kal_traker`（多目标关联草稿） |
| `include/kalman_motion_models.hpp` | 随机游走、离散白加速度匀速、连续白加速度匀速、白 jerk 匀加速模型初始化函数 |
| `src/hsv_range_visualizer/main.cpp` | HSV/H/S/V 滑块、HSV 色相带与 S/V 平面可视化、上下界颜色预览 |
| `src/hsv_range_visualizer/CMakeLists.txt` | 独立的 HSV 可视化程序构建配置；不接入根 CMake |

## 文档与结果图

- `docs/01-image-and-mat.md`：图像读取显示、`Mat`、ROI、颜色转换。
- `docs/02-color-and-morphology.md`：HSV 阈值、掩模、形态学。
- `docs/03-contours-and-bars.md`：轮廓与候选灯条接口。
- `docs/04-pairing-and-perspective.md`：配对几何和单应透视变换。
- `docs/05-pnp.md`：PnP 位姿、重投影与误差。
- `docs/06-video-and-tracking.md`：视频 I/O、计时和中心点关联。
- `docs/07-kalman-filter.md`：线性卡尔曼滤波状态方程与代码映射。
- `docs/08-kalman-models.md`：不同状态模型和过程噪声矩阵的选择与切换。
- `docs/09-hsv-range-visualizer.md`：HSV 指示图、交互工具用法和 `inRange` 阈值示例。
