# OpenCV DNN 多目标跟踪封装与示例

本示例用 OpenCV DNN 读取单类别 YOLO ONNX 的检测框，用 HSV 过滤候选框，再用 KF/EKF 维护二维像素坐标中的目标轨迹。跑通示例后，可按同一段视频对比六种运动模型。[滤波参数调节方法](11-kalman-parameter-tuning.md)

## 文件与接口

| 文件 | 主要接口 | 输入和输出 |
|---|---|---|
| [`include/model_using.hpp`](../include/model_using.hpp) | `model_load`、`image_premdelopt`、`forward_getrect` | 读取 ONNX，等比缩放并填充 BGR 帧，推理和 NMS 后返回映射回原图的 `std::vector<cv::Rect>`，在传入的绘图帧上标出检测框 |
| [`include/common_func.hpp`](../include/common_func.hpp) | `rect_has_hsv`、`filter_rect_by_hsv`、`get_rect_withhsv`、`TrackerManager::update_tracker` | 检查候选框的颜色占比，提取辅助轮廓，关联目标并绘制滤波和预测框 |
| [`include/cerateEKF.hpp`](../include/cerateEKF.hpp) | 三种 EKF、三种 KF | 用二维检测中心初始化和校正，按选定运动模型预测位置 |
| [`src/dnn_test/main.cpp`](../src/dnn_test/main.cpp) | 视频示例 | 串联以上接口，支持逐帧查看或输出标注视频 |
| [`src/dnn_test/CMakeLists.txt`](../src/dnn_test/CMakeLists.txt) | 独立构建入口 | 使用 C++17 和 OpenCV 4 的 `core`、`imgproc`、`video`、`videoio`、`highgui`、`dnn` |

## 导出固定输入尺寸的 ONNX

在终端激活 `yolo` Conda 环境，把 `你的训练目录` 换成实际的训练目录名，然后执行以下命令。

```bash
conda activate yolo
yolo export \
    model=/home/zmasakaza/yolo/runs/detect/你的训练目录/weights/best.pt \
    format=onnx \
    imgsz=1280 \
    batch=1 \
    opset=12 \
    dynamic=False \
    half=False
```

导出的 `best.onnx` 位于 `best.pt` 同一目录。要保留多种输入尺寸，先把本次生成的文件改名为 `best1280.onnx`，再把 `imgsz=1280` 改成 `imgsz=640` 重新导出，并将新文件改名为 `best640.onnx`。运行 `dnn_test` 时，传入的边长应与导出时的 `imgsz` 一致。[Ultralytics 导出参数](https://docs.ultralytics.com/modes/export/)

用同一有标注验证集和相同的置信度、NMS 阈值对比两种输入尺寸，重点看小目标召回率、误检和推理耗时。根据验证结果选择部署尺寸。

## 编译和运行

从仓库根目录执行。准备自己的视频和单类别检测 ONNX。解析器使用 float32 的 `[1, 5, N]` 输出，五个通道依次是 `cx, cy, w, h, score`，坐标以模型输入像素为单位。

```bash
cmake -S src/dnn_test -B build/dnn_test
cmake --build build/dnn_test -j
./build/dnn_test/dnn_test /path/to/best1280.onnx /path/to/input.mp4 1280
```

模型输入是正方形，最后一个数值填写 ONNX 的输入边长，默认值为 `1280`。默认使用 OpenCV CPU 后端。窗口中按任意普通键前进一帧，按 `q` 或 `Esc` 退出。需要无窗口运行时，给出输出路径，视频编解码取决于本机的 OpenCV Video I/O 后端。

```bash
./build/dnn_test/dnn_test /path/to/best640.onnx /path/to/input.mp4 640 result.mp4
```

## 每帧处理顺序

1. `image_premdelopt` 保存缩放比例和左、上填充量。检测输出完成 NMS 后，`forward_getrect` 用这些量还原原图坐标。示例的置信度阈值为 `0.50`，NMS 的 IoU 阈值为 `0.45`。
2. `filter_rect_by_hsv(frame, detections, 0.20)` 对原始 BGR 帧转换一次 HSV，裁剪每个检测框到图像边界，并保留至少有 20% 像素落在指定颜色范围内的框。可在示例中移除此调用以关闭 YOLO 框的颜色筛选。颜色范围通过头文件里的 `yolo_lower_red_*` / `yolo_higher_red_*` 设置。轮廓辅助独立使用 `lower_red_*` / `higher_red_*`。
3. `update_tracker` 先预测所有已初始化的轨迹，再在 100 像素门限内按中心距离从小到大做一对一贪心关联。新轨迹收集三次已关联的检测中心，并用两段帧号差进行初始化。
4. YOLO 未匹配时，已初始化且近期有 YOLO 观测的轨迹可尝试轮廓辅助。轮廓匹配要求中心距离至多 80 像素，面积比在 `[0.25, 2.5]`，并受 `max_contour_frame` 限制。YOLO 或轮廓匹配成功就校正一次。两者均未匹配时沿预测状态推进，连续两帧未关联到任何观测则删除轨迹。
5. 绿色框是 HSV 筛选前的 YOLO 检测，蓝色框是当前帧校正后的估计，红色框由临时复制的滤波器再预测一步得到，用于查看下一帧位置。示例使用原始帧做颜色与轮廓检测，使用绘图副本显示结果。

如果已有几何检测框，可以直接把 `vector<cv::Rect>` 交给 `update_tracker`。纯几何检测时，可在管理器中停用轮廓辅助调用，以免同源轮廓重复参与关联。示例的关联依据是二维中心距离，目标交叉时应检查串轨情况。

## 单独使用或更换滤波器

六种模型共用 `initialize(p0, p1, p2, dt01, dt12)`、`predict(dt)`、`correct(Measurement)`、`getPosition()` 和 `isInitialized()`。在主循环中，每处理一帧先预测一次，再按是否拿到了当前帧观测决定是否校正。示例按顺序处理视频帧，因此 `dt=1.0` 对应一帧。

```cpp
#include "cerateEKF.hpp"

void filter_example()
{
    ConstantAccelerationKF6D filter(1.0, 25.0);
    filter.initialize({10, 20}, {13, 21}, {16, 22}, 1.0, 1.0);
    filter.predict(1.0);  // 当前帧没有检测时，跳过下一行
    filter.correct(ConstantAccelerationKF6D::Measurement(19.0, 23.0));
    const cv::Point2f current = filter.getPosition();
    auto preview = filter;
    preview.predict(1.0);
    const cv::Point2f next = preview.getPosition();
}
```

`TrackerManager` 默认使用 `CoordinatedTurnEKF5D`，示例构造参数依次为 `sigma_position=1.35`、`q_accel=0.90`、`q_alpha=0.90`。在管理器里替换模型时，同步更新 `Track::ekf` 的类型、两个新建轨迹处的构造参数，以及校正处的 `Measurement` 类型；若构造参数的数量或含义有变化，同时更新 `TrackerManager` 构造函数和示例调用。初始化、预测、校正和预览的调用顺序一致。[六种模型的参数与默认值](11-kalman-parameter-tuning.md)
