# 09｜HSV 色彩范围指示器

![OpenCV HSV 色彩范围示意](../assets/figures/hsv_range_indicator.png)

## 为什么 Markdown 里不是可拖动控件

GitHub 的 Markdown 页面会过滤脚本，不会运行文档里嵌入的 JavaScript、表单逻辑或自定义控件。因此，单靠 `.md` 文件不能在浏览页面里安全地执行 H/S/V 滑块。文档里放的是静态范围指示图；可调的版本做成一个独立 OpenCV HighGUI 小程序，布局放在 `src/hsv_range_visualizer/`，有自己的 `CMakeLists.txt`，不改项目根目录的 CMake 配置。

## 独立窗口功能

运行后会打开控制窗口和预览窗口：

- `Selected H/S/V`：选择一个具体 HSV 颜色；右侧色块显示转换到屏幕上的 BGR 颜色。
- `Lower H/S/V`、`Upper H/S/V`：拖动阈值上下界；下、上界色块和 HSV 色相带随数值更新。
- 左侧 S/V 平面固定当前 Selected H；横轴是饱和度 S，纵轴从 V=255 到 V=0。白框表示当前阈值在 S/V 平面上的有效区域，十字标记当前选定的 S/V。
- OpenCV 的 H 取值为 0–179，S/V 为 0–255。H 下界大于上界时表示跨越色相环末端和起点，常见于红色；真正对图像调用 `inRange` 时要分成两段掩模并用 `bitwise_or` 合并。S 或 V 下界大于上界则显示为无效范围。
- 按 `q` 或 `Esc` 关闭两个窗口。

## 单独编译和运行

在项目根目录执行：

```bash
cmake -S src/hsv_range_visualizer -B build-hsv
cmake --build build-hsv -j
./build-hsv/hsv_range_visualizer
```

这个 CMake 配置只查找 `core`、`imgproc`、`highgui`。如果 OpenCV 安装位置不在系统默认搜索路径，可显式提供配置文件目录，例如：

```bash
cmake -S src/hsv_range_visualizer -B build-hsv \
  -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4
```

程序需要桌面图形环境；无桌面的远程会话无法显示 HighGUI 窗口。

## 用滑块值设置颜色掩模

普通的连续 H 范围可直接传入 `inRange`：

```cpp
Scalar lower(hLow, sLow, vLow);
Scalar upper(hHigh, sHigh, vHigh);
inRange(hsvImage, lower, upper, mask);
```

红色跨过 H=179 与 H=0 时，要拆成两段：

```cpp
Mat redA, redB;
inRange(hsvImage, Scalar(hLow, sLow, vLow), Scalar(179, sHigh, vHigh), redA);
inRange(hsvImage, Scalar(0, sLow, vLow), Scalar(hHigh, sHigh, vHigh), redB);
bitwise_or(redA, redB, mask);
```

色块用于直观看单个 HSV 三元组对应的显示色；阈值范围本身是三维 HSV 盒状区域，不是一种唯一颜色。调整 S/V 上下界会改变被保留的颜色种类和亮度范围。
