# 03｜轮廓、旋转矩形与灯条筛选

本章对应 `src/task_03/main.cpp`、`src/task_04/main.cpp` 及 `include/lighting_cap.hpp`。前提是输入单通道二值掩模：`findContours` 提取连通边界，几何特征再用于筛除不像灯条的轮廓。

![候选轮廓筛选结果](../assets/figures/task03_contour_filter.png)

## 主要接口

| 接口 | 作用 | 例子 |
|---|---|---|
| `findContours(image, contours, mode, method)` | 从二值图提取边界点集合。`RETR_EXTERNAL` 只取最外层轮廓；`CHAIN_APPROX_SIMPLE` 压缩水平/垂直/斜线上的冗余点。 | `findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);` |
| `contourArea(contour)` | 计算轮廓包围的面积，可过滤小噪声。 | `if (contourArea(c) > 150) ...` |
| `minAreaRect(contour)` | 返回包围轮廓的最小面积旋转矩形 `RotatedRect`，包含 `center`、`size`、`angle`。angle为x轴沿着顺时针旋转第一次碰到矩形边框的角度 | `RotatedRect r = minAreaRect(c);` |
| `RotatedRect::points(points)` | 把旋转矩形四个顶点写入数组。 | `Point2f p[4]; r.points(p);` |
| `norm(p1-p2)` | 计算两个点差的欧氏长度，可从四个顶点中辨认长边。 | `float len = norm(p[0] - p[1]);` |
| `atan2(y,x)` | 从分量计算带象限信息的弧度角。代码用 `atan2(abs(dx),abs(dy))` 得到相对竖直方向偏角，再转度数。 | `float deg = atan2(abs(dx),abs(dy))*180/CV_PI;` |
| `line`、`circle`、`putText` | 在图像上标注框边、点和编号；颜色仍按 BGR。 | `line(dst,p[0],p[1],Scalar(0,255,0),2);` |
| `Mat::clone()` | 在原图副本上绘制，避免改写输入图。 | `Mat result = src.clone();` |

## 筛选逻辑

示例根据面积、长宽比和方向角选择细长近竖直的候选区域。`RotatedRect::size.width` 与 `height` 的大小顺序不稳定，因此先用 `max(width,height)` 作为长边、`min(...)` 作为短边，再计算比值。旋转矩形的角度字段会随宽高和方向表达而变化；代码改为从顶点向量测角，避免直接把 `RotatedRect::angle` 当成竖直偏角。

当前任务使用的阈值约为面积大于 150 像素、长宽比 3.5–15、偏离竖直方向小于 5°。这些是当前样例的经验筛选条件。远近变化或分辨率变化时，应重新估计阈值。

## 自定义接口

- `calculate_draw(src, contours, index_rect)`：逐个轮廓求最小旋转矩形并按条件筛选；返回画框副本，并把合格轮廓的索引放入 `index_rect`。
- `draw_rect(src, points, color)`：用四条 `line` 连接四个点。
- `draw_doub_light(src, light_id, contours)`：按候选索引对求两矩形中心，返回每对灯条的中点，并在图上标出端点。

同名 `calculate_draw` 在 `task_03`、`task_04` 和两个头文件中实现细节并不完全相同；调用某个目标时要确认实际编译进来的定义，避免把多个定义放入同一翻译单元造成重定义。

## 画轮廓、圆点和几何标注

这些绘图函数会直接修改传入的 `Mat`。如果还要保留原图，先 `clone()` 一份，再在副本上画。

### `circle`：画空心圆或实心点

```cpp
Point center(320, 240);                  // 像素坐标，整数型 Point
Scalar green(0, 255, 0);                 // B、G、R 顺序
circle(image, center, 12, green, 2);    // 半径 12px，线宽 2px
circle(image, center, 3, Scalar(0,0,255), FILLED); // 红色实心点
```

参数依次是：目标图像、圆心 `Point`、半径（像素）、颜色 `Scalar(B,G,R)`、线宽、线型、坐标小数位移（通常省略）。线宽设为 `FILLED` 或 `-1` 表示填充圆形。

若检测中心是 `Point2f`，先显式取整，因为 `circle` 的中心参数是整数像素坐标：

```cpp
Point2f detected(320.4f, 241.7f);
Point pixel(cvRound(detected.x), cvRound(detected.y));
circle(image, pixel, 4, Scalar(0, 255, 255), FILLED);
```

### `drawMarker`：画十字、星形等中心标记

```cpp
drawMarker(image, pixel, Scalar(255, 0, 255),
           MARKER_CROSS, 18, 2, LINE_AA);
```

参数依次是图像、中心点、颜色、标记形状、标记边长（像素）、线宽、线型。常用形状有 `MARKER_CROSS`、`MARKER_TILTED_CROSS`、`MARKER_STAR`、`MARKER_DIAMOND` 和 `MARKER_SQUARE`。标记和 `circle(..., FILLED)` 都能标出一个点；带方向的 marker 更容易和圆形检测结果区分。

### `line`、`rectangle` 与 `putText`

```cpp
Point p1(100, 80), p2(220, 160);
line(image, p1, p2, Scalar(255, 0, 0), 2, LINE_AA); // 蓝线
rectangle(image, Rect(40, 50, 180, 100), Scalar(0, 255, 0), 2);
putText(image, "center", pixel + Point(8, -8),
        FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2, LINE_AA);
```

- `line`：两个端点、颜色、线宽、线型；`LINE_AA` 是抗锯齿线。
- `rectangle`：可传 `Rect(x,y,width,height)`，或传两个对角点。
- `putText`：文字基线起点、字体、缩放比例、颜色、线宽和线型。文字位置是基线位置，不是文字框左上角。
- `Scalar(255,0,0)` 是蓝色，`Scalar(0,0,255)` 是红色，因为 OpenCV 彩色图默认通道顺序是 BGR。

### 在轮廓上画出全部边界

```cpp
vector<vector<Point>> contours;
findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
Mat overlay = image.clone();
drawContours(overlay, contours, -1, Scalar(0, 255, 0), 2);
```

`drawContours` 的第三个参数是轮廓索引；`-1` 表示全部轮廓，指定 `i` 则只画第 `i` 个。这里按绿色、2 像素线宽画轮廓。它适合快速可视化边界；如果要画当前代码算出的最小旋转矩形，应使用 `RotatedRect::points` 后用四次 `line` 连点。

## 如何调用本项目的绘图助手

```cpp
vector<int> validIds;
Mat candidates = calculate_draw(image, contours, validIds); // 画出合格灯条
for (int i : validIds) {
    Point2f c = minAreaRect(contours[i]).center;
    Point pixel(cvRound(c.x), cvRound(c.y));
    circle(candidates, pixel, 3, Scalar(0, 255, 255), FILLED);
}
imwrite("candidates.png", candidates);
```

`calculate_draw` 的第三个参数是输出索引容器，调用前应清空旧索引；函数返回绘制后的图像副本。不同练习文件内的同名函数阈值不同，应使用对应目标里的定义。`draw_rect(src, points, color)` 接收四个角点指针，按 0→1→2→3→0 连线；点坐标来自 `Point2f[4]`，需要传给绘图 API 时可按上面的方式取整。
