# 05｜PnP 位姿估计与重投影误差

本章对应 `src/task_05/main.cpp` 和 `include/task_func.hpp`。PnP（Perspective-n-Point）用已知的物体三维点和图像二维观测点，估计物体相对相机的旋转和平移。它回答的是“这个已知几何物体在相机坐标系里的姿态是什么”。

![观测角点与 PnP 重投影点](../assets/figures/task05_pnp_reprojection.png)

## 几何模型

物体点 `X_i` 经物体到相机的刚体变换后成为相机点：

`X_camera = R X_object + t`

理想针孔投影为：

`u ~ K [R|t] X_object`

- `K` 是相机内参矩阵，含焦距和主点。
- `R` 是三维旋转，代码以旋转向量 `rvec` 表示；`Rodrigues(rvec)` 可将其转换为 3×3 旋转矩阵。
- `t` 是平移向量 `tvec`，表示物体坐标原点在相机坐标系中的位置，单位与三维模型点相同。
- 畸变系数描述镜头非理想投影。相机参数必须来自与当前图像分辨率、裁剪和缩放设置相匹配的标定。

`solvePnP` 通过最小化观测像素点与投影点之间的重投影差来求姿态。四个点的平面目标可能存在多解或对噪声敏感；点应覆盖足够范围、几何顺序要对应、相机标定要准确。非平面分布通常更利于约束姿态。

## 主要接口

| 接口 | 作用 | 例子 |
|---|---|---|
| `FileStorage(path, READ)` | 打开 OpenCV YAML/XML 参数文件。 | `FileStorage fs("camera.yml", FileStorage::READ);` |
| `fs["key"] >> mat` | 从配置节点读出相机内参或畸变系数。 | `fs["camera_matrix"] >> K;` |
| `FileStorage::release()` | 关闭配置文件。 | `fs.release();` |
| `solvePnP(object,image,K,dist,rvec,tvec)` | 根据对应的 3D/2D 点、相机矩阵和畸变估计姿态，返回是否成功。 | `bool ok = solvePnP(obj, img, K, dist, rvec, tvec);` |
| `projectPoints(object,rvec,tvec,K,dist,image_out)` | 用估计姿态把 3D 点重新投影到像素坐标。 | `projectPoints(obj, rvec, tvec, K, dist, projected);` |
| `norm(tvec)` | 计算平移向量长度；`tvec[2]` 是相机坐标系的 Z 深度。 | `double range = norm(tvec);` |
| `calculate_rmse(obs,pred)` | 计算像素重投影误差：`sqrt(sum(dx²+dy²)/N)`，单位为像素。 | `double e = calculate_rmse(image_points, projected);` |
| `setMouseCallback` | 通过 `EVENT_LBUTTONDOWN` 获取用户点击像素，用作二维图像观测点。 | `setMouseCallback("view", onClick, &points);` |

## 从代码到结果

1. 从 `object_points.csv` 读出目标模型的 `Point3f`，点的顺序必须和图像中的对应角点一一一致。
2. 从 `reference_corners.csv` 或人工点击得到 `Point2f` 图像点。
3. 从 `camera.yml` 读取内参和畸变系数。
4. 调用 `solvePnP` 得到 `rvec` 和 `tvec`。
5. 调用 `projectPoints` 生成理论像素位置，再与观测点算 RMSE。

当 `tvec.z > 0`，物体原点位于相机前方；这只是必要的基本合理性检查，不足以证明整组姿态都有效。还应检查各三维点变换后的深度、重投影误差、点序和姿态连续性。RMSE 越小表示在当前标定与点对应下像素拟合更好，但它不等同于真实距离误差，也无法排除整体尺度单位错误。

## 易错点

- `object_points[i]` 与 `image_points[i]` 必须代表同一个物理点。
- `tvec` 单位由模型坐标决定。模型用米则平移为米；用毫米则结果为毫米。
- `solvePnP` 的输入点数量和几何配置必须满足选定求解方法的约束；需要更多容错时可指定 `SOLVEPNP_ITERATIVE`、RANSAC 变体等方法，并检查返回状态。
- 示例用 `at<double>` 读取向量；若更换输出精度或自定义类型，应先检查矩阵深度，避免类型不匹配。
- RMSE 代码要求两组数组等长且非空；正式代码需先检查 `N > 0` 和对应点数一致。
