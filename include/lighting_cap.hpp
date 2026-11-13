#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
using namespace std;
using namespace cv;

/*
根据候选框输出粗略判断筛选条件为：
角度 0-5
面积 >150
长宽比 3.5-15

装甲板灯条的相对关系为：
task04_a.png
Rl: 0.782609   //长度比
delta angle: 0  //角度差
RD: 1.62968   //灯条距离和灯条平均长度比

task04_b.png
Rl: 0.932203
delta angle: 0
RD: 2.22835

最终判断范围
RL 0.6~1
dalta angle 0~1
RD: 1.5-2.5

装甲板丢失条件
中心坐标欧几里得距离上限为 80px
连续丢失最大帧数为 10fps


*/

Mat get_blue_mask(Mat &src)
{
    // blue
    Scalar lower_blue(90, 80, 50);
    Scalar higher_blue(130, 255, 255);

    Mat mask;
    inRange(src, lower_blue, higher_blue, mask);

    return mask;
}

Mat get_red_mask(Mat &src)
{
    // red
    Scalar lower_red1(0, 80, 70);
    Scalar higher_red1(10, 255, 255);

    Scalar lower_red2(170, 80, 70);
    Scalar higher_red2(179, 255, 255);

    Mat mask_1, mask_2, mask;

    inRange(src, lower_red1, higher_red1, mask_1);
    inRange(src, lower_red2, higher_red2, mask_2);
    bitwise_or(mask_1, mask_2, mask);

    return mask;
}

// 传入轮廓获取符合条件的候选框,返回的只含有符合条件的候选框图片矩阵，传入的图片矩阵将画满候选框,传入一个容器记录所有合格候选框id
Mat calculate_draw(Mat &src, vector<vector<Point>> &contours, vector<int> &index_rect)
{
    const float half = 180.0;
    Point2f poin[4];
    Mat result = src.clone();

    for (int i = 0; i < contours.size(); i++)
    {
        // cout << "id: " << i << " area: " << contourArea(contours[i]) << endl;
        auto rect = minAreaRect(contours[i]); // 根据轮廓 获取最小旋转矩形(最小能包裹轮廓的矩形)
        auto center = rect.center;

        // 统一长短定义
        auto long_side = max(rect.size.width, rect.size.height);
        auto short_side = min(rect.size.width, rect.size.height);
        auto l_s = long_side / short_side;
        // cout << "chang / kuan: " << l_s << endl;

        int index = 0;
        rect.points(poin);
        for (int j = 1; j < 4; j++)
        {
            if (abs(norm(poin[0] - poin[j]) - long_side) < 0.01f) // norm根据坐标差计算绝对距离,浮点数计算可能会有误差所以用一个允许的误差范围来判断
            {
                index = j;
                break;
            }
        }

        auto point1 = poin[0];
        auto point2 = poin[index];

        // 计算角度  因为是竖直方向夹角所以是 dx/dy
        auto px = point1.x - point2.x;
        auto py = point1.y - point2.y;

        auto angle = atan2(abs(px), abs(py)) * half / CV_PI; // 通过arctan 求出弧度进而求出角度

        // cout << "px: " << px << " py: " << py << endl;
        // cout << angle << endl;

        // 通过单位向量计算上下端点
        //  长边向量(px,py)
        if (py < 0)
        {
            px = -px;
            py = -py;
        }
        // 确保向量指向图像下方
        Point2f buttom_point(center.x + (px / 2.0f), center.y + (py / 2.0f));
        Point2f top_point(center.x - (px / 2.0f), center.y - (py / 2.0f));

        // 判断条件
        if (angle < 5.0f && contourArea(contours[i]) > 150.0f && l_s > 3.5f && l_s < 16.5f)
        {
            index_rect.push_back(i);
            // 在原图上画出矩形
            for (int k = 0; k < 4; k++)
            {
                line(result, poin[k], poin[(k + 1) % 4], Scalar(0, 255, 0), 2);
            }
            putText(result, to_string(i), poin[0], FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 255), 2);
            circle(result, buttom_point, 4, Scalar(255, 255, 255), -1);
            circle(result, top_point, 4, Scalar(255, 255, 255), -1);
        }
    }

    return result;
}

// 计算匹配灯条,返回包含匹配灯条的id对组
vector<pair<int, int>> calculate_light(vector<vector<Point>> &contours, vector<int> &rect_index)
{
    const float half = 180.0;
    vector<pair<int, int>> light_contour;
    Point2f poin[4];
    for (int i = 0; i < rect_index.size(); i++)
    {
        for (int j = i + 1; j < rect_index.size(); j++)
        {
            auto rect1 = minAreaRect(contours[rect_index[i]]);
            auto rect2 = minAreaRect(contours[rect_index[j]]);

            // 统一长度
            auto long_side1 = max(rect1.size.width, rect1.size.height);
            auto long_side2 = max(rect2.size.width, rect2.size.height);

            auto RL = long_side1 / long_side2;
            if (RL > 1.0f)
            {
                RL = long_side2 / long_side1;
            }

            // cout << "Rl: " << RL << endl; // 灯条长度的比值

            // 计算角度
            rect1.points(poin);
            int index = 0;
            for (int k = 1; k < 4; k++)
            {
                if (abs(norm(poin[0] - poin[k]) - long_side1) < 0.01f) // norm根据坐标差计算绝对距离,浮点数计算可能会有误差所以用一个允许的误差范围来判断
                {
                    index = k;
                    break;
                }
            }

            auto point1 = poin[0];
            auto point2 = poin[index];

            // 计算角度  因为是竖直方向夹角所以是 dx/dy
            auto px = point1.x - point2.x;
            auto py = point1.y - point2.y;

            auto angle1 = atan2(abs(px), abs(py)) * half / CV_PI;

            rect2.points(poin);
            for (int k = 1; k < 4; k++)
            {
                if (abs(norm(poin[0] - poin[k]) - long_side2) < 0.01f) // norm根据坐标差计算绝对距离,浮点数计算可能会有误差所以用一个允许的误差范围来判断
                {
                    index = k;
                    break;
                }
            }
            point1 = poin[0];
            point2 = poin[index];

            // 计算角度  因为是竖直方向夹角所以是 dx/dy
            px = point1.x - point2.x;
            py = point1.y - point2.y;

            auto angle2 = atan2(abs(px), abs(py)) * half / CV_PI;

            auto delta_angle = abs(angle1 - angle2);
            // cout << "delta angle: " << delta_angle << endl;

            auto avg_side = (long_side1 + long_side2) / 2;
            auto RD = abs(norm(rect1.center - rect2.center)) / avg_side; // 计算中心间距和灯条长度的比值(装甲板长宽比)
            // cout << "RD: " << RD << endl;

            if (RL > 0.8f && RL <= 1.0f && delta_angle <= 1.0f && RD > 1.5f && RD < 2.4f)
            {
                pair<int, int> con_index(rect_index[i], rect_index[j]);
                light_contour.push_back(con_index);
            }
        }
    }

    return light_contour;
}

//画出配对好的装甲板灯条，返回装甲板中心坐标
vector<Point2f> draw_doub_light(Mat &src, vector<pair<int, int>> &light_id, vector<vector<Point>> &contours)
{
    Point2f poin[4];
    vector<RotatedRect> v_rect;
    vector<Point2f> center_point;
    vector<Point2f> corn_point;
    for (int i = 0; i < light_id.size(); i++)
    {
        v_rect.clear();
        corn_point.clear();
        auto rect1 = minAreaRect(contours[light_id[i].first]);
        auto rect2 = minAreaRect(contours[light_id[i].second]);
        v_rect.push_back(rect1);
        v_rect.push_back(rect2);

        auto center1 = rect1.center;
        auto center2 = rect2.center;

        auto center = Point2f((center1.x + center2.x)/2,(center1.y + center2.y)/2);
        center_point.push_back(center);



        for (int j = 0; j < v_rect.size(); j++)
        {
            auto rect = v_rect[j];
            auto center = rect.center;

            // 统一长短定义
            auto long_side = max(rect.size.width, rect.size.height);

            int index = 0;
            rect.points(poin);
            for (int k = 1; k < 4; k++)
            {
                if (abs(norm(poin[0] - poin[k]) - long_side) < 0.01f) // norm根据坐标差计算绝对距离,浮点数计算可能会有误差所以用一个允许的误差范围来判断
                {
                    index = k;
                    break;
                }
            }

            auto point1 = poin[0];
            auto point2 = poin[index];

            // 计算角度  因为是竖直方向夹角所以是 dx/dy
            auto px = point1.x - point2.x;
            auto py = point1.y - point2.y;

            // 通过单位向量计算上下端点
            //  长边向量(px,py)
            if (py < 0)
            {
                px = -px;
                py = -py;
            }
            // 确保向量指向图像下方
            Point2f buttom_point(center.x + (px / 2.0f), center.y + (py / 2.0f));
            Point2f top_point(center.x - (px / 2.0f), center.y - (py / 2.0f));
            corn_point.push_back(buttom_point);
            corn_point.push_back(top_point);

            // 绘制矩形
            for (int k = 0; k < 4; k++)
            {
                line(src, poin[k], poin[(k + 1) % 4], Scalar(0, 255, 0), 2);
            }
            // putText(src, to_string(i), poin[0], FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 255), 2);
            circle(src, buttom_point, 4, Scalar(255, 255, 255), -1);
            circle(src, top_point, 4, Scalar(255, 255, 255), -1);
        }

        auto temp = corn_point.at(3);
        corn_point.at(3) = corn_point.at(2);
        corn_point.at(2) = temp;

        for (int j = 0; j < 4; j++)
        {
            line(src, corn_point[j], corn_point[(j + 1) % 4], Scalar(0, 255, 255), 2);
        }

    }
    return center_point;
}

void traker(vector<Point2f>& center_point,vector<pair<Point2f,int>>& track_point)
{
    if (track_point.size() == 0)
    {
        //初始化
        for (int i = 0; i < center_point.size(); i++)
        {
            pair<Point2f,int> point_tracker(center_point[i],0);
            track_point.push_back(point_tracker);
        }
        
    }
    
    if (center_point.size() > track_point.size())
    {
        //挑选最不相关的中心作为新的目标

        int index = 0;
        for (int i = 0; i < center_point.size(); i++)
        {
            double abs_norm = 0;
            for (int j = 0; j < track_point.size(); j++)
            {
                if (abs(norm(center_point[i]-track_point[j].first)) > abs_norm)
                {
                    abs_norm = abs(norm(center_point[i]-track_point[j].first));
                    index = i;
                }
                
            }
            
        }
        
        pair<Point2f,int> point_tracker(center_point[index],0);
        track_point.push_back(point_tracker);

    }
    

    for (int i = 0; i < track_point.size(); i++)
    {
        if (center_point.size() == 0)
        {
            track_point[i].second++;
        }
        

        int index = -1;
        for (int j = 0; j < center_point.size(); j++)
        {
            auto abs_norm = abs(norm(center_point[j] - track_point[i].first));
            if (abs_norm <= 100)
            {
                index = j;
                break;
            }
            
        }

        if (index != -1)
        {
            if (track_point[i].second > 10)
            {
                cout << "target refresh" << endl;
            }

            track_point[i].first = center_point[index];
            track_point[i].second = 0;
            
        }
        else
        {
            //匹配失败,如果超过10帧都失败就输出
            track_point[i].second++;
            if (track_point[i].second > 10)
            {
                cout << "target_lost id: " << i << endl;
            }
        }
        
        
    }
    
}