#pragma once

#include <algorithm>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include "cerateEKF.hpp"

using namespace std;
using namespace cv;

/*
滤波器都需要三个点的数据才能进行初始化
目标一旦关联失效则 计数丢失，丢失到上限则销毁跟踪对象
*/

inline Point2f get_rect_center(const Rect& rect)
{
    Point2f center(rect.x + rect.width/2.0f,rect.y + rect.height/2.0f);
    return center;
}

//基础关联算法，返回bool值判断是不是上一个目标
inline bool is_lasttrack(Point2f& last_center,Point2f& this_center)
{
    //距离限制
    const float mini_norm = 100.0f;
    auto point_norm = abs(norm(last_center - this_center));

    return point_norm <= mini_norm;
}

inline void draw_center_rect(Mat& src,const Point2f& center,const Rect& rect,const Scalar& color,string label)
{
    int width = rect.width;
    int height = rect.height;

    double x = center.x - width/2.0;
    double y = center.y - height/2.0;

    Rect box(cvRound(x),cvRound(y),width,height);

    rectangle(src,box,color,2);

    putText(src,label,Point(box.x,box.y - 5),FONT_HERSHEY_SIMPLEX,0.6,color,2);
}

/*
这些多目标跟踪也可以用于传统视觉几何检测方法
不过要传入的是rect类型的数据
*/

struct Track
{
    // int id;
    //ekf对象 实际使用过程中可以切换5维或6维
    CoordinatedTurnEKF5D ekf;
    // CoordinatedTurnEKF6D ekf;

    //最近的检测框
    Rect rect;

    //目标状态
    int lost_count = 0;
    Point2f last_center;
    vector<Point2f> init_point;
    vector<int> frame_id;

    //距离上次和yolo关联的帧数
    int frames_since_yolo = 0;


};

inline void draw_predict_rect(Mat& src,Track& track)
{
    if (!track.ekf.isInitialized())
    {
        return;
    }

    draw_center_rect(src,track.ekf.getPosition(),track.rect,Scalar(0,0,255),"predict");

}

inline void draw_filtered_rect(Mat& src,Track& track)
{
    if (!track.ekf.isInitialized())
    {
        return;
    }

    draw_center_rect(src,track.ekf.getPosition(),track.rect,Scalar(255,0,0),"");

}


class TrackerManager
{
private:
    vector<Track> tracker;
    double sigma_position;
    double q_accel;
    double q_alpha;
public:
    TrackerManager(double,double,double);
    void update_tracker(vector<Rect>&, int, Mat&, const Mat& observation = Mat());
    vector<pair<int,int>> compare_tracker(const vector<Rect>&);
    vector<pair<int,int>> compare_contour(const vector<Rect>&,const vector<bool>&);
};

inline TrackerManager::TrackerManager(double sigma_position,double q_accel,double q_alpha):sigma_position(sigma_position),q_accel(q_accel),q_alpha(q_alpha)
{
}

// 定义候选匹配结构体进行关联
struct MatchCandidate
{
    int track_index;
    int yolo_index;
    double distance;
};

// 对候选框和已经预测过的轨迹进行匹配，返回下标对。
inline vector<pair<int,int>> TrackerManager::compare_tracker(const vector<Rect>& yolo_rect)
{
    vector<MatchCandidate> candidates;
    vector<pair<int,int>> matches;

    for (int i = 0; i < tracker.size(); i++)
    {
        Point2f last_center;

        if (tracker[i].ekf.isInitialized())
        {
            //更新中心坐标，如果后面有匹配的候选框再用候选框的中心坐标覆盖
            //这里获取的是上一个预测之后的预测中心点
            last_center = tracker[i].ekf.getPosition();
        }
        else
        {
            //如果没有初始化把上次候选框的中心设置为上一个中心点
            last_center = get_rect_center(tracker[i].rect);
        }

        //遍历候选框
        for (int j = 0; j < yolo_rect.size(); j++)
        {
            auto this_center = get_rect_center(yolo_rect[j]);

            //把满足距离条件的候选框加入待筛选容器
            if (is_lasttrack(last_center,this_center))
            {
                double distance = abs(norm(last_center - this_center));
                MatchCandidate temp;

                temp.distance = distance;
                temp.yolo_index = j;
                temp.track_index = i;

                candidates.push_back(temp);
            }
        }
    }

    //按照距离从小到大排序
    sort(candidates.begin(),candidates.end(),[](const MatchCandidate& a,const MatchCandidate& b){return a.distance < b.distance;});

    //记录跟踪容器和候选框的使用情况
    vector<bool> track_use(tracker.size(),false);
    vector<bool> yolo_rect_use(yolo_rect.size(),false);

    //匹配，因为是根据距离大小排列的，距离小的优先匹配
    for (int i = 0; i < candidates.size(); i++)
    {
        int track_index = candidates[i].track_index;
        int yolo_index = candidates[i].yolo_index;

        //保证1对1匹配，没有被使用过
        if (track_use[track_index] || yolo_rect_use[yolo_index])
        {
            continue;
        }

        pair<int,int> match(track_index,yolo_index);
        matches.push_back(match);

        track_use[track_index] = true;
        yolo_rect_use[yolo_index] = true;
    }

    return matches;

}

//传统轮廓分割检测的多目标匹配，只匹配没有被yolo检测到的跟踪器
inline vector<pair<int,int>> TrackerManager::compare_contour(const vector<Rect>& contour_rect,const vector<bool>& tracker_matched)
{
    vector<MatchCandidate> candidates;
    vector<pair<int,int>> mathes;

    const double max_distance = 80.0;

    //辅助最大帧数,记得同步调整最大丢帧数
    const int max_contour_frame = 2;

    for (int i = 0; i < tracker.size(); i++)
    {
        //匹配的不参与关联
        if (tracker_matched[i])
        {
            continue;
        }

        auto& track = tracker[i];

        //未建立滤波的不参与关联
        if (!track.ekf.isInitialized())
        {
            continue;
        }

        //超过yolo丢失帧上限的不参与辅助
        if (track.frames_since_yolo >= max_contour_frame)
        {
            continue;
        }

        auto pre_center = track.ekf.getPosition();
        auto last_area = track.rect.area();

        if (last_area <= 0)
        {
            continue;
        }

        //匹配轮廓
        for (int j = 0; j < contour_rect.size(); j++)
        {
            auto contour_center = get_rect_center(contour_rect[j]);

            double distance = norm(pre_center - contour_center);

            if (distance > max_distance)
            {
                continue;
            }

            auto contour_area = contour_rect[j].area();

            //计算面积比
            const double area_ratio = static_cast<double>(contour_area) / last_area;

            //面积比在0.25倍到2.5倍之间
            if (area_ratio < 0.25 || area_ratio > 2.5)
            {
                continue;
            }

            //加入候选匹配列表
            MatchCandidate temp;

            temp.track_index = i;
            temp.yolo_index = j;
            temp.distance = distance;

            candidates.push_back(temp);
        }
    }

    sort(candidates.begin(),candidates.end(),[](const MatchCandidate& a,const MatchCandidate& b){return a.distance < b.distance;});

    vector<bool> contour_use(contour_rect.size(),false);
    vector<bool> track_used = tracker_matched;

    for (int i = 0; i < candidates.size(); i++)
    {
        int track_index = candidates[i].track_index;
        int contour_index = candidates[i].yolo_index;

        if (track_used[track_index] || contour_use[contour_index])
        {
            continue;
        }

        pair<int,int> comp(track_index,contour_index);
        mathes.push_back(comp);

        track_used[track_index] = true;
        contour_use[contour_index] = true;
    }

    return mathes;

}

inline void draw_next_predict_rect(Mat& src,Track& track)
{
    if (!track.ekf.isInitialized())
    {
        return;
    }

    auto next_ekf = track.ekf;
    next_ekf.predict();

    draw_center_rect(src,next_ekf.getPosition(),track.rect,Scalar(0,0,255),"predict");

}

// 传入图像，包含四个点的数组,传入线条颜色
inline void draw_rect(Mat &src, Point2f *point, Scalar color)
{
    for (int k = 0; k < 4; k++)
    {
        line(src, point[k], point[(k + 1) % 4], color, 2);
    }
}

//hsv参数
inline Scalar lower_red_1(0,50,80);
inline Scalar higher_red_1(25,255,255);

inline Scalar lower_red_2(170,50,80);
inline Scalar higher_red_2(179,255,255);

inline vector<Rect> get_rect_withhsv(const Mat& src)
{
    Mat mask1,mask2,mask,hsv_src;
    cvtColor(src,hsv_src,COLOR_BGR2HSV);
    inRange(hsv_src,lower_red_1,higher_red_1,mask1);
    inRange(hsv_src,lower_red_2,higher_red_2,mask2);

    bitwise_or(mask1,mask2,mask);
    // imshow("111",mask);
    // waitKey(0);

    vector<vector<Point>> contours;
    findContours(mask,contours,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
    contours.erase(remove_if(contours.begin(),contours.end(),[](const vector<Point>& points){return contourArea(points) < 2000;}),contours.end());

    vector<Rect> output;

    for (int i = 0; i < contours.size(); i++)
    {
        auto rect = minAreaRect(contours[i]);

        // Point2f point[4];
        // rect.points(point);

        // draw_rect(src,point,Scalar(0,255,0));

        auto output_rect = rect.boundingRect();
        output.push_back(output_rect);
    }

    return output;

}

//如果使用纯几何算法，往yolo_rect里面传入上面方法的结果，然后禁用轮廓辅助功能
inline void TrackerManager::update_tracker(vector<Rect>& yolo_rect, int frame_id, Mat& frame, const Mat& observation)
{
    //初始化
    if (tracker.size() == 0)
    {
        for (int i = 0; i < yolo_rect.size(); i++)
        {
            Track temp;
            temp.rect = yolo_rect[i];
            CoordinatedTurnEKF5D ekf(sigma_position,q_accel,q_alpha);
            temp.ekf = ekf;

            Point2f center = get_rect_center(yolo_rect[i]);
            temp.last_center = center;
            temp.init_point.push_back(center);
            temp.frame_id.push_back(frame_id);

            tracker.push_back(temp);
        }

        return;
    }

    //执行预测
    for (int i = 0; i < tracker.size(); i++)
    {
        if (tracker[i].ekf.isInitialized())
        {
            tracker[i].ekf.predict();

            // draw_predict_rect(frame,tracker[i]);
        }
    }

    auto matchs = compare_tracker(yolo_rect);

    vector<bool> track_matched(tracker.size(),false);
    vector<bool> yolo_matched(yolo_rect.size(),false);

    //记录被yolo关联的track
    vector<bool> track_yolo_matched(tracker.size(),false);

    //处理匹配结果
    for (int i = 0; i < matchs.size(); i++)
    {
        auto track_index = matchs[i].first;
        auto yolo_index = matchs[i].second;

        auto& track = tracker[track_index];
        auto rect = yolo_rect[yolo_index];

        auto center = get_rect_center(rect);

        track_matched[track_index] = true;
        yolo_matched[yolo_index] = true;

        track_yolo_matched[track_index] = true;

        //更新参数
        track.rect = rect;
        track.lost_count = 0;
        track.last_center = center;
        track.frames_since_yolo = 0;

        //执行修正或者初始化
        if (track.ekf.isInitialized())
        {
            CoordinatedTurnEKF5D::Measurement z(center.x,center.y);

            track.ekf.correct(z);

            // cout << "111" << endl;
            draw_filtered_rect(frame,track);
        }
        else
        {
            track.init_point.push_back(center);
            track.frame_id.push_back(frame_id);

            if (track.init_point.size() == 3)
            {
                double delta_t01 = track.frame_id[1] - track.frame_id[0];
                double delta_t12 = track.frame_id[2] - track.frame_id[1];

                track.ekf.initialize(track.init_point[0],track.init_point[1],track.init_point[2],delta_t01,delta_t12);
            }
        }

    }

    //处理轮廓辅助,记得更改hsv参数
    auto contour_rect = get_rect_withhsv(observation.empty() ? frame : observation);

    auto conotur_matches = compare_contour(contour_rect,track_matched);

    //处理结果
    for (int i = 0; i < conotur_matches.size(); i++)
    {
        int track_index = conotur_matches[i].first;
        int contour_index = conotur_matches[i].second;

        auto& track = tracker[track_index];
        auto rect = contour_rect[contour_index];
        auto center = get_rect_center(rect);

        //修正ekf,由于在匹配时只匹配预测初始化过后的，这里不用进行初始化检测
        CoordinatedTurnEKF5D::Measurement z(center.x,center.y);

        track.ekf.correct(z);

        track_matched[track_index] = true;

        track.lost_count = 0;

        //轮廓辅助只画滤波和预测
        draw_filtered_rect(frame,track);
    }


    //处理没有匹配的对象

    //没有匹配的tracker记录丢失
    for (int i = 0; i < tracker.size(); i++)
    {
        //匹配失败
        if (!track_matched[i])
        {
            tracker[i].lost_count++;
        }
        else
        {
            tracker[i].lost_count = 0;
        }

        //只有轮廓匹配
        if (!track_yolo_matched[i])
        {
            tracker[i].frames_since_yolo++;
        }


    }

    for (int i = 0; i < yolo_rect.size(); i++)
    {
        if (!yolo_matched[i])
        {
            //创建
            Track temp;
            temp.rect = yolo_rect[i];
            CoordinatedTurnEKF5D ekf(sigma_position,q_accel,q_alpha);
            temp.ekf = ekf;

            Point2f center = get_rect_center(yolo_rect[i]);
            temp.last_center = center;
            temp.init_point.push_back(center);
            temp.frame_id.push_back(frame_id);

            tracker.push_back(temp);
        }

    }

    // 连续两帧未关联到任何观测时，删除该轨迹
    tracker.erase(remove_if(tracker.begin(),tracker.end(),[](const Track& track){return track.lost_count > 1;}),tracker.end());

    for (int i = 0; i < tracker.size(); i++)
    {
        if (tracker[i].ekf.isInitialized())
        {
            draw_next_predict_rect(frame,tracker[i]);
        }

    }


}

//yolo rect颜色筛选

//hsv参数
inline Scalar yolo_lower_red_1(0,50,80);
inline Scalar yolo_higher_red_1(25,255,255);

inline Scalar yolo_lower_red_2(170,50,80);
inline Scalar yolo_higher_red_2(179,255,255);

//ration是至少有多少面积的像素符合这个颜色
inline bool rect_has_hsv(Mat& hsv,const Rect& rect,double min_ratio)
{
    //防止rect越界
    Rect image_rect(0,0,hsv.cols,hsv.rows);
    Rect roi_rect = rect & image_rect; //求交集

    if (roi_rect.empty())
    {
        return false;
    }

    auto roi = hsv(roi_rect);

    Mat mask1,mask2,mask;
    inRange(roi,yolo_lower_red_1,yolo_higher_red_1,mask1);
    inRange(roi,yolo_lower_red_2,yolo_higher_red_2,mask2);
    bitwise_or(mask1,mask2,mask);

    //统计非0像素数量
    double valid_pixels = countNonZero(mask);

    if (valid_pixels == 0)
    {
        return false;
    }

    auto ratio = valid_pixels / roi_rect.area();

    return ratio >= min_ratio;

}

//过滤器
inline void filter_rect_by_hsv(const Mat& src,vector<Rect>& rects,double min_ratio)
{
    Mat hsv;
    cvtColor(src,hsv,COLOR_BGR2HSV);

    //删除不符合的框,这里使用 & 是因为要访问外部参数
    rects.erase(remove_if(rects.begin(),rects.end(),[&](const Rect& rect){return !rect_has_hsv(hsv,rect,min_ratio);}),rects.end());
}
