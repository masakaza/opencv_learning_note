#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <fstream>
#include "lighting_cap.hpp"

using namespace std;
using namespace cv;

int main()
{
    VideoCapture cap("../image/task06_a.avi");
    auto fps = cap.get(CAP_PROP_FPS);
    auto width = cap.get(CAP_PROP_FRAME_WIDTH);
    auto height = cap.get(CAP_PROP_FRAME_HEIGHT);
    VideoWriter writer("../image_output/task06/task_06_a_result.avi",VideoWriter::fourcc('M','J','P','G'),fps,Size(width,height));

    VideoCapture cap1("../image/task06_b.avi");
    fps = cap1.get(CAP_PROP_FPS);
    width = cap1.get(CAP_PROP_FRAME_WIDTH);
    height = cap1.get(CAP_PROP_FRAME_HEIGHT);
    VideoWriter writer_1("../image_output/task06/task_06_b_result.avi",VideoWriter::fourcc('M','J','P','G'),fps,Size(width,height));

    Mat frame;
    vector<vector<Point>> contours;  //轮廓集合
    vector<int> rect_index;   //合格的候选框
    vector<pair<int,int>> light_index; //配对灯条id
    vector<pair<Point2f,int>> track_center; //中心坐标
    int frame_count = 0;
    TickMeter timer;

    ofstream csv("../image_output/task06/task06_a.csv");
    ofstream csv1("../image_output/task06/task06_b.csv");
    csv << "frame_id,candidate_count,valid,center_x,center_y,processing_time" << endl;
    csv1 << "frame_id,candidate_count,valid,center_x,center_y,processing_time" << endl;
    while (cap.read(frame))
    {   
        timer.reset();
        timer.start();
        Mat hsv_frame = frame.clone();
        cvtColor(frame,hsv_frame,COLOR_BGR2HSV);
        auto mask = get_blue_mask(hsv_frame);
        findContours(mask,contours,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
        Mat result;
        result = calculate_draw(frame,contours,rect_index); //合格的候选轮廓id在rect_index中
        light_index = calculate_light(contours,rect_index);
        auto v_center = draw_doub_light(frame,light_index,contours); // 返回中心坐标点
        timer.stop();
        auto time = timer.getTimeMilli();
        
        csv << frame_count << "," << rect_index.size() << ",";
        if (light_index.size())
        {
            csv << 1 << ",";
            for (int i = 0; i < v_center.size(); i++)
            {
                csv << v_center[i].x << "/";
            }
            csv << ",";
            for (int i = 0; i < v_center.size(); i++)
            {
                csv << v_center[i].y << "/";
            }
        }
        else
        {
            csv << 0 << ",,";
        }
        csv << "," << time << endl;

        writer.write(frame);

        frame_count++;
        rect_index.clear();
    }

    //换b视频进行基础关联处理
    frame_count = 0;
    while (cap1.read(frame))
    {   
        timer.reset();
        timer.start();
        Mat hsv_frame = frame.clone();
        cvtColor(frame,hsv_frame,COLOR_BGR2HSV);
        auto mask = get_blue_mask(hsv_frame);
        findContours(mask,contours,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
        Mat result;
        result = calculate_draw(frame,contours,rect_index); //合格的候选轮廓id在rect_index中
        light_index = calculate_light(contours,rect_index);
        auto v_center = draw_doub_light(frame,light_index,contours); // 返回中心坐标点
        traker(v_center,track_center);  //检查目标相关性
        timer.stop();
        auto time = timer.getTimeMilli();
        
        csv1 << frame_count << "," << rect_index.size() << ",";
        if (light_index.size())
        {
            csv1 << 1 << ",";
            for (int i = 0; i < v_center.size(); i++)
            {
                csv1 << v_center[i].x << "/";
            }
            csv1 << ",";
            for (int i = 0; i < v_center.size(); i++)
            {
                csv1 << v_center[i].y << "/";
            }
        }
        else
        {
            csv1 << 0 << ",,";
        }
        csv1 << "," << time << endl;

        writer_1.write(frame);

        frame_count++;
        rect_index.clear();
    }
    
    csv.close();
    csv1.close();
    writer.release();
    writer_1.release();
    cap.release();
    cap1.release();

    return 0;
}
