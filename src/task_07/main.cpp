#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <fstream>

using namespace std;
using namespace cv;

//篮球HSV范围
Scalar lower_bask1(0,75,55);
Scalar upper_bask1(28,255,255);

//记录有效帧计算标准差
vector<Point> cap_center;
vector<Point> af_filter_center;

/*
过程噪声强度q：
强度过小时，模型教保守，容易出现滞后于篮球运动轨迹的表现
强度过大时，模型会偏激，容易出现预测框在目标附近的轨迹出现抖动

增大过程噪声强度q时候，滤波器会更快跟上变化，但是预测抖动严重功能几乎失效

测量噪声方差r:
强度过小时，滤波框会靠近实际的测量框，会导致预测框轨迹严重抖动,
强度过大时，滤波框会更偏向于预测框，会导致严重的滞后

增大测量噪声方差r的时候，滤波器运动会很平滑(变化率变小),预测框滞后，但是几乎跟不上实际运动轨迹，导致滤波功能无意义

在本程序中，物体运动速度变化快，运动轨迹会突变
所以测量噪声方差不宜过大，噪声强度q不宜过小
要权衡过程噪声带来的滤波器跟踪效果 和 测量噪声带来的滤波器运动轨迹平滑效果

q=1.0,r=1.0 时候，滤波器的变化跟踪良好，但是预测功能偏差大

调小过程噪声q，减少预测框抖动，把滤波器对检测框的变化程度控制在合理误差范围内
增大测量噪声方差r,让滤波器运动更平滑，补偿过程噪声减小导致的对跟踪目标突然变向的情况下跟不上变化所造成的误差，同时能稳定预测框，使预测框的预测控制在误差范围内
最终确定参数为
q=0.5
r=2.3

*/

/*
卡尔曼滤波主要用于平滑目标运动轨迹，同时在目标轨迹检测失效时根据目标之前的运动状态预测目标当前的位置
*/

//卡尔曼滤波匀速模型建立
void init_kalman_model(KalmanFilter& kf)
{
    kf.init(4,2,0,CV_32F); //4个预测变量 2个测量变量 0个控制变量 使用32位精度浮点数

    //常数定义
    float delta_t = 1.0f;
    float q = 0.5f; //过程噪声强度    强度越大代表运动模型越偏激容易出现波动，强度越小代表运动模型越保守但容易跟不上运动目标
    float r = 2.3f; //测量噪声方差  表示测量结果的不确定性，R越大表示测量结果的正确性权重越低，也就是模型觉得测量结果不准确，R越小表示模型会越相信测量结果

    //状态矩阵
    //矩阵乘法，每一行出来的结果对应预测计算的(x,y,vx,vy)     每一列对应当前的(x,y,vx,vy)其中填写的参数对应相乘系数
    kf.transitionMatrix = (Mat_<float>(4,4) <<
    1,0,delta_t,0,      // x(k+1) = xk + Vx*delta_t 
    0,1,0,delta_t,      // y(k+1) = yK + Vy*delta_t
    0,0,1,0,            // Vx(k+1) = Vx 
    0,0,0,1             // Vy(k+1) = Vy
);  //这个参数矩阵推测出来的是下一帧目标的预测位置和状态是以什么方法计算的 

    //测量矩阵  测量得到的数值跟这个矩阵进行乘法运算后得出滤波器内部所用的测量参数矩阵含义和之前一样
    //通过这个矩阵定义滤波器要怎么获取测量参数
    kf.measurementMatrix = (Mat_<float>(2,4) << 
    1,0,0,0,   //x = xk
    0,1,0,0    //y = yk
);

    //过程噪声  表示物体在运动过程中的不确定性，会影响模型对预测参数的不确定性强度
    //如果不确定性过大则会使预测结果更加偏离计算出的预测结果
    //因为篮球是不确定性运动，这里使用白噪声加速度模型 
    kf.processNoiseCov = q*(Mat_<float>(4,4) << 
    delta_t*delta_t*delta_t*delta_t/4,0,delta_t*delta_t*delta_t/2,0,
    0,delta_t*delta_t*delta_t*delta_t/4,0,delta_t*delta_t*delta_t/2,
    delta_t*delta_t*delta_t/2,0,delta_t*delta_t,0,
    0,delta_t*delta_t*delta_t/2,0,delta_t*delta_t
);

    //测量噪声矩阵  表示对测量结果的置信程度,R越小信任程度越大
    //因为输入的是2维度的测量参数（x,y）因此参数对应跟矩阵乘法一样
    kf.measurementNoiseCov = (Mat_<float>(2,2) << 
    r,0,
    0,r
);

}


int getmax_area_contours(vector<vector<Point>>& contours)
{
    double area = 0;
    int index = 0;
    for (int i = 0; i < contours.size(); i++)
    {
        auto con_area = contourArea(contours[i]);
        if (con_area > area)
        {
            area = con_area;
            index = i;
        }
        
    }

    return index;
    
}

vector<Point2f> init_point;
pair<Point2f,Point2f> basketball_analyse(bool vaild,int index,vector<vector<Point>>& contours_point,Mat& src,KalmanFilter& kf)
{
    //全部选框经过卡尔曼滤波预测目标筛查
    //如果有多个圆框的圆心距离教近，取平均值获得一个新的共同的圆
    pair<Point2f,Point2f> output_center; //1: cap or predit  2: filter
    static bool has_init = 0;
    Point2f center;
    float radius;
    static float last_radius = 0;

    if (vaild)
    {
        minEnclosingCircle(contours_point[index],center,radius);
        circle(src,center,radius,Scalar(0,255,0),2);
        last_radius = radius;
        if (init_point.size() < 2)
        {
            init_point.push_back(center);
        }
        output_center.first = center;
    }

    if (init_point.size() == 2 && !has_init)
    {
        auto v_x = init_point.at(1).x - init_point.at(0).x;
        auto v_y = init_point.at(1).y - init_point.at(0).y;

        kf.statePost = (Mat_<float>(4,1) << 
        center.x,
        center.y,
        v_x,
        v_y
    );
        has_init = 1;
    }
    

    //卡尔曼滤波
    if (has_init)
    {
        //预测
        auto prediction = kf.predict();
        Point2f filterCenter; //滤波后中心
        Point2f predictCenter;

        predictCenter.x = prediction.at<float>(0,0);
        predictCenter.y = prediction.at<float>(1,0);

        if (!vaild)
        {
            //纯predit
            circle(src,predictCenter,last_radius,Scalar(0,0,255),2);
            putText(src,"predict",predictCenter,FONT_HERSHEY_SIMPLEX,0.6,Scalar(0,0,255),2);
            output_center.first = predictCenter;
            return output_center;
        }

        //构建矩阵的时候不能用auto类型
        Mat measurement = (Mat_<float>(2,1) <<   //2行一列
        center.x,
        center.y
        );           //测量数据矩阵

        //滤波修正结果
        auto corrected = kf.correct(measurement);
        filterCenter.x = corrected.at<float>(0,0);
        filterCenter.y = corrected.at<float>(1,0);
        output_center.second = filterCenter;

        cap_center.push_back(center);
        af_filter_center.push_back(filterCenter);

        //因为correct输出的矩阵是(x,y,vx,vy)4行一列的矩阵,用下标获取对应的数据

        //画出滤波后和预测的结果
        circle(src,predictCenter,radius,Scalar(0,0,255),2);
        putText(src,"predict",predictCenter,FONT_HERSHEY_SIMPLEX,0.6,Scalar(0,0,255),2);
        circle(src,filterCenter,radius,Scalar(255,0,0),2);
    }

    return output_center;
    
}

int main()
{
    Mat frame,mask,hsv_frame;
    VideoCapture cap("../image/task07.avi");
    auto fps = cap.get(CAP_PROP_FPS);
    auto width = cap.get(CAP_PROP_FRAME_WIDTH);
    auto height = cap.get(CAP_PROP_FRAME_HEIGHT);
    VideoWriter writer("../image_output/task07/kun.avi",VideoWriter::fourcc('M','J','P','G'),fps,Size(width,height));

    vector<vector<Point>> contours;
    bool vaild = 0;
    int fps_count = 0;    
    KalmanFilter kf;
    ofstream cap_csv("../image_output/task07/cap.csv");
    ofstream fli_csv("../image_output/task07/filter.csv");
    cap_csv << "frame,cx,cy,valid,source" << endl;
    fli_csv << "frame,cx,cy,valid,source" << endl;

    //初始化滤波器
    init_kalman_model(kf);

    while (cap.read(frame))
    {
        cap_csv << fps_count << ",";
        fli_csv << fps_count << ",";
        cvtColor(frame,hsv_frame,COLOR_BGR2HSV);
        inRange(hsv_frame,lower_bask1,upper_bask1,mask);
        findContours(mask,contours,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE); //找寻轮廓
        auto max_contours_index = getmax_area_contours(contours);
        // cout << contourArea(contours[max_contours_index]) << endl;
        // //篮球检测要求 面积大于300

        if (contours.size() != 0)
        {
            vaild = contourArea(contours[max_contours_index]) > 3000;
        }
        else
        {
            vaild = 0;
        }
        auto log_center = basketball_analyse(vaild,max_contours_index,contours,frame,kf);
        
        Point2f point(0,0);

        if (vaild)
        {
            cap_csv << log_center.first.x << "," << log_center.first.y << "," << vaild << "," << "cap_point" << endl;
            //filter
            fli_csv << log_center.second.x << "," << log_center.second.y << "," << vaild << "," << "filter_point" << endl;
        }
        else
        {
            //no filter
            if (log_center.first == point)
            {
                cap_csv << "NaN,NaN," << vaild << "," << "no_cap_point" << endl;
            }
            else
            {
                cap_csv << log_center.first.x << "," << log_center.first.y << "," << vaild << "," << "predict_point" << endl;
            }
            fli_csv << "NaN,NaN," << vaild << "," << "no_cap_point" << endl;
        }
        
        


        fps_count++;
        // imshow("111",frame);
        // waitKey(30);
        writer.write(frame);
    }

    //计算标准差
    vector<double> cap_s_delta;
    vector<double> fli_s_delta;
    double total_cap_s = 0;
    double total_fli_s = 0;
    for (int i = 1; i < cap_center.size(); i++)
    {
        //计算相邻位移
        auto cd = abs(norm(cap_center[i] - cap_center[i-1]));
        auto fd = abs(norm(af_filter_center[i] - af_filter_center[i-1]));

        cap_s_delta.push_back(cd);
        fli_s_delta.push_back(fd);
        total_cap_s += cd;
        total_fli_s += fd;
    }

    auto avg_cap_s = total_cap_s/cap_s_delta.size();
    auto avg_fli_s = total_fli_s/fli_s_delta.size();

    total_cap_s = 0;
    total_fli_s = 0;

    for (int i = 0; i < cap_s_delta.size(); i++)
    {
        auto cap_s_temp = cap_s_delta[i] - avg_cap_s;
        auto fli_s_temp = fli_s_delta[i] - avg_fli_s;

        total_cap_s += cap_s_temp*cap_s_temp;
        total_fli_s += fli_s_temp*fli_s_temp;
    }
    
    auto cap_variance = sqrt(total_cap_s/cap_s_delta.size());
    auto fli_variance = sqrt(total_fli_s/fli_s_delta.size());
    /*
    测量位移标准差43.4793
    滤波位移标准差35.245
     
    增大测量噪声方差的时候，滤波位移标准差会变小，可以初步说明，测量噪声主要来自物体检测时候的测量抖动
    */
    
    cout << "测量位移标准差" << cap_variance << endl;
    cout << "滤波位移标准差" << fli_variance << endl;
    


    cap_csv.close();
    fli_csv.close();
    cap.release();
    writer.release();

    return 0;
}