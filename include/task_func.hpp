#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
using namespace std;
using namespace cv;

/*
如果检测到多个目标
优先检查关联性

卡尔曼滤波多目标问题（我觉得应该不会出到这么难）
如果有多目标要进行卡尔曼滤波，应该先用关联问题来对目标分类
每个新出现的目标对应唯一的target_obj
每帧都挑选距离tarcker_point最近的中心作为该target_obj对应的目标
如果剩下的目标没有对应的target_obj则新建一个并初始化模型（检测target_obj的数量和画面中点的数量是否相等）
如果少来就新建，多了就摧毁最不相关的那个

当目标检测不到时候用预测中心和追踪中心互相关联（阈值应该调大）
如果预测中心出画就摧毁整个关联体
*/

//关联结构体
// id 是关联坐标点的index
struct target_obj
{
    bool kf_has_start = 0;
    bool use_predit_point = 0;
    // vector<Point2f> front_point;
    Point2f center_point;
    Point2f tracker_point;  //上一帧中心点
    KalmanFilter kf;
};


//此为半成品封装，根据题目要求更改条件
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

        // 通过单位向量计算长轴上下端点
        //  长边向量(px,py)
        if (py < 0)
        {
            px = -px;
            py = -py;
        }
        // 确保向量指向图像下方
        Point2f buttom_point(center.x + (px / 2.0f), center.y + (py / 2.0f));
        Point2f top_point(center.x - (px / 2.0f), center.y - (py / 2.0f));

    //     // 判断条件
    //     if (angle < 5.0f && contourArea(contours[i]) > 150.0f && l_s > 3.5f && l_s < 16.5f)
    //     {
    //         index_rect.push_back(i);
    //         // 在原图上画出矩形
    //         for (int k = 0; k < 4; k++)
    //         {
    //             line(result, poin[k], poin[(k + 1) % 4], Scalar(0, 255, 0), 2);
    //         }
    //         putText(result, to_string(i), poin[0], FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 255), 2);
    //         circle(result, buttom_point, 4, Scalar(255, 255, 255), -1);
    //         circle(result, top_point, 4, Scalar(255, 255, 255), -1);
    //     }

    }

    return result;
}

//传入图像，包含四个点的数组,传入线条颜色
void draw_rect(Mat& src,Point2f* point,Scalar color)
{
    for (int k = 0; k < 4; k++)
    {
        line(src, point[k], point[(k + 1) % 4], color, 2);
    }

}

//传入原始图像和需要进行透视变换的四个角 LB LT RB RT 返回展开后的图像
Mat use_perspectTrans(Mat& input_src,vector<Point2f> src_points)   //LB LT RB RT
{
    vector<Point2f> dst_points={{0,0},{0,400},{800,400},{800,0}};  //展开到的图像尺寸
    Mat result;
    auto transform = getPerspectiveTransform(src_points,dst_points);  //获取透视变换矩阵 把src的点平面透视变换到dst的点上
    warpPerspective(input_src,result,transform,Size(800,400));  //应用变换矩阵

    // imshow("111",result);
    // waitKey(0);

    return result;
}

//读取相机的镜头参数用于PNP
void read_camera_config(Mat& cameratrix_ma,Mat& distortion_coefficients,string file)
{
    FileStorage fs(file,FileStorage::READ);   //只读模式打开yml文件

    fs["cameratrix_ma"] >> cameratrix_ma;   //相机内参
    fs["distortion_coefficients"] >> distortion_coefficients;   //畸变系数

    //释放资源
    fs.release();
}


//计算PNP用到的RMSE //计算RMSE（误差分析(方差)）,
double calculate_rmse(vector<Point2f>& image_point , vector<Point2f>& pred_point)
{
    double sum_data = 0;

    for (int i = 0; i < image_point.size(); i++)
    {
        auto delta_x = image_point[i].x - pred_point[i].x;
        auto delta_y = image_point[i].y - pred_point[i].y;

        sum_data += delta_x*delta_x + delta_y*delta_y;
    }

    double rmse = sqrt(sum_data/image_point.size());  //sqrt是开根号
    return rmse;
}



//PNP求解，传入两个相机参数，物体三维点，物体的图像二维点
vector<Point2f> pnp_calculate(const Mat& cameratrix,const Mat& distortion_coefficients,vector<Point3f>& object_points,vector<Point2f>& image_points)
{
    vector<Point2f> calculate_point;

    //调用PNP求解
    Mat rvec,tvec;
    auto success = solvePnP(object_points,image_points,cameratrix,distortion_coefficients,rvec,tvec);
    if (!success)
    {
        cout << "PNP failed" << endl;
        return calculate_point;
    }

    // rvec 旋转向量矩阵   tvec 物体在相机坐标系中相对相机（原点）的平移向量
    cout << "rvec: " << rvec << " tvec: " << tvec << endl;
    cout << "tvec_z: " << tvec.at<double>(2)  << " norm_tvec: " << norm(tvec) << endl; //tvec内部坐标是double精度 按照x,y,z排列
    
    //检查是否在相机正前方（检查PNP结果有效性 ）
    if (tvec.at<double>(2) > 0)
    {
        cout << "in front of the camera" << endl;
    }
    else
    {
        cout << "behind camera" << endl;
    }

    //重投影验证
    projectPoints(object_points,rvec,tvec,cameratrix,distortion_coefficients,calculate_point);  //把重投影的点输出到predict_point

    //计算RMSE（误差分析(方差)）
    auto rmse = calculate_rmse(image_points,calculate_point);
    cout << "rmse: " << rmse << endl;

    return calculate_point;

}

//在使用setMouseCallback("windowsname",call_back_func,userdata)往userdata传入Point2f类型数组地址记录鼠标点击的坐标，可以自定义
void mousecb(int event, int x, int y, int flags, void *userdata)
{
    if (event == EVENT_LBUTTONDOWN)
    {
        auto* points = static_cast<vector<Point2f>*>(userdata);

        Point2f point(x,y);
        points->push_back(point);
    }
    
}

//基础关联,根据中心坐标进行关联,传入当前的所有目标中心数组，传入关联的数组pair<Point2f,int>类型，前面是关联中心点，后面是丢失次数  如果有多目标卡尔曼滤波，应该把失联的中心坐标传入
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
    
    //出现新目标进行关联
    if (center_point.size() > track_point.size())
    {
        //挑选最不相关的中心作为新的目标
        int index = 0;
        double abs_norm = 0;
        for (int i = 0; i < center_point.size(); i++)
        {
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
    
    //检测关联性
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
            if (center_point.size() != 0)
            {
                track_point[i].second++;
            }
            if (track_point[i].second > 10)
            {
                cout << "target_lost id: " << i << endl;
            }
        }
        
        
    }
    
}


/*
过程噪声强度q：
强度过小时，模型教保守，容易出现滞后于篮球运动轨迹的表现
强度过大时，模型会偏激，容易出现预测框在目标附近的轨迹出现抖动

增大过程噪声强度q时候，滤波器会更快跟上变化，但是预测抖动严重功能几乎失效

测量噪声方差r:
强度过小时，滤波框会靠近实际的测量框，会导致预测框轨迹严重抖动,
强度过大时，滤波框会更偏向于预测框，会导致严重的滞后

增大测量噪声方差r的时候，滤波器运动会很平滑(变化率变小),预测框滞后，但是几乎跟不上实际运动轨迹，导致滤波功能无意义
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

//希望不会用到......  该函数只能用于检查当前跟踪器和画面中包含的目标是否对应了，和更新跟踪器的关联中心和当前帧中心 
//多目标卡尔曼关联更新,传入存储跟踪目标的容器，当前画面中的合格的中心坐标点
void update_kal_traker(vector<target_obj>& obj_on_track,vector<Point2f>& center_on_frame)
{
    if (obj_on_track.size() > center_on_frame.size())
    {
        double avg_abs_norm = 0;
        double total_abs_norm = 0;
        auto del_index = obj_on_track.begin();
        for (auto i = obj_on_track.begin() ; i < obj_on_track.end(); i++)
        {
            for (int j = 0; j < center_on_frame.size(); j++)
            {
                total_abs_norm += abs(norm((*i).tracker_point - center_on_frame[j]));
            }
            
            if (total_abs_norm / center_on_frame.size() > avg_abs_norm)
            {
                avg_abs_norm = total_abs_norm / center_on_frame.size();
                del_index = i;
            }

        }
        
        obj_on_track.erase(del_index);
    }

    if (obj_on_track.size() < center_on_frame.size())
    {
        double avg_abs_norm = 0;
        double total_abs_norm = 0;
        auto add_index = center_on_frame.begin();
        for (auto i = center_on_frame.begin() ; i < center_on_frame.end(); i++)
        {
            for (int j = 0; j < obj_on_track.size(); j++)
            {
                total_abs_norm += abs(norm((*i) - obj_on_track[j].tracker_point));
            }
            
            if (total_abs_norm / obj_on_track.size() > avg_abs_norm)
            {
                avg_abs_norm = total_abs_norm / obj_on_track.size();
                add_index = i;
            }

        }
        
        target_obj temp;
        temp.center_point = *add_index;
        temp.tracker_point = *add_index;
        init_kalman_model(temp.kf);
        
        obj_on_track.push_back(temp);        
    }
    
    vector<Point2f> compare_point;

    for (int i = 0; i < obj_on_track.size(); i++)
    {   
        auto index = center_on_frame.begin();
        double abs_norm = abs(norm(obj_on_track[i].tracker_point - center_on_frame[0]));
        for (auto j = center_on_frame.begin(); j < center_on_frame.end(); j++)
        {
            if (abs(norm(obj_on_track[i].tracker_point - *j)) < abs_norm)
            {
                abs_norm = abs(norm(obj_on_track[i].tracker_point - *j));
                index = j;
            }
            
        }

        if (obj_on_track[i].use_predit_point)
        {
            auto prediction = obj_on_track[i].kf.predict();
            Point2f predictCenter;

            predictCenter.x = prediction.at<float>(0,0);
            predictCenter.y = prediction.at<float>(1,0);

            obj_on_track[i].center_point = predictCenter;
            obj_on_track[i].tracker_point = predictCenter;
            continue;
        }
        
        obj_on_track[i].center_point = *index;
        obj_on_track[i].tracker_point = *index;
        compare_point.push_back(*index);
        center_on_frame.erase(index);
        
    }
    
    center_on_frame = compare_point;
    
}

//单目标卡尔曼滤波
pair<Point2f,Point2f> basketball_analyse(bool vaild,int index,vector<vector<Point>>& contours_point,Mat& src,KalmanFilter& kf)
{
    //全部选框经过卡尔曼滤波预测目标筛查
    //如果有多个圆框的圆心距离教近，取平均值获得一个新的共同的圆
    static vector<Point2f> init_point;
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

        //根据初始的两个坐标设置开始值
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

        //因为correct输出的矩阵是(x,y,vx,vy)4行一列的矩阵,用下标获取对应的数据

        //画出滤波后和预测的结果
        circle(src,predictCenter,radius,Scalar(0,0,255),2);
        putText(src,"predict",predictCenter,FONT_HERSHEY_SIMPLEX,0.6,Scalar(0,0,255),2);
        circle(src,filterCenter,radius,Scalar(255,0,0),2);
    }

    return output_center;
    
}