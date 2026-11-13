#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <fstream>
using namespace std;
using namespace cv;

/*
opencv中的Mat类似numpy中的Array

在实际中装甲板四个点的三维物点坐标是不变的（以装甲板中心为原点）
PNP是使用装甲板4个点在平面上的二维坐标反推出装甲板中心在以相机为原点的三维坐标系中的位置
通过这个坐标和旋转向量矩阵和四个三维物点坐标 能反推出装甲板四个点在相机坐标系中的坐标
*/

/*
图片四个端点
[122, 515]
[187, 740]
[552, 592]
[511, 420]

tvec_z: 0.503755 norm_tvec: 0.513133
rmse: 4.42171

tvec_z: 0.503282 norm_tvec: 0.512624
rmse: 4.64131

移动两像素之后，tvec距离没有明显变化，但是重投影的精度下降两
*/

double calculate_rmse(vector<Point2f>& image , vector<Point2f>& pred)
{
    double sum_data = 0;

    for (int i = 0; i < image.size(); i++)
    {
        auto delta_x = image[i].x - pred[i].x;
        auto delta_y = image[i].y - pred[i].y;

        sum_data += delta_x*delta_x + delta_y*delta_y;
    }

    double rmse = sqrt(sum_data/image.size());  //sqrt是开根号
    return rmse;
}

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

vector<Point2f> mouse_point;
void mousecallback(int event, int x, int y, int flags, void *userdata)
{
    if (event == EVENT_LBUTTONDOWN && mouse_point.size() < 4)
    {
        Point2f point(x,y);
        mouse_point.push_back(point);
        cout << point << endl;
    }
    
}

int main()
{
    vector<Point3f> object_points;  //物体三维坐标点
    vector<Point2f> reference_points; //平面像素坐标点

    //读取相机参数
    // 相机参数存储的数据都是矩阵类型的，所以用opencv的Mat处理

    Mat cameratrix_ma,distortion_coefficients;
    FileStorage fs("../image/task05/camera.yml",FileStorage::READ);   //只读模式打开yml文件

    fs["cameratrix_ma"] >> cameratrix_ma;   //相机内参
    fs["distortion_coefficients"] >> distortion_coefficients;   //畸变系数

    //释放资源
    fs.release();

    //读取物体三维坐标
    ifstream obj_point_file("../image/task05/object_points.csv");
    string line;
    getline(obj_point_file,line);

    while (getline(obj_point_file,line))
    {
        stringstream ss(line);

        string model,corn,x,y,z;

        getline(ss,model,',');
        getline(ss,corn,',');
        getline(ss,x,',');
        getline(ss,y,',');
        getline(ss,z,',');

        if (model != "small")
        {
            break;
        }

        float px = stof(x);
        float py = stof(y);
        float pz = stof(z);

        Point3f point(px,py,pz);
        // cout << point << endl;
        object_points.push_back(point);
        
    }

    // cout << object_points.size() << endl;
    
    obj_point_file.close();

    // cout << cameratrix_ma << endl;
    // cout << distortion_coefficients << endl;

    //读取示例平面坐标
    ifstream ref_point("../image/task05/reference_corners.csv");
    string ref_line;
    getline(ref_point,ref_line);

    while (getline(ref_point,ref_line))
    {
        stringstream ss(ref_line);

        string image,model,concer,x,y,type;

        getline(ss,image,',');
        getline(ss,model,',');
        getline(ss,concer,',');
        getline(ss,x,',');
        getline(ss,y,',');
        getline(ss,type,',');

        float px = stof(x);
        float py = stof(y);

        Point2f point(px,py);
        // cout << point << endl;
        reference_points.push_back(point);
        
    }

    ref_point.close();

    pnp_calculate(cameratrix_ma,distortion_coefficients,object_points,reference_points);

    Mat src = imread("../image/task05_a.png");
    //manual
    // namedWindow("111");
    // setMouseCallback("111",mousecallback);
    // imshow("111",src);
    // waitKey(0);
    // destroyAllWindows();

    Point2f LT(122,515);
    Point2f LB(187,740);
    Point2f RB(552,592);
    Point2f RT(511,420);

    mouse_point.push_back(LT);
    mouse_point.push_back(LB);
    mouse_point.push_back(RB);
    mouse_point.push_back(RT);
    
    circle(src,LT,5,Scalar(255,255,255),-1);
    circle(src,LB,5,Scalar(255,255,255),-1);
    circle(src,RB,5,Scalar(255,255,255),-1);
    circle(src,RT,5,Scalar(255,255,255),-1);

    putText(src,"LT",LT + Point2f(8,-8),FONT_HERSHEY_SIMPLEX,0.6,Scalar(255,255,255),2);
    putText(src,"LB",LB + Point2f(8,-8),FONT_HERSHEY_SIMPLEX,0.6,Scalar(255,255,255),2);
    putText(src,"RB",RB + Point2f(8,-8),FONT_HERSHEY_SIMPLEX,0.6,Scalar(255,255,255),2);
    putText(src,"RT",RT + Point2f(8,-8),FONT_HERSHEY_SIMPLEX,0.6,Scalar(255,255,255),2);

    auto cal_point = pnp_calculate(cameratrix_ma,distortion_coefficients,object_points,mouse_point);

    mouse_point[1].x += 2;    
    pnp_calculate(cameratrix_ma,distortion_coefficients,object_points,mouse_point);

    if (cal_point.size() == 0)
    {
        return -1;
    }
    

    for (int i = 0; i < cal_point.size(); i++)
    {
        circle(src,cal_point[i],5,Scalar(255,0,255),-1);
    }
    

    imwrite("../image_output/task05/after_pnp_projectpoint_task05_a.png",src);
    // imshow("111",src);
    // waitKey(0);
    // destroyAllWindows();
    return 0;
}