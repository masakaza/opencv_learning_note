#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <fstream>
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

*/

vector<vector<Point>> contours_a; //轮廓集合
vector<vector<Point>> contours_b;
// vector<Vec4i> hierarchy;  //层级关系

// //red
// Scalar lower_red1(0,80,70);
// Scalar higher_red1(10,255,255);

// Scalar lower_red2(170,80,70);
// Scalar higher_red2(179,255,255);

//blue
Scalar lower_blue(90,80,50);
Scalar higher_blue(130,255,255);

const float half = 180.0;


Mat calculate_draw(Mat& src,vector<vector<Point>>& contours,ofstream& csv,vector<int>& index_rect)
{
    Point2f poin[4];
    Mat result = src.clone();

    for (int i = 0; i < contours.size(); i++)
    {
        // cout << "id: " << i << " area: " << contourArea(contours[i]) << endl;
        auto rect = minAreaRect(contours[i]);   //根据轮廓 获取最小旋转矩形(最小能包裹轮廓的矩形)
        auto center = rect.center;

        //统一长短定义
        auto long_side = max(rect.size.width,rect.size.height);
        auto short_side = min(rect.size.width,rect.size.height);
        auto l_s = long_side/short_side;
        // cout << "chang / kuan: " << l_s << endl;

        int index = 0;
        rect.points(poin);
        for (int j = 1; j < 4; j++)
        {
            if (abs(norm(poin[0]-poin[j]) - long_side) < 0.01f)    //norm根据坐标差计算绝对距离,浮点数计算可能会有误差所以用一个允许的误差范围来判断
            {
                index = j;
                break;
            }
        }

        auto point1 = poin[0];
        auto point2 = poin[index]; 
        
        //计算角度  因为是竖直方向夹角所以是 dx/dy
        auto px = point1.x - point2.x;
        auto py = point1.y - point2.y;
        
        auto angle = atan2(abs(px),abs(py))*half/CV_PI;  //通过arctan 求出弧度进而求出角度
        
        // cout << "px: " << px << " py: " << py << endl;
        // cout << angle << endl;

        //通过单位向量计算上下端点
        // 长边向量(px,py)
        if(py < 0)
        {
            px = -px;
            py = -py;
        }
        //确保向量指向图像下方
        Point2f buttom_point(center.x + (px/2.0f),center.y + (py/2.0f));
        Point2f top_point(center.x - (px/2.0f),center.y - (py/2.0f));

        //写入csv
        csv << fixed << setprecision(2);
        csv << i << "," << contourArea(contours[i]) << "," << center.x << "," << center.y << "," << long_side << "," << short_side << "," << l_s << "," << angle << "," << top_point.x << "," << top_point.y << "," << buttom_point.x << "," << buttom_point.y << ",";

        //判断条件
        if (angle < 5.0f && contourArea(contours[i]) > 150.0f && l_s > 3.5f && l_s < 15.0f)
        {
            index_rect.push_back(i);
            //在原图上画出矩形
            for (int j = 0; j < 4; j++)
            {
                line(result,poin[j],poin[(j+1)%4],Scalar(0,255,0),2);
            }
            putText(result,to_string(i),poin[0],FONT_HERSHEY_SIMPLEX,0.6,Scalar(255,0,255),2);
            circle(result,buttom_point,4,Scalar(255,255,255),-1);
            circle(result,top_point,4,Scalar(255,255,255),-1);
            csv << 1 << endl;
        }
        else
        {
            if (angle >= 5.0f)
            {
                csv << "angle large/";
            }

            if (contourArea(contours[i]) <= 150.0f)
            {
                csv << "less area/"; 
            }

            if (l_s <= 3.5f || l_s >=15.0f)
            {
                csv << "ratio out of range,";
            }

            csv << endl;
            
        }
        

        //在原图上画出矩形
        for (int j = 0; j < 4; j++)
        {
            line(src,poin[j],poin[(j+1)%4],Scalar(0,255,0),2);
        }
        putText(src,to_string(i),poin[0],FONT_HERSHEY_SIMPLEX,0.6,Scalar(255,0,255),2);
        
    }
    
    return result;
}

//计算匹配灯条参数    a: id 12,13   b: id 18,19
vector<pair<int,int>> calculate_light(vector<vector<Point>>& contours,vector<int>& rect_index)
{
    vector<pair<int,int>> light_contour;
    Point2f poin[4];
    for (int i = 0; i < rect_index.size(); i++)
    {
        for (int j = i+1; j < rect_index.size(); j++)
        {
            auto rect1 = minAreaRect(contours[rect_index[i]]);
            auto rect2 = minAreaRect(contours[rect_index[j]]);

            //统一长度
            auto long_side1 = max(rect1.size.width,rect1.size.height);
            auto long_side2 = max(rect2.size.width,rect2.size.height);

            auto RL = long_side1/long_side2;
            if(RL > 1.0f)
            {
                RL = long_side2/long_side1;
            }

            cout << "Rl: " << RL << endl;  //灯条长度的比值

            //计算角度
            rect1.points(poin);
            int index = 0;
            for (int k = 1; k < 4; k++)
            {
                if (abs(norm(poin[0]-poin[k]) - long_side1) < 0.01f)    //norm根据坐标差计算绝对距离,浮点数计算可能会有误差所以用一个允许的误差范围来判断
                {
                    index = k;
                    break;
                }
            }

            auto point1 = poin[0];
            auto point2 = poin[index]; 
            
            //计算角度  因为是竖直方向夹角所以是 dx/dy
            auto px = point1.x - point2.x;
            auto py = point1.y - point2.y;
            
            auto angle1 = atan2(abs(px),abs(py))*half/CV_PI; 

            rect2.points(poin);
            for (int k = 1; k < 4; k++)
            {
                if (abs(norm(poin[0]-poin[k]) - long_side2) < 0.01f)    //norm根据坐标差计算绝对距离,浮点数计算可能会有误差所以用一个允许的误差范围来判断
                {
                    index = k;
                    break;
                }
            }
            point1 = poin[0];
            point2 = poin[index]; 
            
            //计算角度  因为是竖直方向夹角所以是 dx/dy
            px = point1.x - point2.x;
            py = point1.y - point2.y;
            
            auto angle2 = atan2(abs(px),abs(py))*half/CV_PI; 

            auto delta_angle = abs(angle1 - angle2);
            cout << "delta angle: " << delta_angle << endl;

            auto avg_side = (long_side1 + long_side2)/2;
            auto RD = abs(norm(rect1.center - rect2.center)) / avg_side;  //计算中心间距和灯条长度的比值
            cout << "RD: " << RD << endl;

            if (RL > 0.6f && RL <= 1.0f && delta_angle <= 1.0f && RD > 1.5f && RD < 2.5f)
            {
                pair<int,int>con_index(rect_index[i],rect_index[j]);
                light_contour.push_back(con_index);
            }
            
        }
        
    }

    return light_contour;
}

void draw_point(Mat& src ,vector<vector<Point>>& contours,vector<pair<int,int>> c_index)  //LB LT RB RT
{
    Point2f poin[4];
    for (int i = 0; i < c_index.size(); i++)
    {
        auto rect1 = minAreaRect(contours[c_index[i].first]);
        auto rect2 = minAreaRect(contours[c_index[i].second]);
        cout << c_index[i].first << " " << c_index[i].second << endl;
        auto long_side1 = max(rect1.size.width,rect1.size.height);
        auto long_side2 = max(rect2.size.width,rect2.size.height);
        rect1.points(poin);
        int index = 0;
        for (int k = 1; k < 4; k++)
        {
            if (abs(norm(poin[0]-poin[k]) - long_side1) < 0.01f)    //norm根据坐标差计算绝对距离,浮点数计算可能会有误差所以用一个允许的误差范围来判断
            {
                index = k;
                break;
            }
        }

        auto point1 = poin[0];
        auto point2 = poin[index]; 
        
        //计算角度  因为是竖直方向夹角所以是 dx/dy
        auto px = point1.x - point2.x;
        auto py = point1.y - point2.y;
        
        //通过单位向量计算上下端点
        // 长边向量(px,py)
        if(py < 0)
        {
            px = -px;
            py = -py;
        }
        //确保向量指向图像下方
        Point2f buttom_point_1(rect1.center.x + (px/2.0f),rect1.center.y + (py/2.0f));
        Point2f top_point_1(rect1.center.x - (px/2.0f),rect1.center.y - (py/2.0f));

        rect2.points(poin);
        index = 0;
        for (int k = 1; k < 4; k++)
        {
            if (abs(norm(poin[0]-poin[k]) - long_side2) < 0.01f)    //norm根据坐标差计算绝对距离,浮点数计算可能会有误差所以用一个允许的误差范围来判断
            {
                index = k;
                break;
            }
        }
        point1 = poin[0];
        point2 = poin[index]; 
        
        //计算角度  因为是竖直方向夹角所以是 dx/dy
        px = point1.x - point2.x;
        py = point1.y - point2.y;

        //通过单位向量计算上下端点
        // 长边向量(px,py)
        if(py < 0)
        {
            px = -px;
            py = -py;
        }
        //确保向量指向图像下方
        Point2f buttom_point_2(rect2.center.x + (px/2.0f),rect2.center.y + (py/2.0f));
        Point2f top_point_2(rect2.center.x - (px/2.0f),rect2.center.y - (py/2.0f));

        vector<Point2f> src_points;
        vector<Point2f> dst_points={{0,0},{0,400},{800,400},{800,0}};
        Mat result;
        if (rect1.center.x < rect2.center.x)
        {
            src_points.push_back(top_point_1);
            src_points.push_back(buttom_point_1);
            src_points.push_back(buttom_point_2);
            src_points.push_back(top_point_2);
        }
        else
        {
            src_points.push_back(top_point_2);
            src_points.push_back(buttom_point_2);
            src_points.push_back(buttom_point_1);
            src_points.push_back(top_point_1);
        }
        
        auto transform = getPerspectiveTransform(src_points,dst_points);  //获取透视变换矩阵 把src的点平面透视变换到dst的点上
        warpPerspective(src,result,transform,Size(800,400));  //应用变换矩阵

        // imshow("111",result);
        // waitKey(0);
        string filename = "../image_output/task04/task04_b_warpresult_" + to_string(i) + ".png";
        imwrite(filename,result);
    }
    
}

int main()
{
    ofstream csva("../image_output/task04/data_a.csv");
    ofstream csvb("../image_output/task04/data_b.csv");
    csva << "id,area,center_x,center_y,long_side,short_side,ratio,angle,top_x,top_y,bottom_x,bottom_y,vaild" << endl;
    csvb << "id,area,center_x,center_y,long_side,short_side,ratio,angle,top_x,top_y,bottom_x,bottom_y,vaild" << endl;

    Mat a,b,mask_a,mask_b;
    a = imread("../image/task04_a.png");
    b = imread("../image/task04_b.png");
    auto bgr_a = a.clone();
    auto bgr_b = b.clone();
    cvtColor(a,a,COLOR_BGR2HSV);
    cvtColor(b,b,COLOR_BGR2HSV);
    inRange(b,lower_blue,higher_blue,mask_b);
    inRange(a,lower_blue,higher_blue,mask_a);
    findContours(mask_a,contours_a,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);   //计算外部轮廓，只保存转折点
    findContours(mask_b,contours_b,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
    
    //计算面积
    vector<int> rect_index_a;
    vector<int> rect_index_b;
    auto result_a = calculate_draw(bgr_a,contours_a,csva,rect_index_a);
    auto result_b = calculate_draw(bgr_b,contours_b,csvb,rect_index_b);

    //获取判断范围
    // calculate_light(contours_b);
    auto light_contours_b = calculate_light(contours_b,rect_index_b);
    cout << light_contours_b.size() << endl;
    draw_point(result_b,contours_b,light_contours_b);
    
    csva.close();
    csvb.close();

    // imshow("111",result_a);
    // imshow("222",result_b);
    imwrite("../image_output/task04/blue_a_result.png",result_a);
    imwrite("../image_output/task04/blue_b_result.png",result_b);
    // imshow("123",mask_b);
    // waitKey(0);
    destroyAllWindows();
    return 0;
}