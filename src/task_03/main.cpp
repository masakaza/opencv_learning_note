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
长宽比 4-11
*/

vector<vector<Point>> contours_a; //轮廓集合
vector<vector<Point>> contours_b;
// vector<Vec4i> hierarchy;  //层级关系

//red
Scalar lower_red1(0,80,70);
Scalar higher_red1(10,255,255);

Scalar lower_red2(170,80,70);
Scalar higher_red2(179,255,255);

//blue
Scalar lower_blue(90,80,50);
Scalar higher_blue(130,255,255);

const float half = 180.0;


Mat calculate_draw(Mat& src,vector<vector<Point>>& contours,ofstream& csv)
{
    Point2f poin[4];
    Mat result = src.clone();

    for (int i = 0; i < contours.size(); i++)
    {
        cout << "id: " << i << " area: " << contourArea(contours[i]) << endl;
        auto rect = minAreaRect(contours[i]);   //根据轮廓 获取最小旋转矩形(最小能包裹轮廓的矩形)
        auto center = rect.center;

        //统一长短定义
        auto long_side = max(rect.size.width,rect.size.height);
        auto short_side = min(rect.size.width,rect.size.height);
        auto l_s = long_side/short_side;
        cout << "chang / kuan: " << l_s << endl;

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
        cout << angle << endl;

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
        if (angle < 5.0f && contourArea(contours[i]) > 150.0f && l_s > 4.0f && l_s < 11.0f)
        {
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

            if (l_s <= 4.0f || l_s >=11.0f)
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

int main()
{
    ofstream csva("../image_output/task03/data_a.csv");
    ofstream csvb("../image_output/task03/data_b.csv");
    csva << "id,area,center_x,center_y,long_side,short_side,ratio,angle,top_x,top_y,bottom_x,bottom_y,vaild" << endl;
    csvb << "id,area,center_x,center_y,long_side,short_side,ratio,angle,top_x,top_y,bottom_x,bottom_y,vaild" << endl;

    Mat a,b,mask_a,mask_b1,mask_b2,mask_b;
    a = imread("../image/task03_a.png");
    b = imread("../image/task03_b.png");
    auto bgr_a = a.clone();
    auto bgr_b = b.clone();
    cvtColor(a,a,COLOR_BGR2HSV);
    cvtColor(b,b,COLOR_BGR2HSV);
    inRange(b,lower_red1,higher_red1,mask_b1);
    inRange(b,lower_red2,higher_red2,mask_b2);
    bitwise_or(mask_b1,mask_b2,mask_b);
    inRange(a,lower_blue,higher_blue,mask_a);
    findContours(mask_a,contours_a,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);   //计算外部轮廓，只保存转折点
    findContours(mask_b,contours_b,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
    
    //计算面积
    auto result_a = calculate_draw(bgr_a,contours_a,csva);
    auto result_b = calculate_draw(bgr_b,contours_b,csvb);
    imwrite("../image_output/task03/task03_a_withcontours.png",bgr_a);
    imwrite("../image_output/task03/task03_b_withcontours.png",bgr_b);
    imwrite("../image_output/task03/task03_a_result.png",result_a);
    imwrite("../image_output/task03/task03_b_result.png",result_b);
    
    csva.close();
    csvb.close();

    // imshow("111",bgr_a);
    // imshow("222",bgr_b);
    // imshow("123",mask_b);
    // waitKey(0);
    // destroyAllWindows();
    return 0;
}