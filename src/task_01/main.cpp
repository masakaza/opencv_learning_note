#include <iostream>
#include <opencv2/opencv.hpp>
#include <queue>
using namespace std;
using namespace cv;

/*
通过鼠标回调能得出装甲板区域的对角坐标（左上角和右下角）
经过计算可以得出rect的区域获得roi
经过clone的roi不会影响原图，没有经过clone的roi会影响原图


x:578 y: 680
wait for next data...
x:979 y: 974
left corn data: 578 680
weight: 401 hight: 294

*/

queue<pair<int,int>> q_xy;

void mousecb(int event, int x, int y, int flags, void *userdata)
{
    if (event == EVENT_LBUTTONDOWN)
    {
        cout << "x:" << x << " y: " << y << endl;
        pair<int,int>xy;
        xy.first = x;
        xy.second = y;
        q_xy.push(xy);
        // cout << q_xy.size() << endl;
        if (q_xy.size() > 1)
        {
            pair<int,int>temp;
            temp = q_xy.front();
            cout << "left corn data: " << temp.first << " " << temp.second << endl;
            q_xy.pop();
            cout << "weight: " << (q_xy.front().first - temp.first) << " hight: " << (q_xy.front().second - temp.second) << endl;
        }
        else
        {
            cout << "wait for next data..." << endl;
        }
        
        
    }
    
}

int main()
{
    Mat p1,p2;
    p1 = imread("../image/task01_a.png");
    p2 = imread("../image/task01_b.png");
    if (p1.empty() || p2.empty())
    {
        cout << "open picture fail..." << endl;
        return -1;
    }
    else
    {
        cout << "weight: " << p1.cols << endl;
        cout << "hight: " << p1.rows << endl;
        cout << "channel: " << p1.channels() << endl;
        cout << "type: " << p1.type() << endl;
        cout << "weight: " << p2.cols << endl;
        cout << "hight: " << p2.rows << endl;
        cout << "channel: " << p2.channels() << endl;
        cout << "type: " << p2.type() << endl;
    }
    Mat gp1,gp2;
    cvtColor(p1,gp1,COLOR_BGR2GRAY);
    cvtColor(p2,gp2,COLOR_BGR2GRAY);
    // imshow("211",gp1);
    // waitKey(0);
    // destroyAllWindows();
    imwrite("../image_output/task01/gray_a.png",gp1);
    imwrite("../image_output/task01/gray_b.png",gp2);
    // namedWindow("task_a");
    // setMouseCallback("task_a",mousecb);
    // imshow("task_a",p1);
    // waitKey(0);   
    // destroyAllWindows();

    //使用上面代码获得的坐标rect出roi
    Rect rect(578,680,401,294);
    if (rect.x < 0 || rect.y < 0 || rect.width < 0 || rect.height < 0 || rect.x + rect.width > p1.cols || rect.y + rect.height > p1.rows)
    {
        cout << "roi越界" << endl;
        return -1;
    }

    Mat roi = p1(rect);
    // imshow("roi",roi);
    // waitKey(0);
    // destroyAllWindows();
    imwrite("../image_output/task01/roi.png",roi);

    Mat roi_b = p2(rect);
    Mat roi_b_c = p2(rect).clone();
    cvtColor(roi_b_c,roi_b_c,COLOR_BGR2HSV);
    imwrite("../image_output/task01/task01_b_roi_clone.png",p2);
    cvtColor(roi_b,roi_b,COLOR_BGR2HSV);
    imwrite("../image_output/task01/task01_b_roi.png",p2);
    return 0;
}