#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
using namespace std;
using namespace cv;

int main()
{
    Mat src = imread("../image/image.png");  //在c++中，Mat是图像资源类型
    namedWindow("111",WINDOW_FREERATIO); //创建一个自由大小的窗口，然后用imshow把图片显示在这个窗口上面
    resizeWindow("111",800,600);
    // cvtColor(src,src,COLOR_BGR2GRAY);
    Mat m1,m2;
    //图像深拷贝
    m1 = src.clone();
    src.copyTo(m2);

    //创建空白图像
    Mat m4 = Mat::ones(Size2i(8,8),CV_8UC1);
    Mat m3 = Mat::zeros(Size2i(16,8),CV_8UC3); // 8位长度的无符号  单通道 Channel  每个像素点的起始位置的数值赋值0
    m3 = Scalar(154,15,4);
    cout << m3 << endl;
    // resizeWindow("12",128,128);
    // imshow("12",m3);

    // Mat m5 = m4; //这个是浅拷贝，容易出问题

    cout << "weight: " << m3.cols << " height: " << m3.rows << " channels: " << m3.channels() << endl;

    //图像运算操作
    // src = src - Scalar(50,50,50);

    cvtColor(src,src,COLOR_BGR2HSV);
    cvtColor(src,src,COLOR_HSV2BGR);

    // for (int row = 0; row < src.rows; row++)
    // {
    //     auto current_row = src.ptr<uchar>(row);   //获取每行的第一个像素点的指针
    //     for (int col = 0; col < src.cols; col++)
    //     {
    //         if(src.channels() == 1)   //灰度图像
    //         {
    //             *current_row++ = 255-*current_row;   //对每个像素进行操作
    //         }

    //         if(src.channels() == 3)
    //         {
    //             *current_row++ = 255-*current_row;  //B
    //             *current_row++ = 255-*current_row;  //G
    //             *current_row++ = 255-*current_row;  //R
    //         }
    //     }
        
    // }

    vector<Mat> img; 
    split(src,img);   //分离三个通道
    imshow("Blue",img[0]);
    imshow("Green",img[1]);
    imshow("Red",img[2]);
    imshow("111",src);
    waitKey(0);    //waitkey 相当于pause   中间的数字相当于停留时间，单位是ms,0表示阻塞程序
    destroyAllWindows();
    // cout << "1234564897" << endl;

    return 0;
}  