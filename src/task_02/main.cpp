#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

//red
Scalar lower_red1(0,80,70);
Scalar higher_red1(10,255,255);

Scalar lower_red2(170,80,70);
Scalar higher_red2(179,255,255);

//明度V如果使用50容易触发反光误检测，但是如果使用70可以减少这种情况

//blue
Scalar lower_blue(90,80,50);
Scalar higher_blue(130,255,255);

auto Kernel3 = getStructuringElement(MORPH_RECT,Size(3,3));
auto Kernel5 = getStructuringElement(MORPH_RECT,Size(5,5));
auto Kernel15 = getStructuringElement(MORPH_RECT,Size(15,15));
auto Kernel10 = getStructuringElement(MORPH_RECT,Size(10,10));

int main()
{
    Mat src1,src2;
    src1 = imread("../image/task02_blue_a.png");
    src2 = imread("../image/task02_red_a.png");
    auto srcb1 = imread("../image/task02_blue_b.png");
    auto srcb2 = imread("../image/task02_red_b.png");
    cvtColor(src1,src1,COLOR_BGR2HSV);
    cvtColor(src2,src2,COLOR_BGR2HSV);
    cvtColor(srcb1,srcb1,COLOR_BGR2HSV);
    cvtColor(srcb2,srcb2,COLOR_BGR2HSV);
    Mat blue_mask,red_mask1,red_mask2,red_mask,b_mask,r_mask;

    inRange(src1,lower_blue,higher_blue,blue_mask);  //把在范围内的像素变成255,不在范围内的像素变成0,输出二值化后的掩模
    inRange(srcb1,lower_blue,higher_blue,b_mask);
    // imshow("111",blue_mask);
    // waitKey(0);
    // destroyAllWindows();

    inRange(src2,lower_red1,higher_red1,red_mask1);
    inRange(src2,lower_red2,higher_red2,red_mask2);
    bitwise_or(red_mask1,red_mask2,red_mask);
    inRange(srcb2,lower_red1,higher_red1,red_mask1);
    inRange(srcb2,lower_red2,higher_red2,red_mask2);
    bitwise_or(red_mask1,red_mask2,r_mask);
    // imshow("111",red_mask);
    // waitKey(0);
    // destroyAllWindows();

    imwrite("../image_output/task02/blue_mask_a.png",blue_mask);
    imwrite("../image_output/task02/red_mask_a.png",red_mask);
    imwrite("../image_output/task02/blue_mask_b.png",b_mask);
    imwrite("../image_output/task02/red_mask_b.png",r_mask);
    Mat blue_open3,blue_open5,blue_close3,blue_close5;
    morphologyEx(blue_mask,blue_open3,MORPH_OPEN,Kernel3);
    morphologyEx(blue_mask,blue_open5,MORPH_OPEN,Kernel5);
    morphologyEx(blue_mask,blue_close3,MORPH_CLOSE,Kernel3);
    morphologyEx(blue_mask,blue_close5,MORPH_CLOSE,Kernel5);
    imwrite("../image_output/task02/blue_mask_open3.png",blue_open3);
    imwrite("../image_output/task02/blue_mask_open5.png",blue_open5);
    imwrite("../image_output/task02/blue_mask_close3.png",blue_close3);
    imwrite("../image_output/task02/blue_mask_close5.png",blue_close5);

    //对四张图片进行处理
    morphologyEx(blue_mask,blue_mask,MORPH_OPEN,Kernel3);
    morphologyEx(blue_mask,blue_mask,MORPH_CLOSE,Kernel5);
    morphologyEx(blue_mask,blue_mask,MORPH_OPEN,Kernel5);

    morphologyEx(red_mask,red_mask,MORPH_CLOSE,Kernel15);
    morphologyEx(red_mask,red_mask,MORPH_OPEN,Kernel3);

    morphologyEx(b_mask,b_mask,MORPH_OPEN,Kernel3);
    morphologyEx(b_mask,b_mask,MORPH_CLOSE,Kernel5);
    morphologyEx(b_mask,b_mask,MORPH_OPEN,Kernel5);

    morphologyEx(r_mask,r_mask,MORPH_OPEN,Kernel3);
    morphologyEx(r_mask,r_mask,MORPH_CLOSE,Kernel3); 
    //红色 b图片中漏检2个短灯条
    // imshow("111",r_mask);
    // waitKey(0);
    // destroyAllWindows();
    imwrite("../image_output/task02/blue_a_final.png",blue_mask);
    imwrite("../image_output/task02/blue_b_final.png",b_mask);
    imwrite("../image_output/task02/red_a_final.png",red_mask);
    imwrite("../image_output/task02/red_b_final.png",r_mask);

    return 0;
}