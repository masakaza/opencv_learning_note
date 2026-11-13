#include <opencv2/opencv.hpp>
#include <iostream>
using namespace std;
using namespace cv;

int main()
{
    VideoCapture cap("../video/031.mp4");
    
    Mat frame;
    int frame_count = 0;


    while (cap.read(frame))
    {
        String file_name = "../video_val/" + to_string(frame_count) + ".jpg";

        imwrite(file_name,frame);
        frame_count++;
    }
    
    cap.release();

    return 0;
}