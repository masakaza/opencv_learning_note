#pragma once
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <vector>
#include <iostream>

using namespace std;
using namespace cv;

/*
具体使用的时候看视频中目标大小，
如果目标偏大，使用640大小，
目标偏小，使用1280大小
*/

//模型返回空则是加载出错
inline dnn::Net model_load(string model_path)
{
    try
    {
        //加载模型
        dnn::Net net = dnn::readNetFromONNX(model_path);

        //判断是否加载
        if (net.empty())
        {
            cout << "模型为空" << endl;
            return net;
        }

        net.setPreferableBackend(dnn::DNN_BACKEND_OPENCV);
        net.setPreferableTarget(dnn::DNN_TARGET_CPU);
        cout << "模型加载成功" << endl;

        auto layer_names = net.getLayerNames();

        cout << "层数量: " << layer_names.size() << endl;

        auto output_names = net.getUnconnectedOutLayersNames();
        cout << "输出层数量: " << output_names.size() << endl;

        return net;

    }
    catch(const std::exception& e)
    {
        dnn::Net net;
        std::cerr << "异常： " <<e.what() << '\n';
        return net;
    }


}

// 保存Letterbox的缩放和填充信息
struct LetterBoxInfo
{
    float scale;

    int pad_left;
    int pad_top;
};
//保存这些信息在后面用于恢复中心坐标

//Latterbox预处理，把图片等比缩放并添加适量填充，再送进yolo推理
inline Mat letterbox(const Mat& image, int target_width, int target_height, LetterBoxInfo& info)
{
    // 奇数像素的填充余量放在右侧或下侧，保持输出正好是模型输入尺寸。
    const float scale = min(static_cast<float>(target_width) / image.cols,
                            static_cast<float>(target_height) / image.rows);
    const int new_width = max(1, cvRound(image.cols * scale));
    const int new_height = max(1, cvRound(image.rows * scale));
    const int pad_width = target_width - new_width;
    const int pad_height = target_height - new_height;
    const int left = pad_width / 2;
    const int right = pad_width - left;
    const int top = pad_height / 2;
    const int bottom = pad_height - top;

    //缩放
    Mat resized,output;

    resize(image,resized,Size(new_width,new_height),0,0,INTER_LINEAR);

    //填充
    copyMakeBorder(resized,output,top,bottom,left,right,BORDER_CONSTANT,Scalar(114,114,114));

    //保存缩放信息
    info.scale = scale;
    info.pad_left = left;
    info.pad_top = top;

    return output;
}

//图片预处理函数，传入图片，模型需要的尺寸，缩放信息结构,返回预处理后的图片
inline Mat image_premdelopt(Mat& input_src,int model_width,int model_height,LetterBoxInfo& info)
{
    auto after_letter = letterbox(input_src,model_width,model_height,info);
    auto blob = dnn::blobFromImage(after_letter,1.0/255.0,Size(model_width,model_height),Scalar(),true,false,CV_32F);
    return blob;
}

//传入模型对象，预处理后的图片，最低置信度，最低NMS参数，缩放参数,处理的图像，经过模型处理后返回合格的rect框（用于卡尔曼滤波跟踪）,加上画框
inline vector<Rect> forward_getrect(dnn::Net& net,Mat& blob,float confidence,float mininum_NMS,LetterBoxInfo& info,Mat& src)
{
    net.setInput(blob);
    Mat output = net.forward();
    if (output.dims != 3 || output.size[0] != 1 || output.size[1] != 5 ||
        output.type() != CV_32F)
    {
        CV_Error(Error::StsBadArg, "Expected a single-class ONNX output of shape [1, 5, N] (float32)");
    }

    const int N = output.size[2]; //候选框数量

    vector<Rect2d> boxes; //合格候选框
    vector<float> box_confidence; //候选框置信度

    Mat output_contiguous = output.isContinuous() ? output : output.clone(); //判断是否连续，不连续则克隆出一个连续空间

    const float* data = output_contiguous.ptr<float>(); //创建数据指针指向数据的首地址

    // 遍历所有候选框
    for (int i = 0; i < N; i++)
    {
        // 读取置信度
        float score = data[4 * N + i];

        // 置信度筛选
        if (score < confidence)
        {
            continue;
        }

        // 读取中心点和宽高     每个维度的数据有N个，所以用N*int 来表示
        float cx = data[i];
        float cy = data[N + i];
        float w = data[2 * N + i];
        float h = data[3 * N + i];

        // 转换为左上角坐标
        float x = cx - w / 2.0f;
        float y = cy - h / 2.0f;

        // 保存矩形框
        Rect2d rect(x,y,w,h);
        // boxes.emplace_back(x, y, w, h);
        boxes.push_back(rect);

        // 保存置信度
        box_confidence.push_back(score);

    }

    //保存经过NMS抑制之后剩下的候选框的索引
    vector<int> indeices;

    dnn::NMSBoxes(boxes,box_confidence,confidence,mininum_NMS,indeices);

    vector<Rect> output_rect;
    for (int i = 0; i < indeices.size(); i++)
    {
        //还原坐标
        auto box = boxes[indeices[i]];
        float x = (box.x - info.pad_left)/ info.scale;
        float y = (box.y - info.pad_top)/info.scale;

        float w = box.width / info.scale;
        float h = box.height / info.scale;
        Rect rect(cvRound(x),cvRound(y),cvRound(w),cvRound(h));

        //如果要从画框层面直接入手，在这里调用common_func的rect_has_hsv

        //画框
        rectangle(src,rect,Scalar(0,255,0),2);
        //置信度
        string label = format("YOLO %.2f",box_confidence[indeices[i]]);

        putText(src,label,Point(rect.x,rect.y - 10),FONT_HERSHEY_SIMPLEX,0.6,Scalar(0,255,0),2);

        output_rect.push_back(rect);
    }

    return output_rect;
}
