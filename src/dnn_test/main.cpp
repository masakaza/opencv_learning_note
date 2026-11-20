#include <charconv>
#include <iostream>
#include <string_view>

#include "model_using.hpp"
#include "common_func.hpp"

int main(int argc, char* argv[])
{
    if (argc == 2 && std::string_view(argv[1]) == "--help")
    {
        std::cout << "用法: dnn_test <模型.onnx> <输入视频> [输入边长=1280] [输出.mp4]\n";
        return 0;
    }
    if (argc < 3 || argc > 5)
    {
        std::cerr << "用法: dnn_test <模型.onnx> <输入视频> [输入边长=1280] [输出.mp4]\n";
        return 1;
    }

    int input_size = 1280;
    if (argc >= 4)
    {
        const std::string_view size_arg(argv[3]);
        const char* end = size_arg.data() + size_arg.size();
        const auto parsed = std::from_chars(size_arg.data(), end, input_size);
        if (parsed.ec != std::errc{} || parsed.ptr != end || input_size <= 0)
        {
            std::cerr << "模型输入边长必须是正整数。\n";
            return 1;
        }
    }

    try
    {
        auto net = model_load(argv[1]);
        if (net.empty())
        {
            return 1;
        }

        cv::VideoCapture capture(argv[2]);
        cv::Mat frame;
        if (!capture.isOpened() || !capture.read(frame))
        {
            std::cerr << "无法读取输入视频: " << argv[2] << '\n';
            return 1;
        }

        const bool write_video = argc == 5;
        cv::VideoWriter writer;
        if (write_video)
        {
            const double fps = capture.get(cv::CAP_PROP_FPS);
            if (fps <= 0 || !writer.open(argv[4], cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                                         fps, frame.size()))
            {
                std::cerr << "无法创建输出视频: " << argv[4] << '\n';
                return 1;
            }
        }

        constexpr float confidence = 0.50f;
        constexpr float nms_iou = 0.45f;
        constexpr double min_hsv_ratio = 0.20;
        TrackerManager tracker(1.35, 0.90, 0.90);

        int frame_index = 0;
        do
        {
            LetterBoxInfo letterbox_info;
            auto blob = image_premdelopt(frame, input_size, input_size, letterbox_info);
            cv::Mat annotated = frame.clone();
            auto detections = forward_getrect(net, blob, confidence, nms_iou, letterbox_info, annotated);
            filter_rect_by_hsv(frame, detections, min_hsv_ratio);
            tracker.update_tracker(detections, frame_index, annotated, frame);
            ++frame_index;

            if (write_video)
            {
                writer.write(annotated);
            }
            else
            {
                cv::imshow("YOLO + KF/EKF", annotated);
                const int key = cv::waitKey(0);
                if (key == 27 || key == 'q')
                {
                    break;
                }
            }
        } while (capture.read(frame));

        std::cout << "已处理 " << frame_index << " 帧。\n";
    }
    catch (const cv::Exception& error)
    {
        std::cerr << "OpenCV 错误: " << error.what() << '\n';
        return 1;
    }
}
