#include <opencv2/opencv.hpp>

#include <algorithm>
#include <sstream>
#include <string>

namespace {
constexpr int kHueMax = 179;
constexpr int kSaturationMax = 255;
constexpr int kValueMax = 255;

int selected_h = 110;
int selected_s = 255;
int selected_v = 255;
int lower_h = 90;
int upper_h = 130;
int lower_s = 80;
int upper_s = 255;
int lower_v = 50;
int upper_v = 255;

const std::string kControlsWindow = "HSV range controls";
const std::string kPreviewWindow = "HSV color and range preview";

cv::Scalar hsvToBgr(int h, int s, int v)
{
    cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(h, s, v));
    cv::Mat bgr;
    cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
    const cv::Vec3b color = bgr.at<cv::Vec3b>(0, 0);
    return cv::Scalar(color[0], color[1], color[2]);
}

void drawHueStrip(cv::Mat& canvas, const cv::Rect& area)
{
    for (int x = 0; x < area.width; ++x) {
        const int h = std::min(kHueMax, x * (kHueMax + 1) / area.width);
        cv::line(canvas,
                 cv::Point(area.x + x, area.y),
                 cv::Point(area.x + x, area.y + area.height - 1),
                 hsvToBgr(h, 255, 255), 1);
    }

    // Draw the selected HSV interval. H can wrap across 179 -> 0 for red.
    const auto markInterval = [&](int from, int to) {
        const int x1 = area.x + from * area.width / (kHueMax + 1);
        const int x2 = area.x + (to + 1) * area.width / (kHueMax + 1);
        cv::rectangle(canvas, cv::Rect(x1, area.y, std::max(1, x2 - x1), area.height),
                      cv::Scalar(255, 255, 255), 2);
    };
    if (lower_h <= upper_h) {
        markInterval(lower_h, upper_h);
    } else {
        markInterval(lower_h, kHueMax);
        markInterval(0, upper_h);
    }

    const int selectedX = area.x + selected_h * area.width / (kHueMax + 1);
    cv::line(canvas, cv::Point(selectedX, area.y - 5),
             cv::Point(selectedX, area.y + area.height + 5), cv::Scalar(0, 0, 0), 2);
}

void drawSaturationValuePlane(cv::Mat& canvas, const cv::Rect& area)
{
    cv::Mat hsv(area.height, area.width, CV_8UC3);
    for (int y = 0; y < area.height; ++y) {
        const uchar v = static_cast<uchar>(255 - y * 255 / std::max(1, area.height - 1));
        for (int x = 0; x < area.width; ++x) {
            const uchar s = static_cast<uchar>(x * 255 / std::max(1, area.width - 1));
            hsv.at<cv::Vec3b>(y, x) = cv::Vec3b(static_cast<uchar>(selected_h), s, v);
        }
    }

    cv::Mat bgr;
    cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
    bgr.copyTo(canvas(area));

    // Show the S/V portion of the selected range when the bounds are valid.
    if (lower_s <= upper_s && lower_v <= upper_v) {
        const int x1 = area.x + lower_s * (area.width - 1) / 255;
        const int x2 = area.x + upper_s * (area.width - 1) / 255;
        const int y1 = area.y + (255 - upper_v) * (area.height - 1) / 255;
        const int y2 = area.y + (255 - lower_v) * (area.height - 1) / 255;
        cv::rectangle(canvas, cv::Point(x1, y1), cv::Point(x2, y2),
                      cv::Scalar(255, 255, 255), 2);
    }

    const int markerX = area.x + selected_s * (area.width - 1) / 255;
    const int markerY = area.y + (255 - selected_v) * (area.height - 1) / 255;
    cv::drawMarker(canvas, cv::Point(markerX, markerY), cv::Scalar(0, 0, 0),
                   cv::MARKER_CROSS, 15, 2, cv::LINE_AA);
    cv::drawMarker(canvas, cv::Point(markerX, markerY), cv::Scalar(255, 255, 255),
                   cv::MARKER_CROSS, 11, 1, cv::LINE_AA);
}

void drawColorChip(cv::Mat& canvas, const cv::Point& origin, const cv::Scalar& color,
                   const std::string& title, const std::string& hsvText)
{
    const cv::Rect chip(origin.x, origin.y, 150, 70);
    cv::rectangle(canvas, chip, color, cv::FILLED);
    cv::rectangle(canvas, chip, cv::Scalar(235, 235, 235), 1);
    cv::putText(canvas, title, cv::Point(origin.x, origin.y - 8),
                cv::FONT_HERSHEY_SIMPLEX, 0.48, cv::Scalar(235, 235, 235), 1, cv::LINE_AA);
    cv::putText(canvas, hsvText, cv::Point(origin.x, origin.y + 92),
                cv::FONT_HERSHEY_SIMPLEX, 0.43, cv::Scalar(235, 235, 235), 1, cv::LINE_AA);
}

cv::Mat makePreview()
{
    cv::Mat canvas(650, 1000, CV_8UC3, cv::Scalar(38, 42, 48));
    cv::putText(canvas, "OpenCV HSV color range visualizer",
                cv::Point(28, 38), cv::FONT_HERSHEY_SIMPLEX, 0.9,
                cv::Scalar(245, 245, 245), 2, cv::LINE_AA);

    const cv::Rect hueArea(50, 82, 900, 45);
    drawHueStrip(canvas, hueArea);
    cv::putText(canvas, "H: 0 (red)", cv::Point(50, 153), cv::FONT_HERSHEY_SIMPLEX,
                0.48, cv::Scalar(235, 235, 235), 1, cv::LINE_AA);
    cv::putText(canvas, "60", cv::Point(340, 153), cv::FONT_HERSHEY_SIMPLEX,
                0.48, cv::Scalar(235, 235, 235), 1, cv::LINE_AA);
    cv::putText(canvas, "120", cv::Point(650, 153), cv::FONT_HERSHEY_SIMPLEX,
                0.48, cv::Scalar(235, 235, 235), 1, cv::LINE_AA);
    cv::putText(canvas, "179 (red)", cv::Point(872, 153), cv::FONT_HERSHEY_SIMPLEX,
                0.48, cv::Scalar(235, 235, 235), 1, cv::LINE_AA);

    const cv::Rect svArea(50, 205, 300, 300);
    drawSaturationValuePlane(canvas, svArea);
    cv::rectangle(canvas, svArea, cv::Scalar(235, 235, 235), 1);
    cv::putText(canvas, "Saturation (0 -> 255)", cv::Point(50, 532),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(235, 235, 235), 1, cv::LINE_AA);
    cv::putText(canvas, "Value: 255 top, 0 bottom", cv::Point(50, 557),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(235, 235, 235), 1, cv::LINE_AA);

    const cv::Scalar selectedColor = hsvToBgr(selected_h, selected_s, selected_v);
    cv::putText(canvas, "Selected HSV", cv::Point(410, 212), cv::FONT_HERSHEY_SIMPLEX,
                0.58, cv::Scalar(245, 245, 245), 1, cv::LINE_AA);
    cv::rectangle(canvas, cv::Rect(410, 228, 180, 145), selectedColor, cv::FILLED);
    cv::rectangle(canvas, cv::Rect(410, 228, 180, 145), cv::Scalar(235, 235, 235), 1);

    std::ostringstream selectedText;
    selectedText << "HSV(" << selected_h << ", " << selected_s << ", " << selected_v << ")";
    std::ostringstream bgrText;
    bgrText << "BGR(" << static_cast<int>(selectedColor[0]) << ", "
            << static_cast<int>(selectedColor[1]) << ", "
            << static_cast<int>(selectedColor[2]) << ")";
    cv::putText(canvas, selectedText.str(), cv::Point(410, 397), cv::FONT_HERSHEY_SIMPLEX,
                0.55, cv::Scalar(245, 245, 245), 1, cv::LINE_AA);
    cv::putText(canvas, bgrText.str(), cv::Point(410, 423), cv::FONT_HERSHEY_SIMPLEX,
                0.5, cv::Scalar(210, 210, 210), 1, cv::LINE_AA);

    const cv::Scalar lowerColor = hsvToBgr(lower_h, lower_s, lower_v);
    const cv::Scalar upperColor = hsvToBgr(upper_h, upper_s, upper_v);
    drawColorChip(canvas, cv::Point(640, 257), lowerColor, "Lower bound",
                  "(" + std::to_string(lower_h) + ", " + std::to_string(lower_s) + ", " + std::to_string(lower_v) + ")");
    drawColorChip(canvas, cv::Point(810, 257), upperColor, "Upper bound",
                  "(" + std::to_string(upper_h) + ", " + std::to_string(upper_s) + ", " + std::to_string(upper_v) + ")");

    const bool svValid = lower_s <= upper_s && lower_v <= upper_v;
    const std::string hueText = lower_h <= upper_h
        ? "Hue range: one interval [lower H, upper H]"
        : "Hue wraps red: [lower H,179] OR [0,upper H]";
    cv::putText(canvas, hueText, cv::Point(410, 484), cv::FONT_HERSHEY_SIMPLEX,
                0.48, cv::Scalar(235, 235, 235), 1, cv::LINE_AA);
    cv::putText(canvas, svValid ? "S/V range is valid" : "Invalid range: lower S/V must not exceed upper S/V",
                cv::Point(410, 512), cv::FONT_HERSHEY_SIMPLEX, 0.45,
                svValid ? cv::Scalar(120, 230, 140) : cv::Scalar(90, 150, 255), 1, cv::LINE_AA);

    cv::putText(canvas, "OpenCV HSV uses H=0..179 and S,V=0..255. Preview marker: selected H/S/V.",
                cv::Point(410, 555), cv::FONT_HERSHEY_SIMPLEX, 0.43,
                cv::Scalar(190, 195, 202), 1, cv::LINE_AA);
    cv::putText(canvas, "For a real image, pass these bounds to inRange(hsv, lower, upper, mask).",
                cv::Point(410, 580), cv::FONT_HERSHEY_SIMPLEX, 0.43,
                cv::Scalar(190, 195, 202), 1, cv::LINE_AA);
    return canvas;
}
} // namespace

int main()
{
    cv::namedWindow(kControlsWindow, cv::WINDOW_NORMAL);
    cv::resizeWindow(kControlsWindow, 540, 520);
    cv::createTrackbar("Selected H", kControlsWindow, &selected_h, kHueMax);
    cv::createTrackbar("Selected S", kControlsWindow, &selected_s, kSaturationMax);
    cv::createTrackbar("Selected V", kControlsWindow, &selected_v, kValueMax);
    cv::createTrackbar("Lower H", kControlsWindow, &lower_h, kHueMax);
    cv::createTrackbar("Upper H", kControlsWindow, &upper_h, kHueMax);
    cv::createTrackbar("Lower S", kControlsWindow, &lower_s, kSaturationMax);
    cv::createTrackbar("Upper S", kControlsWindow, &upper_s, kSaturationMax);
    cv::createTrackbar("Lower V", kControlsWindow, &lower_v, kValueMax);
    cv::createTrackbar("Upper V", kControlsWindow, &upper_v, kValueMax);

    cv::namedWindow(kPreviewWindow, cv::WINDOW_NORMAL);
    cv::resizeWindow(kPreviewWindow, 1000, 650);
    for (;;) {
        cv::imshow(kPreviewWindow, makePreview());
        const int key = cv::waitKey(30);
        if (key == 27 || key == 'q' || key == 'Q') break;
    }

    cv::destroyAllWindows();
    return 0;
}
