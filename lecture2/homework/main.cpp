#include "io/camera.hpp"
#include "tasks/yolo.hpp"
#include "opencv2/opencv.hpp"
#include "tools/img_tools.hpp"

#include <algorithm>

int main()
{
    Camera camera;
    auto_aim::YOLO yolo("./configs/yolo.yaml", false);
    int frame_count = 0;

    while (true) {
        cv::Mat img = camera.read();
        if (img.empty()) continue;

        const auto armors = yolo.detect(img, frame_count++);
        for (const auto & armor : armors) {
            // draw_points connects the last key point back to the first one.
            tools::draw_points(img, armor.points, cv::Scalar(0, 255, 0));

            const std::string label = auto_aim::COLORS[armor.color] + auto_aim::ARMOR_NAMES[armor.name];
            const double font_scale = std::max(0.5, img.cols / 640.0 * 0.8);
            const int thickness = std::max(1, static_cast<int>(font_scale * 2));
            int baseline = 0;
            const cv::Size text_size =
              cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, font_scale, thickness, &baseline);
            const auto top = std::min_element(
              armor.points.begin(), armor.points.end(),
              [](const cv::Point2f & a, const cv::Point2f & b) { return a.y < b.y; });
            const int x = std::clamp(
              static_cast<int>(armor.center.x - text_size.width / 2), 0,
              std::max(0, img.cols - text_size.width));
            const int y = std::clamp(
              static_cast<int>(top->y - 8), text_size.height, img.rows - baseline);
            tools::draw_text(img, label, {x, y}, cv::Scalar(0, 0, 255), font_scale, thickness);
        }

        cv::Mat display;
        cv::resize(img, display, cv::Size(640, 480));
        cv::imshow("img", display);
        const int key = cv::waitKey(1);
        if (key == 'q' || key == 27) break;
    }

    return 0;
}
