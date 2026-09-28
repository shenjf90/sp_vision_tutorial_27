#include "io/camera.hpp"
#include "tasks/yolo.hpp"
#include "opencv2/opencv.hpp"
#include "tools/img_tools.hpp"

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
        }

        cv::Mat display;
        cv::resize(img, display, cv::Size(640, 480));
        cv::imshow("img", display);
        const int key = cv::waitKey(1);
        if (key == 'q' || key == 27) break;
    }

    return 0;
}
