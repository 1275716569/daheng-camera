#include <opencv2/opencv.hpp>

#include <chrono>
#include <iostream>
#include <string>

int main(int argc, char *argv[]) {
    // 参数:文件路径按文件打开;纯数字按摄像头编号打开
    cv::VideoCapture cap;
    bool opened = false;
    if (argc >= 2) {
        std::string arg = argv[1];
        // 全部是数字 -> 摄像头编号
        if (!arg.empty() && arg.find_first_not_of("0123456789") == std::string::npos) {
            opened = cap.open(std::stoi(arg));
        } else {
            opened = cap.open(arg); // 视频文件
            double fps = cap.get(cv::CAP_PROP_FPS);
            if (fps <= 0 || fps > 120) fps = 30.0; // 防御性编程：防止读取不到FPS或FPS异常
            int delay = floor(1000.0 / fps); // 计算每帧需要多少毫秒

            cv::Mat frame;
            int key = 0;
            while (cap.read(frame)) {
            cv::putText(frame, "FPS: " + std::to_string(fps), cv::Point(10, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

            cv::imshow("Video", frame);
            key = cv::waitKey(delay); 
            if (key == 27){
                break;
            }    

            }
            cap.release();
            return 0;
        }
    } else {
        opened = cap.open(0);
    }

    if (!opened) {
        std::cout << "打开失败!用法: ./video_demo [视频文件路径 或 摄像头编号]" << std::endl;
        return -1;
    }


    int frame_count = 0;
    auto fps_window_start = std::chrono::steady_clock::now();
    double fps = 0.0;

    cv::Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) {
            std::cout << "视频结束或取流失败,退出" << std::endl;
            break;
        }

        ++frame_count;
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - fps_window_start).count();
        if (elapsed >= 1.0) {
            fps = frame_count / elapsed;
            frame_count = 0;
            fps_window_start = now;
        }
        cv::putText(frame, "FPS: " + std::to_string(fps), cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

        cv::imshow("video_demo", frame);
        if (cv::waitKey(1) == 27) { // ESC 退出
            break;
        }
    }
    cap.release();
    return 0;
}
