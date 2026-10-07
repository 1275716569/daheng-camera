//   枚举设备 -> 按序列号打开 -> 配置参数 -> 取流显示 -> 键盘调参
//
// 用法:
//   ./daheng_demo                 枚举所有大恒相机,打印序列号
//   ./daheng_demo <序列号>         打开指定相机取流
//   ./daheng_demo <序列号> <onnx>  打开相机并使用指定 YOLO ONNX 模型
//
//   e / d  增大 / 减小曝光时间(步长 25 us)
//   a / q  增大 / 减小增益(步长 0.1 dB)
//   s / w  增大 / 减小伽马(步长 0.1,失败说明该型号无此节点)
//   ESC    退出
//


// 本文件是一个最小可运行示例,串联了大恒相机的设备发现、参数配置、取流、
// Bayer 解码、OpenCV 显示和键盘调参流程。示例程序采用同步取帧,便于逐步观察
// SDK 调用顺序;实际业务程序还应在断流或异常时增加重连策略。
#include <sdk/GxIAPI.h>      // 大恒 Galaxy SDK 主接口(仓库 camera_stream/include/sdk/daheng/)
#include <sdk/DxImageProc.h> // 图像处理接口:DxRaw8toRGB24Ex 等

#include "rm_log.h"

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>
#include <array>
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace {

struct ArmorDetection {
    cv::Rect box;
    float confidence;
};

// 单根灯条的几何信息。endpoints 是沿灯条长度方向的两个端点，
// 因此它们可以直接用于绘制灯条中心线或后续姿态计算。
struct LightBar {
    cv::Point2f center;
    cv::Point2f endpoints[2];
    float length = 0.0F;
    float width = 0.0F;
    float angle = 0.0F;
    float colorScore = 0.0F;
};

// 从旋转矩形中提取灯条长度方向的两个端点。
// minAreaRect 的短边中点分别对应灯条的两端，比直接取轮廓外接矩形更稳定。
std::array<cv::Point2f, 2> GetLightBarEndpoints(const cv::RotatedRect &rect) {
    cv::Point2f corners[4];
    rect.points(corners);

    const float edge01 = cv::norm(corners[0] - corners[1]);
    const float edge12 = cv::norm(corners[1] - corners[2]);
    const int shortEdgeStart = edge01 <= edge12 ? 0 : 1;
    const int otherShortEdgeStart = (shortEdgeStart + 2) % 4;

    const cv::Point2f first =
        (corners[shortEdgeStart] + corners[(shortEdgeStart + 1) % 4]) * 0.5F;
    const cv::Point2f second =
        (corners[otherShortEdgeStart] + corners[(otherShortEdgeStart + 1) % 4]) * 0.5F;
    return {first, second};
}

// 在指定的 YOLO 装甲板框内检测左右灯条并返回一组配对结果。
// 红色和蓝色使用不同的 HSV 色相区间，最后合并成同一张候选掩膜，
// 因而不需要预先知道当前装甲板是哪一种灯光颜色。
bool DetectArmorLightEndpoints(const cv::Mat &image,
                               const cv::Rect &armorBox,
                               LightBar &leftLight,
                               LightBar &rightLight) {
    if (image.empty() || image.type() != CV_8UC3) {
        return false;
    }

    // YOLO 框可能贴近图像边缘，先裁剪到有效图像范围，避免 ROI 越界。
    const cv::Rect imageBounds(0, 0, image.cols, image.rows);
    const cv::Rect roi = armorBox & imageBounds;
    if (roi.width < 10 || roi.height < 10) {
        return false;
    }
    const cv::Mat roiImage = image(roi);

    cv::Mat hsv;
    cv::cvtColor(roiImage, hsv, cv::COLOR_BGR2HSV);

    cv::Mat redLow, redHigh, blue, colorMask;
    // 红灯边缘可能偏黄、蓝灯可能偏青，适当放宽色相范围。
    // 低饱和度的白色过曝中心不直接作为颜色证据。
    cv::inRange(hsv, cv::Scalar(0, 50, 80), cv::Scalar(35, 255, 255), redLow);
    cv::inRange(hsv, cv::Scalar(165, 50, 80), cv::Scalar(179, 255, 255), redHigh);
    cv::inRange(hsv, cv::Scalar(85, 50, 80), cv::Scalar(135, 255, 255), blue);
    // 转为浮点通道再比较比例，避免 8 位通道乘法饱和影响判断。
    cv::Mat floatBgr;
    roiImage.convertTo(floatBgr, CV_32F);
    std::vector<cv::Mat> channels;
    cv::split(floatBgr, channels);
    const cv::Mat redDominant = (channels[2] > 80.0F) &
                               (channels[2] > channels[0] * 1.25F);
    const cv::Mat blueDominant = (channels[0] > 80.0F) &
                                (channels[0] > channels[2] * 1.20F);
    cv::bitwise_and(redLow, redDominant, redLow);
    cv::bitwise_and(redHigh, redDominant, redHigh);
    cv::bitwise_and(blue, blueDominant, blue);
    cv::bitwise_or(redLow, redHigh, colorMask);
    cv::bitwise_or(colorMask, blue, colorMask);

    // 颜色相同但亮度不足的背景物体不作为灯条候选。
    // HSV 的 V 通道范围是 0~255；150 是当前工业相机画面的初始经验值。
    // 实机过暗时可降到 120，环境干扰较多时可提高到 170~190。
    constexpr int kMinimumLightValue = 150;
    cv::Mat brightMask;
    cv::inRange(hsv,
                cv::Scalar(0, 0, kMinimumLightValue),
                cv::Scalar(180, 255, 255),
                brightMask);
    cv::bitwise_and(colorMask, brightMask, colorMask);

    // 开运算清除孤立噪点，闭运算连接同一灯条内部的断裂区域。
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
    cv::morphologyEx(colorMask, colorMask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(colorMask, colorMask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(colorMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<LightBar> candidates;
    for (const auto &contour : contours) {
        const double area = cv::contourArea(contour);
        if (area < 40.0) {
            continue;
        }

        const cv::RotatedRect rect = cv::minAreaRect(contour);
        const float length = std::max(rect.size.width, rect.size.height);
        const float width = std::min(rect.size.width, rect.size.height);
        if (width < 2.0F || length / width < 2.5F || length / width > 18.0F) {
            continue;
        }

        const float fillRatio = static_cast<float>(area / (length * width));
        if (fillRatio < 0.30F) {
            continue;
        }

        // 凸度可以排除颜色相同但形状破碎或高度不规则的背景物体。
        std::vector<cv::Point> hull;
        cv::convexHull(contour, hull);
        const double hullArea = cv::contourArea(hull);
        const double solidity = hullArea > 1.0 ? area / hullArea : 0.0;
        if (solidity < 0.75) {
            continue;
        }

        // 同时检查轮廓区域的平均饱和度和平均亮度，避免仅靠少量亮色像素成轮廓。
        const cv::Rect candidateRoi = rect.boundingRect() &
                                      cv::Rect(0, 0, roiImage.cols, roiImage.rows);
        if (candidateRoi.empty()) {
            continue;
        }
        const cv::Scalar meanHsv = cv::mean(hsv(candidateRoi), colorMask(candidateRoi));
        if (meanHsv[1] < 50.0 || meanHsv[2] < kMinimumLightValue) {
            continue;
        }

        const auto endpoints = GetLightBarEndpoints(rect);
        candidates.push_back({rect.center, {endpoints[0], endpoints[1]},
                              length, width, rect.angle,
                              static_cast<float>(meanHsv[1] * meanHsv[2])});
    }

    // 从候选灯条中选择几何关系最符合装甲板结构的一对。
    // 配对使用无方向线段夹角，因此灯条同时向左或向右倾斜时仍可配对。
    float bestCost = std::numeric_limits<float>::max();
    bool found = false;
    for (size_t i = 0; i < candidates.size(); ++i) {
        for (size_t j = i + 1; j < candidates.size(); ++j) {
            LightBar a = candidates[i];
            LightBar b = candidates[j];
            const float meanLength = (a.length + b.length) * 0.5F;
            if (meanLength <= 0.0F) {
                continue;
            }

            // 端点定义了灯条的长轴方向；归一化后用 |dot| 判断无方向夹角。
            const cv::Point2f directionA = a.endpoints[1] - a.endpoints[0];
            const cv::Point2f directionB = b.endpoints[1] - b.endpoints[0];
            const float normA = cv::norm(directionA);
            const float normB = cv::norm(directionB);
            if (normA <= 1.0F || normB <= 1.0F) {
                continue;
            }
            const cv::Point2f unitA = directionA * (1.0F / normA);
            const cv::Point2f unitB = directionB * (1.0F / normB);
            const float parallelCos = std::abs(unitA.dot(unitB));
            constexpr float kParallelAngleDeg = 25.0F;
            const float minParallelCos =
                std::cos(kParallelAngleDeg * static_cast<float>(CV_PI) / 180.0F);
            if (parallelCos < minParallelCos) {
                continue;
            }

            // 将两中心连线分解为灯条长轴方向和法向方向。
            const cv::Point2f centerDelta = b.center - a.center;
            const float alongOffset = std::abs(centerDelta.dot(unitA));
            const float normalOffset = std::abs(unitA.x * centerDelta.y -
                                                unitA.y * centerDelta.x);
            const float lengthGap = std::abs(a.length - b.length) / meanLength;
            const float widthGap = std::abs(a.width - b.width) /
                                   std::max(a.width, b.width);

            // 法向间距应明显大于灯条宽度，且不能远到跨越整个 ROI；
            // 长轴方向允许一定错位，以适应透视和装甲板侧视情况。
            if (normalOffset < meanLength * 0.5F ||
                normalOffset > meanLength * 12.0F ||
                alongOffset > meanLength * 1.2F ||
                lengthGap > 0.55F || widthGap > 0.75F) {
                continue;
            }

            const float angleCost = 1.0F - parallelCos;
            const float cost = angleCost * 2.0F +
                               alongOffset / meanLength + lengthGap + widthGap;
            if (cost < bestCost) {
                bestCost = cost;
                // 输出仍按图像 x 坐标命名左右，几何筛选本身不依赖水平轴。
                if (a.center.x <= b.center.x) {
                    leftLight = a;
                    rightLight = b;
                } else {
                    leftLight = b;
                    rightLight = a;
                }
                found = true;
            }
        }
    }
    if (!found) {
        return false;
    }

    // 候选点是在 ROI 局部坐标系中计算的，返回前转换回整幅图像坐标。
    const cv::Point2f offset(static_cast<float>(roi.x), static_cast<float>(roi.y));
    leftLight.center += offset;
    rightLight.center += offset;
    for (int i = 0; i < 2; ++i) {
        leftLight.endpoints[i] += offset;
        rightLight.endpoints[i] += offset;
    }
    return true;
}

// Ultralytics YOLO ONNX 输出为 [x, y, w, h, score]，输入使用 letterbox 保持比例。
std::vector<ArmorDetection> DetectArmor(cv::dnn::Net &net, const cv::Mat &image,
                                        float conf_threshold = 0.35f,
                                        float nms_threshold = 0.45f) {
    constexpr int input_size = 640;
    const float scale = std::min(input_size / static_cast<float>(image.cols),
                                 input_size / static_cast<float>(image.rows));
    const int resized_width = static_cast<int>(std::round(image.cols * scale));
    const int resized_height = static_cast<int>(std::round(image.rows * scale));
    cv::Mat resized;
    cv::resize(image, resized, cv::Size(resized_width, resized_height));
    cv::Mat letterbox(input_size, input_size, CV_8UC3, cv::Scalar(114, 114, 114));
    const int pad_x = (input_size - resized_width) / 2;
    const int pad_y = (input_size - resized_height) / 2;
    resized.copyTo(letterbox(cv::Rect(pad_x, pad_y, resized_width, resized_height)));

    cv::Mat blob = cv::dnn::blobFromImage(letterbox, 1.0 / 255.0,
                                          cv::Size(input_size, input_size),
                                          cv::Scalar(), true, false, CV_32F);
    net.setInput(blob);
    cv::Mat output = net.forward();
    cv::Mat rows = output.reshape(1, output.size[1]); // 5 x 8400

    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    for (int i = 0; i < rows.cols; ++i) {
        const float score = rows.at<float>(4, i);
        if (score < conf_threshold) {
            continue;
        }
        const float cx = rows.at<float>(0, i);
        const float cy = rows.at<float>(1, i);
        const float width = rows.at<float>(2, i);
        const float height = rows.at<float>(3, i);
        int x = static_cast<int>((cx - width * 0.5f - pad_x) / scale);
        int y = static_cast<int>((cy - height * 0.5f - pad_y) / scale);
        int w = static_cast<int>(width / scale);
        int h = static_cast<int>(height / scale);
        cv::Rect box(x, y, w, h);
        box &= cv::Rect(0, 0, image.cols, image.rows);
        if (box.area() > 0) {
            boxes.push_back(box);
            scores.push_back(score);
        }
    }

    std::vector<int> kept;
    cv::dnn::NMSBoxes(boxes, scores, conf_threshold, nms_threshold, kept);
    std::vector<ArmorDetection> detections;
    detections.reserve(kept.size());
    for (int index : kept) {
        detections.push_back({boxes[index], scores[index]});
    }
    return detections;
}

// ----------------------------------------------------------------------------
// 错误码转描述字符串。大恒 SDK 提供 GXGetLastError,可以直接拿到人类可读的
// 错误原因(对比海康只能拿十六进制错误码自己查表)。
// 获取 SDK 错误描述后，使用 RM_LOG_ERROR 记录接口、状态码和原因。
// ----------------------------------------------------------------------------
std::string GetErrorString(GX_STATUS emErrorStatus) {
    char *error_info = nullptr;
    size_t size = 0;
    GX_STATUS emStatus = GXGetLastError(&emErrorStatus, nullptr, &size); // 第一次:只取长度
    if (emStatus != GX_STATUS_SUCCESS) {
        return "<Error when calling GXGetLastError>";
    }
    error_info = new char[size];
    emStatus = GXGetLastError(&emErrorStatus, error_info, &size);        // 第二次:取内容
    std::string error_string = error_info != nullptr ? error_info : "";
    delete[] error_info;
    return emStatus == GX_STATUS_SUCCESS ? error_string : "<Error when calling GXGetLastError>";
}

// ----------------------------------------------------------------------------
// [练习1-大恒版] 枚举设备。
// 大恒没有"设备列表"结构体,而是用 1 开始的序号逐台查询(注意:海康是 0 开始!),
// 这是两套 SDK 最容易踩的差异之一。
// 返回每台设备的 (序列号, 型号) 列表。
// ----------------------------------------------------------------------------
struct DeviceInfo {
    // 只保存后续打开设备所需的稳定标识,避免把 SDK 内部结构体带出枚举函数。
    std::string serial;
    std::string model;
};

// 刷新 SDK 的设备列表并收集 USB3(U3V)设备的序列号和型号。
// 返回值为空既表示没有设备,也表示枚举过程中发生了错误。
std::vector<DeviceInfo> EnumerateDevices() {
    uint32_t device_num = 0;
    // 第二个参数是枚举超时(ms):GigE 相机响应慢,官方建议至少 1000
    GX_STATUS emStatus = GXUpdateAllDeviceList(&device_num, 1000);
    if (emStatus != GX_STATUS_SUCCESS) {
        const std::string error = GetErrorString(emStatus);
        RM_LOG_ERROR("GXUpdateAllDeviceList 枚举失败 (status = {}): {}", emStatus, error);
        RM_LOG_INFO("请检查:1) 相机上电 2) USB3.0 口 3) 设备权限");
        return {};
    }
    if (device_num == 0) {
        RM_LOG_WARN("未找到设备，请检查:1) 相机上电 2) USB3.0 口 3) 设备权限");
        return {};
    }

    std::vector<DeviceInfo> devices;
    RM_LOG_INFO("共找到 {} 台设备:", device_num);
    for (uint32_t i = 1; i <= device_num; ++i) { // 序号从 1 开始!
        GX_DEVICE_INFO info;
        memset(&info, 0, sizeof(GX_DEVICE_INFO));
        emStatus = GXGetDeviceInfo(i, &info);
        if (emStatus != GX_STATUS_SUCCESS) {
            continue;
        }
        // 本项目只支持 USB3 相机(U3V);GigE 相机序列号在 stGigEDevInfo 里
        if (info.emDevType == GX_DEVICE_CLASS_U3V) {
            auto &u3v = info.DevInfo.stU3VDevInfo;
            // SDK 里这些字段是 unsigned char[64],转成 std::string 需要显式强转
            RM_LOG_INFO("  [{}] 型号: {}  序列号: {}", i,
                        reinterpret_cast<const char *>(u3v.chModelName),
                        reinterpret_cast<const char *>(u3v.chSerialNumber));
            devices.push_back({reinterpret_cast<const char *>(u3v.chSerialNumber),
                               reinterpret_cast<const char *>(u3v.chModelName)});
        }
    }
    return devices;
}

int FindDeviceIndexBySerial(const std::vector<DeviceInfo> &devices, const std::string &serial) {
    // 设备序列号是用户可见的选择条件;SDK 打开接口需要的是从 1 开始的索引,
    // 因此这里把 vector 的 0 基下标转换回 SDK 的设备序号。
    for (size_t i = 0; i < devices.size(); ++i) {
        if (devices[i].serial == serial) {
            return static_cast<int>(i) + 1; // 转回 1 开始的大恒设备序号
        }
    }
    return -1;
}

// ----------------------------------------------------------------------------
// 参数调教(与海康版同一套"读-改-限幅-写"模式,仅 API 前缀不同)
// ----------------------------------------------------------------------------
bool AddExposureTime(GX_DEV_HANDLE device, double delta_us) {
    // 先读取当前值再修改,这样每次按键都是相对调节,并且可以使用节点实际值
    // 作为下一次调节的基准。这里的上下限是示例程序设定的安全范围。
    GX_FLOAT_VALUE node;
    memset(&node, 0, sizeof(GX_FLOAT_VALUE));
    GX_STATUS emStatus = GXGetFloatValue(device, "ExposureTime", &node);
    if (emStatus != GX_STATUS_SUCCESS) {
        const std::string error = GetErrorString(emStatus);
        RM_LOG_WARN("GXGetFloatValue ExposureTime 失败 (status = {}): {}", emStatus, error);
        return false;
    }

    double value = node.dCurValue + delta_us;
    value = value < 1.0 ? 1.0 : (value > 10000.0 ? 10000.0 : value);

    emStatus = GXSetFloatValue(device, "ExposureTime", value);
    if (emStatus != GX_STATUS_SUCCESS) {
        const std::string error = GetErrorString(emStatus);
        RM_LOG_WARN("GXSetFloatValue ExposureTime 失败 (status = {}): {}", emStatus, error);
        return false;
    }
    RM_LOG_INFO("曝光时间 -> {} us", value);
    return true;
}

bool AddGain(GX_DEV_HANDLE device, double delta_db) {
    // 增益使用 dB 表示;读写都通过同一个 Float 节点完成,失败时不改变相机状态。
    GX_FLOAT_VALUE node;
    memset(&node, 0, sizeof(GX_FLOAT_VALUE));
    GX_STATUS emStatus = GXGetFloatValue(device, "Gain", &node);
    if (emStatus != GX_STATUS_SUCCESS) {
        const std::string error = GetErrorString(emStatus);
        RM_LOG_WARN("GXGetFloatValue Gain 失败 (status = {}): {}", emStatus, error);
        return false;
    }

    double value = node.dCurValue + delta_db;
    value = value < 0.0 ? 0.0 : (value > 32.0 ? 32.0 : value);

    emStatus = GXSetFloatValue(device, "Gain", value);
    if (emStatus != GX_STATUS_SUCCESS) {
        const std::string error = GetErrorString(emStatus);
        RM_LOG_WARN("GXSetFloatValue Gain 失败 (status = {}): {}", emStatus, error);
        return false;
    }
    RM_LOG_INFO("增益 -> {} dB", value);
    return true;
}

bool AddGamma(GX_DEV_HANDLE device, double delta) {
    // Gamma 并非所有型号都提供,因此获取或设置失败只记录警告并返回 false,
    // 不影响主循环继续显示图像。
    GX_FLOAT_VALUE node;
    memset(&node, 0, sizeof(GX_FLOAT_VALUE));
    GX_STATUS emStatus = GXGetFloatValue(device, "Gamma", &node);
    if (emStatus != GX_STATUS_SUCCESS) {
        const std::string error = GetErrorString(emStatus);
        RM_LOG_WARN("GXGetFloatValue Gamma 失败 (status = {}): {}", emStatus, error);
        return false;
    }

    double value = node.dCurValue + delta;
    value = value < 0.1 ? 0.1 : (value > 3.0 ? 3.0 : value);

    emStatus = GXSetFloatValue(device, "Gamma", value);
    if (emStatus != GX_STATUS_SUCCESS) {
        const std::string error = GetErrorString(emStatus);
        RM_LOG_WARN("GXSetFloatValue Gamma 失败 (status = {}): {}", emStatus, error);
        return false;
    }
    RM_LOG_INFO("伽马 -> {}", value);
    return true;
}

} // namespace

int main(int argc, char *argv[]) {
    // 必须先初始化日志，再调用相机 SDK；路径相对于程序的运行目录。
    try {
        INIT_LOG("logs/daheng_demo.log", "info", "debug", "debug");
    } catch (const std::exception &error) {
        std::fprintf(stderr, "日志初始化失败 (logs/daheng_demo.log): %s\n", error.what());
        return 1;
    }
    // 覆盖正常退出和所有提前 return；先提交刷新，再等待异步日志处理完毕。
    struct LogCleanup {
        ~LogCleanup() {
            utils::RMLOG::instance().getLogger()->flush();
            utils::RMLOG::instance().Close();
        }
    } log_cleanup;
    RM_LOG_INFO("daheng_demo 启动");

    // 大恒 SDK 需要先做一次全局库初始化(海康没有这一步)。从这里开始的所有
    // 设备和数据流句柄都依赖该库,退出前必须调用 GXCloseLib。
    GX_STATUS emStatus = GXInitLib();
    RM_LOG_DEBUG("GXInitLib status = {}", emStatus);
    if (emStatus != GX_STATUS_SUCCESS) {
        const std::string error = GetErrorString(emStatus);
        RM_LOG_ERROR("GXInitLib 失败 (status = {}): {}", emStatus, error);
        return -1;
    }

    // 练习 1:枚举。即使命令行只要求打印设备,也要完成一次枚举刷新以获得
    // 最新的在线设备信息。
    auto devices = EnumerateDevices();
    if (devices.empty()) {
        GXCloseLib(); // 记得与 InitLib 配对
        return -1;
    }
    if (argc < 2) {
        RM_LOG_INFO("用法: ./daheng_demo <相机序列号>");
        GXCloseLib();
        return 0;
    }
    const std::string serial = argv[1];

    // 练习 2:打开设备(按 1 开始的序号)。先在已枚举列表中校验序列号,
    // 再调用 SDK,避免把无效索引交给驱动层。
    int device_index = FindDeviceIndexBySerial(devices, serial);
    if (device_index < 0) {
        RM_LOG_ERROR("序列号 {} 不在枚举列表中!", serial);
        GXCloseLib();
        return -1;
    }

    GX_DEV_HANDLE device = nullptr;
    emStatus = GXOpenDeviceByIndex(device_index, &device);
    if (emStatus != GX_STATUS_SUCCESS) {
        const std::string error = GetErrorString(emStatus);
        RM_LOG_ERROR("GXOpenDeviceByIndex (status = {}): {}", emStatus, error);
        GXCloseLib();
        return -1;
    }
    RM_LOG_INFO("打开相机 {} 成功", serial);

    // ------------------------------------------------------------------
    // 配置相机(对照 daheng_camera_stream.cpp 构造函数 + ConnectCamera):
    // ------------------------------------------------------------------
    // 1. 固定白平衡，避免背景变化导致灯条色相逐帧漂移。
    // 不在彩色灯条场景中自动校白；沿用相机已有的白平衡比例。
    GXSetEnumValueByString(device, "ExposureAuto", "Off");
    GXSetEnumValueByString(device, "GainAuto", "Off");
    emStatus = GXSetEnumValueByString(device, "BalanceWhiteAuto", "Off");
    if (emStatus != GX_STATUS_SUCCESS) {
        RM_LOG_WARN("关闭自动白平衡失败: {}", GetErrorString(emStatus));
    }

    // 防过曝初始参数：曝光只降低到最多 3000 us，增益设为设备最小值。
    // 每个节点都先查询范围；节点不支持或设置失败时记录日志，不中断采集。
    // Gamma 恢复为 1，曝光和增益仍可使用原有快捷键现场微调。
    const auto setInitialFloat = [&](const char *name, double requested,
                                     bool onlyReduce, bool useMinimum) {
        GX_FLOAT_VALUE node{};
        GX_STATUS status = GXGetFloatValue(device, name, &node);
        if (status != GX_STATUS_SUCCESS) {
            RM_LOG_WARN("读取 {} 失败: {}", name, GetErrorString(status));
            return;
        }
        double target = useMinimum ? node.dMin : requested;
        if (onlyReduce) {
            target = std::min(target, node.dCurValue);
        }
        target = std::clamp(target, node.dMin, node.dMax);
        status = GXSetFloatValue(device, name, target);
        if (status != GX_STATUS_SUCCESS) {
            RM_LOG_WARN("设置 {} 失败: {}", name, GetErrorString(status));
        } else {
            RM_LOG_INFO("{} 初始值 {} -> {}", name, node.dCurValue, target);
        }
    };
    setInitialFloat("ExposureTime", 3000.0, true, false);
    setInitialFloat("Gain", 0.0, false, true);
    setInitialFloat("Gamma", 1.0, false, false);

    // 2. 连续采集 + 关触发
    emStatus = GXSetEnumValueByString(device, "AcquisitionMode", "Continuous");
    if (emStatus != GX_STATUS_SUCCESS) {
        const std::string error = GetErrorString(emStatus);
        RM_LOG_WARN("GXSetEnumValueByString AcquisitionMode 失败 (status = {}): {}", emStatus, error);
    }
    emStatus = GXSetEnumValueByString(device, "TriggerMode", "Off");
    if (emStatus != GX_STATUS_SUCCESS) {
        const std::string error = GetErrorString(emStatus);
        RM_LOG_WARN("GXSetEnumValueByString TriggerMode 失败 (status = {}): {}", emStatus, error);
    }

    // 3. 像素格式:传感器原始数据 Bayer RG8(与海康版同一思路)
    emStatus = GXSetEnumValue(device, "PixelFormat", GX_PIXEL_FORMAT_BAYER_RG8);
    if (emStatus != GX_STATUS_SUCCESS) {
        const std::string error = GetErrorString(emStatus);
        RM_LOG_WARN("GXSetEnumValue PixelFormat 失败 (status = {}): {}", emStatus, error);
    }

    // 4. 大恒 USB3 相机出厂限速!把 DeviceLinkThroughputLimit 拉到最大值,
    //    否则帧率只有几十帧。这是大恒相机"帧率上不去"的头号原因。
    GX_INT_VALUE limit_node;
    memset(&limit_node, 0, sizeof(GX_INT_VALUE));
    if (GXGetIntValue(device, "DeviceLinkThroughputLimit", &limit_node) == GX_STATUS_SUCCESS) {
        emStatus = GXSetIntValue(device, "DeviceLinkThroughputLimit", limit_node.nMax);
        if (emStatus != GX_STATUS_SUCCESS) {
            const std::string error = GetErrorString(emStatus);
            RM_LOG_WARN("GXSetIntValue DeviceLinkThroughputLimit 失败 (status = {}): {}", emStatus, error);
        }
    }

    // 5. 读分辨率(设置 PixelFormat 之后读,分辨率会随格式变化)
    GX_INT_VALUE width_node, height_node;
    memset(&width_node, 0, sizeof(GX_INT_VALUE));
    memset(&height_node, 0, sizeof(GX_INT_VALUE));
    GXGetIntValue(device, "Width", &width_node);
    GXGetIntValue(device, "Height", &height_node);
    int width = static_cast<int>(width_node.nCurValue);
    int height = static_cast<int>(height_node.nCurValue);
    RM_LOG_INFO("分辨率 {}x{}", width, height);

    // 6. 大恒的"流"概念:payload(单帧字节数)要从数据流句柄查询。
    //    项目只取 1 号流,多相机场景需要对应到正确的流下标。
    uint32_t stream_num = 0;
    GX_DS_HANDLE stream_handle = nullptr;
    emStatus = GXGetDataStreamNumFromDev(device, &stream_num);
    if (emStatus != GX_STATUS_SUCCESS || stream_num < 1) {
        if (emStatus != GX_STATUS_SUCCESS) {
            const std::string error = GetErrorString(emStatus);
            RM_LOG_ERROR("GXGetDataStreamNumFromDev 获取数据流失败 (status = {}): {}", emStatus, error);
        } else {
            RM_LOG_ERROR("GXGetDataStreamNumFromDev 未返回可用数据流 (stream_num = {})", stream_num);
        }
        GXCloseDevice(device);
        GXCloseLib();
        return -1;
    }
    GXGetDataStreamHandleFromDev(device, 1, &stream_handle);
    uint32_t payload_size = 0;
    GXGetPayLoadSize(stream_handle, &payload_size);

    // 7. 设置 SDK 内部取流缓存个数(5 个),并分配转换缓冲区。转换缓冲区按
    // RGB24 计算,其生命周期覆盖整个取流循环,直到停止数据流后才释放。
    GXSetAcqusitionBufferNumber(device, 5);
    const unsigned int buffer_size = sizeof(unsigned char) * width * height * 3;
    unsigned char *rgb_buffer = static_cast<unsigned char *>(malloc(buffer_size));

    // 8. 大恒做 Bayer 转换前要先查相机的滤镜排列(PixelColorFilter 节点):
    //    RG / GB / GR / BG 四种排列,转换函数必须传对,否则颜色错乱。
    int64_t color_filter = GX_COLOR_FILTER_NONE;
    GX_ENUM_VALUE filter_value;
    memset(&filter_value, 0, sizeof(GX_ENUM_VALUE));
    if (GXGetEnumValue(device, "PixelColorFilter", &filter_value) == GX_STATUS_SUCCESS) {
        color_filter = filter_value.stCurValue.nCurValue;
        RM_LOG_INFO("Bayer PixelColorFilter = {}，转换输出 BGR", color_filter);
    } else {
        RM_LOG_WARN("读取 PixelColorFilter 失败，需检查 Bayer 颜色排列");
    }

    // 9. 开始取流。只有 StreamOn 成功后才能调用 GXDQBuf 获取帧,
    // 失败路径要立即释放前面已经申请的资源。
    emStatus = GXStreamOn(device);
    if (emStatus != GX_STATUS_SUCCESS) {
        const std::string error = GetErrorString(emStatus);
        RM_LOG_ERROR("GXStreamOn (status = {}): {}", emStatus, error);
        free(rgb_buffer);
        GXCloseDevice(device);
        GXCloseLib();
        return -1;
    }

    // 帧率统计
    int frame_count = 0;
    auto fps_window_start = std::chrono::steady_clock::now();
    double fps = 0.0;

    PGX_FRAME_BUFFER frame_buffer = nullptr;
    const std::string model_path = argc >= 3
        ? argv[2]
        : "/home/zhongchengyi/Desktop/desktop/RM/runs/detect/train-3/weights/best.onnx";
    cv::dnn::Net armor_net;
    try {
        armor_net = cv::dnn::readNetFromONNX(model_path);
        armor_net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        armor_net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        RM_LOG_INFO("YOLO 模型加载成功: {}", model_path);
    } catch (const cv::Exception &error) {
        RM_LOG_ERROR("YOLO 模型加载失败 {}: {}", model_path, error.what());
        GXStreamOff(device);
        free(rgb_buffer);
        GXCloseDevice(device);
        GXCloseLib();
        return -1;
    }

    RM_LOG_INFO("取流开始,按 e/d 调曝光、a/q 调增益、s/w 调伽马、ESC 退出");

    // 主循环每次处理一帧:出队 -> 校验 -> Bayer 转换 -> 显示/调参 -> 还队。
    // 无论中间哪一步失败,都要把已出队的 frame_buffer 归还给 SDK。
    while (true) {
        // 1000ms 超时取一帧(DQ = DeQueue,从 SDK 队列取出一帧)
        emStatus = GXDQBuf(device, &frame_buffer, 1000);
        if (emStatus != GX_STATUS_SUCCESS) {
            const std::string error = GetErrorString(emStatus);
            RM_LOG_ERROR("GXDQBuf 超时/失败 (status = {}): {}", emStatus, error);
            break; // 练习程序直接退出;主工程在这里做断流重连(见 1.3.4)
        }

        // 帧状态检查:丢包、传输错误的帧要跳过,但必须照常 QB 还回去!
        if (frame_buffer->nStatus != GX_FRAME_STATUS_SUCCESS) {
            RM_LOG_WARN("帧状态异常: 0x{:x}", static_cast<std::uint32_t>(frame_buffer->nStatus));
            GXQBuf(device, frame_buffer); // 无论帧好坏都要还,否则队列会被掏空
            continue;
        }

        // Bayer RG8 -> BGR24。参数含义:
        //   RAW2RGB_NEIGHBOUR   邻域插值(简单快速;官方还有 2x2/3x3 等可选)
        //   DX_PIXEL_COLOR_FILTER(滤镜排列) 由 PixelColorFilter 节点查询得到
        //   false                不翻转图像
        //   DX_ORDER_BGR         输出 OpenCV 的 BGR 通道序
        VxInt32 dx_status = DxRaw8toRGB24Ex(frame_buffer->pImgBuf, rgb_buffer,
                                            frame_buffer->nWidth, frame_buffer->nHeight,
                                            RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(color_filter),
                                            false, DX_ORDER_BGR);
        if (dx_status != DX_OK) {
            RM_LOG_ERROR("DxRaw8toRGB24Ex 失败: 0x{:x}", static_cast<std::uint32_t>(dx_status));
            GXQBuf(device, frame_buffer); // 同样要还
            continue;
        }

        // image 仅引用转换缓冲区,display 用 clone 保留一份可叠加 FPS 的独立图像;
        // 这样下一帧转换时不会覆盖当前正在显示的数据。
        cv::Mat image(frame_buffer->nHeight, frame_buffer->nWidth, CV_8UC3, rgb_buffer);
        cv::Mat display = image.clone();

        ++frame_count;
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - fps_window_start).count();
        if (elapsed >= 1.0) {
            fps = frame_count / elapsed;
            frame_count = 0;
            fps_window_start = now;
        }
        cv::putText(display, "FPS: " + std::to_string(fps), cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);


        // 先由 YOLO 得到装甲板框，再把每个框作为灯条检测的唯一搜索区域。
        // 这样背景中的红蓝高亮物体不会进入灯条候选集合。
        const std::vector<ArmorDetection> detections = DetectArmor(armor_net, image);
        for (const ArmorDetection &detection : detections) {
            cv::rectangle(display, detection.box, cv::Scalar(0, 0, 255), 2);
            const std::string text = "armor_plate " +
                                     cv::format("%.2f", detection.confidence);
            const int text_y = std::max(detection.box.y - 8, 18);
            cv::putText(display, text, cv::Point(detection.box.x, text_y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);

            LightBar leftLight;
            LightBar rightLight;
            if (!DetectArmorLightEndpoints(image, detection.box, leftLight, rightLight)) {
                continue;
            }

            const cv::Scalar lightColor(0, 255, 255);
            // 四点按凸包边界的环形顺序连接，不再分别按 y 值判断上下。
            // 这样在装甲板旋转、侧视或灯条端点顺序翻转时不会绘制交叉对角线。
            const std::vector<cv::Point2f> borderPoints = {
                leftLight.endpoints[0], leftLight.endpoints[1],
                rightLight.endpoints[0], rightLight.endpoints[1]};
            std::vector<cv::Point2f> borderHull;
            cv::convexHull(borderPoints, borderHull);
            // 重合、共线或某一点落入其余三点内部时，不强行画四边框；
            // 仍保留下面的四点标记和坐标，便于观察检测结果。
            if (borderHull.size() == 4) {
                for (size_t i = 0; i < borderHull.size(); ++i) {
                    cv::line(display, borderHull[i],
                             borderHull[(i + 1) % borderHull.size()],
                             lightColor, 2, cv::LINE_AA);
                }
            }

            // 端点使用独立圆点标记，便于确认坐标是否落在灯条两端。
            for (const cv::Point2f endpoint : leftLight.endpoints) {
                cv::circle(display, endpoint, 5, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
            }
            for (const cv::Point2f endpoint : rightLight.endpoints) {
                cv::circle(display, endpoint, 5, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
            }

            // 在画面中标注四个端点的像素坐标，便于调试和直接读取识别结果。
            const std::array<cv::Point2f, 4> endpoints = {
                leftLight.endpoints[0], leftLight.endpoints[1],
                rightLight.endpoints[0], rightLight.endpoints[1]};
            const std::array<const char *, 4> labels = {"L1", "L2", "R1", "R2"};
            for (size_t i = 0; i < endpoints.size(); ++i) {
                const cv::Point textOrigin(
                    static_cast<int>(endpoints[i].x) + 8,
                    static_cast<int>(endpoints[i].y) - 8);
                const std::string endpointText =
                    std::string(labels[i]) + " (" +
                    std::to_string(static_cast<int>(std::lround(endpoints[i].x))) + "," +
                    std::to_string(static_cast<int>(std::lround(endpoints[i].y))) + ")";
                cv::putText(display, endpointText, textOrigin, cv::FONT_HERSHEY_SIMPLEX,
                            0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
            }
        }

        cv::imshow("daheng_demo", display);

        // waitKey 同时负责处理 HighGUI 事件和读取按键。调参函数内部会再次
        // 从相机节点读取当前值,因此连续按键不会依赖本地缓存状态。
        bool quit = false;
        switch (cv::waitKey(1)) {
            case 27: quit = true; break;                              // ESC
            case 'e': AddExposureTime(device, 25.0); break;           // 增大曝光
            case 'd': AddExposureTime(device, -25.0); break;          // 减小曝光
            case 'a': AddGain(device, 0.1); break;                    // 增大增益
            case 'q': AddGain(device, -0.1); break;                   // 减小增益
            case 's': AddGamma(device, 0.1); break;                   // 增大伽马
            case 'w': AddGamma(device, -0.1); break;                  // 减小伽马
            default: break;
        }

        // 用完后把帧还回 SDK 队列(QB = EnQueue),漏还的症状与海康相同:
        // 队列被掏空,GXDQBuf 永远超时,帧率掉到 0
        GXQBuf(device, frame_buffer);
        if (quit) break;
    }

    // 逆序关闭:停流 -> 释放缓冲 -> 关设备 -> 关库(与 InitLib 配对)
    GXStreamOff(device);
    free(rgb_buffer);
    GXCloseDevice(device);
    GXCloseLib();
    RM_LOG_INFO("相机已关闭");
    return 0;
}
