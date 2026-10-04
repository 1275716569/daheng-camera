//   枚举设备 -> 按序列号打开 -> 配置参数 -> 取流显示 -> 键盘调参
//
// 用法:
//   ./daheng_demo                 枚举所有大恒相机,打印序列号
//   ./daheng_demo <序列号>         打开指定相机取流
//
//   e / d  增大 / 减小曝光时间(步长 25 us)
//   a / q  增大 / 减小增益(步长 0.1 dB)
//   s / w  增大 / 减小伽马(步长 0.1,失败说明该型号无此节点)
//   ESC    退出
//


#include <sdk/GxIAPI.h>      // 大恒 Galaxy SDK 主接口(仓库 camera_stream/include/sdk/daheng/)
#include <sdk/DxImageProc.h> // 图像处理接口:DxRaw8toRGB24Ex 等

#include "rm_log.h"

#include <opencv2/opencv.hpp>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>
#include <string>
#include <vector>

namespace {

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
    std::string serial;
    std::string model;
};

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

    // 大恒 SDK 需要先做一次全局库初始化(海康没有这一步)
    GX_STATUS emStatus = GXInitLib();
    RM_LOG_DEBUG("GXInitLib status = {}", emStatus);
    if (emStatus != GX_STATUS_SUCCESS) {
        const std::string error = GetErrorString(emStatus);
        RM_LOG_ERROR("GXInitLib 失败 (status = {}): {}", emStatus, error);
        return -1;
    }

    // 练习 1:枚举
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

    // 练习 2:打开设备(按 1 开始的序号)
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
    // 1. 自动挡全关(调参前必须关,理由与海康版相同)
    GXSetEnumValueByString(device, "ExposureAuto", "Off");
    GXSetEnumValueByString(device, "GainAuto", "Off");
    GXSetEnumValueByString(device, "BalanceWhiteAuto", "Continuous");

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

    // 7. 设置 SDK 内部取流缓存个数(5 个),并分配转换缓冲区
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
    }

    // 9. 开始取流
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
    RM_LOG_INFO("取流开始,按 e/d 调曝光、a/q 调增益、s/w 调伽马、ESC 退出");

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
        cv::imshow("daheng_demo", display);

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
