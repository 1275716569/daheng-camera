#include <sdk/GxIAPI.h>
#include <sdk/DxImageProc.h> // 图像处理接口:DxRaw8toRGB24Ex 等

#include <vector>
#include <cstring>
#include <chrono>

#include <opencv2/opencv.hpp>

#include <iostream>

struct DeviceInfo {
    std::string serial;
    std::string model;
};

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

std::vector<DeviceInfo> EnumerateDevices() {
    uint32_t device_num = 0;
    // 第二个参数是枚举超时(ms):GigE 相机响应慢,官方建议至少 1000
    GX_STATUS emStatus = GXUpdateAllDeviceList(&device_num, 1000);
    if (emStatus != GX_STATUS_SUCCESS || device_num == 0) {
        std::cout << "枚举失败或未找到设备: " << GetErrorString(emStatus) << std::endl
                  << "请检查:1) 相机上电 2) USB3.0 口 3) 设备权限" << std::endl;
        return {};
    }

    std::vector<DeviceInfo> devices;
    std::cout << "共找到 " << device_num << " 台设备:" << std::endl;
    for (uint32_t i = 1; i <= device_num; ++i) { // 序号从 1 开始!
        GX_DEVICE_INFO info;
        memset(&info, 0, sizeof(GX_DEVICE_INFO));
        emStatus = GXGetDeviceInfo(i, &info);
        if (emStatus != GX_STATUS_SUCCESS) {
            continue;
        }
        
        if (info.emDevType == GX_DEVICE_CLASS_U3V) {
            auto &u3v = info.DevInfo.stU3VDevInfo;
            // SDK 里这些字段是 unsigned char[64],转成 std::string 需要显式强转
            std::cout << "  [" << i << "] 型号: " << reinterpret_cast<const char *>(u3v.chModelName)
                      << "  序列号: " << reinterpret_cast<const char *>(u3v.chSerialNumber) << std::endl;
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

bool AddExposureTime(GX_DEV_HANDLE device, double delta_us) {
    GX_FLOAT_VALUE node;
    memset(&node, 0, sizeof(GX_FLOAT_VALUE));
    GX_STATUS emStatus = GXGetFloatValue(device, "ExposureTime", &node);
    if (emStatus != GX_STATUS_SUCCESS) { std::cout << "Get ExposureTime: " << GetErrorString(emStatus) << std::endl; return false; }

    double value = node.dCurValue + delta_us;
    value = value < 1.0 ? 1.0 : (value > 10000.0 ? 10000.0 : value);

    emStatus = GXSetFloatValue(device, "ExposureTime", value);
    if (emStatus != GX_STATUS_SUCCESS) { std::cout << "Set ExposureTime: " << GetErrorString(emStatus) << std::endl; return false; }
    std::cout << "曝光时间 -> " << value << " us" << std::endl;
    return true;
}

bool AddGain(GX_DEV_HANDLE device, double delta_db) {
    GX_FLOAT_VALUE node;
    memset(&node, 0, sizeof(GX_FLOAT_VALUE));
    GX_STATUS emStatus = GXGetFloatValue(device, "Gain", &node);
    if (emStatus != GX_STATUS_SUCCESS) { std::cout << "Get Gain: " << GetErrorString(emStatus) << std::endl; return false; }

    double value = node.dCurValue + delta_db;
    value = value < 0.0 ? 0.0 : (value > 32.0 ? 32.0 : value);

    emStatus = GXSetFloatValue(device, "Gain", value);
    if (emStatus != GX_STATUS_SUCCESS) { std::cout << "Set Gain: " << GetErrorString(emStatus) << std::endl; return false; }
    std::cout << "增益 -> " << value << " dB" << std::endl;
    return true;
}

bool AddGamma(GX_DEV_HANDLE device, double delta) {
    GX_FLOAT_VALUE node;
    memset(&node, 0, sizeof(GX_FLOAT_VALUE));
    GX_STATUS emStatus = GXGetFloatValue(device, "Gamma", &node);
    if (emStatus != GX_STATUS_SUCCESS) { std::cout << "Get Gamma: " << GetErrorString(emStatus) << std::endl; return false; }

    double value = node.dCurValue + delta;
    value = value < 0.1 ? 0.1 : (value > 3.0 ? 3.0 : value);

    emStatus = GXSetFloatValue(device, "Gamma", value);
    if (emStatus != GX_STATUS_SUCCESS) { std::cout << "Set Gamma: " << GetErrorString(emStatus) << std::endl; return false; }
    std::cout << "伽马 -> " << value << std::endl;
    return true;
}

int main(int argc, char *argv[]) {

    GX_STATUS emStatus = GXInitLib();
    std::cout << "GXInitLib status = " << emStatus << std::endl;
    
    auto devices = EnumerateDevices();
    if (devices.empty()) {
    GXCloseLib();
    return 1;
    }

    if (argc < 2) {
    std::cout << "用法: ./daheng_demo <相机序列号>" << std::endl;
    GXCloseLib();
    return 0;
    }
     
    const std::string serial = argv[1];

    int device_index = FindDeviceIndexBySerial(devices, serial);
    if (device_index < 0) {
    std::cout << "serial not found" << std::endl;
    GXCloseLib();
    return 1;
    }

    GX_DEV_HANDLE device = nullptr;
    emStatus = GXOpenDeviceByIndex(device_index, &device);
    if (emStatus != GX_STATUS_SUCCESS) {
        std::cout << "GXOpenDeviceByIndex: " << GetErrorString(emStatus) << std::endl;
        GXCloseLib();
        return -1;
    }
    std::cout << "打开相机 " << serial << " 成功" << std::endl;

    // 配置相机(对照 daheng_camera_stream.cpp 构造函数 + ConnectCamera):
    // ------------------------------------------------------------------
    // 1. 自动挡全关
    GXSetEnumValueByString(device, "ExposureAuto", "Off");
    GXSetEnumValueByString(device, "GainAuto", "Off");
    GXSetEnumValueByString(device, "BalanceWhiteAuto", "Continuous");

    // 2. 连续采集 + 关触发
    emStatus = GXSetEnumValueByString(device, "AcquisitionMode", "Continuous");
    if (emStatus != GX_STATUS_SUCCESS) std::cout << "AcquisitionMode: " << GetErrorString(emStatus) << std::endl;
    emStatus = GXSetEnumValueByString(device, "TriggerMode", "Off");
    if (emStatus != GX_STATUS_SUCCESS) std::cout << "TriggerMode: " << GetErrorString(emStatus) << std::endl;

    // 3. 像素格式:传感器原始数据 Bayer RG8
    emStatus = GXSetEnumValue(device, "PixelFormat", GX_PIXEL_FORMAT_BAYER_RG8);
    if (emStatus != GX_STATUS_SUCCESS) std::cout << "PixelFormat: " << GetErrorString(emStatus) << std::endl;

    // 4. 把 DeviceLinkThroughputLimit 拉到最大值,
    GX_INT_VALUE limit_node;
    memset(&limit_node, 0, sizeof(GX_INT_VALUE));
    if (GXGetIntValue(device, "DeviceLinkThroughputLimit", &limit_node) == GX_STATUS_SUCCESS) {
        emStatus = GXSetIntValue(device, "DeviceLinkThroughputLimit", limit_node.nMax);
        if (emStatus != GX_STATUS_SUCCESS)
            std::cout << "DeviceLinkThroughputLimit: " << GetErrorString(emStatus) << std::endl;
    }

        // 5. 读分辨率(设置 PixelFormat 之后读,分辨率会随格式变化)
    GX_INT_VALUE width_node, height_node;
    memset(&width_node, 0, sizeof(GX_INT_VALUE));
    memset(&height_node, 0, sizeof(GX_INT_VALUE));
    GXGetIntValue(device, "Width", &width_node);
    GXGetIntValue(device, "Height", &height_node);
    int width = static_cast<int>(width_node.nCurValue);
    int height = static_cast<int>(height_node.nCurValue);
    std::cout << "分辨率 " << width << "x" << height << std::endl;

    // 6. 大恒的"流"概念:payload(单帧字节数)要从数据流句柄查询。
    //    项目只取 1 号流,多相机场景需要对应到正确的流下标。
    uint32_t stream_num = 0;
    GX_DS_HANDLE stream_handle = nullptr;
    if (GXGetDataStreamNumFromDev(device, &stream_num) != GX_STATUS_SUCCESS || stream_num < 1) {
        std::cout << "获取数据流失败" << std::endl;
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

    // 8.做 Bayer 转换前要先查相机的滤镜排列(PixelColorFilter 节点):
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
        std::cout << "GXStreamOn: " << GetErrorString(emStatus) << std::endl;
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
    std::cout << "取流开始,按 e/d 调曝光、a/q 调增益、s/w 调伽马、ESC 退出" << std::endl;

    while (true) {
        // 1000ms 超时取一帧(DQ = DeQueue,从 SDK 队列取出一帧)
        emStatus = GXDQBuf(device, &frame_buffer, 1000);
        if (emStatus != GX_STATUS_SUCCESS) {
            std::cout << "GXDQBuf 超时/失败: " << GetErrorString(emStatus) << std::endl;
            break; // 练习程序直接退出;主工程在这里做断流重连(见 1.3.4)
        }

        // 帧状态检查:丢包、传输错误的帧要跳过,但必须照常 QB 还回去!
        if (frame_buffer->nStatus != GX_FRAME_STATUS_SUCCESS) {
            std::cout << "帧状态异常: 0x" << std::hex << frame_buffer->nStatus << std::dec << std::endl;
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
            std::cout << "DxRaw8toRGB24Ex 失败: 0x" << std::hex << dx_status << std::dec << std::endl;
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
    std::cout << "相机已关闭" << std::endl;
    return 0;

}