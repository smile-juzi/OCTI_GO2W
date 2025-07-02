#include<unistd.h>
#include<cstdlib>
using namespace std;
struct commonFrameHead {
    uint32_t frameType;
    uint32_t source;
    uint32_t dest;
    uint32_t subObj;
};

struct commonFrameData {
    uint32_t hasData;
    uint32_t data1;  // 随意填充
    uint32_t data2;  // 随意填充
    float data3;     // 根据 subObj 返回相应的浓度值
    float data4;     // 根据 subObj 返回相应的浓度值
};

struct commonFrame {
    struct commonFrameHead frameHead;
    struct commonFrameData framedata;
};
//robot command
enum class Command {
    ResumeInspection = 1,  // 恢复巡检
    StopInspection,    // 停止巡检
    MoveForward,       // 前进
    MoveBackward,      // 后退
    TurnLeft,          // 左转
    TurnRight,         // 右转
    StandUp,           // 起立
    BowDown,           // 俯下
    MoveLeft,          // 左移
    MoveRight          // 右移
};
struct Gas_concentration {
    float globalO2;  // O2 浓度
    float globalCO;  // CO 浓度
    float globalH2S; // H2S 浓度
    float globalEX;  // EX 浓度
};

// 定义仪表数据结构体
struct Meter_data {
    float XIAOFANG;  // 仪表1读数
    float AIR;       // 仪表2读数
    float JIXIE;     // 仪表3读数
};

// 定义温度信息结构体
struct Tem_data {
    float average_tem;  // 平均温度信息
    float medium_tem;
};

// 定义视觉数据结构体
struct Vision_data {
    uint32_t face;    // 人脸信息
    uint32_t smoke;   // 吸烟信息
    uint32_t hat;     // 安全帽信息
    uint32_t inf;     // 人员入侵信息
    uint32_t flame;
};

// 定义处理数据参数结构体
struct ProcessDataParams {
    struct Gas_concentration gas;
    struct Meter_data meter;
    struct Tem_data tem;
    struct Vision_data vision;
};
//巡检位置坐标
struct pathpoint
    {
        uint32_t number;//
        uint32_t seq;
        float x;
        float y;
        //double yaw;
        uint32_t keyPointType;
    };
struct Robot_Status{
 uint32_t work_status;
 struct pathpoint now_point;
 uint32_t task_type;//
 string task_begin_time;
 string task_end_time;
};
#pragma pack(push, 1) 
struct PGMHeader {
    uint32_t magic;      // 魔数标识"P5P2"
    uint32_t width;      // 图像宽度
    uint32_t height;     // 图像高度
    uint32_t max_gray;   // 最大灰度值
    uint32_t data_size;  // 像素数据总字节数
    uint32_t map_number;
    uint32_t datalen;
    float  resolution; 
};
struct PGMData{
    PGMHeader header;
    uint8_t* pixel_data;  // 像素数据
};
#pragma pack(pop)

