#include<iostream>
#include <queue>
#include <chrono>
#include <mutex>
#include <thread>
#include<sys/socket.h>
#include<netinet/in.h>
#include<arpa/inet.h>
#include<cstring>
#include<unistd.h>
#include<cstdlib>
#include<nlohmann/json.hpp>
#include<cstdint>
#include"httplib.h"
#include<unordered_map>
#include"data_type.h"
#include<nlohmann/adl_serializer.hpp>
#include <filesystem>
#include <system_error>
#include <sys/mman.h>
#include"base64_code.h"
#include"data_handle.h"
#include <opencv2/opencv.hpp>
#define UDP_PORT 8111
#define HTTP_PORT 8080
#define destination_ip "127.0.0.1"
#define INF_PORT 8870
#define ROBOT_PORT 8880
#define MAP_PATH "./map/scans2_ps.pgm"
#define video_path "udpsrc address=230.1.1.1 port=1720 multicast-iface=enP4p65s0 ! application/x-rtp, media=video, encoding-name=H264 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,width=1280,height=720,format=BGR ! appsink drop=1"

using  namespace httplib;
using json=nlohmann::json;
std::atomic<bool> stopReceiving(false);
std::vector<struct pathpoint> globalPathPoints;
std::vector<struct pathpoint> global_revPathPoints;
//data
struct ProcessDataParams processdata={
    {0.0f,0.0f, 0.0f, 0.0f},
    // 初始化仪表数据
    {0.0f, 0.0f, 0.0f},
    // 初始化温度信息
    {0.0f, 0.0f},
    // 初始化视觉数据
    {0, 0, 0, 0, 0}
};

struct pathpoint revPothPoints;
std::atomic<bool> dataProcessed(false); // 原子变量确保线程安全
// 定义全局变量存储数据
std::atomic<bool> pgmdataprocess(false);


// 映射命令到相应的请求帧
std::unordered_map<Command, std::string> commandFrameMap = {
    {Command::ResumeInspection, "ResumeInspection_Frame"},
    {Command::StopInspection, "StopInspection_Frame"},
    {Command::MoveForward, "MoveForward_Frame"},
    {Command::MoveBackward, "MoveBackward_Frame"},
    {Command::TurnLeft, "TurnLeft_Frame"},
    {Command::TurnRight, "TurnRight_Frame"},
    {Command::StandUp, "StandUp_Frame"},
    {Command::BowDown, "BowDown_Frame"},
    {Command::MoveLeft, "MoveLeft_Frame"},
    {Command::MoveRight, "MoveRight_Frame"}
};



commonFrame request_yanwu1Frame = {
    {0, 0, 4, 0},       // 初始化 frameHead，分别对应 frameType, source, dest 和 subObj
//    {0, 0, 0, 0.0f, 0.0f} // 初始化 framedata，分别对应 hasData, data1, data2, data3 和 data4
};

commonFrame request_yanwu2Frame = {
    {0, 0, 4, 1},       // 初始化 frameHead，分别对应 frameType, source, dest 和 subObj
//    {0, 0, 0, 0.0f, 0.0f} // 初始化 framedata，分别对应 hasData, data1, data2, data3 和 data4
};

commonFrame request_FACEFrame = {
    {0, 0, 1, 0},       // 初始化 frameHead，分别对应 frameType, source, dest 和 subObj
    {0, 0, 0, 0.0f, 0.0f} // 初始化 framedata，分别对应 hasData, data1, data2, data3 和 data4
};

commonFrame request_SMOKE_HATFrame = {
    {0, 0, 1, 2},       // 初始化 frameHead，分别对应 frameType, source, dest 和 subObj
    {0, 0, 0, 0.0f, 0.0f} // 初始化 framedata，分别对应 hasData, data1, data2, data3 和 data4
};

commonFrame request_METERFrame = {
    {0, 0, 3, 1},       // 初始化 frameHead，分别对应 frameType, source, dest 和 subObj
    {0, 0, 0, 0.0f, 0.0f} // 初始化 framedata，分别对应 hasData, data1, data2, data3 和 data4
};

commonFrame request_INFFrame = {
    {0, 0, 1, 3},       // 初始化 frameHead，分别对应 frameType, source, dest 和 subObj
    {0, 0, 0, 0.0f, 0.0f} // 初始化 framedata，分别对应 hasData, data1, data2, data3 和 data4
};

commonFrame request_TEMFrame = {
    {0, 0, 5, 0},       // 初始化 frameHead，分别对应 frameType, source, dest 和 subObj
    {0, 0, 0, 0.0f, 0.0f} // 初始化 framedata，分别对应 hasData, data1, data2, data3 和 data4
};

commonFrame request_FlameFrame = {
    {0, 0, 1, 4},       // 初始化 frameHead，分别对应 frameType, source, dest 和 subObj
    {0, 0, 0, 0.0f, 0.0f} // 初始化 framedata，分别对应 hasData, data1, data2, data3 和 data4
};

//----------
commonFrame close_infframe = {
        {2, 0, 1, 3}, // 初始化 frameHead，frameType, source, dest 和 subObj
        {0, 2, 0, 0.0f, 0.0f} // 初始化 framedata
};

commonFrame close_TEMframe = {
        {2, 0, 5, 0}, // 初始化 frameHead，frameType, source, dest 和 subObj
        {1, 2, 0, 0.0f, 0.0f} // 初始化 framedata
    };
commonFrame time_frame = {
   
        {2, 0, 2, 0}, // 初始化 frameHead，frameType, source, dest 和 subObj
        {1, 0, 0, 0.0f, 0.0f} // 初始化 framedata
    };

struct Robot_Status robot_status={};
/*std::mutex rev_points_mutex;
void handleSource6(const std::vector<commonFrame> & frames) {
    // 预分配内存优化
    std::vector<pathpoint> temp_points;
    temp_points.reserve(frames.size());

    for (const auto& frame : frames) { // 遍历可变数组
        // 
        if (frame.framedata.hasData != 1 || 
            frame.frameHead.dest != 0 
           ) {
            continue;
        }

        try {
            
            pathpoint new_point{
                .number = frame.frameHead.source,
                .seq =frame.frameHead.data1,
                .x = frame.framedata.data3,
                .y = frame.framedata.data4,
                .keyPointType = frame.framedata.data2
            };

            // 使用emplace_back减少拷贝
            temp_points.emplace_back(new_point);

        } catch (const std::exception& e) {
            std::cerr << "数据转换错误: " << e.what() << std::endl;
        }
    }

    // 线程安全写入全局容器
    {
        std::lock_guard<std::mutex> lock(rev_points_mutex);
        global_revPathPoints.insert(
            global_revPathPoints.end(),
            std::make_move_iterator(temp_points.begin()),
            std::make_move_iterator(temp_points.end())
        );
        
        // 容量优化策略
        if (global_revPathPoints.capacity() > global_revPathPoints.size() * 2) {
            global_revPathPoints.shrink_to_fit();
        }
    }
}*/

// 在全局作用域添加RAII包装器定义
struct MMapGuard {
    void* addr = nullptr;
    size_t length = 0;
    
    MMapGuard(void* ptr, size_t len) : addr(ptr), length(len) {}
    ~MMapGuard() { 
        if(addr) munmap(addr, length); 
    }
    
    // 禁用拷贝构造和赋值
    MMapGuard(const MMapGuard&) = delete;
    MMapGuard& operator=(const MMapGuard&) = delete;
};

int setup_udp_socket(int port) {
    // 创建 UDP 套接字
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "Failed to create UDP socket: " << strerror(errno) << std::endl;
        return -1;
    }

    // 允许在同一端口上多次绑定套接字，即使之前的绑定还未完全释放
    int optval = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) < 0) {
        std::cerr << "Failed to set SO_REUSEADDR: " << strerror(errno) << std::endl;
        close(sockfd);
        return -1;
    }

    //  设置接收缓冲区大小
    int buf_size = 1024 * 1024;
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, &buf_size, sizeof(buf_size)) < 0) {
        std::cerr << "Failed to set SO_RCVBUF: " << strerror(errno) << std::endl;
    }

    sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    //绑定套接字到指定端口
    if (bind(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Failed to bind UDP socket: " << strerror(errno) << std::endl;
        close(sockfd);
        return -1;
    }

    return sockfd;
}

void printCommonFrame(const commonFrame& frame) {
    std::cout << "=== Received Frame ===" << std::endl;
    std::cout << "Frame Head:" << std::endl;
    std::cout << "  frameType: " << frame.frameHead.frameType << std::endl;
    std::cout << "  source: " << frame.frameHead.source << std::endl;
    std::cout << "  dest: " << frame.frameHead.dest << std::endl;
    std::cout << "  subObj: " << frame.frameHead.subObj << std::endl;
    
    std::cout << "Frame Data:" << std::endl;
    std::cout << "  hasData: " << frame.framedata.hasData << std::endl;
    std::cout << "  data1: " << frame.framedata.data1 << std::endl;
    std::cout << "  data2: " << frame.framedata.data2 << std::endl;
    std::cout << "  data3: " << frame.framedata.data3 << std::endl;
    std::cout << "  data4: " << frame.framedata.data4 << std::endl;
    std::cout << "======================" << std::endl;
}

// 打印全局变量值的函数
void printGlobalVariables() {
    std::cout << "Global Variables:" << std::endl;
    std::cout << "O2 Concentration: " << processdata.gas.globalO2 << std::endl;
    std::cout << "CO Concentration: " << processdata.gas.globalCO << std::endl;
    std::cout << "H2S Concentration: " << processdata.gas.globalH2S << std::endl;
    std::cout << "EX Concentration: " <<processdata.gas. globalEX << std::endl;
    std::cout << "XIAOFANG Reading: " <<processdata.meter.XIAOFANG << std::endl;
    std::cout << "AIR Reading: " << processdata.meter.AIR << std::endl;
    std::cout << "JIXIE Reading: " << processdata.meter.JIXIE << std::endl;
    std::cout << "Face Info: " << processdata.vision.face << std::endl;
    std::cout << "Smoke Info: " << processdata.vision.smoke << std::endl;
    std::cout << "Hat Info: " << processdata.vision.hat << std::endl;
    std::cout << "Average Temperature: " << processdata.tem.average_tem << std::endl; 
    std::cout << "Medium_tem Temperature: " << processdata.tem.medium_tem << std::endl; 
    std::cout << "Intrusion Info: " << processdata.vision.inf << std::endl;
}

// 向机器人发送指令
void sendCommand(int sockfd, const commonFrame& commandFrame, const char* ip, uint16_t port)
{
    // 设置服务器地址
    sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);  // 与服务器端口对应
    server_addr.sin_addr.s_addr = inet_addr(ip);  // 指定服务器IP地址

    // 发送控制指令
    int send_len = sendto(sockfd, &commandFrame, sizeof(commandFrame), 0, (struct sockaddr*)&server_addr, sizeof(server_addr));
    if (send_len < 0) {
        std::cerr << "Failed to send control command!" << std::endl;
        // 不关闭套接字，这样可以重用它
    } else {
        std::cout << "Control command sent successfully!" << std::endl;
    }
}

void receiveData(int sockfd) {
    std::cout << "Listening for incoming data..." << std::endl;

    while (!stopReceiving) {
        commonFrame responseFrame;
        sockaddr_in remote_addr;
        socklen_t remote_addr_len = sizeof(remote_addr);

        int recv_len = recvfrom(sockfd, &responseFrame, sizeof(responseFrame), 0, (struct sockaddr*)&remote_addr, &remote_addr_len);
        if (recv_len < 0) {
            std::cerr << "Failed to receive data: " << strerror(errno) << std::endl;
            // 可以添加重试机制
            continue;
        }
        if(responseFrame.frameHead.frameType != 2) {
            printCommonFrame(responseFrame);
            processData(responseFrame,processdata,revPothPoints,global_revPathPoints,robot_status);
            printGlobalVariables();
            dataProcessed = true;
            
        }
        printCommonFrame(responseFrame);
        processData(responseFrame,processdata,revPothPoints,global_revPathPoints,robot_status);
        const bool isComplete = (global_revPathPoints.size() == responseFrame.frameHead.subObj);
        if(responseFrame.frameHead.subObj==1&&responseFrame.frameHead.dest==0){
            robot_status.now_point=revPothPoints;
        }
        //std::cout<<"1"<<global_revPathPoints.size()<<std::endl;
      //  std::cout<<responseFrame.frameHead.subObj<<std::endl;
        
        dataProcessed = isComplete;
    }
        close(sockfd);
}

struct PGMServerState {
    std::mutex data_mutex;
    std::shared_ptr<PGMData> current_data;
    std::atomic<bool> data_ready{false};
    std::condition_variable data_cv;
};

PGMServerState pgm_state;

void pgm_receiver_thread() {
    PGMReceiver receiver;
    if (!receiver.start(1234)) return;

    while (true) {
        PGMDataPtr raw_data;
        if (receiver.get_data(raw_data)) {
            std::lock_guard<std::mutex> lock(pgm_state.data_mutex);
            
            // 直接转移所有权到shared_ptr
            pgm_state.current_data = std::shared_ptr<PGMData>(
                raw_data.release(),  // 释放unique_ptr所有权
                PGMDataDeleter{}     // 保留删除器
            );
            
            pgm_state.data_ready.store(true);
            pgm_state.data_cv.notify_all();
        }
      
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
/*struct PGMHeader {
    std::string magic;
    uint32_t width;
    uint32_t height;
    uint16_t max_value;
    size_t data_offset;  // 像素数据起始偏移量
};

PGMHeader parse_pgm_header(const char* mapped_data, size_t file_size) {
    PGMHeader header;
    size_t offset = 0;
    int header_lines = 0;
    
    // 逐行解析头信息（兼容注释行）
    while(header_lines < 3 && offset < file_size) {
        std::string line;
        // 提取单行（查找换行符0x0A）
        while(offset < file_size && mapped_data[offset] != '\n') {
            line += mapped_data[offset++];
        }
        offset++; // 跳过换行符
        
        if(line.empty()) continue;
        
        // 过滤注释行（
        if(line[0] == '#') {
           // std::cout << "跳过注释: " << line << std::endl;
            continue;
        }
        
        // 解析关键参数
        switch(header_lines) {
            case 0: // 魔数（P5/P2）
                header.magic = line.substr(0,2);
                if(header.magic != "P5" && header.magic != "P2") 
                    throw std::runtime_error("非法PGM魔数: " + header.magic);
                break;
            case 1: { // 宽高
                std::istringstream iss(line);
                if(!(iss >> header.width >> header.height))
                    throw std::runtime_error("尺寸解析失败");
                break;
            }
            case 2: // 最大灰度值
                header.max_value = std::stoi(line);
                if(header.max_value < 1 || header.max_value > 65535)
                    throw std::runtime_error("灰度值越界");
                break;
        }
        header_lines++;
    }
    
    header.data_offset = offset; // 记录数据起始位置
    return header;
}

*/

struct ScheduledTask {
    std::chrono::system_clock::time_point exec_time;
};

struct CompareScheduledTask {
    bool operator()(const ScheduledTask& a, const ScheduledTask& b) {
        return a.exec_time > b.exec_time;
    }
};

std::priority_queue<ScheduledTask, std::vector<ScheduledTask>, CompareScheduledTask> taskQueue;
std::mutex queueMutex;
bool scheduler_running = true;
std::mutex path_mutex;

void scheduler_loop(int socket) {
    while (scheduler_running) {
        auto now = std::chrono::system_clock::now();
        
        {
            std::lock_guard<std::mutex> lock(queueMutex);
            
            while (!taskQueue.empty() && taskQueue.top().exec_time <= now) {
                auto task = taskQueue.top();
                taskQueue.pop();

                // 获取触发时的精确时间
                auto trigger_time = std::chrono::system_clock::now();
                std::time_t trigger_t = std::chrono::system_clock::to_time_t(trigger_time);
                std::tm local_tm = *std::localtime(&trigger_t);
                sendCommand(socket, time_frame, "127.0.0.1", 8111);
                // 格式化输出当前时间
                char time_str[32];
                strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &local_tm);
                std::cout << "[Scheduled Task] Triggered at: " << time_str << std::endl;
               // scheduler_running = false;
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}


using namespace cv;

// 全局变量
//VideoCapture camera;
std::mutex camera_mutex;
//const std::string video_file = "../video/test.mp4";
// 生成 MJPEG 流
/*
void generate_mjpeg_stream(const httplib::Request &, httplib::Response &res) {
    res.set_chunked_content_provider(
        "multipart/x-mixed-replace; boundary=frame",
        [&](size_t , httplib::DataSink &sink) {
            Mat frame;
            {
                std::lock_guard<std::mutex> lock(camera_mutex);
                camera >> frame;
            }

            if (frame.empty()) {
                sink.done();  // 结束流
                return false;
            }

            // 编码为 JPEG
            std::vector<uchar> buffer;
            if (!imencode(".jpg", frame, buffer)) {
                sink.done();
                return false;
            }

            // 构建 HTTP 响应块
            const std::string part = 
                "--frame\r\n"
                "Content-Type: image/jpeg\r\n"
                "Content-Length: " + std::to_string(buffer.size()) + "\r\n\r\n";

            // 分块写入数据
            sink.write(part.data(), part.size());
            sink.write(reinterpret_cast<const char*>(buffer.data()), buffer.size());
            sink.write("\r\n", 2);

            return true;  // 继续发送下一帧
        }
    );
}
*/
bool fileExists(const string& path) {
    struct stat buffer;
    return (stat(path.c_str(), &buffer) == 0);
}

void start_http_server(int udp_socket){
    Server svr;
    //当客户端访问/smoke_concentration路径时，触发该处理函数
    svr.Get("/smoke_concentration",[&](const httplib::Request& req,
        httplib::Response& res){
            //std::cout<<"waiting for data"<<std::endl;
           while(!dataProcessed){
               std::this_thread::sleep_for(std::chrono::milliseconds(10));
                //wait for dataprocess
            }
            json response_data={
                {"O2_concentration",processdata.gas.globalO2},
                {"CO_concentration",processdata.gas.globalCO},
                {"H2S_concentration",processdata.gas.globalH2S},
                {"ex_concentration",processdata.gas.globalEX}
            };
            res.set_content(response_data.dump(),"application/json");
        });
    //客户端发送指令数据
    svr.Post("/vision",[&](const Request& req ,Response& res){
        auto jsonData =json::parse(req.body);
        std::string action = jsonData["action"];
        std::string object = jsonData["object"];
        commonFrame frame={
            {2,0,1,0},
            {0,0,0,0.0f,0.0f}
        };
        //指令映射
        if (object == "face") frame.frameHead.subObj = 0; // FACE
         else if (object == "smoke_hat") frame.frameHead.subObj = 2; // SMOKE_HAT
         else if (object == "inf") frame.frameHead.subObj = 3; // INF
         else if (object == "flame") frame.frameHead.subObj = 4; // INF    
         else {
        res.set_content("Invalid object", "text/plain");
        return;
    }

    // 根据操作填充 data1
        if (action == "open") {
    	if(frame.frameHead.subObj == 3) 
    	{
    		sendCommand(udp_socket, close_TEMframe, "127.0.0.1", 9010);
    		std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        }
        frame.framedata.data1 = static_cast<uint32_t>(1); // OPEN
        } 
        else if (action == "close") 
        {
        frame.framedata.data1 = static_cast<uint32_t>(2); // CLOSE
        } 
        else {
        res.set_content("Invalid action", "text/plain");
        return;
        }
        std::cout<<"frame.framedata.data1 :"<<frame.framedata.data1<<std::endl;
        std::cout<<"frame.frameHead.subObj :"<<frame.frameHead.subObj<<std::endl;
        res.set_content("Command executed: " + action + " " , "text/plain");
        }
    );
    svr.Get("/vision/inf", [&](const Request& req, Response& res) {
        sendCommand(udp_socket,request_INFFrame,"127.0.0.1", INF_PORT);
        
        while (!dataProcessed) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        printf("iray = %d\n", processdata.vision.inf);
        json response_data = {
            {"人员入侵状态", (processdata.vision.inf == 0) ? "无人入侵" : "有人入侵"}
        };

        // 返回JSON格式的响应
        res.set_content(response_data.dump(), "application/json");
        dataProcessed = false;
    }); 
    svr.Get("/robot/temp", [&](const Request& req, Response& res) {
        sendCommand(udp_socket,request_TEMFrame,"127.0.0.1", 9010);
        
        while (!dataProcessed) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
    // 创建 JSON 响应数据
    json response_data = {
        {"average_temperature", processdata.tem.average_tem},
        {"medium_temperature", processdata.tem.medium_tem}
    };

        // 返回JSON格式的响应
        res.set_content(response_data.dump(), "application/json");
        dataProcessed = false;
    });

    svr.Get("/vision/flame", [&](const Request& req, Response& res) {
        sendCommand(udp_socket,request_FlameFrame,"127.0.0.1", 8870);
        
        while (!dataProcessed) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
    // 创建 JSON 响应数据
    json response_data = {
        {"flame_status", (processdata.vision.flame == 1) ? "有火焰" : "无火焰"}  // 根据 flame 的值返回状态
    };

        // 返回JSON格式的响应
        res.set_content(response_data.dump(), "application/json");
        dataProcessed = false;
    });

    svr.Get("/vision/face", [&](const Request& req, Response& res) {
        sendCommand(udp_socket, request_FACEFrame, "127.0.0.1", 8870);
        
        while (!dataProcessed) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        json response_data = {
            {"FACE_concentration", (processdata.vision.face == 9999) ? "unknown" : std::to_string(processdata.vision.face)}
        };
        
        res.set_content(response_data.dump(), "application/json");
        dataProcessed = false;
    });

    svr.Get("/vision/smoke_hat", [&](const Request& req, Response& res) {
        sendCommand(udp_socket, request_SMOKE_HATFrame, "127.0.0.1", 8870);
        
        while (!dataProcessed) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        json response_data = {
            {"smoke_status", (processdata.vision.smoke == 1) ? "有吸烟" : "没有吸烟"},
            {"hat_status", (processdata.vision.hat == 1) ? "有安全帽" : "没有安全帽"}
        };
        
        res.set_content(response_data.dump(), "application/json");
        dataProcessed = false;
    });

    svr.Get("/vision/meter", [&](const Request& req, Response& res) {
        sendCommand(udp_socket, request_METERFrame, "127.0.0.1", 8889);
        
        while (!dataProcessed) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
	
        json response_data = {
            {"消防仪表读数", processdata.meter.XIAOFANG},
            {"空气仪表读数", processdata.meter.AIR},
            {"机械仪表读数", processdata.meter.JIXIE}
        };
        
        res.set_content(response_data.dump(), "application/json");
        dataProcessed = false;
    });

    svr.Get("/get_robot_data",[&](const Request& req,Response&res){
        while(!processData){
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::string Tasktype ;
        std::string Workstatus;
        if(robot_status.task_type==1) Tasktype="定时任务";
        else Tasktype="非定时任务";
        if(robot_status.work_status==1) Workstatus="正常";
        else Workstatus="异常";
        json response_data={
            {"工作状态",Workstatus},
            {"当前巡检位置",{
                           {"巡检路径编号",robot_status.now_point.number},
                           {"巡检点编号",robot_status.now_point.seq},
                           {"坐标x", robot_status.now_point.x},
                           {"坐标y",robot_status.now_point.y},
                           
                           {"巡检点类型", robot_status.now_point.keyPointType}}},
            {"任务类型",Tasktype},
            {"任务开始时间",robot_status.task_begin_time},
            {"任务结束时间",robot_status.task_end_time}
        };
        res.set_content(response_data.dump(),"application/json");
    });
    
    svr.Get("/get_all_pathpoint", [&](const Request& req, Response& res) {
       while (!dataProcessed) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        try {
            // 线程安全访问全局路径点
            std::vector<pathpoint> current_path;
            {
                std::lock_guard<std::mutex> lock(path_mutex);
                current_path = global_revPathPoints; // 复制数据避免长期锁定
            }
    
            // 序列化为JSON数组
            nlohmann::json j_array = nlohmann::json::array();
            for (const auto& point : current_path) {
                j_array.push_back({
                    {"path", point.number},
                    {"point_seq",point.seq},
                    {"x", point.x},
                    {"y", point.y},
                    {"keyPointType", point.keyPointType}
                });
            }
    
            // 设置分块传输编码
            res.set_chunked_content_provider("application/json", 
                [j_array = std::move(j_array)](size_t offset, DataSink& sink) {
                    const std::string json_str = j_array.dump();
                    
                    if(offset >= json_str.size()) {
                        sink.done();
                        return false;
                    }
    
                    size_t chunk_size = std::min(4096UL, json_str.size() - offset);
                    sink.write(json_str.data() + offset, chunk_size);
                    return true;
                },
                [](bool success) { std::cout<<"done"<<std::endl;}
            );
    
        } catch (const std::exception& e) {
            res.status = 500;
            res.set_content(json{{"error", e.what()}}.dump(), "application/json");
        }
    });
    svr.Get("/map", [](const Request& req, Response& res) {
        // 带超时的等待
        std::shared_ptr<PGMData> active_pgm;
        {
            std::unique_lock<std::mutex> lock(pgm_state.data_mutex);
            if (!pgm_state.data_cv.wait_for(lock, std::chrono::seconds(1),
                []{ return pgm_state.data_ready.load(); })) 
            {
                res.status = 504;
                res.set_content("Data Not Ready", "text/plain");
                return;
            }
            active_pgm = pgm_state.current_data;
        }
        
        // 数据有效性检查
        if (!active_pgm || !active_pgm->pixel_data) {
            res.status = 500;
            res.set_content("Invalid Data", "text/plain");
            return;
        }
    
        std::ostringstream header_stream;

	// 标准头
	header_stream << "P5\n"                    // 强制换行符统一为LF
             << active_pgm->header.width << " " 
             << active_pgm->header.height << "\n"
             << active_pgm->header.max_gray << "\n"; 

	// 附加元数据注释（确保在标准头之后）
	header_stream << "# MAP_NUM:" << active_pgm->header.map_number << "\n"
             << "# RESOLUTION:" << active_pgm->header.resolution << "\n";

	std::string header_str = header_stream.str();
        // 构造分块传输上下文
        struct ChunkContext {
            std::string header;
            std::shared_ptr<PGMData> pgm_data;
            size_t total_size;
            size_t header_size;
        };
        
        auto ctx = std::make_shared<ChunkContext>(ChunkContext{
            header_str,
            active_pgm,
            header_str.size() + active_pgm->header.data_size,
            header_str.size()
        });
    
        // 正确设置分块传输
        res.set_chunked_content_provider(
            "image/x-portable-graymap",  // 内容类型
            [ctx](size_t offset, DataSink& sink) -> bool {  // 内容提供器
                try {
                    constexpr size_t CHUNK_SIZE = 131072;
    
                    if (offset >= ctx->total_size) {
                        sink.done();
                        return false;
                    }
    
                    // 判断当前数据位置
                    if (offset < ctx->header_size) {
                        // 发送头部数据
                        size_t remain_header = ctx->header_size - offset;
                        size_t send_size = std::min(CHUNK_SIZE, remain_header);
                        sink.write(ctx->header.data() + offset, send_size);
                    } else {
                        // 发送像素数据
                        size_t pixel_offset = offset - ctx->header_size;
                        size_t remain_pixels = ctx->pgm_data->header.data_size - pixel_offset;
                        size_t send_size = std::min(CHUNK_SIZE, remain_pixels);
                        
                        sink.write(
                            reinterpret_cast<const char*>(ctx->pgm_data->pixel_data + pixel_offset),
                            send_size
                        );
                    }
                    return true;
                } catch (...) {
                    sink.done();
                    return false;
                }
            },
            [](bool success) {}// 必需的资源释放器
        );
    });
    /*
    svr.Get("/map", [](const Request& req, Response& res) {
        try {
            namespace fs = std::filesystem;
            const fs::path map_path(MAP_PATH);
    
            // 使用RAII管理文件描述符
            int fd = open(map_path.c_str(), O_RDONLY);
            if(fd == -1) throw fs::filesystem_error(
                "Open failed", map_path,
                std::error_code(errno, std::system_category())
            );
            
            // 获取文件状态
            struct stat st;
            if(fstat(fd, &st) == -1) throw fs::filesystem_error(
                "Stat failed", map_path,
                std::error_code(errno, std::system_category())
            );
    
            // 内存映射与RAII管理
            void* mapped = mmap(nullptr, st.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
            close(fd); // 立即释放文件描述符
            auto guard = std::make_shared<MMapGuard>(mapped, st.st_size);
            const PGMHeader header = parse_pgm_header(
              static_cast<const char*>(mappe
            
            // 设置HTTP响应头
            //res.set_header("Content-Type", "image/x-portable-graymap");
            std::ostringstream content_header;
            // 增强Content-Type元数据携带能力
          content_header << "image/x-portable-graymap"
                         << "; magic=" << header.magic
                         << "; width=" << header.width
                        << "; height=" << header.height
                         << "; max_value=" << header.max_value
                        << "; data_offset=" << header.data_offset;  // 添加数据偏移量[3,5](@ref)
            //res.set_header("Content-Type", content_header.str());
            res.set_header("Cache-Control", "public, max-age=3600");
            res.set_header("X-Accel-Buffering", "no"); // 禁用代理缓冲
            res.set_chunked_content_provider(
                content_header.str(),
               // "image/png"
                [guard, size=st.st_size, sent_bytes=0ULL](size_t offset, DataSink& sink) mutable {
                    try {
                        if(offset >= size) {
                            std::cout << "Total sent: " << sent_bytes << " bytes\n"; // 最终输出
                            sink.done();
                            return false;
                        }
                        
                        const size_t chunk = std::min(131072UL, size - offset);
                        sink.write(static_cast<char*>(guard->addr) + offset, chunk);
                        sent_bytes += chunk;  // 累计发送量
                        std::cout << "Progress: " << sent_bytes*100/size << "%\n"; // 实时进度
                        return true;
                    } catch (...) {
                        sink.done();
                        return false;
                    }
                });
           
        } catch (const std::exception& e) {
            res.status = 500;
            res.set_content(json{
                {"error", e.what()},
                {"timestamp", std::chrono::system_clock::now().time_since_epoch().count()}
            }.dump(), "application/json");
        }
        
    }
    );
    */
    //巡检位置坐标
    svr.Post("/set_pathpoint", [&](const Request& req, Response& res) {
        try {
            auto jsonData = json::parse(req.body);
            
            // 参数校验
            if (!jsonData.contains("points") || !jsonData["points"].is_array()) {
                throw std::runtime_error("Invalid request format: 'points' array required");
            }
    
            std::vector<pathpoint> receivedPoints;
            for (const auto& pointData : jsonData["points"]) {
                pathpoint point;
                
                // 提取必需字段
                point.number = pointData.value("number", 0);
                point.seq=pointData.value("point_seq",0);
                point.x = pointData.value("x", -1.0);
                point.y = pointData.value("y", -1.0);
                point.keyPointType = pointData.value("type", 0);
    
                // 坐标有效性验证
               /* if (point.x < 0 || point.x >= PGM_WIDTH || 
                    point.y < 0 || point.y >= PGM_HEIGHT) {
                    throw std::runtime_error("Coordinate out of bounds: (" + 
                        std::to_string(point.x) + "," + std::to_string(point.y) + ")");
                }
                */
                // 类型校验
                if (point.keyPointType > 2) { // 假设允许0-2类型
                    throw std::runtime_error("Invalid keyPointType: " + 
                        std::to_string(point.keyPointType));
                }
                
                std::cout<<"number:"<<point.number<<std::endl;
                std::cout<<"x:"<<point.x<<std::endl;
                std::cout<<"y:"<<point.y<<std::endl;
                std::cout<<"type:"<<point.keyPointType<<std::endl;
                receivedPoints.push_back(point);
            }
    
            // 原子化更新全局路径点
            auto& points = globalPathPoints;
            points.clear();
            points.insert(points.end(), receivedPoints.begin(), receivedPoints.end());
            //std::cout<<"pathpoint sent"<<std::endl;
            commonFrame frame={
                {2,0,1,0},
                {0,0,0,0.0f,0.0f}
            };
            frame.frameHead.subObj=points.size();  
            for (auto it=points.begin();it!=points.end();++it){
              
                frame.frameHead.source=it->number;
                frame.framedata.data1=it->seq;
                frame.framedata.data2=it->keyPointType;
                frame.framedata.data3=it->x;
                frame.framedata.data4=it->y;
                sendCommand(udp_socket, frame, "127.0.0.1", 8870);
            }
            //std::cout<<points<<std::endl;
            res.set_content(json{{"status", "success"}, {"received_points", points.size()}}.dump(), 
                           "application/json");
    
        } catch (const std::exception& e) {
            res.status = 400;
            res.set_content(json{{"error", e.what()}}.dump(), "application/json");
        }
    });
    svr.Post("/robot", [&](const Request& req, Response& res) {
        auto jsonData = json::parse(req.body);
        std::string commandStr = jsonData["robot"];
    
        commonFrame frame = {
            {2, 0, 2, 0}, // 初始化 frameHead，frameType, source, dest 和 subObj
            {1, 0, 0, 0.0f, 0.0f} // 初始化 framedata
        };
    
        // 根据不同的指令填充 data1
        if (commandStr == "ResumeInspection") frame.framedata.data1 = static_cast<uint32_t>(Command::ResumeInspection);
        else if (commandStr == "StopInspection") frame.framedata.data1 = static_cast<uint32_t>(Command::StopInspection);
        else if (commandStr == "MoveForward") frame.framedata.data1 = static_cast<uint32_t>(Command::MoveForward);
        else if (commandStr == "MoveBackward") frame.framedata.data1 = static_cast<uint32_t>(Command::MoveBackward);
        else if (commandStr == "TurnLeft") frame.framedata.data1 = static_cast<uint32_t>(Command::TurnLeft);
        else if (commandStr == "TurnRight") frame.framedata.data1 = static_cast<uint32_t>(Command::TurnRight);
        else if (commandStr == "StandUp") frame.framedata.data1 = static_cast<uint32_t>(Command::StandUp);
        else if (commandStr == "BowDown") frame.framedata.data1 = static_cast<uint32_t>(Command::BowDown);
        else if (commandStr == "MoveLeft") frame.framedata.data1 = static_cast<uint32_t>(Command::MoveLeft);
        else if (commandStr == "MoveRight") frame.framedata.data1 = static_cast<uint32_t>(Command::MoveRight);
        else {
            res.set_content("Invalid command", "text/plain");
            return;
        }
    
        // 发送控制指令
        sendCommand(udp_socket, frame, "127.0.0.1", 8081);
        res.set_content("Command executed: " + commandStr, "text/plain");
    });
    
    svr.Post("/scheduled_task", [&](const Request& req, Response& res) {
        try {
            auto jsonData = nlohmann::json::parse(req.body);
            std::string commandStr = jsonData["scheduled_task"];
            int path =jsonData["path"]; 
            std::string datetimeStr = jsonData["task_begin_time"];
            std::string datatimeStr1 =jsonData["task_end_time"];
            
            // 解析时间
            struct tm tm = {};
            std::istringstream ss(datetimeStr);
            ss >> std::get_time(&tm, "%Y-%m-%d %H:%M");
            tm.tm_isdst = -1;
            
            time_t tt = std::mktime(&tm);
            if (tt == -1) throw std::runtime_error("Invalid datetime");
            auto execTime = std::chrono::system_clock::from_time_t(tt);
            //
            if (commandStr == "ResumeInspection") {
                time_frame.framedata.data1 = static_cast<uint32_t>(Command::ResumeInspection);
            } else if (commandStr == "StopInspection") {
                time_frame.framedata.data1 = static_cast<uint32_t>(Command::StopInspection);
            } else {
                res.set_content("Invalid command", "text/plain");
                return;
            }
            {
                std::lock_guard<std::mutex> lock(queueMutex);
                taskQueue.push({execTime});
            }
            
             
            res.set_content("Scheduled successfully", "text/plain");
        } catch (const std::exception& e) {
            res.status = 400;
            res.set_content(e.what(), "text/plain");
        }
    });
    /*svr.Post("/scheduled_task", [&](const Request& req, Response& res) {
        try {
            auto jsonData = nlohmann::json::parse(req.body);
            std::string commandStr = jsonData["scheduled_task"];
            int  path=jsonData["path"];
            std::string datetimeStr = jsonData["task_begin_time"];
            std::string datetimeStr1 =jsonData["task_end_time"];
            auto get_precise_time = [](const std::string& str) {
                std::tm tm = {};
                std::istringstream ss(str);
                ss >> std::get_time(&tm, "%Y-%m-%d %H:%M");
                tm.tm_isdst = -1;
                return std::mktime(&tm); // 转换为秒级时间戳
            };
            commonFrame frame = {
                {2, 0, 1, 0}, // 初始化 frameHead，frameType, source, dest 和 subObj
                {1, 0, 0, 0.0f, 0.0f} // 初始化 framedata
            };
            
            frame.framedata.data1=1;
            frame.framedata.data2=path;
            frame.framedata.data3= get_precise_time(datetimeStr); 
            frame.framedata.data4= get_precise_time(datetimeStr1); 
            sendCommand(udp_socket, frame, "127.0.0.1", INF_PORT);


        }catch (const std::exception& e) {
            res.status = 400;
            res.set_content(e.what(), "text/plain");
        }
    });*/
    svr.Get("/video_feed", [&](const httplib::Request &, httplib::Response &res) {
    // 使用 shared_ptr 管理 VideoCapture 以便在多次回调中保持状态
    
    //auto camera = std::make_shared<cv::VideoCapture>(video_file);
    auto camera = std::make_shared<cv::VideoCapture>(video_path,cv::CAP_GSTREAMER);
    if (!camera->isOpened()) {
        res.status = 500;
        res.set_content("Error opening video file", "text/plain");
        return;
    }

    // 设置视频参数（可选）
    camera->set(cv::CAP_PROP_FRAME_WIDTH, 640);
    camera->set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    res.set_chunked_content_provider(
        "multipart/x-mixed-replace; boundary=frame",
        [camera](size_t /*offset*/, httplib::DataSink &sink) {
            cv::Mat frame;
            *camera >> frame;

            // 视频结束处理（循环播放）
            if (frame.empty()) {
                // 方式1：重置播放位置
                camera->set(cv::CAP_PROP_POS_FRAMES, 0);
                *camera >> frame;  // 重新读取第一帧
                
                // 方式2：重新打开文件（更可靠）
                // camera->release();
                // if (!camera->open(video_file)) {
                //     sink.done();
                //     return false;
                // }
                // *camera >> frame;
                
                if (frame.empty()) {
                    sink.done();
                    return false;
                }
            }

            // 编码为 JPEG
            std::vector<uchar> buffer;
            if (!cv::imencode(".jpg", frame, buffer)) {
                sink.done();
                return false;
            }

            // 构建响应块
            const std::string part = 
                "--frame\r\n"
                "Content-Type: image/jpeg\r\n"
                "Content-Length: " + std::to_string(buffer.size()) + "\r\n\r\n";

            // 发送数据块
            sink.write(part.data(), part.size());
            sink.write(reinterpret_cast<const char*>(buffer.data()), buffer.size());
            sink.write("\r\n", 2);

            // 控制帧率（按视频实际帧率）
            const double fps = camera->get(cv::CAP_PROP_FPS);
            if (fps > 0) {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(static_cast<int>(1000 / fps))
                );
            }

            return true;  // 继续发送下一帧
        }
    );
});
   
        svr.listen("0.0.0.0", HTTP_PORT);
    }
    
   
   
    
int  main(){

    int sockfd=setup_udp_socket(UDP_PORT);//绑定UDP端口
    if (sockfd < 0) {
        std::cerr << "Failed to set up UDP socket!" << std::endl;
        return -1;
    }
    // 启动数据接收线程
    std::thread receiverThread(receiveData, sockfd);
    std::thread httpThread([&](){
        start_http_server(sockfd);
    });
 
    std::thread receivepgmThread(pgm_receiver_thread);
    /* {
        std::lock_guard<std::mutex> lock(camera_mutex);
        camera.open(0);
        if (!camera.isOpened()) {
            std::cerr << "Error opening camera" << std::endl;
            return -1;
        }
    }*/
    std::thread scheduler(scheduler_loop,sockfd);
    
    receiverThread.join();
    httpThread.join();
    receivepgmThread.join();
    scheduler.join();
   return 0;
   
}  
        
    
    
    
    
    


