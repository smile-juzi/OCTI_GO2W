#include <sys/select.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <thread>
#include <atomic>
#include<iomanip>
#include<sstream>

#define UDP_PORT 8111
#define UDP_IP "127.0.0.1"
#define INF_PORT 8870

struct commonFrameHead {
    uint32_t frameType;  // 注意驼峰命名
    uint32_t source;
    uint32_t dest;
    uint32_t subObj;
};

struct commonFrameData {
    uint32_t hasData;
    uint32_t data1;
    uint32_t data2;
    float data3;
    float data4;
};

struct commonFrame {
    struct commonFrameHead frameHead;
    struct commonFrameData frameData;  // 修正结构体字段名
};
std::atomic<bool> running(true);
/*void senddata(int sockfd){
    sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(UDP_PORT);
    server_addr.sin_addr.s_addr = inet_addr(UDP_IP);  // 正确IP转换

    // 发送循环
    while (true) {
        int send_len = sendto(sockfd, &frame, sizeof(commonFrame), 0,
                             (struct sockaddr*)&server_addr, sizeof(server_addr));
        
        if (send_len < 0) {
            std::cerr << "发送失败: " << strerror(errno) << std::endl;
            break;
        } else {
            //std::cout << "已发送" << send_len << "字节数据" << std::endl;
        }
        
        sleep(1); // 添加发送间隔
    }
}*/
void receiver_thread(int sockfd) {
   
    sockaddr_in sender_addr{};
    socklen_t addr_len = sizeof(sender_addr);

    // 初始化绑定地址
    

    while (running) {
        commonFrame recv_frame;
        fd_set fds;
        timeval tv{1, 0};  // 1秒超时

        FD_ZERO(&fds);
        FD_SET(sockfd, &fds);

        int ret = select(sockfd + 1, &fds, NULL, NULL, &tv);
        if (ret < 0) {
            std::cerr << "Select error: " << strerror(errno) << std::endl;
            break;
        }

        if (FD_ISSET(sockfd, &fds)) {
            ssize_t recv_len = recvfrom(sockfd, &recv_frame, sizeof(recv_frame), 0,
                                       (sockaddr*)&sender_addr, &addr_len);

            if (recv_len == sizeof(commonFrame)) {
                // 字节序转换
                recv_frame.frameHead.source = ntohl(recv_frame.frameHead.source);
                recv_frame.frameData.data3 = ntohl(*(uint32_t*)&recv_frame.frameData.data3);

                // 打印信息
                std::cout << "\nReceived from " << inet_ntoa(sender_addr.sin_addr) 
                         << ":" << ntohs(sender_addr.sin_port) << "\n"
                         
                         << "FrameType: " << recv_frame.frameHead.frameType << "\n"
                         << "Source: " << recv_frame.frameHead.source << "\n"
                         <<"Data2: "<<recv_frame.frameData.data2<<"\n"
                         << "Data3: " << recv_frame.frameData.data3 << "\n"
                         << "Data4: " << recv_frame.frameData.data4 << std::endl;
            } else if (recv_len < 0) {
                std::cerr << "Recv error: " << strerror(errno) << std::endl;
            } else {
                std::cerr << "Incomplete frame (" << recv_len << "/" 
                          << sizeof(commonFrame) << " bytes)" << std::endl;
            }
        }
    }
}
float get_utc_timestamp() {
   
    using namespace std::chrono;
    
    // 单次获取时间点
    auto now = system_clock::now();
    
    // 直接转换为秒级浮点（UTC 时间）
    return duration_cast<duration<double>>(now.time_since_epoch()).count()+8*3600;
}
int main() {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "创建套接字失败: " << strerror(errno) << std::endl;
        return -1;
    }
    int reuse = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        std::cerr << "Set sockopt failed: " << strerror(errno) << std::endl;
    }

    // 绑定到所有接口的正确写法
    sockaddr_in local_addr{};
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(INF_PORT);  // 必须使用htons转换
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);  // 监听所有网卡
    
    if (bind(sockfd, (sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
        std::cerr << "绑定失败: " << strerror(errno) 
                 << " (错误码=" << errno << ")" << std::endl;
        close(sockfd);
        return -1;
    }
    	/*
    // 初始化数据帧
    auto get_precise_time = [](const std::string& str) -> double {
    std::tm tm = {};
    tm.tm_isdst = 0;  
    
    std::istringstream ss(str);
    if (!(ss >> std::get_time(&tm, "%Y-%m-%d %H:%M"))) {
        throw std::invalid_argument("Invalid datetime format: " + str);
    }
    
    //tm.tm_mon -= 1;    // 修正月份（输入4月对应tm_mon=3）
    return static_cast<float>(timegm(&tm));  // 转换为双精度浮点数
    };
    */
    
    commonFrame frame;
    commonFrame frame2;
    std::string datetimeStr="2025-04-03 19:24";
    std::string datetimeStr1="2025-04-03 20:24";
    //float data=get_precise_time(datetimeStr);
    //float data1=get_precise_time(datetimeStr1);
    float data=get_utc_timestamp();
    float data1=get_utc_timestamp();
    frame2={
    {3,1,1,1},
    {1,1,1,data,data1}
    };
    frame={
    
        {3, 1, 0, 1}, 
        {1, 2, 1, 12.45, 12.34} 
    
   
    };
    //上报信息分两帧发
    //frameType=3，dest=0，则为上报的所有路径点信息，source--路径编号，subObj--路径点数，data1--路径点序号，
    //data2--路径类型，data3--x坐标，data4--y坐标
    //frameType=3，dest=1, 上报状态信息，source，任务类型（定时非定时），sub0bj--工作状态 , data3--开始时间，data4--结束时间
    std::thread receiver(receiver_thread, sockfd);
    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(UDP_PORT);
    server_addr.sin_addr.s_addr = inet_addr(UDP_IP);
    int count=0;
   
    ssize_t send_len = sendto(sockfd, &frame, sizeof(commonFrame), 0,
                             (struct sockaddr*)&server_addr, sizeof(server_addr));
        if(send_len != sizeof(frame)) {
            std::cerr << "发送失败: " << strerror(errno) << std::endl;
        } else {
            std::cout << "已发送数据包#" << ++count << std::endl;
        }
	ssize_t send_len2 = sendto(sockfd, &frame2, sizeof(commonFrame), 0,
                             (struct sockaddr*)&server_addr, sizeof(server_addr));
        if(send_len2 != sizeof(frame)) {
            std::cerr << "发送失败: " << strerror(errno) << std::endl;
        } else {
            std::cout << "已发送数据包#" << ++count << std::endl;
        }

        // 更新发送数据（示例）
        //frame.frameData.data3 += 0.5f;
       // frame.frameData.data4 += 0.3f;
	//frame.frameData.data1 +=1;
        //sleep(5);
     while(running){};
    //running=false;
    receiver.join();
    // 配置目标地址
   
    close(sockfd);
    return 0;
}
