#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sstream>
#include <string>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <thread>
#include <mutex>

struct commonFrameHead {
    uint32_t frameType;
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
    struct commonFrameData framedata;
};

struct GasConcentrationData {
    float O2_value;   
    float CO_value;   
    float H2S_value;  
    float EX_value;   
};

GasConcentrationData gasData;
std::mutex dataMutex;  // 互斥锁

// 设置串口选项
void set_serial_options(int fd) {
    struct termios options;
    tcgetattr(fd, &options);
    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD; 
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0; 
    options.c_cc[VMIN] = 0;  
    options.c_cc[VTIME] = 10; 
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &options);
}

// 配置UDP套接字
int setup_udp_socket(int port) {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "Failed to create UDP socket!" << std::endl;
        return -1;
    }
    sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Failed to bind UDP socket!" << std::endl;
        close(sockfd);
        return -1;
    }
    return sockfd;
}

void clearSerialBuffer(int fd) {
    tcflush(fd, TCIFLUSH); // 清空输入缓冲区
}

// 读取串口数据并更新结构体的线程函数
void read_serial_data(int fd) {
    while (true) {
        clearSerialBuffer(fd);
        char buffer[256];
        memset(buffer, 0, sizeof(buffer));
        int bytesRead = read(fd, buffer, sizeof(buffer) - 1);

        if (bytesRead > 0) {
            buffer[bytesRead] = '\0'; // 添加字符串结束符
            std::cout << "Raw serial data: " << buffer << std::endl;

            std::stringstream ss(buffer);
            char comma; 
            GasConcentrationData newData;

            if (ss >> newData.O2_value >> comma >> newData.CO_value >> comma >> newData.H2S_value >> comma >> newData.EX_value) {
                std::lock_guard<std::mutex> lock(dataMutex); // 加锁
                gasData = newData; // 更新全局数据
                std::cout << "Updated values: O2 = " << gasData.O2_value 
                          << ", CO = " << gasData.CO_value 
                          << ", H2S = " << gasData.H2S_value 
                          << ", EX = " << gasData.EX_value << std::endl;
            } else {
                std::cerr << "Error in parsing data!" << std::endl;
            }
        }
        usleep(100000); // 适当延时以避免CPU占用过高
    }
}

// 根据请求内容生成响应
void prepare_response(commonFrame &responseFrame) {
    std::lock_guard<std::mutex> lock(dataMutex); // 加锁
    responseFrame.frameHead.frameType = 1;  
    responseFrame.frameHead.source = 4;     
    responseFrame.frameHead.dest = 0;       

    if (responseFrame.frameHead.subObj == 0) {
        responseFrame.framedata.data3 = gasData.O2_value;
        responseFrame.framedata.data4 = gasData.CO_value;
    } else if (responseFrame.frameHead.subObj == 1) {
        responseFrame.framedata.data3 = gasData.H2S_value;
        responseFrame.framedata.data4 = gasData.EX_value;
    }

    responseFrame.framedata.hasData = 1;
    responseFrame.framedata.data1 = 0;    
    responseFrame.framedata.data2 = 0;    
}

// 处理UDP请求
void handle_udp_request(int udp_sockfd) {
    sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);
    commonFrame receivedFrame;

    // 接收UDP数据
    int recv_len = recvfrom(udp_sockfd, &receivedFrame, sizeof(receivedFrame), 0, (struct sockaddr*)&client_addr, &addr_len);
    if (recv_len > 0) {
        std::cout << "Received frame from client!" << std::endl;
        std::cout << "Received data from client IP: " << inet_ntoa(client_addr.sin_addr) << " and port: " << ntohs(client_addr.sin_port) << std::endl;

        if (receivedFrame.frameHead.frameType == 0 &&
            receivedFrame.frameHead.source == 0 &&
            receivedFrame.frameHead.dest == 4) {

            std::cout << "Valid frame received, preparing response..." << std::endl;
            prepare_response(receivedFrame);

            int send_len = sendto(udp_sockfd, &receivedFrame, sizeof(receivedFrame), 0, (struct sockaddr*)&client_addr, addr_len);
            if (send_len > 0) {
                std::cout << "Response sent to client!" << std::endl;
            } else {
                std::cerr << "Failed to send response!" << std::endl;
            }
        } else {
            std::cerr << "Invalid frame received!" << std::endl;
        }
    } else {
        std::cerr << "Failed to receive UDP data!" << std::endl;
    }
}

int main() {
    int serial_fd = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY);
    if (serial_fd == -1) {
        std::cerr << "Failed to open serial port!" << std::endl;
        return -1;
    }
    set_serial_options(serial_fd);  

    int udp_sockfd = setup_udp_socket(8083);
    if (udp_sockfd == -1) {
        close(serial_fd);
        return -1;
    }

    // 创建串口读取线程
    std::thread serialThread(read_serial_data, serial_fd);

    while (true) {
        handle_udp_request(udp_sockfd);
        usleep(100000); // 避免CPU占用过高
    }

    serialThread.join(); // 等待线程结束
    close(serial_fd);
    close(udp_sockfd);
    return 0;
}

