#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cstdlib>  // for atoi

// 定义结构体
//struct MyData {
//    int member1 = 2;
//    int member2 = 2;
//    int serialValue;  // 存储从串口读取并转化后的整数值
//};



struct commonFrameHead {
    uint32_t frameType = 2;
    uint32_t source = 6;
    uint32_t dest = 2;
    uint32_t subObj;
};

struct commonFrameData {
    uint32_t hasData = 1;
    uint32_t data1;  // 随意填充
    uint32_t data2;  // 随意填充
    float data3;     // 根据 subObj 返回相应的浓度值
    float data4;     // 根据 subObj 返回相应的浓度值
};

struct motionFrame {
    struct commonFrameHead frameHead;
    struct commonFrameData framedata;
};

// 配置串口函数
int configureSerialPort(int serial_fd) {
    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(serial_fd, &tty) != 0) {
        std::cerr << "获取串口配置失败" << std::endl;
        return -1;
    }

    // 设置波特率
    cfsetospeed(&tty, B9600);  // 输出波特率
    cfsetispeed(&tty, B9600);  // 输入波特率

    // 设置数据位
    tty.c_cflag &= ~CSIZE; // 清除当前数据位设置
    tty.c_cflag |= CS8;    // 8位数据位

    // 无校验
    tty.c_cflag &= ~PARENB;

    // 一位停止位
    tty.c_cflag &= ~CSTOPB;

    // 开启接收
    tty.c_cflag |= CREAD | CLOCAL;

    // 设置非规范模式
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // 禁止软件流控
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);

    // 禁止硬件流控
    tty.c_cflag &= ~CRTSCTS;

    // 设置立即读取的字节数
    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 0;

    // 刷新串口设置
    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        std::cerr << "设置串口参数失败" << std::endl;
        return -1;
    }

    return 0;
}

// 串口读取函数
int readSerialData(int serial_fd, unsigned char *buffer, size_t size) {
    int bytesRead = read(serial_fd, buffer, size);
    if (bytesRead < 0) {
        std::cerr << "读取串口数据失败" << std::endl;
        return -1;
    }
    if(bytesRead == 0)
    {
    std::cout << "No data\n";
    	return 0;
    }
    // 打印读取到的数据，调试用
    std::cout << "读取到的数据: ";
    std::cout << "read bytes = " << bytesRead << std::endl;
    for (int i = 0; i < bytesRead; i++) {
        std::printf("%02X ", buffer[i]);
    }
    std::cout << std::endl;
    
    buffer[bytesRead] = '\0'; // 确保数据以空字符结束
    return bytesRead;
}

// 通过UDP发送结构体数据
void sendUDPData(const motionFrame& data, const char* ip, int port) {
    int udpSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udpSocket < 0) {
        std::cerr << "创建UDP套接字失败" << std::endl;
        return;
    }

    sockaddr_in serverAddr{};
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    inet_pton(AF_INET, ip, &serverAddr.sin_addr);

    sendto(udpSocket, &data, sizeof(data), 0, (sockaddr*)&serverAddr, sizeof(serverAddr));
    close(udpSocket);
}

int sumOfDigits(int number) {
    int sum = 0;

    while (number > 0) {
        sum += number % 10;  // 提取最后一位并加到总和
        number /= 10;        // 去掉最后一位
    }

    return sum;
}
// 主函数
int main() {
    const char* ip = "127.0.0.1";  // 目标IP地址
    int port = 8081;                   // 目标端口
    int serial_fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);  // 打开串口设备
    if (serial_fd < 0) {
	serial_fd = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY);
	if(serial_fd < 0)
	{
		serial_fd = open("/dev/ttyUSB2", O_RDWR | O_NOCTTY);
		if(serial_fd < 0)
		{
        		std::cerr << "打开串口失败" << std::endl;
        		return -1;
        	}
	}
    }

    // 配置串口参数
    if (configureSerialPort(serial_fd) != 0) {
        std::cerr << "配置串口失败" << std::endl;
        close(serial_fd);
        return -1;
    }

    unsigned char buffer[2];  // 使用unsigned char以便读取原始字节
    char hexStr[5]; // 两个字节加上一个结束符
    motionFrame data;

    while (true) {
        // 从串口读取数据
        if (readSerialData(serial_fd, buffer, 2) > 0) {
            // 如果设备使用小端序，将字节顺序反转
	    int value = buffer[0]; // (buffer[1] << 8) | buffer[0];

	   	
		
           std::cout << "转换后的整数: " << value << std::endl;

            // 将转换后的整数填入结构体
            data.framedata.data1 = value;

            // 通过UDP发送数据
            sendUDPData(data, ip, port);
        }
        usleep(100000);  // 每次读取后延迟100毫秒
    }

    close(serial_fd);
    return 0;
}

