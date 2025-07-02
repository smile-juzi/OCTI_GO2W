#include<iostream>
//#include"data_type.h"
using namespace std;
void handleSource1SubObj0(const commonFrame& frame,uint32_t& face) {
    face = (frame.framedata.hasData == 1) ? frame.framedata.data1 : 9999;
}

// 处理子对象 2
void handleSource1SubObj2(const commonFrame& frame,uint32_t& face,uint32_t& smoke,uint32_t& hat) {
    if (frame.framedata.hasData == 1) {
        switch (frame.framedata.data1) {
            case 0: hat = 0; break;
            case 1: hat = 1; break;
            case 2: smoke = 0; break;
            case 3: smoke = 1; break;
            default: break; // 未知值不做处理
        }
    } else {
        hat = 9999;
        smoke = 9999;
    }
}

// 处理子对象 3
void handleSource1SubObj3(const commonFrame& frame,uint32_t& inf) {
    inf = frame.framedata.hasData;
}

// 处理子对象 4
void handleSource1SubObj4(const commonFrame& frame,uint32_t& flame) {
    // 将 hasData 的值赋给 flame
    flame = frame.framedata.hasData;
}

// 处理源 1 的数据
void handleSource1(const commonFrame& frame,Vision_data& vision) {
    switch (frame.frameHead.subObj) {
        case 0: handleSource1SubObj0(frame,vision.face); break;
        case 2: handleSource1SubObj2(frame,vision.face,vision.smoke,vision.hat) ; break;
        case 3: handleSource1SubObj3(frame,vision.inf); break; 
        case 4: handleSource1SubObj4(frame,vision.flame);// 处理子对象 4
        default: break; // 未知值不做处理
    }
}

// 处理源 3 的数据
void handleSource3(const commonFrame& frame,Meter_data& meter ) {
    if (frame.framedata.hasData == 1) {
        switch (frame.framedata.data1) {
            case 0: meter.XIAOFANG = frame.framedata.data3; break;
            case 1: meter.AIR = frame.framedata.data3; break;
            case 2: meter.JIXIE = frame.framedata.data3; break;
            default: break; // 未知值不做处理
        }
    }
}

// 处理源 4 的数据
void handleSource4(const commonFrame& frame,Gas_concentration& gas ) {
    switch (frame.frameHead.subObj) {
        case 0:
            gas.globalO2 = frame.framedata.data3;
            gas.globalCO = frame.framedata.data4;
            break;
        case 1:
            gas.globalH2S = frame.framedata.data3;
            gas.globalEX = frame.framedata.data4;
            break;
        default: break; // 未知值不做处理
    }
}




// 处理源为 5 的数据
void handleSource5(const commonFrame& frame,Tem_data& tem) {
    // 检查条件
    if (frame.framedata.hasData == 1 &&
        frame.frameHead.source == 5 && 
        frame.frameHead.dest == 0 &&
        frame.frameHead.subObj == 4 ) {  // 根据条件进行判断
        tem.average_tem = frame.framedata.data3; // 保存 data3 到 average_tem
        tem.medium_tem = frame.framedata.data4;  // 保存 data4 到 medium_tem
        std::cout << "Average Temperature: " << tem.average_tem << std::endl;
        std::cout << "Medium Temperature: " << tem.medium_tem << std::endl;
    }
}

void handleSource6(const commonFrame& frame,pathpoint &revPothPoints,std::vector<pathpoint>& global_revPathPoints) {
    // 检查条件
    if (frame.framedata.hasData == 1 &&
       
        frame.frameHead.dest == 0
        ) {  // 根据条件进行判断
        //pathpoint revPothPoints;
        revPothPoints.number=frame.frameHead.source;//点归属路径
        revPothPoints.seq=frame.framedata.data1; //点序号
        revPothPoints.x = frame.framedata.data3; // 保存 data1到 x
        revPothPoints.y = frame.framedata.data4;  // 保存 data2 到 y
        revPothPoints.keyPointType=frame.framedata.data2; 
        global_revPathPoints.emplace_back(revPothPoints);
    }
   
}
std::string timestamp_to_utc_string(float timestamp) {
    const time_t t = static_cast<time_t>(timestamp);
    
    std::tm tm;

    static std::mutex tm_mutex;
    std::lock_guard<std::mutex> lock(tm_mutex);
    tm = *std::gmtime(&t); // 通用非线程安全版本
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M");
    
    // 验证转换结果
    if (oss.fail()) {
        throw std::runtime_error("Failed to format timestamp");
    }
    
    return oss.str();
}
void handleSource7(const commonFrame& frame, Robot_Status& robot_status){
	if (frame.framedata.hasData == 1 &&
	       
		frame.frameHead.dest == 1
		) {  // 根据条件进行判断
		
		robot_status.work_status=frame.frameHead.subObj;//点归属路径
		robot_status.task_type=frame.frameHead.source;
		
		robot_status.task_begin_time=  timestamp_to_utc_string(frame.framedata.data3);  
		//std::cout<<robot_status.task_begin_time<<std::endl;// 保存 data2 到 
		robot_status.task_end_time=  timestamp_to_utc_string(frame.framedata.data4);
		
	    }
}
void processData(const commonFrame& frame,ProcessDataParams& processdata,struct pathpoint& revPothPoints,vector<struct pathpoint>& global_revPathPoints,Robot_Status& robot_status) {
    if (frame.frameHead.frameType == 1) { // 根据 frameType 进行判断
        switch (frame.frameHead.source) {
            case 1: handleSource1(frame,processdata.vision); break;
            case 3: handleSource3(frame,processdata.meter ); break;
            case 4: handleSource4(frame,processdata.gas ); break;
            case 5: handleSource5(frame,processdata.tem); break; 
            //ase 6: handleSource6(frame);break;// 处理源为 5 的数据
            default: break; // 未知值不做处理
        }
    }
    else if(frame.frameHead.frameType == 3){
      switch(frame.frameHead.dest){
      case 0:
      handleSource6(frame,revPothPoints,global_revPathPoints);
      break;
      case 1:
      handleSource7(frame,robot_status);
      break;
      default:break;
      }
    }
}
struct PGMDataDeleter {
    void operator()(PGMData* p) {
        if (p) {
            delete[] p->pixel_data;
            delete p;
        }
    }
};

using PGMDataPtr = std::unique_ptr<PGMData, PGMDataDeleter>;

class PGMReceiver {
private:
    std::atomic<bool> running_{false};
    std::thread listener_thread_;
    std::mutex queue_mutex_;
    int server_fd_{-1};
    std::queue<PGMDataPtr> data_queue_;

    void enqueue_data(PGMDataPtr data) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        data_queue_.push(std::move(data));
    }

    void listener_loop() {
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);

        while (running_) {
            int client_fd = accept(server_fd_, 
                                 (struct sockaddr*)&client_addr,
                                 &addr_len);
            if (client_fd < 0) {
                if (running_) perror("accept failed");
                continue;
            }

            std::thread([this, client_fd]() {
                handle_client(client_fd);
            }).detach();
        }
    }

    void handle_client(int client_fd) {
        try {
            PGMHeader network_header;
            if (!receive_all(client_fd, &network_header, sizeof(PGMHeader))) {
                throw std::runtime_error("Header接收失败");
            }

            // 创建带自定义删除器的智能指针
            PGMDataPtr pgm_data(new PGMData(), PGMDataDeleter{});

            // 转换字节序
            pgm_data->header = {
                ntohl(network_header.magic),
                ntohl(network_header.width),
                ntohl(network_header.height),
                ntohl(network_header.max_gray),
                ntohl(network_header.data_size),
                ntohl(network_header.map_number),
                ntohl(network_header.datalen),
                network_header.resolution  // float不需要转换字节序
            };

            // 验证格式
            if (pgm_data->header.magic != 0x5035 &&  // P5
                pgm_data->header.magic != 0x5032) {  // P2
                throw std::runtime_error("无效的PGM格式");
            }

            // 分配并接收像素数据
            const size_t data_size = pgm_data->header.data_size;
            pgm_data->pixel_data = new uint8_t[data_size];
            if (!receive_all(client_fd, pgm_data->pixel_data, data_size)) {
                throw std::runtime_error("像素数据接收失败");
            }

            enqueue_data(std::move(pgm_data));
        } catch (const std::exception& e) {
            std::cerr << "处理客户端错误: " << e.what() << std::endl;
        }
        close(client_fd);
    }

    bool receive_all(int fd, void* buffer, size_t length) {
        size_t received = 0;
        while (received < length && running_) {
            ssize_t n = recv(fd, static_cast<char*>(buffer) + received, 
                           length - received, 0);
            if (n <= 0) return false;
            received += n;
        }
        return received == length;
    }

public:
    PGMReceiver() = default;
    ~PGMReceiver() { stop(); }

    bool start(uint16_t port) {
        
        server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd_ < 0) return false;
        
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        addr.sin_addr.s_addr = INADDR_ANY;
        
        if (bind(server_fd_, (sockaddr*)&addr, sizeof(addr)) < 0) return false;
        listen(server_fd_, 5);
        
        running_ = true;
        listener_thread_ = std::thread(&PGMReceiver::listener_loop, this);
        return true;
    }

    void stop() {
        if (running_.exchange(false)) {
            shutdown(server_fd_, SHUT_RDWR);
            close(server_fd_);
            if (listener_thread_.joinable()) listener_thread_.join();
        }
    }

    bool get_data(PGMDataPtr& out_data) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (data_queue_.empty()) return false;
        out_data = std::move(data_queue_.front());
        data_queue_.pop();
        return true;
    }
};

