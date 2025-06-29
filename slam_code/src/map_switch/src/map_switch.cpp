#include <ros/ros.h>
#include <sys/socket.h>
#include <std_msgs/Bool.h>  
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#include <cstdlib>
#include <vector>
#include <thread>
#include <mutex>
#include <signal.h> 

#define map1 1
#define map2 2

// 定义请求结构体
struct req_frame
{
    unsigned long frame_type;
    unsigned long seq;
    float x;
    float y;
    float yaw;
};

// 定义回复结构体
struct replay_frame
{
    bool result;
    unsigned long seq;
};

class TcpMapSwitchNode
{
public:
    TcpMapSwitchNode() : sockfd_(-1), listenfd_(-1), isRunning_(false), currentMapPid(-1) 
    {
        // 从ROS参数服务器获取服务端地址和端口
        nh.param<std::string>("server_addr", server_addr, "192.168.2.100");
        nh.param<int>("map_switch_PORT", map_switch_PORT, 6050);
    }
    ~TcpMapSwitchNode()
    {
        stop();
    }

    bool init()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        listenfd_ = createAndBindTcpSocket();
        if (listenfd_ == -1)
        {
            ROS_ERROR("Failed to create and bind TCP socket");
            return false;
        }
        if (listen(listenfd_, 5) < 0)
        {
            ROS_FATAL("Failed to listen on TCP socket: %s", strerror(errno));
            close(listenfd_);
            return false;
        }
        ROS_WARN("TCP Map Switch Node started. Waiting for commands...");
        return true;
    }

    void start()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (listenfd_ != -1 && !isRunning_)
        {
            isRunning_ = true;
            tcpThread_ = std::thread(&TcpMapSwitchNode::handleTcpData, this, listenfd_);
        }
    }

    void stop()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (isRunning_)
        {
            isRunning_ = false;
            if (tcpThread_.joinable())
            {
                tcpThread_.join();
            }
            if (listenfd_ != -1)
            {
                close(listenfd_);
                listenfd_ = -1;
            }
            if (sockfd_ != -1)
            {
                close(sockfd_);
                sockfd_ = -1;
            }
        }
    }

private:
    ros::NodeHandle nh;
    bool isRunning_;
    int sockfd_;
    int listenfd_;

    int map_switch_PORT;
    std::string server_addr;

    std::thread tcpThread_;
    std::mutex mutex_;
    std::mutex pidMutex_;  

    struct sockaddr_in serverAddr;
    pid_t currentMapPid;

    // 两张地图的转换参数
    float tx_map1_to_map2 = 7.85;       //地图2原点在地图1坐标系中的位置
    float ty_map1_to_map2 = 1.62;       //地图2原点在地图1坐标系中的位置
    float theta_map1_to_map2 = -1.55;   //地图2原点的x坐标方向在地图1坐标系中的方向

    const std::string map1_ScriptPath = "/home/orangepi/slam_ws/src/map_switch/scripts/start_map1.sh";
    const std::string map2_ScriptPath = "/home/orangepi/slam_ws/src/map_switch/scripts/start_map2.sh";
    const std::string stop_ScriptPath = "/home/orangepi/slam_ws/src/map_switch/scripts/stop_current_map.sh";
    const std::string initial_pose_script_path = "/home/orangepi/slam_ws/src/FAST_LIO_GLOBAL/scripts/publish_initial_pose.py";


    // 启动脚本并获取其进程ID
    pid_t startScript(const std::string& scriptPath) {
        // 构建命令，在后台运行脚本并输出其进程ID
        std::string command = "bash " + scriptPath + " & echo $!";
        FILE* pipe = popen(command.c_str(), "r");
        if (!pipe) {
            std::cerr << "Failed to start the script. Error executing popen." << std::endl;
            return -1;
        }

        char buffer[128];
        std::string pidStr;
        if (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            pidStr = buffer;
            // 去除换行符
            pidStr.erase(pidStr.find_last_not_of(" \n\r\t") + 1);
        } else {
            std::cerr << "Failed to read the PID of the started script." << std::endl;
            pclose(pipe);
            return -1;
        }

        pclose(pipe);

        try {
            pid_t pid = std::stoi(pidStr);
            std::cout << "Script started with PID: " << pid << std::endl;
            return pid;
        } catch (const std::invalid_argument& e) {
            std::cerr << "Failed to parse PID: " << e.what() << std::endl;
            return -1;
        } catch (const std::out_of_range& e) {
            std::cerr << "PID out of range: " << e.what() << std::endl;
            return -1;
        }
    }
    // 执行停止脚本
    bool executeStopScript() {
        std::string command = "bash " + stop_ScriptPath;
        int result = std::system(command.c_str());
        if (result == 0) {
            std::cout << "Stop script executed successfully." << std::endl;
            return true;
        } else {
            std::cerr << "Failed to execute stop script. Error code: " << result << std::endl;
            return false;
        }
    }

    // 启动指定地图
    void launchMap(pid_t& currentMapPid, unsigned long frame_type, unsigned long req_seq, float x, float y, float yaw)
    {
        auto switchTask = [this, &currentMapPid, frame_type, req_seq, x, y, yaw]() {
            const std::string& mapScriptPath = (frame_type == map1)? map1_ScriptPath : map2_ScriptPath;
            replay_frame reply;

            if (frame_type == map1 || frame_type == map2)
            {
                reply.seq = req_seq;
                {
                    std::lock_guard<std::mutex> lock(pidMutex_);

                    if (this->executeStopScript()) {
                        std::cout << "Current map script stopped." << std::endl;
                    }
                    // // 关闭当前运行的地图脚本
                    // if (currentMapPid!= -1) {
                    //     if (this->executeStopScript()) {
                    //         std::cout << "Current map script stopped." << std::endl;
                    //     }
                    // }

                }
                pid_t newPid = this->startScript(mapScriptPath);
                {
                    std::lock_guard<std::mutex> lock(pidMutex_);
                    currentMapPid = newPid;

                }
                bool success = (currentMapPid!= -1);
                if (success) {
                    ROS_WARN("Switching maps...");

                    std_msgs::Bool::ConstPtr initial_pose_flag = ros::topic::waitForMessage<std_msgs::Bool>("waiting_for_initial_pose", nh);
                    if (initial_pose_flag && initial_pose_flag->data) {
                        ROS_WARN("Received initial pose flag, publishing initial pose...");
                        PublishInitialPose(x, y, yaw);
                    } else {
                        ros::Duration(5.0).sleep();//没接收到正确标志再等5s
                        ROS_WARN("Has been waiting for 20 seconds, publishing initial pose...");
                        PublishInitialPose(x, y, yaw);
                    }

                    std_msgs::Bool::ConstPtr relocalization_msg = ros::topic::waitForMessage<std_msgs::Bool>("map_to_odom_flag", nh);
                    ROS_WARN("Map switch succeeded...");
                    if (relocalization_msg && relocalization_msg->data) {
                        reply.result = true;
                    } else {
                        reply.result = false;
                    }
                } else {
                    reply.result = false;
                }

               this->sendReply(reply);
            }
            else
            {
                ROS_WARN("Received unknown frame_type: %lu", frame_type);
                reply.result = false;
                reply.seq = 2;
                this->sendReply(reply);
            }
        };

        std::thread launchThread(switchTask);
        launchThread.detach();
    }

    // 创建并绑定 TCP 套接字
    int createAndBindTcpSocket()
    {
        int sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd < 0){
            ROS_FATAL("Failed to create TCP socket: %s", strerror(errno));
            return -1;
        }

        memset(&serverAddr, 0, sizeof(serverAddr));
        serverAddr.sin_family = AF_INET;
        serverAddr.sin_port = htons(map_switch_PORT);
        if (inet_pton(AF_INET, server_addr.c_str(), &(serverAddr.sin_addr)) <= 0) {
            ROS_FATAL("Failed to convert IP address string to network format");
            close(sockfd);
            return -1;
        }

        if (bind(sockfd, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0)
        {
            ROS_FATAL("Failed to bind TCP socket: %s", strerror(errno));
            close(sockfd);
            return -1;
        }

        return sockfd;
    }

    // map1到map2的转换函数
    req_frame map1_to_map2(const req_frame& map1_frame_old, float tx, float ty, float theta) {
        req_frame map2_frame_new;
        map2_frame_new.frame_type = map1_frame_old.frame_type;
        map2_frame_new.seq = map1_frame_old.seq;
        map2_frame_new.x = (map1_frame_old.x - tx) * std::cos(theta) + (map1_frame_old.y - ty) * std::sin(theta);
        map2_frame_new.y = -(map1_frame_old.x - tx) * std::sin(theta) + (map1_frame_old.y - ty) * std::cos(theta);
        map2_frame_new.yaw = map1_frame_old.yaw - theta;
        return map2_frame_new;
    }

    //  map2到map1的转换函数
    req_frame map2_to_map1(const req_frame& map2_frame_old, float tx, float ty, float theta) {
        req_frame map1_frame_new;
        map1_frame_new.frame_type = map2_frame_old.frame_type;
        map1_frame_new.seq = map2_frame_old.seq;
        map1_frame_new.x = map2_frame_old.x * std::cos(theta) - map2_frame_old.y * std::sin(theta) + tx;
        map1_frame_new.y = map2_frame_old.x * std::sin(theta) + map2_frame_old.y * std::cos(theta) + ty;
        map1_frame_new.yaw = map2_frame_old.yaw + theta;
        return map1_frame_new;
    }

    // 处理 TCP 数据接收
    void handleTcpData(int listenfd){
        struct sockaddr_in socket_Client;
        socklen_t clientAddrLen = sizeof(socket_Client);
        static unsigned long current_map = map1; 

        while (ros::ok() && isRunning_){
            sockfd_ = accept(listenfd, (struct sockaddr*)&socket_Client, &clientAddrLen);
            if (sockfd_ < 0){
                ROS_ERROR("accept failed: %s", strerror(errno));
                continue;
            }

            req_frame request;
            ssize_t recv_len = recv(sockfd_, &request, sizeof(request), 0);
            if (recv_len < 0){
                ROS_ERROR("recv failed: %s", strerror(errno));
                close(sockfd_);
                continue;
            }
            else if (recv_len == 0){
                // 客户端关闭连接
                close(sockfd_);
                continue;
            }

            req_frame new_request;
            if((current_map == 0) && ((request.frame_type == map1) || (request.frame_type == map2))){
                // 不需要转换
                new_request = request;
                if (request.frame_type == map1){
                    current_map = map1;
                }
                else if (request.frame_type == map2){
                    current_map = map2;
                }
            } else if (current_map == map1 && request.frame_type == map2) {
                // 从 map1 转换到 map2
                new_request = map1_to_map2(request, tx_map1_to_map2, ty_map1_to_map2, theta_map1_to_map2);
                current_map = map2;
            } else if (current_map == map2 && request.frame_type == map1) {
                // 从 map2 转换到 map1
                new_request = map2_to_map1(request, tx_map1_to_map2, ty_map1_to_map2, theta_map1_to_map2);
                current_map = map1;
            } else {
                new_request = request;
                // new_request.frame_type = 0;
                ROS_WARN("Already at the target map. No need to switch!!!");
            }

             // 打印旧坐标和新坐标
             ROS_WARN("Old Coordinates: x = %f, y = %f, yaw = %f", request.x, request.y, request.yaw);
             ROS_WARN("New Coordinates: x = %f, y = %f, yaw = %f", new_request.x, new_request.y, new_request.yaw);

            // 提取参数并传递给launchMap函数
            if (recv_len == sizeof(request)){
                launchMap(currentMapPid, new_request.frame_type, new_request.seq, new_request.x, new_request.y, new_request.yaw);
            }

            else{
                ROS_ERROR("Received incomplete or incorrect data length for req_frame");
                continue;
            }
        }
    }


    // 发送回复给客户端
    void sendReply(const replay_frame& reply)
    {
        ssize_t send_len = send(sockfd_, &reply, sizeof(reply), 0);
        if (send_len < 0){
            ROS_ERROR("send reply failed: %s", strerror(errno));
        }
        else{
            ROS_WARN("send reply succeeded!!!");
        }
    }

    // 启动初始位姿发布节点
    void PublishInitialPose(float x, float y, float yaw)
    {
        std::stringstream ss;
        ss << initial_pose_script_path << " " << x << " " << y << " " << 0 << " " << yaw << " " << 0 << " " << 0;
        std::string cmd = ss.str();
        int result = system(cmd.c_str());
        if (result != 0) {
            ROS_ERROR("Failed to execute initial pose script: %s", cmd.c_str());
        }
    }  
};

int main(int argc, char* argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "tcp_map_switch_node");
    ros::NodeHandle nh;

    TcpMapSwitchNode node;
    if (!node.init())
    {
        return -1;
    }
    node.start();

    ros::spin();

    return 0;
}    

