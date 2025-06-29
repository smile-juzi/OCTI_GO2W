#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <curl/curl.h>
#include <nlohmann/json.hpp>

#include <thread>
#include <mutex>
#include <iostream>
#include <functional>
#include <vector>
#include <algorithm>

#include <chrono>
#include <iomanip>
#include <sstream>

using json = nlohmann::json;

#define HTTP_URL "http://222.75.204.39:7072/api/iot/IotLampData/receive/ConstructionMachinery"

#define LINE_NUM 181




typedef struct coordination_data{
    float x;
    float y;
    float z;
}dog_now_coordition, obstacle_now_coordition;

typedef struct dog_info{
    dog_now_coordition coordination;
    float angle;
}dog_info;

typedef struct obstacle_info{
    obstacle_now_coordition coordination;
    float len_x;
    float len_y;
}obstacle_info;


/*****************<dwa test>****************/

struct laddar_data
{
    double x;
    double y;
    double yaw;
    double dist[LINE_NUM];
};

struct laddar_receive_frame
{
    unsigned long frame_type;
    unsigned long seq;
    laddar_data data;
};

struct laddar_req_frame
{
    unsigned long frame_type;
    unsigned long seq;
};

/**********************************/

typedef struct udp_data{
    int command;
    float x;
    float y;
    float z;
    float yaw_angle;
    float left_obs_dist;
    float right_obs_dist;
}udp_data;


//取范围内均值函数
template<typename T>
float ArrayGetAverage(const std::vector<T>& arry, int start, int end){

    int array_size = arry.size();
    float total_value = 0;
    float get_average = 0;
    int filter_flag = 0;

    if(start > (array_size - 1) || end > (array_size - 1) || start > end) 
        return 0;
    for(int i = start; i <= end; i++){
        if(std::isinf(arry[i])){
            
            continue;
        }
        total_value += arry[i];
        filter_flag++;

    }

    if (filter_flag == 0) {
        return std::numeric_limits<float>::infinity();  //避免除以零(若都为inf则返回inf)
    }

    get_average = total_value/filter_flag;

    //std::cout << "distance:" <<  << std::endl;
    return get_average;
}

// 获取当前时间戳字符串（毫秒精度）
std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S");
    
    // 获取毫秒部分
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    ss << '.' << std::setfill('0') << std::setw(3) << ms.count();
    
    return ss.str();
}

class TfPublish{
    private:
        ros::NodeHandle nh;

        ros::Publisher tf_pub;
        ros::Subscriber sub_LaserScan;

        tf::TransformListener listener;

        std::string pub_topic;
        std::string source_frame;
        std::string target_frame;
        std::string sub_LaserScan_topic;
        double publish_tf_rate;
        double socket_rate;
        bool http_en;
        bool socket_en;        
        bool socket_print_en;

        int get_command;
        double roll, pitch, yaw;
        double rel_coords_data[6];
        dog_now_coordition robot_position;
        obstacle_now_coordition obstacle_position;
        dog_info robot_nav_info;
        obstacle_info obstacle_nav_info;
        udp_data nav_data;
        udp_data recv_data;
        float left_distance = std::numeric_limits<float>::infinity();
        float right_distance = std::numeric_limits<float>::infinity();
        //dwa应答数据
        laddar_receive_frame locationData_send;
        laddar_req_frame receive_flag;


        int server_PORT;
        int client_PORT;
        std::string client_addr;
        std::string server_addr;
        int sockfd;
        struct sockaddr_in socket_ServerAddr;
        struct sockaddr_in socket_Client;

        json http_data;
        std::string http_json2strData;

        std::mutex mtx;

    public:
        laddar_data location_with_2dLidar;



    public:
        TfPublish(){

            nh.param<std::string>("pub_topic",pub_topic,"/robot_position");
            nh.param<std::string>("source_frame",source_frame,"/robot");
            nh.param<std::string>("target_frame",target_frame,"/robot_body");
            nh.param<bool>("socket_print_en",socket_print_en,true);                   //终端打印使能
            nh.param<double>("publish_tf_rate",publish_tf_rate,10.0);         //发布速率

            nh.param<std::string>("server_addr",server_addr,"192.168.110.206");
            nh.param<int>("server_PORT",server_PORT,6001);
            nh.param<std::string>("client_addr",client_addr,"192.168.110.206");
            nh.param<int>("client_PORT",client_PORT,6002);

            nh.param<bool>("socket_en",socket_en,true);
            nh.param<bool>("http_en",http_en,true);

            tf_pub = nh.advertise<std_msgs::Float32MultiArray>(pub_topic,100);  //队列大小100



            nh.param<std::string>("sub_LaserScan_topic",sub_LaserScan_topic,"/mid360/scan");
            sub_LaserScan = nh.subscribe(sub_LaserScan_topic,1000,&TfPublish::LaserscanCallback,this);  //缓冲区为1000


        } 


        //接收点云回调
        void LaserscanCallback(const sensor_msgs::LaserScan::Ptr &msg){
            //ROS_INFO(" get data ,into callback function ");      
            std::vector<float> angle_range(std::begin(msg->ranges),std::end(msg->ranges));

            //解算方向角如下:
            //       180
            //        |
            // 270<-     -> 90
            //        |
            //        0

            left_distance = ArrayGetAverage<float>(angle_range, 265, 275);
            right_distance = ArrayGetAverage<float>(angle_range, 85, 95);

            //location_with_2dLidar.dist
            std::copy(angle_range.rbegin()+90,angle_range.rbegin()+271,locationData_send.data.dist);
            //std::copy(angle_range.begin(),angle_range.begin()+90,locationData_send.data.dist+90);

            //test  
            // std::cout << "left_array:{ " << angle_range[269] << "," << angle_range[270] << "," << angle_range[271] << " }" 
            //           << "right_array:{ " << angle_range[89] << "," << angle_range[90] << "," << angle_range[91] << " }" << std::endl;

            // std::cout << "Array after copy: ";
            // for (int i = 0; i <= 180; i++) {
            //     std::cout << locationData_send.data.dist[i] << " ";
            // }
            // std::cout << std::endl;

        }

        //类中使能返回
        bool socketEn_get() const{
            return socket_en;
        }
        bool httpEn_get() const{
            return http_en;
        }



        int socketServer(){

            std::cout << "Waiting for relocalization to succeed..." << std::endl;
            std_msgs::Bool::ConstPtr relocalization_msg = ros::topic::waitForMessage<std_msgs::Bool>("map_to_odom_flag", nh);
            std::cout << "get for relocalization to succeed..." << std::endl;


            std::cout << "<--------into socket_thread--------->" << std::endl;
            ros::Rate socket_server_rate(100);
            /****创建socket连接(TCP:SOCK_STREAM  UDP:SOCK_DGRAM )*****/
            if((sockfd = socket(AF_INET,SOCK_DGRAM,0)) == -1){
                ROS_ERROR("Socket creation failed");
                return -1;
            }
            memset(&socket_ServerAddr, 0,sizeof(socket_ServerAddr));
            socket_ServerAddr.sin_family = AF_INET;
            socket_ServerAddr.sin_port = htons(server_PORT);

            if(inet_pton(AF_INET,server_addr.c_str(),&socket_ServerAddr.sin_addr) <= 0){
                ROS_ERROR("Invalid address/ Address not supported");
                return -1;
            }

            /***bind本机地址***/ 
            if(bind(sockfd,(struct sockaddr *)&socket_ServerAddr,sizeof(socket_ServerAddr)) < 0){
                ROS_ERROR("bind failed");
                return -1;
            }



            /************TCP************/
            // if(listen(sockfd,1) < 0){
            //     ROS_ERROR("listen failed");
            //     return -1;
            // }

            // if(connect(sockfd,(struct sockaddr *)&socket_ServerAddr,sizeof(socket_ServerAddr)) == -1){
            //     std::cerr << "Socket creation failed: " << strerror(errno) << std::endl;
            //     ROS_ERROR("Connection failed");
            // }

            //socklen_t clientAddrLen = sizeof(socket_ServerAddr);
            // int clientSocket =accept(sockfd,(struct sockaddr *)&socket_ServerAddr,&clientAddrLen);

            // std::cout << "socket created successfully" << std::endl;




            /**************UDP***************/
            /***UDP接收***/

            // ROS_INFO("socket accept successfully!!!!!!");
            memset(&socket_Client, 0,sizeof(socket_Client));
            socket_Client.sin_family = AF_INET;
            socket_Client.sin_port = htons(client_PORT);
            if(inet_pton(AF_INET,client_addr.c_str(),&socket_Client.sin_addr) <= 0){
                ROS_ERROR("Invalid client address/ Address not supported");
                return -1;
            }
            
             socklen_t clientAddrLen = sizeof(socket_Client);



            while (ros::ok())
            {
                // ROS_INFO("The location information is initialized successfully.");
                // if(recvfrom(sockfd, &recv_data, sizeof(recv_data), 0, (struct sockaddr *)&socket_Client, &clientAddrLen) < 0){
                //     ROS_ERROR("recv failed");
                // }

                // if(recv_data.command == 1){

                //     std::unique_lock<std::mutex> lock(mtx);
                //     recv_data.command = 2;
                //     nav_data.command = 2;
                //     sendto(sockfd, &nav_data, sizeof(nav_data), 0, (struct sockaddr *)&socket_Client, sizeof(socket_Client));
                //     // std::cout << "socket send successfully" << std::endl;
                //     std::cout << "x: " << nav_data.x << "y: " << nav_data.y << "yaw: " << nav_data.yaw_angle 
                //                 << "left: " << nav_data.left_obs_dist << "right: " << nav_data.right_obs_dist << std::endl;
                //     std::cout << "socket send successfully" << std::endl;
                        
                // }
                
                // // try{
                // //     /********TCP********/
                // //     // sendto(sockfd, &nav_data, sizeof(nav_data), 0);

                // //     /*********UDP**********/ 
                    
                // // }
                // // catch(const std::exception& e){
                // //     std::cerr<<"send failed: "<<e.what()<<std::endl;
                // // }


                if(recvfrom(sockfd, &receive_flag, sizeof(receive_flag), 0, (struct sockaddr *)&socket_Client, &clientAddrLen) < 0){
                    ROS_ERROR("recv failed");
                }

                if(receive_flag.frame_type == 1){
                    
                    std::unique_lock<std::mutex> lock(mtx);
                    receive_flag.frame_type = 2;
                    locationData_send.frame_type = 2;
                    locationData_send.seq = receive_flag.seq;
                    sendto(sockfd, &locationData_send, sizeof(locationData_send), 0, (struct sockaddr *)&socket_Client, sizeof(socket_Client));

                    //test

                    // std::cout << "x: " << nav_data.x << "y: " << nav_data.y << "yaw: " << nav_data.yaw_angle 
                    //           << "left: " << nav_data.left_obs_dist << "right: " << nav_data.right_obs_dist << std::endl;

                    std::cout << "[" << getCurrentTimestamp() << "] "
                              << "x: " << nav_data.x 
                              << " y: " << nav_data.y 
                              << " yaw: " << nav_data.yaw_angle << std::endl;


                    // std::cout << "Array after copy: ";
                    // for (int i = 0; i <= 180; i++) {
                    //     std::cout << locationData_send.data.dist[i] << " ";
                    // }
                    // std::cout << std::endl;
                    // std::cout << "socket send successfully" << std::endl;
                        
                }


                socket_server_rate.sleep();
            }
            return 0;
            
        }

        void httpPostRequest(const std::string& url) {
            ros::Rate http_client_rate(5);
            std::string messageData;
            while(ros::ok())
            {
                messageData = http_json2strData;
                CURL* curl = curl_easy_init();
                
                if (curl) {
                    std::cout << "init successfully" << std::endl;
                    CURLcode res;
                    struct curl_slist* headers = NULL;

                    // 设置要发送的HTTP请求的URL
                    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());

                    // 设置HTTP请求头
                    headers = curl_slist_append(headers, "Content-Type: application/json");
                    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

                    // 设置要发送的数据
                    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, messageData.c_str());

                    // 执行HTTP POST请求
                    res = curl_easy_perform(curl);
                    if (res != CURLE_OK) {
                        ROS_ERROR("curl_easy_perform() failed: %s", curl_easy_strerror(res));
                        std::cout << "http send fail" << std::endl;
                    }else{
                        std::cout << "http send successfully" << std::endl;
                    }

                    // 清理资源
                    curl_slist_free_all(headers);
                    curl_easy_cleanup(curl);
                    // std::cout << "http send successfully" << std::endl;
                    http_client_rate.sleep();
                }
            }
        }


        void run(){
            ros::Rate rate(publish_tf_rate);
            memset(&robot_position, 0,sizeof(robot_position));
            memset(&obstacle_position, 0,sizeof(obstacle_position));
            memset(&robot_nav_info, 0,sizeof(robot_nav_info));
            memset(&obstacle_nav_info, 0,sizeof(obstacle_nav_info));
            memset(&nav_data, 0,sizeof(nav_data));
            http_data["device_id"] = "robot507";                        
            http_data["x"] = 0;
            http_data["y"] = 0;
            http_data["yaw"] = 0;

            std::cout << "<--------Transform is running-------->" << std::endl;

            while (ros::ok())
            {
                try
                {
                    tf::StampedTransform TF;
                    tf::Quaternion rotation;
                    listener.waitForTransform(target_frame,source_frame,ros::Time(0),ros::Duration(5.0));   //等待数据(每一个监听器都有一个缓冲器),保证安全接收tf tree(look要对应时间戳)
                    if (!listener.canTransform(target_frame, source_frame, ros::Time(0))) {
                        ROS_WARN("Failed to get transform from %s to %s", source_frame.c_str(), target_frame.c_str());
                        continue;  // 重试
                    }
                    listener.lookupTransform(target_frame,source_frame,ros::Time(0),TF);    //获得两个坐标系的TF关系(必须确保在同一个tree)
                    rotation = TF.getRotation();
                    tf::Matrix3x3 rotation_matrix(rotation);
                    rotation_matrix.getRPY(roll,pitch,yaw);

                    {
                        std::unique_lock<std::mutex> lock(mtx);
                        // robot_position = {float(TF.getOrigin().x()), float(TF.getOrigin().y()), float(TF.getOrigin().z())};
                        // robot_nav_info = {robot_position,float(yaw)};
                        nav_data = {0,float(TF.getOrigin().x()),float(TF.getOrigin().y()),float(TF.getOrigin().z()),float(yaw),left_distance,right_distance};

                        locationData_send.frame_type = 0;
                        locationData_send.data.x = TF.getOrigin().x();
                        locationData_send.data.y = TF.getOrigin().y();
                        locationData_send.data.yaw = yaw;

                        http_data["x"] = TF.getOrigin().x();
                        http_data["y"] = TF.getOrigin().y();
                        http_data["yaw"] = yaw;
                        http_json2strData = http_data.dump();

                        rel_coords_data[0]=TF.getOrigin().x();
                        rel_coords_data[1]=TF.getOrigin().y();
                        rel_coords_data[2]=TF.getOrigin().z();
                        rel_coords_data[3]=roll;
                        rel_coords_data[4]=pitch;
                        rel_coords_data[5]=yaw;
                    }

                    // tf_pub.publish(rel_coords_data);

                    if(socket_print_en)
                    {
                        ROS_INFO("Got transform from %s to %s: [%f, %f, %f, %f, %f]",
                            source_frame.c_str(),target_frame.c_str(),
                            TF.getOrigin().x(),
                            TF.getOrigin().y(),
                            yaw,
                            left_distance,
                            right_distance);                 
                    } 
                    
                }
                catch(tf::LookupException &e)
                {
                    ROS_WARN("Failed to get transform");
                }
                ros::spinOnce();
                rate.sleep();
                // ros::spinOnce();
            }  
        }



};

int main(int argc,char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"tf_publish");
    TfPublish Tf_pub;

    
    if(Tf_pub.socketEn_get()){
        std::thread socketServerThread(&TfPublish::socketServer,&Tf_pub);
        socketServerThread.detach();
    }

    if(Tf_pub.httpEn_get()){
        std::thread httpClientThread(&TfPublish::httpPostRequest,&Tf_pub,HTTP_URL);
        httpClientThread.detach();
    }

    std::thread mainThread(&TfPublish::run,&Tf_pub);
    mainThread.detach();
    //Tf_pub.run();

    ros::spin();

    return 0;

}

