#ifndef LADDAR_DEVICE__HPP
#define LADDAR_DEVICE__HPP

#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <thread>
#include <functional> //std::bind
#include <iostream>
#include <atomic>
#include <mutex>
#include "path_control/include/deviceBase.hpp"

#define LADDAR_MAX_SEQ (unsigned long)1000
#define LADDAR_LINE_NUM 181
namespace LADDAR
{
    struct dataStruct
    {
        double x;
        double y;
        double yaw;
        double dist[LADDAR_LINE_NUM];
    };
    struct receiveFrameStruct
    {
        unsigned long frameType;
        unsigned long seq;
        dataStruct data;
    };
    struct requestFrameStruct
    {
        unsigned long frameType;
        unsigned long seq;
    };
};

class laddarDevice : public deviceBase
{
private:
    std::thread *ThreadHandler;
    const char *deviceIP;
    unsigned short devicePort;
    const char *myIp;
    unsigned short myPort;
    std::atomic<bool> threadEndFlag;
    std::atomic<bool> DataUpdateFlag;
    std::atomic<bool> laddarBeginFlag;
    std::mutex DataAccessMutex;

private:
    LADDAR::dataStruct dataBuf;

public:
    laddarDevice(const char *deviceIp_, unsigned short devicePort_, const char *myIp_, unsigned short myPort_);
    ~laddarDevice();
    void deviceWorkTHreadFunc(int fd) override;
    bool startWork() override;
    bool endWork() override;
    void watingDeviceEnding() override;
    bool readDeviceData(void *dataBuf, unsigned int &dataSize) override;
    bool sendDeviceData(void *dataBuf, unsigned int dataSize) override;
    bool isDeviceBeginWorking() override;
};

#endif