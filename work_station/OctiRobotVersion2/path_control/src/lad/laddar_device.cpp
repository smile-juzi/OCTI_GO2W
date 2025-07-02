#include "path_control/include/laddar_device.hpp"

laddarDevice::laddarDevice(const char *deviceIp_, unsigned short devicePort_, const char *myIp_, unsigned short myPort_)
{
    this->threadEndFlag = false;
    this->DataUpdateFlag = false;
    this->deviceIP = deviceIp_;
    this->devicePort = devicePort_;
    this->myIp = myIp_;
    this->myPort = myPort_;
    this->ThreadHandler = NULL;
    this->laddarBeginFlag = false;
}

laddarDevice::~laddarDevice()
{
    if (this->ThreadHandler != NULL)
    {
        if (this->ThreadHandler->joinable())
        {
            this->ThreadHandler->join();
            std::cout << "Device close success\n";
        }
    }
}

bool laddarDevice::endWork()
{
    this->threadEndFlag = true;
    return true;
}

bool laddarDevice::readDeviceData(void *dataBuf, unsigned int &dataSize)
{
    unsigned int temptTimes = 0;
    while (true)
    {
        if (this->DataUpdateFlag == true)
        {
            this->DataAccessMutex.lock();
            memcpy(dataBuf, &(this->dataBuf), sizeof(this->dataBuf));
            this->DataUpdateFlag = false;
            this->DataAccessMutex.unlock();
            return true;
        }
        else
        {
            temptTimes++;
            if(temptTimes >= 20)
            {
                std::cout << "laddar device may disconnect!\n";
                return false;
            }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(50000));
    }

    return true;
}

bool laddarDevice::sendDeviceData(void *dataBuf, unsigned int dataSize)
{
    return true;
}

bool laddarDevice::isDeviceBeginWorking()
{
    return this->laddarBeginFlag;
}

void laddarDevice::deviceWorkTHreadFunc(int fd)
{
    std::cout << "device thread begin!\n";
    // 先发数据请求
    unsigned long seq = 0;
    sockaddr_in server;
    sockaddr_in send_c;
    socklen_t size = sizeof(send_c);
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = inet_addr(this->deviceIP);
    server.sin_port = htons(this->devicePort);
    LADDAR::requestFrameStruct send_data;
    LADDAR::receiveFrameStruct receive_data;
    ssize_t num = 0;
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    do
    {
        if (this->threadEndFlag == true)
        {
            close(fd);
            std::cout << "Laddar Device close success\n";
            pthread_exit(NULL);
        }
        send_data.frameType = 1;
        send_data.seq = seq;
        sendto(fd, &send_data, sizeof(send_data), 0, (sockaddr *)&server, sizeof(server));
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fd, &readfds);
        //
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 50000;
        //

        int result = select(fd + 1, &readfds, NULL, NULL, &timeout);
        if (result < 0)
        {
            perror("select failed");
            close(fd);
            this->threadEndFlag == true;
            pthread_exit(NULL);
        }
        else if (result == 0)
        {
            continue;
        }
        // 接受雷达数据
        num = recvfrom(fd, &receive_data, sizeof(receive_data), 0, (sockaddr *)&send_c, &size);
        if (num == sizeof(LADDAR::receiveFrameStruct) && receive_data.frameType == 2 && receive_data.seq == seq)
        {
            seq = (seq + 1) % LADDAR_MAX_SEQ;
            // exit waiting laddar begin
            this->laddarBeginFlag = true;
            break;
        }
        else if(receive_data.seq < seq)
        {
            // std::cout << "receive req error recSeq = " << receive_data.seq << " Seq = " << seq << std::endl;
            std::this_thread::sleep_for(std::chrono::microseconds(50000));
        }
    } while (true);


    do
    {
        if (this->threadEndFlag == true)
        {
            close(fd);
            std::cout << "Laddar Device close success\n";
            pthread_exit(NULL);
        }
        send_data.frameType = 1;
        send_data.seq = seq;
        sendto(fd, &send_data, sizeof(send_data), 0, (sockaddr *)&server, sizeof(server));
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fd, &readfds);
        //
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 50000;
        //

        int result = select(fd + 1, &readfds, NULL, NULL, &timeout);
        if (result < 0)
        {
            perror("select failed");
            close(fd);
            this->threadEndFlag == true;
            pthread_exit(NULL);
        }
        else if (result == 0)
        {
            continue;
        }
        // 接受雷达数据
        num = recvfrom(fd, &receive_data, sizeof(receive_data), 0, (sockaddr *)&send_c, &size);
        if (num == sizeof(LADDAR::receiveFrameStruct) && receive_data.frameType == 2 && receive_data.seq == seq)
        {
            seq = (seq + 1) % LADDAR_MAX_SEQ;
            this->DataAccessMutex.lock();
            this->dataBuf = receive_data.data;
            DataUpdateFlag = true;
            this->DataAccessMutex.unlock();
            std::this_thread::sleep_for(std::chrono::microseconds(50000));
        }
        else if(receive_data.seq < seq)
        {
            // std::cout << "receive req error recSeq = " << receive_data.seq << " Seq = " << seq << std::endl;
            std::this_thread::sleep_for(std::chrono::microseconds(50000));
        }
    } while (true);
}

bool laddarDevice::startWork()
{
    this->threadEndFlag = false;
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
    {
        std::cout << "sock error\n";
        return false;
    }
    sockaddr_in mySockAddrIn;
    memset(&mySockAddrIn, 0, sizeof(mySockAddrIn));
    mySockAddrIn.sin_family = AF_INET;
    mySockAddrIn.sin_port = htons(this->myPort);
    mySockAddrIn.sin_addr.s_addr = inet_addr(this->myIp);
    int ret = bind(fd, (sockaddr *)&mySockAddrIn, sizeof(mySockAddrIn));
    if (ret < 0)
    {
        std::cout << "bind error\n";
        close(fd);
        return false;
    }
    this->ThreadHandler = new std::thread(std::bind(&laddarDevice::deviceWorkTHreadFunc, this, fd));
    if(this->ThreadHandler == NULL)
    {
        std::cout << "thread start error\n";
        close(fd);
        return false;
    }
    std::cout << "Laddar Device start success!\n";
    return true;
}

void laddarDevice::watingDeviceEnding()
{
    if (this->ThreadHandler != NULL)
    {
        this->ThreadHandler->join();
        this->ThreadHandler = NULL;
    }
    std::cout << "Laddar Device finished\n";
}