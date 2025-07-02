#ifndef COMMUNICATION__HPP
#define COMMUNICATION__HPP

struct commonFrameHead
{
    uint32_t frameType;
    uint32_t source;
    uint32_t dest;
    uint32_t subObj;
};

struct commonFrameData
{
    uint32_t hasData;
    uint32_t data1;
    uint32_t data2;
    float data3;
    float data4;
};

struct commonFrame
{
    struct commonFrameHead frameHead;
    struct commonFrameData framedata;
};



#define SERVER 0
#define THR 5

// frame type
#define REQUESTDATA 0
#define DATA 1
#define CONTROL 2

//control
#define OPENTHR 1
#define CLOSETHR 2

#endif