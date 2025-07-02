

#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <linux/v4l2-controls.h>
#include <termio.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <stdbool.h>
#include <math.h>
#include <semaphore.h>
#include <pthread.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/core/core_c.h>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <fstream>
#include <thread>
#include <mutex>
#include "./communication.hpp"
#include "thermometry.h"
#include "SimplePictureProcessing.h"
#include "Usb.h"
#include "pot.h"

using namespace std;
using namespace cv;

#define SAFE_DELETE(p)  \
    {                   \
        if (p)          \
        {               \
            delete (p); \
            (p) = NULL; \
        }               \
    }

/**
 * OUTPUTMODE 4:配合libthermometry.so可以输出全局温度数据。
 *              配合simple.so使用专业级图像算法，可得到优秀画质的图像，但是需要主频1.8ghz。
 *              也可配合代码里的线性图像算法，画质稍低于高性能算法，但是对主频几乎没要求。
 *              输出数据格式：按照yuyv的形式，实际输出每个像素16bit中14bit为灰度数据，最后四行为参数。
 * OUTPUTMODE 4:Combine with libthermometry.so to output all around temperature data.
 *              Obtain high quality image with professional algorithms from simple.so, requires basic frequency at least 1.8ghz.
 *              Linear Algorithms produce lower quality image than professional algorithms but there are almost no requirements in basic frequency.
 *              Output data format: in yuyu form, the actual transferred is 16bit NUC data with 14 bits to store the wide dynamic grayscale value of one pixel.
 *              The last four lines of yuyu data are parameters.
 *
 *
 * OUTPUTMODE    5:配合libthermometry.so，可以直接输出中心点，最高温，最低温和额外指定的三个点的信息，不可输出全帧温度数据。
 *              输出数据格式：输出yuyv格式的图像，最后四行为参数。
 *  OUTPUTMODE 5:With libthermometry.so to directly output temperature of the center, highest, lowest and extra three points. Can't output full frame temperature data.
 *               Output data format: graphs in yuyv format, the last four lines are parameters.
 */
#define OUTPUTMODE 4
// #define OUTPUTMODE 5
// #define OUTPUTMODE 6

#define TRUE 1
#define FALSE 0
#define MAX_BUFFER 2048

#define FILE_VIDEO1 "video"
#define FILE_DEV "/dev"
#define IRRGBA 0
#define IRRGB 1
// 注意配置和设备相应的分辨率 Caution: set resolution that correspond to specification of the device
int IMAGEWIDTH = 640;
int IMAGEHEIGHT = 516;

pthread_t renderThreadId, handlerThreadId, bulkThreadPotId, bulkThreadFirmwareId; // 显示线程 / 处理线程（根据键盘按键，进行具体处理） / Bulk线程
static int fd = 0;                                                                // 设备描述符 Descriptor of device
struct v4l2_streamparm stream_para;                                               // 结构体v4l2_streamparm来描述视频流的属性 Struct v4l2_streamparm to describe the property of video stream
struct v4l2_capability cap;                                                       // 取得设备的capability，看看设备具有什么功能，比如是否具有视频输入,或者音频输入输出等
                                                                                  // Obtain capability of device, check if it has video input or audio input/output abilities.
struct v4l2_fmtdesc fmtdesc;                                                      // 枚举设备所支持的image format:  VIDIOC_ENUM_FMT    Enumerate image format the deivce supported: VIDIOC_ENUM_FMT
struct v4l2_format fmt, fmtack;                                                   // 子结构体struct v4l2_pix_format设置摄像头采集视频的宽高和类型：V4L2_PIX_FMT_YYUV V4L2_PIX_FMT_YUYV
                                                                                  // Substruct struct v4l2_pix_format for setting Height, Width and Type of captured video
struct v4l2_requestbuffers req;                                                   // 向驱动申请帧缓冲的请求，里面包含申请的个数 Send request of frame buffer from driver, including amount of requests
struct v4l2_buffer buf;                                                           // 代表驱动中的一帧 One frame in the driver

struct buffer // 从相机获得的数据缓存 Capture data buffer from Camera
{
    void *start;
    unsigned int length;
    long long int timestamp;
} *buffers;

struct irBuffer // 使用专业级图像算法所需要的缓存   The buffer required by professional algorithm
{
    size_t **midVar;
    unsigned char *destBuffer;
} *irBuffers;

/**
 *temperatureData:最终输出的温度数据，采用10+全帧温度数据格式；例如10+384（宽）×288（高），前10个格式如下
 *The final output temperature data, in the form of "10 + Full Frame Temperature Data"; such as 10+384(width)×288(height), the top 10 as below
 *temperatureData[0]=centerTmp;
 *temperatureData[1]=(float)maxx1;
 *temperatureData[2]=(float)maxy1;
 *temperatureData[3]=maxTmp;
 *temperatureData[4]=(float)minx1;
 *temperatureData[5]=(float)miny1;
 *temperatureData[6]=minTmp;
 *temperatureData[7]=point1Tmp;
 *temperatureData[8]=point2Tmp;
 *temperatureData[9]=point3Tmp;
 *根据8004或者8005模式来查表，8005模式下仅输出以上注释的10个参数，8004模式下数据以上参数+全局温度数据
 *Search on the table with 8004 or 8005 mode, 8005 mode only outputs the 10 parameters above, 8004 mode include above parameters with overall temperature data
 *参见：thermometrySearch函数 Refer to function thermometrySearch
 */
float *temperatureData = NULL; // 10个温度值

float *temperatureDataFixed = NULL;
float maxTmp = 0; // 最高温度变量
float minTmp = 0; // 最低温度变量

/**
 *设置三个单独点 Set three points
 *温度会出现在temperatureData[7]，temperatureData[8]，temperatureData[9] shows temperature
 *0<=viewX1<IMAGEWIDTH
 *0<=viewY1<IMAGEHEIGHT-4
 */
void setPoint(int viewX1, int viewY1, int indexOfPoint);
enum v4l2_buf_type type; // 帧类型  Type of buffer
struct v4l2_control ctrl;

struct tempdata
{
    float avgTmp;
    float centerTmp;
    float maxTmp;
    float minTmp;
    float pointTmp;
};

struct data_receive
{
    int type;
    int state;
    struct tempdata temp;
};

int sockfd;
struct sockaddr_in clientaddr;

/**
 *temperatureTable:温度映射表
 */
float temperatureTable[16384];

int init_v4l2(string videoX); // 初始化 Initialization
int v4l2_grab(void);          // 采集 Capture
int v4l2_control(int);        // 控制 Control
int traversalVideo(void);     // 遍历video，如果是多个UVC设备可以在此处增加判断，是不是红外UVC
                              // Traveral video, may add determine statements if there are multiple UVC devices, weither infrared UVC

int delayy;
void sendCorrection(float correction); // 设置修正，一般取（-3.0)-3.0,用于整体温度向上或者向下修正  Set correction, usally -3.0/3.0 to correct temperature lower or higher

void sendReflection(float reflection); // 设置反射温度，一般情况下为环境温度  Set reflection temperature, usually ambient temperature

/*反射温度的确定：
当周围没有热源时，目标反射的是环境温度，所以一般情况下反射温度等于环境温度。
当周围有热源时，目标会反射热源的温度。如何确定反射温度：
1）取一张铝箔（红外反射镜），弄皱后再展平（朗伯面），将铝箔放在纸板上，亮面朝上。
2）调节热像仪发射率为1。
3）将铝箔放在目标表面前面并与之平行，用热像仪测量反射镜表面温度，此温度即反射温度。*/
/*How to find out reflection temperature:
When there is no heat source nearby, the object reflects ambient temperature, so usually refection equals ambient temperature.
When there is heat source, object reflects temperature of heat source. How to know the reflection temperature:
1)Take an aluminum foil as Mirror to reflect infrared ray, wrinkle the foil and then flat it. Put the foil on cardboard, the bright surface upwards
2)Adjust the emissivity of device to 1;
3)Parallel foil with object, measure surface temperature of foil with thermal imager. The measured temperature is reflection temperature.
*/

void sendAmb(float amb); // 设置环境温度   Set ambient temperature

void sendHumidity(float humidity); // 设置湿度（0-1.0），一般0.45   Set humidity (0-1.0), usually 0.45

void sendEmissivity(float emiss); // 发射率（0-1.0），具体发射率见附表数据   Emissivity(0-1.0), refer to emissivity table

void sendDistance(unsigned short distance); // 距离（单位米）  Distance (Unit: Meter)

void sendC_fix_NT(float c_fix);

void sendShutter_fix_NT(float Shutter_fix);

void sendC_fix_HT(float c_fix);

void sendShutter_fix_HT(float Shutter_fix);

void getCenterNuc(unsigned short *centernuc);

void savePara();    // 保存参数到机芯，断电不丢失   Save parameter to the chipset
int v4l2_release(); // 释放v4l2  Release v4l2
int palette = 0;
int displayMode = 0;
int lowTmpPaletteIndex = 0;
int computeMethod = 2;
bool autoCalibtateShutter_Flag = FALSE; // 计算c值修正和快门修正
bool fix_Calculate_Done = FALSE;        // 修正值计算是否完成  1表示完成
bool temperautreOutput_Flag = FALSE;    // 输出当前中心点温度
bool distanceFix_flag = FALSE;          // 模拟计算距离补偿
bool hightemperature_Flag;
bool saveImage_Flag = 0;
bool KsendFlag = false;
int kLineCnt = 0;
bool isKreceived = false;
int shutterCnt = 0;
int nucFrameCnt = 0;
bool isBCaculated = false;
bool firstFrame = true;
bool nucFreeze = false;
bool firstShutFrame = false;
bool grab_done = FALSE; // 采集是否完成  TRUE表示完成
unsigned short center_nuc_100 = 2289;
unsigned short center_nuc_300 = 3621;
unsigned short center_nuc_500 = 5825;

const unsigned char *paletteIronRainbow = getPalette(0);   // 256*3 铁虹   Iron Rainbow
const unsigned char *palette3 = getPalette(1);             // 256*3 彩虹1    Rainbow 1
const unsigned char *paletteRainbow = getPalette(2);       // 224*3 彩虹2    Rainbow 2  实际可用220
const unsigned char *paletteHighRainbow = getPalette(3);   // 448*3 高动态彩虹   HDR rainbow   实际可用444
const unsigned char *paletteHighContrast = getPalette(4);  // 448*3 高对比彩虹    High Contrast rainbow
const unsigned char *lavaRainbow = getPalette(5);          // 256*3 lava Rainbow
const unsigned char *paletteThIronRainbow = getPalette(6); // 256*3 th Iron Rainbow
const static unsigned char colormap_Iron888[256 * 3] =
    {
        0x0, 0x14, 0x18, 0x0, 0x10, 0x20, 0x0, 0xc, 0x20, 0x0, 0x8, 0x28, 0x0, 0x8, 0x30, 0x0, 0x8,
        0x38, 0x0, 0x8, 0x38, 0x0, 0x8, 0x38, 0x0, 0x4, 0x40, 0x0, 0x4, 0x48, 0x0, 0x4, 0x50,
        0x0, 0x0, 0x50, 0x0, 0x0, 0x50, 0x8, 0x0, 0x58, 0x8, 0x0, 0x58, 0x8, 0x0, 0x60, 0x10,
        0x0, 0x60, 0x10, 0x0, 0x60, 0x18, 0x0, 0x68, 0x18, 0x0, 0x68, 0x18, 0x0, 0x70, 0x20, 0x0,
        0x78, 0x20, 0x0, 0x78, 0x28, 0x0, 0x78, 0x28, 0x0, 0x80, 0x28, 0x0, 0x80, 0x30, 0x0, 0x80,
        0x30, 0x0, 0x88, 0x38, 0x0, 0x88, 0x38, 0x0, 0x88, 0x38, 0x0, 0x88, 0x40, 0x0, 0x90, 0x40,
        0x0, 0x90, 0x40, 0x0, 0x90, 0x48, 0x0, 0x90, 0x48, 0x0, 0x90, 0x48, 0x0, 0x98, 0x50, 0x0,
        0x98, 0x50, 0x0, 0x98, 0x50, 0x0, 0x98, 0x58, 0x0, 0x98, 0x58, 0x0, 0x98, 0x58, 0x0, 0x98,
        0x60, 0x0, 0x98, 0x60, 0x0, 0xa0, 0x68, 0x0, 0xa0, 0x68, 0x0, 0xa0, 0x68, 0x0, 0xa0, 0x68,
        0x0, 0xa0, 0x70, 0x0, 0xa0, 0x70, 0x0, 0xa0, 0x70, 0x0, 0xa8, 0x70, 0x0, 0xa8, 0x78, 0x0,
        0xa8, 0x78, 0x0, 0xa8, 0x78, 0x0, 0xa8, 0x80, 0x0, 0xa0, 0x80, 0x0, 0xa8, 0x80, 0x0, 0xa8,
        0x88, 0x0, 0xa8, 0x88, 0x0, 0xa0, 0x88, 0x0, 0xa0, 0x88, 0x0, 0xa0, 0x88, 0x0, 0xa8, 0x90,
        0x0, 0xa0, 0x90, 0x0, 0xa0, 0x90, 0x4, 0xa0, 0x98, 0x4, 0xa0, 0x98, 0x4, 0xa0, 0xa0, 0x4,
        0xa0, 0xa0, 0x4, 0xa0, 0xa0, 0x4, 0xa0, 0xa0, 0x4, 0xa0, 0xa0, 0x8, 0xa0, 0xa0, 0x8, 0x98,
        0xa8, 0xc, 0x98, 0xa8, 0xc, 0x98, 0xa8, 0xc, 0x98, 0xa8, 0x10, 0x98, 0xb0, 0x10, 0x98, 0xb0,
        0x10, 0x98, 0xb0, 0x10, 0x98, 0xb0, 0x10, 0x90, 0xb0, 0x10, 0x90, 0xb8, 0x14, 0x90, 0xb8, 0x14,
        0x88, 0xb8, 0x18, 0x88, 0xb8, 0x18, 0x88, 0xb8, 0x18, 0x88, 0xc0, 0x1c, 0x88, 0xc0, 0x1c, 0x88,
        0xc0, 0x1c, 0x80, 0xc0, 0x1c, 0x80, 0xc8, 0x20, 0x80, 0xc8, 0x20, 0x80, 0xc8, 0x20, 0x80, 0xc8,
        0x20, 0x80, 0xc8, 0x20, 0x78, 0xc8, 0x24, 0x78, 0xc8, 0x28, 0x78, 0xd0, 0x28, 0x70, 0xd0, 0x28,
        0x70, 0xd0, 0x28, 0x70, 0xd0, 0x2c, 0x70, 0xd8, 0x2c, 0x68, 0xd8, 0x2c, 0x68, 0xd8, 0x30, 0x68,
        0xd8, 0x30, 0x68, 0xd8, 0x30, 0x60, 0xd8, 0x34, 0x60, 0xd8, 0x34, 0x60, 0xd8, 0x38, 0x60, 0xd8,
        0x38, 0x58, 0xe0, 0x3c, 0x58, 0xe0, 0x3c, 0x58, 0xe0, 0x3c, 0x58, 0xe0, 0x40, 0x50, 0xe0, 0x40,
        0x50, 0xe0, 0x40, 0x50, 0xe0, 0x44, 0x48, 0xe0, 0x44, 0x48, 0xe8, 0x48, 0x48, 0xe8, 0x48, 0x48,
        0xe8, 0x4c, 0x40, 0xe8, 0x4c, 0x40, 0xe8, 0x50, 0x40, 0xe8, 0x50, 0x40, 0xe8, 0x50, 0x38, 0xf0,
        0x54, 0x38, 0xf0, 0x54, 0x38, 0xf0, 0x58, 0x38, 0xf0, 0x58, 0x38, 0xf0, 0x58, 0x30, 0xf0, 0x5c,
        0x30, 0xf0, 0x5c, 0x30, 0xf0, 0x60, 0x28, 0xf0, 0x60, 0x28, 0xf0, 0x64, 0x28, 0xf0, 0x64, 0x28,
        0xf8, 0x64, 0x20, 0xf8, 0x68, 0x20, 0xf8, 0x68, 0x20, 0xf8, 0x6c, 0x20, 0xf8, 0x6c, 0x20, 0xf8,
        0x70, 0x20, 0xf8, 0x70, 0x18, 0xf8, 0x74, 0x18, 0xf8, 0x74, 0x18, 0xf8, 0x74, 0x10, 0xf8, 0x78,
        0x10, 0xf8, 0x78, 0x10, 0xf8, 0x7c, 0x10, 0xf8, 0x7c, 0x10, 0xf8, 0x80, 0x10, 0xf8, 0x80, 0x10,
        0xf8, 0x80, 0x8, 0xf8, 0x84, 0x8, 0xf8, 0x88, 0x8, 0xf8, 0x88, 0x0, 0xf8, 0x88, 0x0, 0xf8,
        0x8c, 0x0, 0xf8, 0x8c, 0x0, 0xf8, 0x8c, 0x0, 0xf8, 0x90, 0x0, 0xf8, 0x90, 0x0, 0xf8, 0x90,
        0x0, 0xf8, 0x94, 0x0, 0xf8, 0x94, 0x0, 0xf8, 0x98, 0x0, 0xf8, 0x98, 0x0, 0xf8, 0x9c, 0x0,
        0xf8, 0x9c, 0x0, 0xf8, 0x9c, 0x0, 0xf8, 0xa0, 0x0, 0xf8, 0xa4, 0x0, 0xf8, 0xa4, 0x0, 0xf8,
        0xa4, 0x0, 0xf8, 0xa8, 0x0, 0xf8, 0xa8, 0x0, 0xf8, 0xa8, 0x0, 0xf8, 0xac, 0x0, 0xf8, 0xac,
        0x0, 0xf8, 0xb0, 0x0, 0xf8, 0xb0, 0x0, 0xf8, 0xb4, 0x0, 0xf8, 0xb4, 0x0, 0xf8, 0xb8, 0x0,
        0xf8, 0xb8, 0x0, 0xf8, 0xb8, 0x0, 0xf8, 0xbc, 0x0, 0xf8, 0xbc, 0x0, 0xf8, 0xbc, 0x0, 0xf8,
        0xc0, 0x0, 0xf8, 0xc0, 0x0, 0xf8, 0xc0, 0x0, 0xf8, 0xc4, 0x0, 0xf8, 0xc4, 0x0, 0xf8, 0xc8,
        0x0, 0xf8, 0xc8, 0x0, 0xf8, 0xc8, 0x0, 0xf8, 0xcc, 0x0, 0xf8, 0xcc, 0x8, 0xf8, 0xcc, 0x8,
        0xf8, 0xd0, 0x8, 0xf8, 0xd0, 0x10, 0xf8, 0xd0, 0x10, 0xf8, 0xd4, 0x10, 0xf8, 0xd4, 0x10, 0xf8,
        0xd4, 0x18, 0xf8, 0xd8, 0x18, 0xf8, 0xd8, 0x18, 0xf8, 0xdc, 0x20, 0xf8, 0xdc, 0x20, 0xf8, 0xdc,
        0x20, 0xf8, 0xe0, 0x28, 0xf8, 0xe0, 0x28, 0xf8, 0xe0, 0x28, 0xf8, 0xe0, 0x30, 0xf8, 0xe4, 0x30,
        0xf8, 0xe4, 0x38, 0xf8, 0xe4, 0x38, 0xf8, 0xe8, 0x40, 0xf8, 0xe8, 0x40, 0xf8, 0xe8, 0x40, 0xf8,
        0xec, 0x48, 0xf8, 0xec, 0x48, 0xf8, 0xec, 0x50, 0xf8, 0xec, 0x50, 0xf8, 0xec, 0x58, 0xf8, 0xf0,
        0x58, 0xf8, 0xf0, 0x60, 0xf8, 0xf0, 0x60, 0xf8, 0xf0, 0x68, 0xf8, 0xf4, 0x70, 0xf8, 0xf4, 0x78,
        0xf8, 0xf4, 0x78, 0xf8, 0xf4, 0x80, 0xf8, 0xf4, 0x88, 0xf8, 0xf8, 0x88, 0xf8, 0xf8, 0x90, 0xf8,
        0xf8, 0x98, 0xf8, 0xf8, 0xa0, 0xf8, 0xfc, 0xa0, 0xf8, 0xfc, 0xa8, 0xf8, 0xfc, 0xb0, 0xf0, 0xfc,
        0xb8, 0xf0, 0xfc, 0xc0, 0xf0, 0xfc, 0xc8, 0xf0, 0xfc, 0xc8, 0xf0, 0xfc, 0xd0, 0xf0, 0xfc, 0xd8,
        0xf0, 0xfc, 0xe0, 0xf0, 0xfc, 0xe8, 0xf0, 0xfc, 0xf0, 0xf0, 0xfc, 0xf0, 0xf0, 0xfc, 0xf0}; // 铁红
const static unsigned char colormap_RainBowHC888[256 * 3] =
    {
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x8, 0x0, 0x8, 0x8, 0x0, 0x8, 0x8, 0x0,
        0x10, 0x10, 0x0, 0x10, 0x18, 0x0, 0x18, 0x20, 0x0, 0x18, 0x20, 0x0, 0x20, 0x28, 0x0, 0x20,
        0x28, 0x0, 0x28, 0x30, 0x0, 0x30, 0x30, 0x0, 0x30, 0x38, 0x0, 0x38, 0x40, 0x0, 0x40, 0x48,
        0x0, 0x48, 0x48, 0x0, 0x48, 0x50, 0x0, 0x50, 0x58, 0x0, 0x50, 0x58, 0x0, 0x58, 0x60, 0x0,
        0x58, 0x68, 0x0, 0x60, 0x68, 0x0, 0x68, 0x68, 0x0, 0x68, 0x70, 0x0, 0x70, 0x78, 0x0, 0x78,
        0x80, 0x0, 0x80, 0x88, 0x0, 0x88, 0x88, 0x0, 0x88, 0x90, 0x0, 0x90, 0x98, 0x0, 0x98, 0xa0,
        0x0, 0x98, 0xa0, 0x0, 0xa0, 0xa8, 0x0, 0xa8, 0xa8, 0x0, 0xa8, 0xb0, 0x0, 0xb0, 0xb0, 0x0,
        0xb0, 0xb8, 0x0, 0xb8, 0xc0, 0x0, 0xc0, 0xc8, 0x0, 0xc8, 0xc8, 0x0, 0xc8, 0xd0, 0x0, 0xd0,
        0xd0, 0x0, 0xd0, 0xd8, 0x0, 0xd8, 0xd8, 0x0, 0xd8, 0xe0, 0x0, 0xe0, 0xe0, 0x0, 0xe0, 0xd8,
        0x0, 0xe0, 0xd8, 0x0, 0xe0, 0xd8, 0x0, 0xe0, 0xd0, 0x0, 0xe0, 0xd0, 0x0, 0xe0, 0xc8, 0x0,
        0xd8, 0xc8, 0x0, 0xd8, 0xc0, 0x0, 0xd8, 0xc0, 0x0, 0xd8, 0xb8, 0x0, 0xd8, 0xb0, 0x0, 0xd0,
        0xb0, 0x0, 0xd0, 0xb0, 0x0, 0xd0, 0xa8, 0x0, 0xd0, 0xa8, 0x0, 0xd0, 0xa0, 0x0, 0xc8, 0x98,
        0x0, 0xc8, 0x98, 0x0, 0xc8, 0x90, 0x0, 0xc8, 0x90, 0x0, 0xc8, 0x88, 0x0, 0xc8, 0x88, 0x0,
        0xc0, 0x88, 0x0, 0xc0, 0x80, 0x0, 0xc0, 0x78, 0x0, 0xc0, 0x78, 0x0, 0xc0, 0x70, 0x0, 0xb8,
        0x68, 0x0, 0xb8, 0x60, 0x0, 0xb8, 0x60, 0x0, 0xb8, 0x58, 0x0, 0xb8, 0x58, 0x0, 0xb0, 0x50,
        0x0, 0xb0, 0x48, 0x0, 0xb0, 0x40, 0x0, 0xa8, 0x38, 0x0, 0xa8, 0x38, 0x0, 0xa8, 0x30, 0x0,
        0xa8, 0x28, 0x0, 0xa0, 0x20, 0x0, 0xa0, 0x18, 0x0, 0xa0, 0x8, 0x0, 0x98, 0x0, 0x0, 0x98,
        0x0, 0x4, 0x98, 0x0, 0x8, 0x98, 0x0, 0x18, 0xa0, 0x0, 0x20, 0xa0, 0x0, 0x30, 0xa8, 0x0,
        0x40, 0xb0, 0x0, 0x4c, 0xb0, 0x0, 0x5c, 0xb8, 0x0, 0x60, 0xb8, 0x0, 0x6c, 0xb8, 0x0, 0x78,
        0xb8, 0x0, 0x88, 0xc0, 0x0, 0x98, 0xc8, 0x0, 0xa0, 0xc8, 0x0, 0xb0, 0xd0, 0x0, 0xb8, 0xd0,
        0x0, 0xc4, 0xd8, 0x0, 0xc8, 0xd8, 0x0, 0xd4, 0xd8, 0x0, 0xe0, 0xe0, 0x0, 0xdc, 0xd8, 0x0,
        0xd8, 0xd0, 0x0, 0xcc, 0xc0, 0x0, 0xc8, 0xb8, 0x0, 0xc4, 0xb0, 0x0, 0xc0, 0xa8, 0x0, 0xbc,
        0xa8, 0x0, 0xb8, 0xa0, 0x0, 0xb0, 0x98, 0x0, 0xa8, 0x88, 0x0, 0xa4, 0x80, 0x0, 0xa0, 0x80,
        0x0, 0x9c, 0x78, 0x0, 0x98, 0x70, 0x0, 0x98, 0x68, 0x0, 0x90, 0x60, 0x0, 0x8c, 0x58, 0x0,
        0x80, 0x48, 0x0, 0x80, 0x48, 0x0, 0x7c, 0x48, 0x0, 0x78, 0x40, 0x0, 0x74, 0x38, 0x0, 0x70,
        0x30, 0x0, 0x68, 0x28, 0x0, 0x64, 0x20, 0x0, 0x64, 0x20, 0x0, 0x60, 0x18, 0x0, 0x5c, 0x10,
        0x0, 0x58, 0x10, 0x0, 0x50, 0x0, 0x0, 0x4c, 0x0, 0x8, 0x54, 0x0, 0x10, 0x58, 0x0, 0x18,
        0x64, 0x0, 0x20, 0x64, 0x0, 0x20, 0x64, 0x0, 0x28, 0x68, 0x0, 0x30, 0x70, 0x0, 0x38, 0x74,
        0x0, 0x38, 0x78, 0x0, 0x40, 0x7c, 0x0, 0x50, 0x80, 0x0, 0x58, 0x88, 0x0, 0x60, 0x90, 0x0,
        0x68, 0x90, 0x0, 0x68, 0x94, 0x0, 0x78, 0x9c, 0x0, 0x78, 0xa0, 0x0, 0x80, 0xa8, 0x0, 0x88,
        0xac, 0x0, 0x98, 0xb8, 0x0, 0xa0, 0xb8, 0x0, 0xa8, 0xbc, 0x0, 0xb0, 0xc0, 0x0, 0xb8, 0xcc,
        0x0, 0xc0, 0xd0, 0x0, 0xc8, 0xd4, 0x0, 0xd0, 0xdc, 0x0, 0xd8, 0xe0, 0x0, 0xe0, 0xe0, 0x0,
        0xe0, 0xdc, 0x0, 0xd8, 0xd0, 0x0, 0xd0, 0xc4, 0x0, 0xc8, 0xb4, 0x0, 0xc8, 0xac, 0x0, 0xc0,
        0x9c, 0x0, 0xb8, 0x94, 0x0, 0xb8, 0x94, 0x0, 0xb8, 0x88, 0x0, 0xb0, 0x7c, 0x0, 0xb0, 0x74,
        0x0, 0xa8, 0x68, 0x0, 0xa0, 0x60, 0x0, 0xa0, 0x58, 0x0, 0x98, 0x4c, 0x0, 0x90, 0x40, 0x0,
        0x98, 0x40, 0x0, 0x90, 0x30, 0x0, 0x88, 0x24, 0x0, 0x80, 0x18, 0x0, 0x78, 0x10, 0x0, 0x78,
        0x4, 0x0, 0x78, 0x0, 0x0, 0x78, 0x0, 0x0, 0x80, 0x0, 0x0, 0x80, 0x4, 0x0, 0x88, 0x4,
        0x0, 0x88, 0x8, 0x8, 0x90, 0xc, 0x8, 0x90, 0xc, 0x8, 0x98, 0x10, 0x10, 0x98, 0x10, 0x10,
        0xa0, 0x14, 0x10, 0xa0, 0x14, 0x10, 0xa8, 0x14, 0x18, 0xa8, 0x18, 0x18, 0xb0, 0x1c, 0x18, 0xb0,
        0x1c, 0x18, 0xb8, 0x20, 0x20, 0xb8, 0x20, 0x20, 0xc0, 0x20, 0x20, 0xc0, 0x24, 0x20, 0xc0, 0x24,
        0x28, 0xc8, 0x24, 0x28, 0xc8, 0x28, 0x28, 0xd0, 0x2c, 0x28, 0xd0, 0x2c, 0x28, 0xd8, 0x30, 0x30,
        0xd8, 0x34, 0x30, 0xd8, 0x3c, 0x38, 0xd8, 0x40, 0x40, 0xd8, 0x48, 0x48, 0xd8, 0x50, 0x50, 0xd8,
        0x58, 0x58, 0xe0, 0x64, 0x60, 0xe0, 0x64, 0x68, 0xe0, 0x6c, 0x68, 0xe0, 0x70, 0x70, 0xe0, 0x78,
        0x78, 0xe0, 0x80, 0x80, 0xe8, 0x88, 0x88, 0xe8, 0x8c, 0x90, 0xe8, 0x90, 0x90, 0xe8, 0x98, 0x98,
        0xe8, 0x9c, 0x98, 0xe8, 0xa4, 0xa0, 0xe8, 0xac, 0xa8, 0xe8, 0xb0, 0xb0, 0xf0, 0xb8, 0xb8, 0xf0,
        0xb8, 0xb8, 0xf0, 0xc0, 0xb8, 0xf0, 0xc0, 0xc0, 0xf0, 0xc8, 0xc0, 0xf0, 0xcc, 0xc8, 0xf0, 0xd0,
        0xc8, 0xf0, 0xd4, 0xd0, 0xf0, 0xd8, 0xd8, 0xf0, 0xd8, 0xd8, 0xf0, 0xe0, 0xe0, 0xf8, 0xe4, 0xe0,
        0xf8, 0xe8, 0xe8, 0xf8, 0xe8, 0xe8, 0xf8, 0xf0, 0xf0, 0xf8, 0xf0, 0xf0, 0xf8, 0xf4, 0xf0}; // 高对比彩虹

int keyBoardNum = 0;
int KeyTestNew();
int isOn = 1;
int counter = 0;
int uvcState = 1;
int nextTrans = 0;
short *collectedData = NULL;
unsigned short *tempBuffer;
int rightData = 1;
bool needRecal = 0;
sem_t sem; // 信号量数据类型

struct data_receive send_data;

struct tempdata sendTempData;
std::mutex dataMutex;
int dataUpdateFlag = 0;

int data_send_func(int instruction, int state, struct tempdata *temp)
{
    // send_data.type = instruction;
    // send_data.state = 1;
    // if (temp != NULL)
    // {
    //     send_data.temp.avgTmp = temp->avgTmp;
    //     send_data.temp.centerTmp = temp->centerTmp;
    //     send_data.temp.maxTmp = temp->maxTmp;
    //     send_data.temp.minTmp = temp->minTmp;
    //     send_data.temp.pointTmp = temp->pointTmp;
    // }
    // int err = sendto(sockfd, &send_data, sizeof(send_data), 0, (struct sockaddr *)&clientaddr, sizeof(clientaddr));
    // printf("send err = %d\n", err);
    // memset(&send_data, 0, sizeof(send_data));
    // return 1;
}

int KeyTestNew()
{
    struct termios oldt, newt;
    int c, oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    c = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if (EOF != c)
    {
        if (c >= 'a' && c <= 'z')
        {
            return (c - 'a' + 'A');
        }
        else
        {
            return c;
        }
    }
    else
    {
        return 0;
    }
}

// SetMethodTwoParameter(13,0,3,3,50,50);//目标突出
int methodTwoParameterA = 30;
int methodTwoParameterB = 1;
int methodTwoParameterC = 1;
int methodTwoParameterD = 1;
int methodTwoParameterBright = 50;
int methodTwoParameterContra = 50;

void on_Trackbar(int, void *)
{
    SetMethodTwoParameter(methodTwoParameterA, methodTwoParameterB, methodTwoParameterC, methodTwoParameterD, methodTwoParameterBright, methodTwoParameterContra);
}

float divTmp1 = 25; // divTmp1摄氏度以下黑白显示，可根据实际温度需求修改
float divTmp2 = 40; // divTmp1-divTmp2黄到红色渐变，divTmp2以上红色，可根据实际温度需求修改
int divTmp1x10 = 250;
int divTmp2x10 = 400;
int divTmpPercent = 50;
int divTmpNuc1 = 6500;
int divTmpNuc2 = 9000;
int rangeMode = 120;

void on_Trackbar_divTmp(int, void *)
{
    int i = 0, divTmpNuc1State = 1;
    divTmp1 = divTmp1x10 / 10;
    divTmp2 = divTmp2x10 / 10;
    for (i = 0; i < 16384; i++)
    {
        if (divTmp1 < temperatureTable[i] && divTmpNuc1State)
        {
            divTmpNuc1 = i;
            divTmpNuc1State = 0;
            printf("divTmpNuc1:%d\n", divTmpNuc1);
        }
        if (divTmp2 < temperatureTable[i])
        {
            divTmpNuc2 = i;
            printf("divTmpNuc2:%d\n", divTmpNuc2);
            break;
        }
    }
    SetDivTemp(divTmpNuc1, divTmpNuc2);
}
void on_Trackbar_PaletteIndex(int, void *)
{
}

void *renderThread(void *arg);
void *bulkThreadPot(void *arg);
void *bulkThreadFirmware(void *arg);
void *handlerThread(void *arg)
{
    int palette05 = 0x8800;
    int temperatueParameter = 0;
    int grabbedNuc_Num = 0;
    while (isOn)
    {
        sem_wait(&sem); // P操作，等主线程获取完键盘内容后主线程会释放信号量，然后该处理线程会执行
        printf("keyBoardNum:%d\n", keyBoardNum);
        if (keyBoardNum == 10)
        { // 10 : enter:shutter
            if (uvcState == 1)
                // v4l2输出模式，摄像头控制
                v4l2_control(0x8000);
            if (OUTPUTMODE == 6)
            {
                shutterCnt = 1;
                nucFreeze = true;
                firstShutFrame = true;
                data_send_func('/10', 1, NULL);
            }
        } // enter:shutter

        if (keyBoardNum == 52)
        {
            if (uvcState == 1)
            {
                rightData = 0;
                usleep(200);
                savePara();
                usleep(100000);
                rightData = 1;
            }
        } // 4:save para

        if (keyBoardNum == 81)
        {
            if (uvcState == 1)
            {
                palette = 0;
            }

        } // q white hot

        if (keyBoardNum == 87)
        {
            if (uvcState == 1)
            {
                palette = 1;
            }
        } // w black hot
        if (keyBoardNum == 69)
        {
            if (uvcState == 1)
            {
                palette = 2;
            }
        } // e Iron Rainbow
        if (keyBoardNum == 82)
        {
            if (uvcState == 1)
            {
                palette = 3;
            }
        } // r Rainbow 1
        if (keyBoardNum == 84)
        {
            if (uvcState == 1)
            {
                palette = 4;
            }
        } // t Rainbow 2
        if (keyBoardNum == 89)
        {
            if (uvcState == 1)
            {
                palette = 5;
            }
        } // y HDR rainbow
        if (keyBoardNum == 85)
        {
            if (uvcState == 1)
            {
                palette = 6;
                ;
            }
        } // u High Contrast rainbow
        if (keyBoardNum == 73)
        {
            if (uvcState == 1)
            {
                palette = 7;
                v4l2_control(0x8800);
            }
        } // i lava Rainbow
        if (keyBoardNum == 79)
        {
            if (uvcState == 1)
            {
                palette = 8;
                v4l2_control(0x8802);
            }
        } // o th Iron Rainbow

        // if(keyBoardNum==73) { if(uvcState==1){SetUserPalette((unsigned char*)colormap_Iron888,100); palette=100;} }//i lava Rainbow
        // if(keyBoardNum==79) { if(uvcState==1){SetUserPalette((unsigned char*)colormap_RainBowHC888,100); palette=100;} }//o th Iron Rainbow

        if (keyBoardNum == 80)
        {
            if (uvcState == 1)
            {
                if (displayMode == 0)
                {
                    displayMode = 1;
                }
                else if (displayMode == 1)
                {
                    displayMode = 2;
                }
                else if (displayMode == 2)
                {
                    displayMode = 0;
                }
            }
        } // p  change displayMode

        if (keyBoardNum == 65)
        {
            uvcState = 1;
            pthread_attr_t attr;
            pthread_attr_init(&attr);
            pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
            int ret = pthread_create(&renderThreadId, &attr, renderThread, NULL);
            pthread_attr_destroy(&attr);
        } // a open uvc device

        //-----------------------------
        if (keyBoardNum == 83)
        {
            uvcState = 0;
        } // s close uvc device

        if (keyBoardNum == 70)
        {
            if (uvcState == 1)
                ;
            v4l2_control(0x8021);
            rangeMode = 400;
            hightemperature_Flag = 1;
            sleep(1);
            v4l2_control(0x8000);
            sleep(1);
            needRecal = 1;
            sleep(10);
            v4l2_control(0x8000);
            sleep(1);
            needRecal = 1;
            data_send_func('F', 1, NULL);
        } // f change rangeMode 400

        if (keyBoardNum == 71)
        {
            if (uvcState == 1)
                ;
            v4l2_control(0x8020);
            rangeMode = 120;
            sleep(1);
            v4l2_control(0x8000);
            sleep(1);
            needRecal = 1;
            sleep(10);
            v4l2_control(0x8000);
            sleep(1);
            needRecal = 1;
            data_send_func('G', 1, NULL);
        } // g change rangeMode 120

        if (keyBoardNum == 72)
        {
            if (uvcState == 1 && temperatueParameter < 6)
            {
                switch (temperatueParameter)
                {
                case 0:
                    sendCorrection(2.1f);
                    break;
                case 1:
                    sendAmb(26.0f);
                    break;
                case 2:
                    sendHumidity(0.47f);
                    break;
                case 3:
                    sendEmissivity(1.0f);
                    break;
                default:
                    sendDistance(3);
                    break;
                }
                sleep(1);
                needRecal = 1;
                temperatueParameter++;
            }
            else
            {
                temperatueParameter = 0;
            }
        } // h send temperatue parameter

        if (keyBoardNum == 74)
        {
            int frate = 0;
            float new_float_tmp006 = 6.0f;
            new_float_tmp006 = new_float_tmp006 * 10;
            if (new_float_tmp006 < 0)
            {
                frate = (int)(256 + new_float_tmp006);
            }
            else
            {
                frate = (int)(new_float_tmp006);
            }
            frate = 0x9500 | frate;
            v4l2_control(frate);
            sleep(1);
            v4l2_control(0x8000);
            sleep(2);
            v4l2_control(0x8000);
            rightData = 0;
            sleep(2);
            v4l2_control(0x80fe);
            usleep(100000);
            rightData = 1;
            needRecal = 1;
        } // j TMP006

        if (keyBoardNum == 90)
        {
            if (uvcState == 1 && OUTPUTMODE == 5)
            {
                if (palette05 < 0x8807)
                {
                    palette05++;
                }
                else
                {
                    palette05 = 0x8800;
                }
                v4l2_control(palette05);
            }
        } // z 8005 change palette

        if (keyBoardNum == 75)
        {
            if (uvcState == 1)
            {
                autoCalibtateShutter_Flag = TRUE;
                sleep(1);
            }
        }
        if (keyBoardNum == 76)
        {
            if (uvcState == 1 && grabbedNuc_Num < 3)
            {
                switch (grabbedNuc_Num)
                {
                case 0:
                    getCenterNuc(&center_nuc_100);
                    printf("center_nuc_100:%d\n", center_nuc_100);
                    break;
                case 1:
                    getCenterNuc(&center_nuc_300);
                    printf("center_nuc_300:%d\n", center_nuc_300);
                    break;
                default:
                    getCenterNuc(&center_nuc_500);
                    printf("center_nuc_500:%d\n", center_nuc_500);
                    grab_done = TRUE;
                    break;
                }
                sleep(1);
                needRecal = 1;
                grabbedNuc_Num++;
            }
            else
                grabbedNuc_Num = 0;
        }
        if (keyBoardNum == 77)
        {
            if (uvcState == 1)
            {
                temperautreOutput_Flag = TRUE;
                // sleep(1);
            }
        } // m 输出中心点温度

        if (keyBoardNum == 78)
        {
            if (uvcState == 1)
            {
                distanceFix_flag = TRUE;
                sleep(1);
            }
        } // n 距离补偿

        if (keyBoardNum == 66)
        {
            if (uvcState == 1)
            {
                saveImage_Flag = TRUE;
                sleep(1);
            }
        } // b 保存8004模式下的图片
    }
}

unsigned int n_buffers = 0;
void *renderThread(void *arg) //
{
    if (traversalVideo() == FALSE) // 打开摄像头，但没有开始图像数据传输，返回fd
    {
        printf("Init fail~~\n");
        uvcState = 0;
    }

    if (v4l2_grab() == FALSE) // 采集数据命令
    {
        printf("grab fail~~\n");
        uvcState = 0;
    }
    //
    // data_send_func('A', 1, NULL);
    //
    delayy = 0;
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; // Stream 或者Buffer的类型。此处肯定为V4L2_BUF_TYPE_VIDEO_CAPTURE  Stream or Buffer Type, constantly as V4L2_BUF_TYPE_VIDEO_CAPTURE
    buf.memory = V4L2_MEMORY_MMAP;          // Memory Mapping模式，则此处设置为：V4L2_MEMORY_MMAP   Memory Mapping mode，set as V4L2_MEMORY_MMAP

    uint16_t *KBuffer = new uint16_t[IMAGEWIDTH * (IMAGEHEIGHT - 4)];
    uint16_t *BBuffer = new uint16_t[IMAGEWIDTH * (IMAGEHEIGHT - 4)];
    uint16_t *kbBuffer = new uint16_t[IMAGEWIDTH * (IMAGEHEIGHT - 4)];
    uint16_t *RemoveStripeBuffer = new uint16_t[IMAGEWIDTH * (IMAGEHEIGHT - 4)];
    uint16_t *OutBuffer = new uint16_t[IMAGEWIDTH * (IMAGEHEIGHT - 4)];
    uint16_t *HoldBuffer = new uint16_t[IMAGEWIDTH * (IMAGEHEIGHT - 4)];
    uint16_t *shutterBuffer = new uint16_t[IMAGEWIDTH * (IMAGEHEIGHT - 4)];

    if (OUTPUTMODE == 4)
    {
        if (v4l2_control(0x8004) == FALSE) // 控制机芯切换为8004 原始数据输出   Switch to 8004 mode, output raw data
        {
            printf("control fail~~\n");
            // uvcState=0;
        }
        // 初始化专业级图像算法 Initialize Professional Algorithm
        irBuffers = (irBuffer *)malloc(4 * sizeof(*irBuffers));
        if (!irBuffers)
        {
            printf("Out of memory\n");
            return 0;
        }
        SimplePictureProcessingInit(IMAGEWIDTH, (IMAGEHEIGHT - 4));
        SetMethodTwoParameter(methodTwoParameterA, methodTwoParameterB, methodTwoParameterC, methodTwoParameterD, methodTwoParameterBright, methodTwoParameterContra);
        ; // 均衡
        for (n_buffers = 0; n_buffers < 4; n_buffers++)
        {
            irBuffers[n_buffers].midVar = (size_t **)calloc(7, sizeof(size_t *));
            SimplePictureProcessingInitMidVar(irBuffers[n_buffers].midVar);
        }
        // end -初始化高性能图像算法
        temperatureData = (float *)calloc(IMAGEWIDTH * (IMAGEHEIGHT - 4) + 10, sizeof(float));
        temperatureDataFixed = (float *)calloc(IMAGEWIDTH * (IMAGEHEIGHT - 4) + 10, sizeof(float));
    }

    int i = 100;
    double t;
    long long int extra_time = 0;
    long long int cur_time = 0;
    long long int last_time = 0;

    /*---------------------图像生成--------------------*/
    // 生成的rgb图像，cv使用bgr格式，所以除了黑热和白热，其他伪彩存在通道偏差，需要再转换一下。
    // formed rgba image, CV with bgr format. So there is channel deviation in pseudo color, need to convert,except Black Hot and White Hot.
    cv::Mat bgrImg(IMAGEHEIGHT - 4, IMAGEWIDTH, CV_8UC4);
    cv::Mat rgbImgBuffer[4];
    for (n_buffers = 0; n_buffers < 4; n_buffers++)
    {
        rgbImgBuffer[n_buffers].create(IMAGEHEIGHT - 4, IMAGEWIDTH, CV_8UC4);
    }

    // 灰度图像 luminous image.
    cv::Mat lumiImg(IMAGEHEIGHT - 4, IMAGEWIDTH, CV_8UC1);

    // 8005模式输出的yuyv图像。 8005 mode output yuyv image.
    cv::Mat yuvImg;
    yuvImg.create(IMAGEHEIGHT - 4, IMAGEWIDTH, CV_8UC2);

    // 测温相关参数，详见thermometry.h    Refer to thermometry.h for relevant parameters

    int haveChangedrangeMode = 0;
    float floatFpaTmp;
    float correction = 0.0f;
    float Refltmp = 25.0f;
    float ambient = 25.0f;
    float humi = 0.45f;
    float emiss = 0.98f;
    unsigned short distance = 1;
    int cameraLens = 130;
    float shutterFix;
    float c_fix = 0;
    // end -测温相关参数

    int time = 0; // 自动标定次数
    int err = 0;  // 错误的标定结果返回

    char sn[32];                // camera序列码   camera serial number
    char cameraSoftVersion[16]; // camera软件版本   camera Software Version
    unsigned short shutTemper;
    float floatShutTemper; // 快门温度    Shutter Temperature
    unsigned short coreTemper;
    float floatCoreTemper; // 外壳温度   Shell temperature
    printf("renderThread 0\n");
    printf("uvcstate:%d\n", uvcState);
    // uvcState = 1时会进入while死循环
    while (uvcState)
    {
        for (n_buffers = 0; n_buffers < 4; n_buffers++)
        {
            t = (double)cvGetTickCount();
            // 出队  out of queue
            buf.index = n_buffers;
            ioctl(fd, VIDIOC_DQBUF, &buf);
            delayy++;
            if (KsendFlag)
            {
                uint16_t *orgBuffer = (uint16_t *)buffers[n_buffers].start;

                int KlineBytes = IMAGEWIDTH * 2;
                int curLocation;
                for (int i = 0; i < (IMAGEHEIGHT - 4); ++i)
                {
                    curLocation = i * IMAGEWIDTH;
                    unsigned long sumRowPixel = 0;
                    for (int j = 0; j < IMAGEWIDTH; j++)
                    {
                        sumRowPixel += orgBuffer[j + i * IMAGEWIDTH];
                    }
                    sumRowPixel = sumRowPixel / IMAGEWIDTH;
                    if (sumRowPixel > 10000)
                    {
                        if (((orgBuffer[2 + i * IMAGEWIDTH] > 14000) || (orgBuffer[2 + i * IMAGEWIDTH] == 0)) && ((orgBuffer[13 + i * IMAGEWIDTH] > 14000) || (orgBuffer[13 + i * IMAGEWIDTH] == 0)))
                        {
                            memcpy(&KBuffer[kLineCnt * IMAGEWIDTH], &orgBuffer[curLocation], KlineBytes);
                            kLineCnt++;
                        }
                    }
                }
                if (kLineCnt == (IMAGEHEIGHT - 4))
                {
                    callFindBadPix(KBuffer);
                    KsendFlag = false;
                    isKreceived = true;
                    printf("isKreceived successfully\n");
                }
                else
                {
                    printf("current kLineCnt:%d\n", kLineCnt);
                    ioctl(fd, VIDIOC_QBUF, &buf);
                    continue;
                }
            }

            int checkresult = dataCheck((unsigned short *)buffers[n_buffers].start, IMAGEWIDTH, IMAGEHEIGHT - 4, OUTPUTMODE);
            // printf ("checkresult:%d\n", checkresult);
            if (buf.bytesused != IMAGEHEIGHT * IMAGEWIDTH * 2 || !rightData || checkresult == -1) // 数据不完整直接入队  incomplete data ,enqueueing cycle
            {
                ioctl(fd, VIDIOC_QBUF, &buf);
                printf("throw data\n");
            }

            if (rightData && (checkresult == 0))
            {
                // 查看采集数据的时间戳之差，单位为微妙   Check the timestamp of data capturing, unit: microsecond
                buffers[n_buffers].timestamp = buf.timestamp.tv_sec * 1000000 + buf.timestamp.tv_usec;
                cur_time = buffers[n_buffers].timestamp;
                extra_time = cur_time - last_time;
                last_time = cur_time;

                if (shutterCnt > 0)
                {
                    // printf("shutterCnt:%d\n",shutterCnt);
                    if ((shutterCnt > 19) && (nucFrameCnt < 5))
                    {
                        if (nucFrameCnt == 0)
                        {
                            memset(BBuffer, 0x00, IMAGEWIDTH * (IMAGEHEIGHT - 4) * sizeof(uint16_t));
                        }
                        uint16_t *orgBuffer = (uint16_t *)buffers[n_buffers].start;
                        for (int i = 0; i < IMAGEWIDTH * (IMAGEHEIGHT - 4); ++i)
                        {
                            BBuffer[i] += orgBuffer[i];
                        }
                        nucFrameCnt++;
                    }
                    if (nucFrameCnt == 3)
                    {
                        for (int i = 0; i < IMAGEWIDTH * (IMAGEHEIGHT - 4); ++i)
                        {
                            BBuffer[i] = (uint16_t)((BBuffer[i]) / 3.0f + 0.5f);
                        }
                        isBCaculated = true;
                        nucFrameCnt = 5;

                        printf("isBCaculated\n");
                    }
                    else if (shutterCnt < 31)
                    {
                        shutterCnt++;
                    }
                }

                unsigned short *orgData = (unsigned short *)buffers[n_buffers].start;
                unsigned short *fourLinePara = (unsigned short *)(buffers[n_buffers].start) + IMAGEWIDTH * (IMAGEHEIGHT - 4); // 后四行参数  Parameters in last four lines

                if (isBCaculated && isKreceived)
                {
                    // printf("nucFreeze:%d\n",nucFreeze);
                    if (!nucFreeze)
                    {
                        kbAdjust(KBuffer, BBuffer, orgData, kbBuffer, IMAGEWIDTH, IMAGEHEIGHT - 4, 0);

                        // callFindBadPix(KBuffer);

                        processFindBadK(kbBuffer);

                        remove_stripe(kbBuffer, RemoveStripeBuffer, IMAGEWIDTH, IMAGEHEIGHT - 4);

                        if (firstFrame)
                        {
                            printf("firstFrame:%d\n", firstFrame);
                            unsigned short *temp = OutBuffer;
                            OutBuffer = RemoveStripeBuffer;
                            RemoveStripeBuffer = temp;
                            firstFrame = false;
                            ioctl(fd, VIDIOC_QBUF, &buf);
                            continue;
                        }

                        temporalFilter(RemoveStripeBuffer, OutBuffer, HoldBuffer, IMAGEWIDTH, IMAGEHEIGHT - 4);

                        orgData = HoldBuffer;
                    }
                    else
                    {

                        if (firstShutFrame)
                        {
                            memcpy(shutterBuffer, HoldBuffer, IMAGEWIDTH * (IMAGEHEIGHT - 4) * sizeof(unsigned short));
                            firstShutFrame = false;
                        }
                        if (shutterCnt == 31)
                        {
                            shutterCnt = 0;
                            nucFrameCnt = 0;
                            nucFreeze = false;
                        }

                        orgData = shutterBuffer;
                    }
                }

                if (counter != 0 && (OUTPUTMODE == 4 || OUTPUTMODE == 6))
                {
                    // printf("collectedData address4:%X\n",collectedData);
                    collectData(&counter, orgData, collectedData, IMAGEHEIGHT - 4, IMAGEWIDTH, &tempBuffer);
                    if (counter == 0)
                    {
                        for (int i = 0; i < IMAGEHEIGHT - 4; i++)
                        {
                            for (int j = 0; j < IMAGEWIDTH; j++)
                            {

                                int gray = 0;
                                printf(" %d", collectedData[i * IMAGEWIDTH + j]);
                                gray = (collectedData[i * IMAGEWIDTH + j] > -127 ? (collectedData[i * IMAGEWIDTH + j] < 127 ? collectedData[i * IMAGEWIDTH + j] : 127) : -127) + 127;
                                lumiImg.at<uchar>(i, j) = (uchar)gray;
                            }
                        }
                        // cv::imshow("lumiImg", lumiImg);
                    }
                    printf("collectData counter:%d,tempBuffer:%X\n", counter, tempBuffer);
                }

                int amountPixels = 0;

                /*--------------------------------------*/
                // 定时打快门，“回车键”
                if (delayy % 4500 == 30) // 每三分钟打一次快门，人体高精度版本每分钟打一次   Shutter calibrates once every three minutes, for body temperature measuring, once per minute
                                         // 注意这里是取余，不是除，所以得改变除数，如果一分钟打一次快门，得将4500改为1500
                {
                    if (v4l2_control(0x8000) == FALSE)
                    {
                        printf("shutter fail~~\n");
                    }

                    shutterCnt = 1;
                    nucFreeze = true;
                    firstShutFrame = true;
                }
                /*--------------------------------------*/
                // 重新计算温度表，定时计算温度表
                if (delayy % 1500 == 55 || needRecal) // 打快门前5帧重新计算table，打快门时不可计算表，数据可能有错误   Recalculate the table during the five frames before shutter calibrates, otherwise may be errors
                {
                    // 用后四行参数来计算表   Calculate table with last four lines
                    if (grab_done == FALSE && fix_Calculate_Done == FALSE)
                    {
                        printf("rangmode:%d\n", rangeMode);
                        printf("计算温度对应表\n");
                        if (OUTPUTMODE == 4)
                        {
                            thermometryT4Line(IMAGEWIDTH, IMAGEHEIGHT, temperatureTable, fourLinePara, &floatFpaTmp, &correction, &Refltmp, &ambient, &humi, &emiss, &distance, cameraLens, shutterFix, rangeMode);
                        }
                        else if (OUTPUTMODE == 6)
                        {
                            EAthermometryT4Line(IMAGEWIDTH, IMAGEHEIGHT, temperatureTable, fourLinePara, &floatFpaTmp, &correction, &Refltmp, &ambient, &humi, &emiss, &distance, cameraLens, shutterFix, rangeMode);
                        }
                    }
                    else if (fix_Calculate_Done == TRUE && OUTPUTMODE == 4)
                    {
                        printf("rangmode:%d\n", rangeMode);
                        printf("计算添加修正后的温度对应表\n");
                        thermometryT4Line_auto_fix(IMAGEWIDTH, IMAGEHEIGHT, temperatureTable, fourLinePara, &floatFpaTmp, &correction, &Refltmp, &ambient, &humi, &emiss, &distance, cameraLens, rangeMode);
                    }

                    needRecal = 0;
                    if (computeMethod == 3 || computeMethod == 4)
                    {
                        int i = 0, divTmpNuc1State = 1;
                        for (i = 0; i < 16384; i++)
                        {
                            if (divTmp1 < temperatureTable[i] && divTmpNuc1State)
                            {
                                divTmpNuc1 = i;
                                divTmpNuc1State = 0;
                                printf("divTmpNuc1:%d\n", divTmpNuc1);
                            }
                            if (divTmp2 < temperatureTable[i])
                            {
                                divTmpNuc2 = i;
                                printf("divTmpNuc2:%d\n", divTmpNuc2);
                                break;
                            }
                        }
                        SetDivTemp(divTmpNuc1, divTmpNuc2);
                    }
                    if (delayy > 9000)
                    {
                        delayy = 0;
                    }
                }
                if (autoCalibtateShutter_Flag == TRUE && OUTPUTMODE == 4)
                {
                    printf("center100:%d  center300:%d  center500:%d\n", center_nuc_100, center_nuc_300, center_nuc_500);
                    int refix_result = autoCalibrate(IMAGEWIDTH, IMAGEHEIGHT, temperatureTable, fourLinePara, &floatFpaTmp,
                                                     &correction, &Refltmp, &ambient, &humi, &emiss, &distance, cameraLens, &shutterFix, rangeMode,
                                                     center_nuc_100, center_nuc_300, center_nuc_500, 120.0f, 290.0f, 450.0f, 2.0f, &c_fix, &time, &err);
                    if (rangeMode == 120 && refix_result == 0)
                    {
                        sendC_fix_NT(c_fix);
                        sendShutter_fix_NT(shutterFix);
                        sleep(1);
                        fix_Calculate_Done = TRUE;
                    }
                    else if (rangeMode == 400 && refix_result == 0)
                    {
                        sendC_fix_HT(c_fix);
                        sendShutter_fix_HT(shutterFix);
                        sleep(1);
                        fix_Calculate_Done = TRUE;
                    }
                    else
                    {
                        printf("标定失败\n");
                    }
                    autoCalibtateShutter_Flag = FALSE;
                    if (refix_result == 0)
                    {
                        printf("c_fix:%f\n Shutter_fix:%f\n", c_fix, shutterFix);
                    }

                    printf("err:%d, result:%d\n", err, refix_result);
                }

                // 根据8004或者8005模式来查表，8005模式下仅输出11个参数(参考thermometry.h)，8004模式下数据以上参数+全局温度数据
                // Search on the table with 8004 or 8005 mode, 8005 mode only outputs the 11 parameters(refer to thermometry.h) , 8004 mode include above parameters with overall temperature data
                if (distanceFix_flag == TRUE)
                {
                    float temperature = 459.2f;
                    float envir_temp = 25.0f;
                    printf("距离补偿前的：%.2f\n", temperature);
                    float centertemp_fix = distanceFix(temperature, 2.0f, 25.0f, 130); // temperatureData[0]
                    printf("距离补偿后的：%.2f\n", centertemp_fix);

                    distanceFix_flag = FALSE;
                }
                if (temperautreOutput_Flag == TRUE)
                {
                    if (OUTPUTMODE == 4)
                    {
                        // 温度9值的获取
                        thermometrySearch(IMAGEWIDTH, IMAGEHEIGHT, temperatureTable, orgData, temperatureData, rangeMode, OUTPUTMODE);
                        printf("centerTmp:%.2f,maxTmp:%.2f,minTmp:%.2f,avgTmp:%.2f,point1Tmp:%.2f\n", temperatureData[0], temperatureData[3], temperatureData[6], temperatureData[9], temperatureData[7]);
                        struct tempdata temp;
                        temp.avgTmp = temperatureData[9];
                        temp.centerTmp = temperatureData[0];
                        temp.maxTmp = temperatureData[3];
                        temp.minTmp = temperatureData[6];
                        temp.pointTmp = temperatureData[7];
                        dataMutex.lock();
                        dataUpdateFlag = 1;
                        sendTempData = temp;
                        dataMutex.unlock();
                    }

                    temperautreOutput_Flag = FALSE;
                }
                // 指定要查询的个数及数据位置，仅可输入8004模式下的数据   Set the count of query and data address, only for data in 8004 mode
                amountPixels = IMAGEWIDTH * (IMAGEHEIGHT - 4);
                unsigned short detectAvg = orgData[amountPixels];
                // printf("cpyPara  detectAvg:%d ",detectAvg);
                amountPixels++;
                unsigned short fpaTmp = orgData[amountPixels];
                amountPixels++;
                unsigned short maxx1 = orgData[amountPixels];
                amountPixels++;
                unsigned short maxy1 = orgData[amountPixels];
                amountPixels++;
                unsigned short max = orgData[amountPixels];
                // printf("cpyPara  max:%d \n",max);
                amountPixels++;

                unsigned short minx1 = orgData[amountPixels];
                amountPixels++;
                unsigned short miny1 = orgData[amountPixels];
                amountPixels++;
                unsigned short min = orgData[amountPixels];
                // printf("cpyPara  min:%d \n",min);
                amountPixels++;
                unsigned short avg = orgData[amountPixels];

                // 以下用于8004/8082模式中计算出rgba来显示，opencv使用bgra来显示，故而通道有差异。
                // Following codes for computing rgba outputs in 8004 mode; using OpenCV to show bgra, so there are differences in channel
                /*-------------------图像处理相关-----------------*/
                // 只用专业的图像处理computeMethod = 1
                Compute(orgData, rgbImgBuffer[n_buffers].data, palette, irBuffers[n_buffers].midVar, IRRGBA); // 0-6 to change platte
                cv::cvtColor(rgbImgBuffer[n_buffers], bgrImg, cv::COLOR_RGB2BGR);
                if (saveImage_Flag == TRUE)
                {
                    cv::imwrite("save.bmp", bgrImg);
                    saveImage_Flag = FALSE;
                }
                /*-----------------------------------------------------------*/
                // 图像显示部分代码，但这里还没有创建显示窗口的代码
                cv::Point p1((IMAGEWIDTH / 2) - 10, (IMAGEHEIGHT - 4) / 2), p2((IMAGEWIDTH / 2) + 10, (IMAGEHEIGHT - 4) / 2);
                cv::Point p3(IMAGEWIDTH / 2, (IMAGEHEIGHT - 4) / 2 + 10), p4(IMAGEWIDTH / 2, (IMAGEHEIGHT - 4) / 2 - 10);
                cv::Scalar colorLine(0, 0, 255);
                int thicknessLine = 2;
                cv::line(bgrImg, p1, p2, colorLine, thicknessLine);
                cv::line(bgrImg, p3, p4, colorLine, thicknessLine);
                cv::imshow("methodTwo", bgrImg);

                // 在 driver 内部管理着两个 buffer queues ，一个输入队列，一个输出队列。
                // 对于 capture device 来说，当输入队列中的 buffer
                // 被塞满数据以后会自动变为输出队列，
                // There are two buffer queues inside driver: input queues and output queues
                // For capture device, when buffer in input queue is fulled by data, it will transfer to output queue
                // 重新调用 VIDIOC_QBUF 将 buffer 重新放进输入队列.  Calling VIDIOC_DQBUF again, place buffer into output queue
                ioctl(fd, VIDIOC_QBUF, &buf);
                if ((cv::waitKey(1) & 255) == 27)
                    exit(0);
                t = (double)cvGetTickCount() - t;
                // printf("used time is %gms\n",(t/(cvGetTickFrequency()*1000)));
            }
        }
    }

    printf("renderThread 1\n");
    for (n_buffers = 0; n_buffers < 4; n_buffers++)
    {
        // 释放专业图像算法占用的资源   Release buffers captured by professional image algorithm
        if (OUTPUTMODE == 4)
        {
            SimplePictureProcessingDeinit();
            if (irBuffers[n_buffers].midVar != NULL)
            {
                SimplePictureProcessingDeinitMidVar(irBuffers[n_buffers].midVar);
                free(irBuffers[n_buffers].midVar);
                irBuffers[n_buffers].midVar = NULL;
            }
        }
        // end -释放专业图像算法占用的资源   Release buffers

        if (temperatureData != NULL)
        {
            free(temperatureData);
            temperatureData = NULL;
        }
        if (temperatureDataFixed != NULL)
        {
            free(temperatureDataFixed);
            temperatureDataFixed = NULL;
        }
    }
    if (fd != 0)
    {
        v4l2_release(); // 停止视频采集命令，应用程序调用VIDIOC_ STREAMOFF停止视频采集命令后，视频设备驱动程序不在采集视频数据。
                        // Calling VIDIOC_ STREAMOFF to stop capturing video, video device driver stops capturing video
        fd = 0;
    }

    SAFE_DELETE(KBuffer);
    SAFE_DELETE(BBuffer);
    SAFE_DELETE(kbBuffer);
    SAFE_DELETE(RemoveStripeBuffer);
    SAFE_DELETE(OutBuffer);
    SAFE_DELETE(HoldBuffer);
    SAFE_DELETE(shutterBuffer);
    cv::destroyWindow("methodTwo");
    printf("pthread_exit\n");
    pthread_exit(0);
}

#define SERVER_PORT 9010
#define SERVER_IP "127.0.0.1"

int init_udp_server(int *sockfd)
{
    *sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (*sockfd < 0)
    {
        perror("socket error");
        exit(1);
    }

    struct sockaddr_in serveraddr;
    memset(&serveraddr, 0, sizeof(serveraddr));
    serveraddr.sin_family = AF_INET;
    serveraddr.sin_port = htons(SERVER_PORT);
    serveraddr.sin_addr.s_addr = inet_addr(SERVER_IP);
    if (bind(*sockfd, (struct sockaddr *)&serveraddr, sizeof(serveraddr)) < 0)
    {
        perror("bind error");
        exit(1);
    }
}

// void* irayReceiveThread(void* arg)
// {
//     int* sock = (int* )arg;
//     int sockfd = *sock;
//     struct sockaddr_in client;
//     struct commonFrame receiveFrame;
//     struct commonFrame sendFrame;
//     struct tempdata sendtemp;
//     socklen_t len = sizeof(client);
//     while (1)
//     {
//         if (recvfrom(sockfd, &receiveFrame, sizeof(receiveFrame), 0, (struct sockaddr *)&client, &len) > 0)
//         {
//             if (receiveFrame.frameHead.source == SERVER)
//             {
//                 if (receiveFrame.frameHead.frameType == REQUESTDATA)
//                 {
//                     keyBoardNum = 'M';
//                     sem_post(&sem);
//                     do
//                     {
//                         std::this_thread::sleep_for(std::chrono::milliseconds(100));
//                         dataMutex.lock();
//                         if (dataUpdateFlag == 1)
//                         {
//                             dataUpdateFlag = 0;
//                             sendtemp = sendTempData;
//                             dataMutex.unlock();
//                             sendFrame.frameHead.dest = SERVER;
//                             sendFrame.frameHead.source = THR;
//                             sendFrame.frameHead.frameType = DATA;
//                             sendFrame.framedata.hasData = 1;
//                             sendFrame.framedata.data3 = sendtemp.avgTmp;
//                             sendFrame.framedata.data4 = sendtemp.centerTmp;
//                             int err = sendto(sockfd, &sendFrame, sizeof(sendFrame), 0, (struct sockaddr *)&client, sizeof(client));
//                             break;
//                         }
//                         else
//                         {
//                             dataMutex.unlock();
//                         }
//                     } while (true);
//                 }
//                 else if (receiveFrame.frameHead.frameType == CONTROL)
//                 {
//                     if (receiveFrame.framedata.hasData)
//                     {
//                         switch (receiveFrame.framedata.data1)
//                         {
//                         case OPENTHR:
//                             // open UVC device，create renderThread
//                             keyBoardNum = 'A';
//                             sem_post(&sem);
//                             break;
//                         case CLOSETHR:
//                             // open UVC device，create renderThread
//                             keyBoardNum = 'S';
//                             sem_post(&sem);
//                             break;
//                         default:
//                             break;
//                         }
//                     }
//                 }
//             }
//         }
//         std::this_thread::sleep_for(std::chrono::milliseconds(100));
//     }
// }

// char GetInstruction(int sockfd)
// {
//     char instruction;
//     int flag = 1;
//     socklen_t len = sizeof(clientaddr);
//     char buffer[1024];
//     memset(buffer, 0, sizeof(buffer));
//     // 接收客户端的数据报文
//     while (flag)
//     {
//         if (recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr *)&clientaddr, &len) > 0)
//         {
//             flag = 0;
//         }
//     }
//     printf("received data = %c\n", buffer[0]);
//     return buffer[0];
// }

//--------------------------------main-----------------------------------//
/*主线程中改为接收网络套接字传来的操作指令，然后交给handlerThread线程处理*/

int main()
{
    // 创建显示窗口
    namedWindow("methodTwo", 0);

    int ret = 0;
    char instruction;
    pthread_t irayReceiveThreadId;
    init_udp_server(&sockfd);
    // 创建handlerThread线程，处理线程（根据键盘按键，进行具体处理）
    ret = pthread_create(&handlerThreadId, NULL, handlerThread, NULL);
    // ret = pthread_create(&irayReceiveThreadId, NULL, irayReceiveThread, &sockfd);
    ret = sem_init(&sem, 0, 0);

    struct sockaddr_in client;
    struct commonFrame receiveFrame;
    struct commonFrame sendFrame;
    struct tempdata sendtemp;
    socklen_t len = sizeof(client);
    while (1)
    {
        if (recvfrom(sockfd, &receiveFrame, sizeof(receiveFrame), 0, (struct sockaddr *)&client, &len) > 0)
        {
            if (receiveFrame.frameHead.source == SERVER)
            {
                if (receiveFrame.frameHead.frameType == REQUESTDATA)
                {
                    keyBoardNum = 'M';
                    sem_post(&sem);
                    do
                    {
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        dataMutex.lock();
                        if (dataUpdateFlag == 1)
                        {
                            dataUpdateFlag = 0;
                            sendtemp = sendTempData;
                            dataMutex.unlock();
                            sendFrame.frameHead.dest = SERVER;
                            sendFrame.frameHead.source = THR;
                            sendFrame.frameHead.frameType = DATA;
                            sendFrame.frameHead.subObj = 4;
                            sendFrame.framedata.hasData = 1;
                            sendFrame.framedata.data3 = sendtemp.avgTmp;
                            sendFrame.framedata.data4 = sendtemp.centerTmp;
                            int err = sendto(sockfd, &sendFrame, sizeof(sendFrame), 0, (struct sockaddr *)&client, sizeof(client));
                            break;
                        }
                        else
                        {
                            dataMutex.unlock();
                        }
                    } while (true);
                }
                else if (receiveFrame.frameHead.frameType == CONTROL)
                {
                    if (receiveFrame.framedata.hasData)
                    {
                        switch (receiveFrame.framedata.data1)
                        {
                        case OPENTHR:
                            // open UVC device，create renderThread
                            keyBoardNum = 'A';
                            sem_post(&sem);
                            break;
                        case CLOSETHR:
                            // open UVC device，create renderThread
                            keyBoardNum = 'S';
                            pthread_join(renderThreadId, NULL);
                            sem_post(&sem);
                            break;
                        default:
                            break;
                        }
                    }
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    /*
     int sem_init (sem_t *sem, int pshared, unsigned int value); 这个函数的作用是对由sem指定的信号量进行初始化，设置好它的共享选项，并指定一个整数类型的初始值。pshared参数控制着信号量的类型。如果 pshared的值是0，就表示它是当前进程的局部信号量；否则，其它进程就能够共享这个信号量。我们现在只对不让进程共享的信号量感兴趣。 （这个参数 受版本影响）， pshared传递一个非零将会使函数调用失败，属于无名信号量。
    //  */
    // while (isOn)
    // {
    //     // 加两个指令来选择是否显示传输过来的图片，添加显示状态的标志变量Display_flag
    //     // char nNub = GetInstruction(sockfd);
    //     switch (nNub)
    //     {
        // handlerThread线程处理后由handlerThread线程将结果通过网络套接字返回
        // case 10:  // enter:shutter 10为回车键的ascii码，这里代表打快门
        // case 52:  // 4:save para
        // case 57:  // 9 nextTrans
        // case 81:  // q white hot
        // case 87:  // w black hot
        // case 69:  // e Iron Rainbow
        // case 82:  // r Rainbow 1
        // case 84:  // t Rainbow 2
        // case 89:  // y HDR rainbow
        // case 85:  // u High Contrast rainbow
        // case 73:  // i lava Rainbow
        // case 79:  // o th Iron Rainbow
        // case 'A': // a open uvc device，create renderThread
        // case 83:  // s close uvc device
        // case 70:  // f change rangemode 400
        // case 71:  // g change rangemode 120
        // case 72:  // h send temperatue parameter
        // case 74:  // j send TMP006 parameter
        // case 75:  // k auto calculate kc_fix and shutter_fix
        // case 76:  // l collect center nuc
        // case 'M': // m output center temperature
        // case 78:  // n caculate distanceFix
        // case 66:  // b save the image
        // case 90:  // z 8005 change pallets
        // case 86:  // v 8081 receive the K
    //         keyBoardNum = nNub;
    //         sem_post(&sem);
    //         printf("nub=%c\n", nNub);
    //         if (nNub != 'M')
    //         {
    //             data_send_func(nNub, 1, NULL);
    //         }
    //         /*
    //          函数sem_post( sem_t *sem )类似V操作，用来增加信号量的值。当有线程阻塞在这个信号量上时，调用这个函数会使其中的一个线程不在阻塞，选择机制同样是由线程的调度策略决定的。int sem_post(sem_t *sem);sem_post() 成功时返回 0；错误时，信号量的值没有更改，-1 被返回，并设置errno 来指明错误。错误   EINVAL sem 不是一个有效的信号量。 EOVERFLOW 信号量允许的最大值将要被超过。
    //          */
    //         break;
    //     }
    // }
    pthread_join(handlerThreadId, NULL);
    // pthread_join(irayReceiveThreadId, NULL);
    /*
     主线程等待子线程的终止。也就是在子线程调用了pthread_join()方法后面的代码，只有等到子线程结束了才能执行。
     */
    sem_destroy(&sem);
    close(sockfd);
    return 0;
}

int v4l2_release()
{
    unsigned int n_buffers;
    enum v4l2_buf_type type;

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(fd, VIDIOC_STREAMOFF, &type);

    // 关闭内存映射   Memory map OFF
    for (n_buffers = 0; n_buffers < 4; n_buffers++)
    {
        munmap(buffers[n_buffers].start, buffers[n_buffers].length);
    }

    // 释放自己申请的内存   Free buffers
    free(buffers);

    // 关闭设备   Device OFF
    close(fd);
    return TRUE;
}

int init_v4l2(string videoX)
{
    const char *videoXConst = videoX.c_str();
    if ((fd = open(videoXConst, O_RDWR)) == -1) // 打开video1  Open Video1
    {
        printf("Opening video device error\n");
        return FALSE;
    }
    printf("init_v4l2 fd :%d\n", fd);
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == -1) // 查询视频设备的功能   Query capabilities of video device
    {
        printf("unable Querying Capabilities\n");
        return FALSE;
    }
    else
    {
        printf("Driver Caps:\n"
               "  Driver: \"%s\"\n"
               "  Card: \"%s\"\n"
               "  Bus: \"%s\"\n"
               "  Version: %d\n"
               "  Capabilities: %x\n",
               cap.driver,
               cap.card,
               cap.bus_info,
               cap.version,
               cap.capabilities);
        string str = "";
        str = (char *)cap.card;
        /*if(!str.find("T3S")){
            close(fd);
            return FALSE;
        }*/
    }
    /* if((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == V4L2_CAP_VIDEO_CAPTURE){//是否支持V4L2_CAP_VIDEO_CAPTURE
         printf("Camera device %s: support capture\n",FILE_VIDEO1);
     }
     if((cap.capabilities & V4L2_CAP_STREAMING) == V4L2_CAP_STREAMING){//是否支持V4L2_CAP_STREAMING
         printf("Camera device %s: support streaming.\n",FILE_VIDEO1);
     }
    */
    fmtdesc.index = 0;
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    printf("Support format: \n");
    while (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) != -1) // 获取当前视频设备支持的视频格式   Capture current video format of the device
    {
        printf("\t%d. %s\n", fmtdesc.index + 1, fmtdesc.description);
        fmtdesc.index++;
    }
    // set fmt
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = IMAGEWIDTH;
    fmt.fmt.pix.height = IMAGEHEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV; // 使用V4L2_PIX_FMT_YUYV  Calling V4L2_PIX_FMT_YUYV
    // fmt.fmt.pix.field = V4L2_FIELD_NONE;
    fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) == -1) // 设置视频设备的视频数据格式，例如设置视频图像数据的长、宽，图像格式（JPEG、YUYV格式）
    {                                        // Set video data format, such as Width, Height of image, format (JPEG, YUYV etc)
        printf("Setting Pixel Format error\n");
        return FALSE;
    }
    if (ioctl(fd, VIDIOC_G_FMT, &fmt) == -1) // 获取图像格式   Get video format
    {
        printf("Unable to get format\n");
        return FALSE;
    }
    IMAGEWIDTH = fmt.fmt.pix.width;   // 更正宽  fix Width
    IMAGEHEIGHT = fmt.fmt.pix.height; // 更正高   fix Height
    printf("IMAGEWIDTH:%d,IMAGEHEIGHT:%d\n", IMAGEWIDTH, IMAGEHEIGHT);
    memset(&stream_para, 0, sizeof(struct v4l2_streamparm));
    stream_para.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    stream_para.parm.capture.timeperframe.denominator = 25;
    stream_para.parm.capture.timeperframe.numerator = 1;

    if (ioctl(fd, VIDIOC_S_PARM, &stream_para) == -1)
    {
        printf("Unable to set frame rate\n");
        return FALSE;
    }
    if (ioctl(fd, VIDIOC_G_PARM, &stream_para) == -1)
    {
        printf("Unable to get frame rate\n");
        return FALSE;
    }
    {
        printf("numerator:%d\ndenominator:%d\n", stream_para.parm.capture.timeperframe.numerator, stream_para.parm.capture.timeperframe.denominator);
    }
    //        else

    /*        {
                printf("fmt.type:\t%d\n",fmt.type);         //可以输出图像的格式   Output image format
                printf("pix.pixelformat:\t%c%c%c%c\n",fmt.fmt.pix.pixelformat & 0xFF,(fmt.fmt.pix.pixelformat >> 8) & 0xFF,\
                       (fmt.fmt.pix.pixelformat >> 16) & 0xFF, (fmt.fmt.pix.pixelformat >> 24) & 0xFF);
                printf("pix.height:\t%d\n",fmt.fmt.pix.height);
                printf("pix.field:\t%d\n",fmt.fmt.pix.field);
            }
    */
    return TRUE;
}

int v4l2_grab(void)
{
    // struct v4l2_requestbuffers req = {0};
    // 4  request for 4buffers 缓存不可少于两个   count of buffer >= 2
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_REQBUFS, &req) == -1) // 开启内存映射或用户指针I/O    Start memory map or pointer I/O
    {
        printf("Requesting Buffer error\n");
        return FALSE;
    }
    // 5 mmap for buffers
    buffers = (buffer *)malloc(req.count * sizeof(*buffers));
    if (!buffers)
    {
        printf("Out of memory\n");
        return FALSE;
    }
    unsigned int n_buffers;
    for (n_buffers = 0; n_buffers < req.count; n_buffers++)
    {
        // struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = n_buffers;
        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) == -1) // 查询已经分配的V4L2的视频缓冲区的相关信息，包括视频缓冲区的使用状态、
        {
            // 在内核空间的偏移地址、缓冲区长度等。 Search info of allocated V4L2 video buffer, including status, offset address,buffer length etc
            printf("Querying Buffer error\n");
            return FALSE;
        }
        buffers[n_buffers].length = buf.length;

        buffers[n_buffers].start = (unsigned char *)mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);

        if (buffers[n_buffers].start == MAP_FAILED)
        {
            printf("buffer map error\n");
            return FALSE;
        }
    }
    // 6 queue
    for (n_buffers = 0; n_buffers < req.count; n_buffers++)
    {
        buf.index = n_buffers;
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (ioctl(fd, VIDIOC_QBUF, &buf)) // 投放一个空的视频缓冲区到视频缓冲区输入队列中  Put in empty video buffer to input queue of video buffer
        {
            printf("query buffer error\n");
            return FALSE;
        }
    }
    // 7 starting
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) == -1) //
    {
        printf("stream on error\n");
        return FALSE;
    }
    return TRUE;
}

int v4l2_control(int value)
{
    ctrl.id = V4L2_CID_ZOOM_ABSOLUTE;
    ctrl.value = value; // change output mode 0x8004/0x8005
    int result = 0;
    // shutter 0x8000
    // VIDIOC_S_CTRL为命令码
    if (ioctl(fd, VIDIOC_S_CTRL, &ctrl) == -1)
    {
        /*
         如果ioctl执行成功，它的返回值就是驱动程序中ioctl接口给的返回值，驱动程序可以通过返回值向用户程序传参。但驱动程序最好返回一个非负数，因为用户程序中的ioctl运行失败时一定会返回-1并设置全局变量errorno。
         */
        printf("v4l2_control error:%d\n", result);
        return FALSE;
    }
    return TRUE;
}

int traversalVideo(void)
{
    string device = "/dev";
    string video = "video";
    char *KEY_PTR = (char *)video.c_str();
    char *FILE_PTR = (char *)device.c_str();

    DIR *dir;
    struct dirent *ptr;
    char base[1000];

    if ((dir = opendir(FILE_PTR)) == NULL)
    {
        perror("Open dir error...");
        exit(1);
    }

    while ((ptr = readdir(dir)) != NULL)
    {
        string name = "";
        name = (char *)ptr->d_name;
        if (name.find("video") != string::npos)
        {
            string allName = device + "/" + name;
            if (init_v4l2(allName))
            {
                closedir(dir);
                return 1;
            }
        }
        printf("d_name:%s/%s\n", FILE_PTR, ptr->d_name);
    }

    closedir(dir);
    return 0;
}

void sendFloatCommand(int position, unsigned char value0, unsigned char value1, unsigned char value2, unsigned char value3, int interval0,
                      int interval1, int interval2, int interval3, int interval4)
{
    int psitionAndValue0 = (position << 8) | (0x000000ff & value0);
    printf("psitionAndValue0:%X\n", psitionAndValue0);
    // v4l2_control(psitionAndValue0);
    if (v4l2_control(psitionAndValue0) == FALSE)
    {
        printf("control fail psitionAndValue0~~\n");
        // exit(EXIT_FAILURE);
    }
    usleep(50000);
    int psitionAndValue1 = ((position + 1) << 8) | (0x000000ff & value1);
    printf("psitionAndValue1:%X\n", psitionAndValue1);
    if (v4l2_control(psitionAndValue1) == FALSE)
    {
        printf("control fail psitionAndValue1~~\n");
        // exit(EXIT_FAILURE);
    }
    usleep(50000);
    int psitionAndValue2 = ((position + 2) << 8) | (0x000000ff & value2);
    printf("psitionAndValue2:%X\n", psitionAndValue2);
    if (v4l2_control(psitionAndValue2) == FALSE)
    {
        printf("control fail psitionAndValue2~~\n");
        // exit(EXIT_FAILURE);
    }
    usleep(50000);
    int psitionAndValue3 = ((position + 3) << 8) | (0x000000ff & value3);
    printf("psitionAndValue3:%X\n", psitionAndValue3);
    if (v4l2_control(psitionAndValue3) == FALSE)
    {
        printf("control fail psitionAndValue3~~\n");
        exit(EXIT_FAILURE);
    }
    usleep(50000);
}
void sendUshortCommand(int position, unsigned char value0, unsigned char value1)
{
    int psitionAndValue0 = (position << 8) | (0x000000ff & value0);
    printf("psitionAndValue0:%X\n", psitionAndValue0);
    // v4l2_control(psitionAndValue0);
    if (v4l2_control(psitionAndValue0) == FALSE)
    {
        printf("control fail psitionAndValue0~~\n");
        // exit(EXIT_FAILURE);
    }
    int psitionAndValue1 = ((position + 1) << 8) | (0x000000ff & value1);
    printf("psitionAndValue1:%X\n", psitionAndValue1);
    usleep(50000);
    if (v4l2_control(psitionAndValue1) == FALSE)
    {
        printf("control fail psitionAndValue1~~\n");
        // exit(EXIT_FAILURE);
    }
    usleep(50000);
}
void sendByteCommand(int position, unsigned char value0, int interval0)
{
    int psitionAndValue0 = (position << 8) | (0x000000ff & value0);
    v4l2_control(psitionAndValue0);
    usleep(50000);
}

void sendCorrection(float correction)
{
    unsigned char iputCo[4];
    memcpy(iputCo, &correction, sizeof(float));
    sendFloatCommand(0 * 4, iputCo[0], iputCo[1], iputCo[2], iputCo[3], 20, 40, 60, 80, 120);
}
void sendReflection(float reflection)
{
    unsigned char iputRe[4];
    memcpy(iputRe, &reflection, sizeof(float));
    sendFloatCommand(1 * 4, iputRe[0], iputRe[1], iputRe[2], iputRe[3], 20, 40, 60, 80, 120);
}
void sendAmb(float amb)
{
    unsigned char iputAm[4];
    memcpy(iputAm, &amb, sizeof(float));
    sendFloatCommand(2 * 4, iputAm[0], iputAm[1], iputAm[2], iputAm[3], 20, 40, 60, 80, 120);
}
void sendHumidity(float humidity)
{
    unsigned char iputHu[4];
    memcpy(iputHu, &humidity, sizeof(float));
    sendFloatCommand(3 * 4, iputHu[0], iputHu[1], iputHu[2], iputHu[3], 20, 40, 60, 80, 120);
}
void sendEmissivity(float emiss)
{
    unsigned char iputEm[4];
    memcpy(iputEm, &emiss, sizeof(float));
    sendFloatCommand(4 * 4, iputEm[0], iputEm[1], iputEm[2], iputEm[3], 20, 40, 60, 80, 120);
}

void sendDistance(unsigned short distance)
{
    unsigned char iputDi[2];
    memcpy(iputDi, &distance, sizeof(unsigned short));
    sendUshortCommand(5 * 4, iputDi[0], iputDi[1]);
}

void sendC_fix_NT(float c_fix)
{
    unsigned char iputC_fix[4];
    memcpy(iputC_fix, &c_fix, sizeof(float));
    sendFloatCommand(26, iputC_fix[0], iputC_fix[1], iputC_fix[2], iputC_fix[3], 20, 40, 60, 80, 120);
}

void sendShutter_fix_NT(float Shutter_fix)
{
    unsigned char iputShutter_fix[4];
    memcpy(iputShutter_fix, &Shutter_fix, sizeof(float));
    sendFloatCommand(30, iputShutter_fix[0], iputShutter_fix[1], iputShutter_fix[2], iputShutter_fix[3], 20, 40, 60, 80, 120);
}

void sendC_fix_HT(float c_fix)
{
    unsigned char iputC_fix[4];
    memcpy(iputC_fix, &c_fix, sizeof(float));
    sendFloatCommand(34, iputC_fix[0], iputC_fix[1], iputC_fix[2], iputC_fix[3], 20, 40, 60, 80, 120);
}

void sendShutter_fix_HT(float Shutter_fix)
{
    unsigned char iputShutter_fix[4];
    memcpy(iputShutter_fix, &Shutter_fix, sizeof(float));
    sendFloatCommand(38, iputShutter_fix[0], iputShutter_fix[1], iputShutter_fix[2], iputShutter_fix[3], 20, 40, 60, 80, 120);
}

void getCenterNuc(unsigned short *centernuc)
{
    unsigned short *orgData = (unsigned short *)buffers[n_buffers].start;
    unsigned short *fourLinePara = orgData + IMAGEWIDTH * (IMAGEHEIGHT - 4); // 后四行参数  Parameters in last four lines
    int amountPixels = IMAGEWIDTH * (IMAGEHEIGHT - 4);
    amountPixels += 12;
    *centernuc = orgData[amountPixels];
    // printf("centernuc:%d\n",*centernuc);
}

void savePara()
{
    v4l2_control(0x80ff);
}
void setPoint(int viewX1, int viewY1, int indexOfPoint)
{
    int x1 = 0, x2 = 0, y1 = 0, y2 = 0;
    if (IMAGEWIDTH == 640)
    {
        switch (indexOfPoint)
        {
        case 0:
            x1 = 0xf300 + (viewX1 & 0x00ff); // low 8bit
            x2 = 0xf400 + (viewX1 >> 8);     // high 8bit
            y1 = 0xf500 + (viewY1 & 0x00ff);
            y2 = 0xf600 + (viewY1 >> 8);
            break;
        case 1:
            x1 = 0xf700 + (viewX1 & 0x00ff); // low 8bit
            x2 = 0xf800 + (viewX1 >> 8);     // high 8bit
            y1 = 0xf900 + (viewY1 & 0xff);
            y2 = 0xfa00 + (viewY1 >> 8);
            break;
        case 2:
            x1 = 0xfb00 + (viewX1 & 0x00ff); // low 8bit
            x2 = 0xfc00 + (viewX1 >> 8);     // high 8bit
            y1 = 0xfd00 + (viewY1 & 0x00ff);
            y2 = 0xfe00 + (viewY1 >> 8);
            break;
        }
        printf("x:0x%x,x1:0x%x,x2:0x%x,y:0x%x,y1:0x%x,y2:0x%x\n", viewX1, x1, x2, viewY1, y1, y2);
        v4l2_control(x1);
        usleep(50000);
        v4l2_control(x2);
        usleep(50000);
        v4l2_control(y1);
        usleep(50000);
        v4l2_control(y2);
    }
    else if (IMAGEWIDTH == 384)
    {
        switch (indexOfPoint)
        {
        case 0:
            x1 = 0xf300 + viewX1;
            y1 = 0xf500 + viewY1;
            break;
        case 1:
            x1 = 0xf700 + viewX1;
            y1 = 0xf900 + viewY1;
            break;
        case 2:
            x1 = 0xfb00 + viewX1;
            y1 = 0xfd00 + viewY1;
            break;
        }
        v4l2_control(x1);
        usleep(50000);
        v4l2_control(y1);
    }
    else
    {
        switch (indexOfPoint)
        {
        case 0:
            x1 = 0xf000 + viewX1;
            y1 = 0xf200 + viewY1;
            break;
        case 1:
            x1 = 0xf400 + viewX1;
            y1 = 0xf600 + viewY1;
            break;
        case 2:
            x1 = 0xf800 + viewX1;
            y1 = 0xfa00 + viewY1;
            break;
        }
        v4l2_control(x1);
        usleep(50000);
        v4l2_control(y1);
    }
}

// 设置低温区参数
void setLTConfig(unsigned short temp)
{
    v4l2_control(0x8600);
    usleep(1);
    v4l2_control(0x8700);
    usleep(1);
    v4l2_control(temp);
    usleep(1);
    v4l2_control(0x80fe);
    sleep(1);
}

// 设置常温区参数
void setNTConfig(unsigned short temp)
{
    v4l2_control(0x8600);
    sleep(1);
    v4l2_control(0x8701);
    sleep(1);
    v4l2_control(temp);
    sleep(1);
    v4l2_control(0x80fe);
}

// 设置高温区参数
void setHTConfig(unsigned short temp)
{
    v4l2_control(0x8600);
    usleep(1);
    v4l2_control(0x8702);
    usleep(1);
    v4l2_control(temp);
    usleep(1);
    v4l2_control(0x80fe);
    sleep(1);
}

// 获得3172探测器的NTCC
void getNTCC(int width, unsigned short *fourLinePara, int position, int rangMode, unsigned short *NTCC_L, unsigned short *NTCC_H)
{
    int amountPixels = width * (4 - 1);
    amountPixels = amountPixels + 255 + position;
    unsigned short *orgData = fourLinePara;
    unsigned char NTCC_L1 = 0;
    unsigned short temp = 0;
    unsigned char NTCC_H1 = 0;
    memcpy(&temp, orgData + amountPixels, sizeof(unsigned short));
    // printf("temp:%x\n",temp);
    NTCC_L1 = temp >> 8;
    amountPixels = amountPixels + 1;
    memcpy(&NTCC_H1, orgData + amountPixels, sizeof(unsigned char));
    // printf("NTCC_L1:%d  NTCC_H1:%d\n",NTCC_L1,NTCC_H1);
    *NTCC_L = (0x83 << 8) | (NTCC_L1);
    *NTCC_H = (0x84 << 8) | (NTCC_H1);
    // printf("NTCC_L:%x  NTCC_H:%x\n",*NTCC_L,*NTCC_H);
}

// 获取6122/3122探测器的OOC
void getOOC(int width, unsigned short *fourLinePara, int position, int rangMode, unsigned short *OOC)
{
    int amountPixels = width * (4 - 1);
    amountPixels = amountPixels + 255 + position;

    unsigned char OOC_temp = 0;
    memcpy(&OOC_temp, fourLinePara + amountPixels, sizeof(unsigned char));
    *OOC = (0x83 << 8) | OOC_temp;
    // printf("OOC:%x\n",*OOC);
}

void getAvg(unsigned short *fourLinePara, unsigned short *detectAvg)
{
    *detectAvg = fourLinePara[0];
}

// 设置三个温区的参数
void setConfig(int width, int height, unsigned short *fourLinePara, int rangMode)
{
    int amountPixels = 0;
    unsigned short *orgData = fourLinePara;
    unsigned short L = 0;
    unsigned short H = 0;
    unsigned char SFB_OOC = 0;
    unsigned short OOC = 0;

    if (width == 384)
    {
        if (rangeMode == 120)
        {
            // 获取常温区NTCC
            getNTCC(width, fourLinePara, 89, rangeMode, &L, &H);
            // printf("L:%x\n",L);
            setLTConfig(L);
            setLTConfig(H);

            setHTConfig(L);
            setHTConfig(H);

            printf("以下是低温区：\n");
            getNTCC(width, fourLinePara, 85, rangeMode, &L, &H);

            printf("以下是高温区：\n");
            getNTCC(width, fourLinePara, 93, rangeMode, &L, &H);
        }
        else if (rangeMode == 400)
        {
            // 获取常温区NTCC
            getNTCC(width, fourLinePara, 91, rangeMode, &L, &H);

            setLTConfig(L);
            setLTConfig(H);

            setHTConfig(L);
            setHTConfig(H);

            printf("以下是低温区：\n");
            getNTCC(width, fourLinePara, 87, rangeMode, &L, &H);

            printf("以下是高温区：\n");
            getNTCC(width, fourLinePara, 95, rangeMode, &L, &H);
        }
    }
    // v4l2_control();
    else if (width == 640)
    {
        if (rangeMode == 120)
        {
            // 获取常温区OOC
            getOOC(width, fourLinePara, 86, rangMode, &OOC);

            setLTConfig(OOC);

            setHTConfig(OOC);

            printf("以下是低温区：\n");
            getOOC(width, fourLinePara, 84, rangMode, &OOC);

            printf("以下是高温区：\n");
            getOOC(width, fourLinePara, 88, rangMode, &OOC);
        }
        else if (rangeMode == 400)
        {
            unsigned short temp;
            amountPixels = width * (4 - 1);
            amountPixels = amountPixels + 255 + 123;
            memcpy(&temp, orgData + amountPixels, sizeof(unsigned short));
            SFB_OOC = temp >> 8;
            printf("SFB_OOC:%d\n", SFB_OOC);
        }
    }
}
