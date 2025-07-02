
#ifndef THERMOMETRY_H_
#define THERMOMETRY_H_
#define DEBUG 0
#include <stdbool.h>
#ifdef __cplusplus
extern "C"  //C++
{
#endif
/**
* 计算温度对应表，用camera数据首地址  Calculate thermometry, using camera data starting address 
* @para width:宽width
* @para height:高height
* @para temperatureTable:输出，调用完此函数后，温度对应表会被填充 Output, after calling this function, temperature table will be filled
* @para orgData :输入，camera数据首地址   Input, camera data starting address
* @para floatFpaTmp: 输出，fpa温度即机芯温度  Output, fpa/chipset temperature
* @para correction :输出，设定的温度整体修正值   Output, setting correction of temperature
* @para Refltmp :输出，设定的反射温度，一般情况下为环境温度   Output, setting reflection temperature, usually ambient temperature
* @para Airtmp :输出，设定的环境温度值    Output, setting Air temperature
* @para humi :输出，设定的湿度  Output, setting humidity
* @para emiss :输出，设定的发射率   Output, setting emissivity
* @para distance :输出，设定的距离    Output, settting distance
* @para cameraLens :输入，镜头大小:目前支持两种，68：使用6.8mm镜头，130：使用13mm镜头,默认130。 
* Input, set camera lens, 68=6.8mm lens, 130=130mm lens, default 130. 
* @para shutterFix :输入，快门校正，一般为0.    Input, shutter fixing, usually 0.
* @para rangeMode :输入，测温范围：120：温度范围为-20-120摄氏度。400：温度范围为-20-400摄氏度,另外人体测温不同产品有不同定义
* Input, measuring range: 120: ranges = -20 ~ 120 celsius; 400: ranges = -20 ~ 400 celsius, differences in body temperature measuring. 
*/
void thermometryT(int width,
                  int height,
                  float *temperatureTable,
                  unsigned short *orgData,
                  float* floatFpaTmp,
                  float* correction,
                  float* Refltmp,
                  float* Airtmp,
                  float* humi,
                  float* emiss,
                  unsigned short* distance,
                  int cameraLens,
                  float shutterFix,
                  int rangeMode);

  /**
  * 计算温度对应表，用camera数据后四行参数
  * @para width:宽
  * @para height:高
  * @para temperatureTable:输出，调用完此函数后，温度对应表会被填充
  * @para orgData :输入，camera数据首地址
  * @para floatFpaTmp: 输出，fpa温度
  * @para correction :输出，温度整体修正值
  * @para Refltmp :输出，发射温度
  * @para Airtmp :输出，环境温度
  * @para humi :输出，湿度
  * @para emiss :输出，发射率
  * @para distance :输出，距离
  * @para cameraLens :设置，镜头大小:目前支持两种，68：使用6.8mm镜头，130：使用13mm镜头,默认130。177：定焦6m状态下使用90w镜头 90+w:87=177 177：定焦6m状态下使用90w镜头+锗玻璃90+w:87+锗玻璃:1=178
  * @para shutterFix :设置，快门校正，一般为0.
  * @para rangeMode :设置，测温范围：120：温度范围为-20-120摄氏度。400：温度范围为-20-400摄氏度,另外人体测温产品T3H也使用这种模式
  */

void thermometryT4Line(int width,
                       int height,
                       float *temperatureTable,
                       unsigned short *fourLinePara,
                       float* floatFpaTmp,
                       float* correction,
                       float* Refltmp,
                       float* Airtmp,
                       float* humi,
                       float* emiss,
                       unsigned short* distance,
                       int cameraLens,
                       float shutterFix,
                       int rangeMode);


void EAthermometryT4Line( int width,
                          int height,
                          float *temperatureTable,
                          unsigned short *fourLinePara,
                          float* floatFpaTmp,
                          float* correction,
                          float* Refltmp,
                          float* Airtmp,
                          float* humi,
                          float* emiss,
                          unsigned short* distance,
                          int cameraLens,
                          float shutterFix,
                          int rangeMode);



 /*
 * 计算温度对应表
 * 测温参数直接传值进入
 */
void thermomtryT4Line_non_point(int width,
    int height,
    float* temperatureTable,
    unsigned short* fourLinePara,
    float* floatFpaTmp,
    float correction,
    float Refltmp,
    float Airtmp,
    float humi,
    float emiss,
    float distance,
    int cameraLens,
    float shutterFix,
    int rangeMode);
 /*
 * 添加kc修正和快门矫正后的温度对应表计算 * 
 */
void thermometryT4Line_auto_fix(
                       int width,
                       int height,
                       float *temperatureTable,
                       unsigned short *fourLinePara,
                       float* floatFpaTmp,
                       float* correction,//
                       float* Refltmp,//
                       float* Airtmp,//
                       float* humi,//
                       float* emiss,//
                       unsigned short* distance,
                       int cameraLens,
                       int rangeMode);

/**thermometrySearch
  * 使用温度对应表来查询温度 Search temperature on the table
  * @para width:宽 Width
  * @para height:高 Height
  * @para temperatureTable:输入温度对应表，用于查询对应温度 Input temperature table for searching temperature
  * @para orgData :输入camera数据，用于查询对应温度 Input camera data for search temperature
  * @para temperatureData: 输出，根据8004或者8005模式来查表，8005模式下仅输出以下10个参数，8004模式下数据以下参数+全局温度数据
  * Output, search table in 8004 or 8005 mode, only 10 parameters in 8005; following 10 parameters + global temperature in 8004
  *          temperatureData[0]=centerTmp;
  *          temperatureData[1]=(float)maxx1;
  *          temperatureData[2]=(float)maxy1;
  *          temperatureData[3]=maxTmp;
  *          temperatureData[4]=(float)minx1;
  *          temperatureData[5]=(float)miny1;
  *          temperatureData[6]=minTmp;
  *          temperatureData[7]=point1Tmp;
  *          temperatureData[8]=point2Tmp;
  *          temperatureData[9]=avgTmp;
  * @para rangeMode :输入，测温范围：120：温度范围为-20-120摄氏度。400：温度范围为-20-400摄氏度,另外人体测温不同产品有不同定义
  * Input, measuring range: 120: ranges = -20 ~ 120 celsius; 400: ranges = -20 ~ 400 celsius, differences in body temperature measuring.
  * @para outputMode：4：8004模式。5：8005模式
  */


void thermometrySearch( int width,
                        int height,
                        float *temperatureTable,
                        unsigned short *orgData,
                        float* temperatureData,
                        int rangeMode,
                        int outputMode);

/**EAthermometrySearch
  * 使用温度对应表来查询温度 Search temperature on the table
  * @para width:宽 Width
  * @para height:高 Height
  * @para temperatureTable:输入温度对应表，用于查询对应温度 Input temperature table for searching temperature
  * @para orgData :输入camera数据，用于查询对应温度 Input camera data for search temperature
  * @para temperatureData: 输出，根据8004或者8005模式来查表，8005模式下仅输出以下10个参数，8004模式下数据以下参数+全局温度数据
  * Output, search table in 8004 or 8005 mode, only 10 parameters in 8005; following 10 parameters + global temperature in 8004
  *          temperatureData[0]=centerTmp;
  *          temperatureData[1]=(float)maxx1;
  *          temperatureData[2]=(float)maxy1;
  *          temperatureData[3]=maxTmp;
  *          temperatureData[4]=(float)minx1;
  *          temperatureData[5]=(float)miny1;
  *          temperatureData[6]=minTmp;
  *          temperatureData[7]=0;
  *          temperatureData[8]=0;
  *          temperatureData[9]=0;
  * @para rangeMode :输入，测温范围：120：温度范围为-20-120摄氏度。400：温度范围为-20-400摄氏度,另外人体测温不同产品有不同定义
  * Input, measuring range: 120: ranges = -20 ~ 120 celsius; 400: ranges = -20 ~ 400 celsius, differences in body temperature measuring.
  * @para outputMode：6：输出全帧温度；5：仅输出中心点、最高点和最低点温度。
  */
void EAthermometrySearch( int width,
                          int height,
                          float *temperatureTable,
                          unsigned short *orgData,
                          float* temperatureData,
                          float correction,
                          int rangeMode,
                          int outputMode);


/**
 * 使用温度对应表来查询个别温度 Search single temperature on the table 
 * @para width:宽 Width
 * @para height:高  Height
 * @para temperatureTable:输入，温度对应表，用于查询对应温度  Input, temperature table for searching temperature
 * @para fourLinePara :输入，后四行参数  Input, last four line parameters
 * @para count :输入，要查询的个数  Input, count of temperature to search
 * @para queryData :输入，要查询的8004模式下的数据，连续的，符合查询个数 Input, search data in 8004 mode, continuous
 * @para temperatureData: 输出，要查询的数据的温度 Output, temperature data
 * @para rangeMode :输入，测温范围：120：温度范围为-20-120摄氏度。400：温度范围为-20-400摄氏度,另外人体测温不同产品有不同定义
 * input, measuring range: 120: ranges = -20 ~ 120 celsius; 400: ranges = -20 ~ 400 celsius, differences in body temperature measuring.
 */
void thermometrySearchSingle(      int width,
                                   int height,
                                   float* temperatureTable,
                                   unsigned short* fourLinePara,
                                   int count,
                                   unsigned short* queryData,
                                   float* temperatureData,
                                   int rangeMode);
/**
  * 使用温度对应表来查询温度
  * @para width:宽 Width
  * @para height:高  Height 
  * @para temperatureTable:输入温度对应表，用于查询对应温度  Input temperature table for searching temperature
  * @para fourLinePara :输入，后四行参数   Input, last four line parameters
  * @para temperatureData: 输出，仅输出以下10个参数  Output following 10 parameters
  *          temperatureData[0]=centerTmp;
  *          temperatureData[1]=(float)maxx1;
  *          temperatureData[2]=(float)maxy1;
  *          temperatureData[3]=maxTmp;
  *          temperatureData[4]=(float)minx1;
  *          temperatureData[5]=(float)miny1;
  *          temperatureData[6]=minTmp;
  *          temperatureData[7]=point1Tmp;
  *          temperatureData[8]=point2Tmp;
  *          temperatureData[9]=point3Tmp;
  *          temperatureData[10]=avgTmp;
  * @para rangeMode :输入，测温范围：120：温度范围为-20-120摄氏度。400：温度范围为-20-400摄氏度,另外人体测温不同产品有不同定义
  *  Input, measuring range: 120: ranges = -20 ~ 120 celsius; 400: ranges = -20 ~ 400 celsius, differences in body temperature measuring.
  */

void thermometrySearchCMM(         int width,
                                   int height,
                                   float* temperatureTable,
                                   unsigned short* fourLinePara,
                                   float* temperatureData,
                                   int rangeMode);
/**
  * 用于同一场景不同距离的温度修正：比如整体设置距离3m，但是某点位距离5m，那么把该点的温度提取出来，输入为inputTemp，距离5输入为distance
  * For fixing temperatures of different distances in same scene: such as the total distance is 3m and one spot is 5m;
  * Input this spot as inputTemp, set distance as 5
  * @para inputTemp:输入温度 
  * @para distance:距离 
  * @para Airtmp:输入，环境温度
  * @para cameraLens :设置，镜头大小:目前支持两种，68：使用6.8mm镜头，130：使用13mm镜头,默认130。
  * Input, set camera lens, 68=6.8mm lens, 130=130mm lens, default 130
  */
float distanceFix(float inputTemp,float distance,float Airtmp,int cameraLens);

/**
 * 用温度参数来修正温度值
 * 
 * @param temp_in_t  指向待修正温度值的指针，温度值单位为摄氏度
 * @param in_count   待修正温度值个数
 * @param temp_out_t 指向已完成修正温度值的指针，温度值单位为摄氏度
 * @param temp_atm   环境温度，温度值单位为摄氏度
 * @param temp_ref   反射温度，温度值单位为摄氏度，一般等于环境温度
 * @param emiss      发射率
 * @param trans      透过率
 *说明：当emiss=1时，设置trans，则是透过率修正；
		   当trans=1时，设置emiss，则是发射率修正；
		   当设置emiss、trans时，则是叠加发射率和透过率修正。
 */
void thermFix (float* temp_in_t, int in_count, float* temp_out_t,
              float temp_atm, float temp_ref, float emiss,
              float trans);


/*
二次标定
  * @para width:宽
  * @para height:高
  * @para temperatureTable:输出，调用完此函数后，温度对应表会被填充
  * @para orgData :输入，camera数据首地址
  * @para floatFpaTmp: 输出，fpa温度
  * @para correction :输出，温度整体修正值
  * @para Refltmp :输出，发射温度
  * @para Airtmp :输出，环境温度
  * @para humi :输出，湿度
  * @para emiss :输出，发射率
  * @para distance :输出，距离
  * @para cameraLens :设置，镜头大小:目前支持两种，68：使用6.8mm镜头，130：使用13mm镜头,默认130。177：定焦6m状态下使用90w镜头 90+w:87=177 177：定焦6m状态下使用90w镜头+锗玻璃90+w:87+锗玻璃:1=178
  * @para shutter_fix :输出，快门修正
  * @para rangeMode :设置，测温范围：120：温度范围为-20-120摄氏度。400：温度范围为-20-400摄氏度,另外人体测温产品T3H也使用这种模式
  * @para center_nuc_20 设置
  * @para center_nuc_30 设置
  * @para center_nuc_100 设置
  * @para target_temperature_20 设置
  * @para target_temperature_30 设置
  * @para target_temperature_100 设置  以上六个参数是一一对应的。例如center_nuc_20为采集探测器照射20℃均匀面时的NUC值，target_temperature_20即为非均匀性校正后的预期温度（这里即为20℃）。
  *                                   这里的目标温度可以依据实际工作要求改变，只是做标定使用。例如可以将20℃改为40℃，即变为center_nuc_40和target_temperature_20，并设置为40℃情况下对应的参数值。
  * @para precision：设置，温度精度。若设置为2.0f,则对应高增益模式为正负2摄氏度，对应低增益模式为正负2%。
  * @para c_fix：输出，修正c
  * @para time：输出，标定迭代次数
  * @para err：输出，根据精度要求，输出错误情况代码（仅低增益模式下十进制输出，查看时请转换成六位二进制数，如010101）
  * @return  0:标定成功 -1：标定失败查看，查看错误代码 -2：超时 -3：rangeMode设置错误
*/

int autoCalibrate(int width,
                       int height,
                       float *temperatureTable,
                       unsigned short *fourLinePara,
                       float* floatFpaTmp,
                       float* correction,
                       float* Refltmp,
                       float* Airtmp,
                       float* humi,
                       float* emiss,
                       unsigned short* distance,
                       int cameraLens,
                       float* shutter_fix,
                       int rangeMode,
                       unsigned short center_nuc_20,
                       unsigned short center_nuc_30,
                       unsigned short center_nuc_100,
                       float target_temperature_20,
                       float target_temperature_30,
                       float target_temperature_100,
                       float precision,
                       float* c_fix,
                       int* time,
                       int* err);


#ifdef __cplusplus
}
#endif

#endif
