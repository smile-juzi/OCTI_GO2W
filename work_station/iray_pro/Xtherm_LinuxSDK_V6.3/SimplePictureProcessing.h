#ifndef SIMPLE_PICTURE_PROCESSING_H
#define SIMPLE_PICTURE_PROCESSING_H



#include<stdio.h>
#include<stdlib.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C"  //C++
{
#endif
	 /**
     *设置方法二参数
	 * @para input：point to  data
	 * @para width： width
	 * @para height： height
     * @para outputMode：4：8004模式。5：8005模式
     * return 0:right -1:err
	 */

    int dataCheck(unsigned short* input,int width,int height,int outputMode);
    void SimplePictureProcessingInit(int width,int height);//初始化Initialize
    void SimplePictureProcessingInitMidVar(size_t** midVar);//初始化中间量initialize Mid Variables
	 /**
     *设置方法二参数
	 * @para a：该值越高，能够看到的背景信息越明显，但是噪声也越严重。该值越低，对背景会抑制，目标突出。范围0-500,推荐值30.
	 * @para b：该值越高，可使比较少的点也能明显显示。当b趋向于0时，可减少高温物体进入，画面变暗的现象。范围0-50,推荐值29.
	 * @para c：高亮度点数调整，该值越大，高亮度点越多。范围0-500,推荐值3
	 * @para d：低亮度点数调整，该值越大，低亮度点越多。范围0-500,推荐值3
     * @para bright：亮度。范围0-100,推荐值50
     * @para contra：对比度。范围0-100,推荐值50
	 */
    void SetMethodTwoParameter(int a,int b,int c,int d,int bright,int contra);
    void SetParameter(float a,float b,float c,float d,float e,float f);
    void SetDivTemp(int div1,int div2);

	/*
	*kb校正
	*bufferK：输入。K值
	*bufferB：输入。B值
	*input：输入。原始数据。
	*output：输出。经过kb校正后的原始数据。
	*width：输入。图像宽度。
	*height：输入。图像高度，不包括后四行。即例如原始图像分辨率为256*196，在此即为256*192。
	*targetValue：输入。kb校正目标值。“0”，目标值为6000；“1”，目标值为2000。
					在高增益模式下，应选择“0”；在低增益模式下应选择“1”。
	*/
	void kbAdjust(unsigned short *bufferK,unsigned short* bufferB,unsigned short* input,unsigned short* output,int width,int height,int targetValue);

	/*
	*callFindBadPix 盲元查找
	*bufferK：输入。K值。
	*/
	void callFindBadPix(unsigned short* bufferK);

	/*
	*processFindBadK  盲元替换
	*bufferCur：输入。当前帧的原始数据。
	*/
	void processFindBadK(unsigned short* bufferCur);

	/*
	*remove_stripe  去条纹
	*input:输入。原始数据。
	*output：输出。去条纹后的数据。
	*width：输入。图像宽度。
	*height：输入。图像高度，不包括后四行。即例如原始图像分辨率为256*196，在此即为256*192。
	*/
	void remove_stripe(unsigned short* input, unsigned short* output,int width,int height);

	/*
	*temporalFilter  时域滤波
	*bufferCur:输入。当前帧的原始数据。
	*bufferLast：输入。前一帧的原始数据。
	*output:输出。时域滤波后的数据。
	*width：输入。图像宽度。
	*height：输入。图像高度，不包括后四行。即例如原始图像分辨率为256*196，在此即为256*192。
	*/
	void temporalFilter(unsigned short* bufferCur,unsigned short* bufferLast,unsigned short* output,int width,int height);


	//设置参数，参数已经调校好，请勿修改 Set parameters, all set, please dont modify

	 /**
	 * 使用专业图像算法将8004数据转变为图像 Transfer 8004 data into graphs via algorithm
	 * @para input：输入指针，指向8004数据开始 Input pointer, point to 8004 data
	 * @para output：输出指针，指向rgba图像开始 Output pointer, point to rgba graph
	 * @para kindOfPalette1: 0：白热。White Hot 1：黑热。Black Hot 2：铁虹。 Iron Raindow 3：彩虹1。Rainbow 1 
	 * 4、彩虹2. Rainbow 2 5:高对比彩虹1.High-contrast rainbow 1 6:高对比彩虹2. High-contrast rainbow 2 >=7、用户色板 Customize palette
	 */
    void Compute(unsigned short* input,unsigned char* output,int kindOfPalette1,size_t** midVar,int format);
    void ComputeMethodTwo(unsigned short* input,unsigned char* output,int kindOfPalette1,size_t** midVar,int format);//0:rgba 1:rgb
    void ComputeDivTemp(unsigned short* input,unsigned char* output,int kindOfPalette1,size_t** midVar,int format);
    void ComputeDivTempType2(unsigned short* input,unsigned char* output,int kindOfPalette1,int lowTmpPaletteIndex,size_t** midVar,int displayMode,int format);
	 /**
	 * 设置用户色板 Set User Palette
	 * @para palette：输入指针，指向用户色板开始 input pointer, point to user palette
	 * @para typeOfPalette：需要>=7 needs >=7
	 */
    void SetUserPalette(unsigned char* palette,int typeOfPalette);
    void SimplePictureProcessingDeinit();//释放资源 Release resources 
    void SimplePictureProcessingDeinitMidVar(size_t** midVar);//释放中间变量 Release mid variables

	/**
	 * 获得色板Capture palette
	 * @para type：
	 * (0)：256*3 铁虹 iorn rainbow
	 * (1)：256*3 彩虹1 Rainbow 1
	 * (2)：224*3 彩虹2 Rainbow 2
	 * (3)：448*3 高对比彩虹1 High-Contrast Rainbow 1
	 * (4)：448*3 高对比彩虹2 High-Contrast Rainbow 2
	 */
    const unsigned char* getPalette(int type);


#ifdef __cplusplus
}
#endif

#endif
