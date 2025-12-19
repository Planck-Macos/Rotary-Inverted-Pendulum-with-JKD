#include "stm32f10x.h"                  // Device header

void AD_Init()
{	//开启时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);//选择6分频，防止超频（72/6MHz<14MHz）
	//GPIO的配置
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;//ADC/DAC的专属模式，截断该引脚对于GPIO的复用功能，专用于模拟信号采集
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	//选择规则组的输入通道
	ADC_RegularChannelConfig(ADC1,ADC_Channel_8,1,ADC_SampleTime_55Cycles5);//参数的选择要根据引脚的定义来选择

	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_ContinuousConvMode=DISABLE;
	ADC_InitStructure.ADC_DataAlign=ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_Mode=ADC_Mode_Independent;
	ADC_InitStructure.ADC_NbrOfChannel=1;
	ADC_InitStructure.ADC_ScanConvMode=DISABLE;
	ADC_Init(ADC1,&ADC_InitStructure);

	ADC_Cmd(ADC1,ENABLE);//开启ADC的电源

	//对ADC进行校准
	ADC_ResetCalibration(ADC1);// 第1步：复位校准寄存器（清空之前的校准值）
	// 相当于："忘记之前的调零数据，重新开始"

	while(ADC_GetResetCalibrationStatus(ADC1)==SET);// 第2步：等待复位完成
	// 相当于："等待电子秤归零完成"，所以需要一个条件循环

	ADC_StartCalibration(ADC1);// 第3步：开始自动校准
	// ADC内部：测量自己的偏移和增益误差，计算补偿值

	while(ADC_GetCalibrationStatus(ADC1)==SET);// 第4步：等待校准完成
	// 相当于："等待电子秤完成自动调零和标定"
}
//启动转换获取结果
uint16_t AD_GetValue()
{
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);/*假如连续转化的话，把这个函数放上个模块处*/
	//因为转化需要时间，所以给一个标志位获取的函数
	while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);/*然后就可以不需要这个了*/
	return ADC_GetConversionValue(ADC1);
}

