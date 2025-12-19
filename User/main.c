#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "LED.h"
#include "Timer.h"
#include "Key.h"
#include "RP.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"
#include "AD.h"
#include "PID.h"

#define CENTER_ANGEL	2030	//中心角度值
#define CENTER_RANGE	500		//中心区间范围
#define START_PWM		38		//启摆时电机转动的力度
#define START_TIME		120		//启摆时电机施加瞬时驱动力的时间，一般在80~120之间

uint8_t KeyNum;
uint8_t RunState;//倒立摆启动的标志位，来控制是否启动

uint16_t Angle;
int16_t Speed,Location;

PID_t Angle_PID={
	.Target=CENTER_ANGEL,

	.Kp=0.3,
	.Ki=0.009,
	.Kd=0.41,
	
	.OutMax=+100,
	.OutMin=-100,
};

PID_t Location_PID={
	.Target=0,

	.Kp=0.4,
	.Ki=0.01,//略微有点积分超调，不过问题不大，积分偏移不多，补救方案可使用积分分离
	.Kd=5.34,
	
	.OutMax=+100,
	.OutMin=-100,
};

int main(void)
{
	OLED_Init();
	LED_Init();
	Key_Init();
	RP_Init();
	Motor_Init();
	Encoder_Init();
	Serial_Init();
	AD_Init();
	
	Timer_Init();
	
	while (1)
	{
		KeyNum=Key_GetNum();
		if(KeyNum==1)
		{
			if(RunState==0)
				RunState=21;
			else
				RunState=0;
		}
			
		if(KeyNum==4)	
			RunState=31;
		if (KeyNum == 2)			//如果K2按下
		{
			Location_PID.Target += 408;		//位置环目标位置加408，控制横杆正转一圈
			if (Location_PID.Target > 4080)	//如果正转了10圈
			{
				Location_PID.Target = 4080;	//不再允许正转，避免位置越界
			}
		}
		if (KeyNum == 3)			//如果K3按下
		{
			Location_PID.Target -= 408;		//位置环目标位置减408，控制横杆反转一圈
			if (Location_PID.Target < -4080)	//如果反转了10圈
			{
				Location_PID.Target = -4080;	//不再允许反转，避免位置越界
			}
		}
		//采用LED来合理显示是否工作
		if(RunState)
			LED_ON();
		else
			LED_OFF();
//		//26min处调参，重新调
//		/*RP_GetValue函数返回电位器旋钮的AD值，范围：0~4095*/
//		/* 除4095.0可以把AD值归一化，再乘上一个系数，可以调整到一个合适的范围*/
//		Angle_PID.Kp=RP_GetValue(1)/4095.0*1;//调节范围是0-1
//		Angle_PID.Ki=RP_GetValue(2)/4095.0*1;
//		Angle_PID.Kd=RP_GetValue(3)/4095.0*1;
//		
		
		//36min
		Location_PID.Kp=RP_GetValue(1)/4095.0*1;//调节范围是0-1
		Location_PID.Ki=RP_GetValue(2)/4095.0*1;
		Location_PID.Kd=RP_GetValue(3)/4095.0*9;//阻尼不足，加大范围

		
		OLED_Printf(42,0,OLED_6X8,"%02d",RunState);
		
		OLED_Printf(0,0,OLED_6X8,"Angle");
		OLED_Printf(0,12,OLED_6X8,"Kp:%05.3f",Angle_PID.Kp);
		OLED_Printf(0,20,OLED_6X8,"Ki:%05.3f",Angle_PID.Ki);
		OLED_Printf(0,28,OLED_6X8,"Kd:%05.3f",Angle_PID.Kd);
		OLED_Printf(0,40,OLED_6X8,"Tar:%04.0f",Angle_PID.Target);
		OLED_Printf(0,48,OLED_6X8,"Act:%04d",Angle);
		OLED_Printf(0,56,OLED_6X8,"Out:%+04.0f",Angle_PID.Out);
		
		OLED_Printf(64,0,OLED_6X8,"Location");
		OLED_Printf(64,12,OLED_6X8,"Kp:%05.3f",Location_PID.Kp);
		OLED_Printf(64,20,OLED_6X8,"Ki:%05.3f",Location_PID.Ki);
		OLED_Printf(64,28,OLED_6X8,"Kd:%05.3f",Location_PID.Kd);
		OLED_Printf(64,40,OLED_6X8,"Tar:%+05.0f",Location_PID.Target);
		OLED_Printf(64,48,OLED_6X8,"Act:%+05d",Location);
		OLED_Printf(64,56,OLED_6X8,"Out:%+04.0f",Location_PID.Out);
		
		
		OLED_Update();
	}
}

//定时器中断函数，可以复制到使用它的地方
void TIM1_UP_IRQHandler(void)
{
	/*定义静态变量（默认初值为0，函数退出后保留值和存储空间）*/
	static uint16_t Count0,Count1,Count2,CountTime; //分别计时
	static uint16_t Angle0,Angle1,Angle2;
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		/*每隔1ms，程序执行到这里一次*/
		Key_Tick();
		
		Angle=AD_GetValue();
		Speed=Encoder_Get();
		Location+=Speed;//位置是速度对时间的累积
		
		
		switch (RunState)
		{
			case 0://倒地后的处理			
				Motor_SetPWM(0);
				//倒地后对程序的各个参数归零处置，不再需要手动复位
				Angle_PID.ErrorInt=0;
				Location_PID.ErrorInt=0;
				break;
			
			case 1:
				Count0++;//状态计时，40ms后切换自动启摆的状态
				if(Count0>=40)//30min处调试采样周期
				{
					Count0=0;
					Angle2=Angle1;
					Angle1=Angle0;
					Angle0=Angle;
					
					if(Angle0>CENTER_ANGEL+CENTER_RANGE//保证右侧区间
					 &&Angle1>CENTER_ANGEL+CENTER_RANGE
					 &&Angle2>CENTER_ANGEL+CENTER_RANGE
					 &&Angle1<Angle0//保证最高点
					 &&Angle2<Angle1)
					{
						RunState=21;
					}
					
					if(Angle0<CENTER_ANGEL+CENTER_RANGE//保证左侧区间
					 &&Angle1<CENTER_ANGEL+CENTER_RANGE
					 &&Angle2<CENTER_ANGEL+CENTER_RANGE
					 &&Angle1>Angle0//保证最高点
					 &&Angle2>Angle1)
					{
						RunState=31;
					}
					
					if(Angle0<CENTER_ANGEL+CENTER_RANGE//进入中心区间
					 &&Angle0>CENTER_ANGEL-CENTER_RANGE
					 &&Angle1<CENTER_ANGEL+CENTER_RANGE
					 &&Angle1>CENTER_ANGEL-CENTER_RANGE
					 &&Angle2<CENTER_ANGEL+CENTER_RANGE
					 &&Angle2>CENTER_ANGEL-CENTER_RANGE)
					{
						Location=0;//重新设置启摆成功的位置为倒立摆的位置
						RunState=4;
					}
				}
				break;
			
			case 21:
				
				Motor_SetPWM(START_PWM);
				RunState=22;
				CountTime=START_TIME;
				break;
			
			case 22:
				CountTime--;
				if(CountTime==0)
				{
					RunState=23;
				}	
				break;
			
			case 23:
				Motor_SetPWM(-START_PWM);
				RunState=24;
				CountTime=START_TIME;
				break;
			
			case 24:
				CountTime--;
				if(CountTime==0)
				{
					Motor_SetPWM(0);
					RunState=1;
				}
				break;
			
			case 31:
				Motor_SetPWM(-START_PWM);
				RunState=32;
				CountTime=START_TIME;
				break;
			
				break;
			
			case 32:
				CountTime--;
				if(CountTime==0)
				{
					RunState=33;
				}	
				break;
			
			case 33:
				Motor_SetPWM(START_PWM);
				RunState=34;
				CountTime=START_TIME;
				break;
			
			case 34:
				CountTime--;
				if(CountTime==0)
				{
					Motor_SetPWM(0);
					RunState=1;
				}
				break;
			
			case 4:
				//work time
					if(!(Angle < CENTER_ANGEL+CENTER_RANGE
					&& Angle >CENTER_ANGEL-CENTER_RANGE))
				{//判断是否倒立摆倒在地上，如果倒了就停止
					RunState=0;
				}
				
				/*根据运行状态执行PID程序或者停止*/
				if(RunState)
				{
					/*角度环计次分频*/
					Count1++;
					if(Count1>=5)//手动分频，直接降速5倍，即if每隔5ms进一次
					{
						Count1=0;
						
						Angle_PID.Actual=Angle;//获取输入值，内环为角度环，实际值为角度值
						PID_Update(&Angle_PID);//不断计算相关参数和更新
						Motor_SetPWM(Angle_PID.Out);//输出PID值到电机的PWM
					}
					
					Count2++;
					if(Count2>=50)
					{
						Count2=0;
						
						Location_PID.Actual=Location;
						PID_Update(&Location_PID);
						
						
						
						//37min处检查极性，保证有楞次定律一样的调控阻碍作用，所以-+
						
						
						Angle_PID.Target=-Location_PID.Out+CENTER_ANGEL;//外环的输出值是内环的目标值
					}
			
				break;
				}					
		}
					
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}

