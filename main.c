#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include <stdio.h>
#include <math.h>

char data[100];
int counter = 0;
int button_counter = 0;
double output_signal = 0.0;
uint16_t adc_value = 0;

double outputToPC  = 0.0;
double inputToSystem = 0.0;


double K = 1, zeta = 1.261, wn = 0.674;
double stepSize = 0.2;


double eq1(double y1, double y2, double u)
{
 return y2;
}

double eq2(double y, double y1, double u)
{
 return (K*wn*wn*u)-(2*zeta*wn*y)-(wn*wn*y1);
}


double ExplicitEulerEq1(double (*f)(double,double,double), double y2, double ts,
double uk)
{
 static double yk = 0;
 double yk1 = yk + ts * f(yk, y2, uk);
 yk = yk1;
 return yk1;
}

double ExplicitEulerEq2(double (*f)(double,double,double), double y1, double ts,
double uk)
{
 static double yk = 0; // Previous value of the output
 double yk1 = yk + ts * f(yk, y1, uk);
 yk = yk1;
 return yk1;
}


// PID parameters
volatile double Kp = 2.5, Ki = 0.0, Kd = 0;
// PID Controller

volatile double stepsize = 0.2;

double derivative_pid(double ek, double ek_1)
{
 return (ek - ek_1) / stepsize;
}
// Trapezoidal integration
double integral_pid(double ek, double ek_1)
{
 return (ek + ek_1) * (stepsize / 2);
}

double PID(double r, double y)
{
 static double ek_1 = 0, integralSum = 0;
 double outputPID;
 double ek = r - y;
 integralSum += integral_pid(ek, ek_1);
 outputPID = Kp * ek + Ki * integralSum + Kd * derivative_pid(ek, ek_1);
 ek_1 = ek;
 return outputPID;
}



void UART_Transmit(char *string)
{
	while(*string)
	{
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC));
		USART_SendData(USART3,*string);
		*string++;
	}
}

void PWM_TIMER_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct;

    /* Enable clock for GPIOD */
    //***********
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    //***********

	/* Alternating functions for pins */
    //***********
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	//
	/* Set pins */
	//***********
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;   //All chosen push pull
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; //All chosen as nopull
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;     //Alternating function to use PWM
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;   //Full speed
	GPIO_Init(GPIOD, &GPIO_InitStruct);               //Load the configuration
    //***********

	TIM_TimeBaseInitTypeDef TIM_BaseStruct;
	/* Enable clock for TIM4 */
	//***********
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	//***********

	/* TIM4 Base Settings */
	//***********
	TIM_BaseStruct.TIM_Prescaler = 0;
    TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_BaseStruct.TIM_Period = 33000;
    TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_BaseStruct.TIM_RepetitionCounter = 0;
	//***********

	/* Initialize TIM4 */
	//***********
    TIM_TimeBaseInit(TIM4, &TIM_BaseStruct);
	//***********

	/* Start count on TIM4 */
    TIM_Cmd(TIM4, ENABLE);

}

void ADC_Config()
{
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_25MHz;

	GPIO_Init(GPIOA,&GPIO_InitStruct);

	ADC_InitTypeDef ADC_InitStruct;
	ADC_CommonInitTypeDef ADC_CommonInitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);

	ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div6;

	ADC_CommonInit(&ADC_CommonInitStruct);

	ADC_InitStruct.ADC_Resolution = ADC_Resolution_8b;
	ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;

	ADC_Init(ADC1,&ADC_InitStruct);

	ADC_Cmd(ADC1,ENABLE);
}

uint16_t Read_adc()
{

	ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_3Cycles);

	ADC_SoftwareStartConv(ADC1);

	//while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC == RESET));

	return ADC_GetConversionValue(ADC1);

}



void counter_TIMER_Init()
{
    TIM_TimeBaseInitTypeDef TIM_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);	 	 //TIMER TIM1 clock enable
	TIM_InitStruct.TIM_Prescaler = 479;	//479	//524					//Prescaler set is 249
	TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;  	   //Counter mode up
	TIM_InitStruct.TIM_Period = 63999;						   // I used this formula to calculate the period--->Update Event = TIM clk/((PSC+1)*(ARR+1)*(RCR+1))
	TIM_InitStruct.TIM_ClockDivision = 0;                      // however it missed with small numbers  than I incremented that value a little bit experimentally.
	TIM_InitStruct.TIM_RepetitionCounter = 0;                  //Repetition counter set 0.

	TIM_TimeBaseInit(TIM1,&TIM_InitStruct);                    //Load the configuration


	TIM_Cmd(TIM1,ENABLE);																		   //TIM1 enable
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);                   //Configures the TIMx event by the software

	NVIC_InitStruct.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;      //NVIC channel selected.
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority  = 1;    //NVIC Preemption Priority set as 0
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;				//NVIC Sub Priority set as 0
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;               //NVIC Channel enabled when interrupt comes

	NVIC_Init(&NVIC_InitStruct);                               //Load

}

void TIM1_UP_TIM10_IRQHandler()
{
	static volatile double y2=0, y1=0;

	if(TIM_GetITStatus(TIM1,TIM_IT_Update) == SET)
	{

		if(button_counter == 0)
		{
			inputToSystem = 0.0;

		}

		else {
			inputToSystem = 1.0;

		}

		y1 = ExplicitEulerEq1(eq1, y2, stepSize, inputToSystem);
		y2 = ExplicitEulerEq2(eq2, y1, stepSize, inputToSystem);

		TIM_OCInitTypeDef TIM_OCStruct;           // Timer struct defined for pwm.
		TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM2;  //PWM2 mode selected
		TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;  //Output state enable
		TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;       //Polarity chosen as low

		TIM_OCStruct.TIM_Pulse = inputToSystem*(11500); //1v 10909 = 36000 /3.3
		TIM_OC1Init(TIM4, &TIM_OCStruct);
		TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

		adc_value = Read_adc();
		output_signal = adc_value*(0.01294117647);   // 3.3/255

		USART_SendData(USART3,output_signal*100);    //output_signal *100;

		TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
	}
}
void CONFIG_USART3()
{
        GPIO_InitTypeDef GPIO_InitStructure;                      			//GPIO_InitTypeDef for USART pins.
		USART_InitTypeDef USART_InitStructure;                    			//USART_InitTypeDef definition.

		// USART IO Settings
	    //**************
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);   			//USART3 clock enable.
	    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;                       //Alternate function mode chosen.
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9;            //I have used port D, pins 8 and 9 for USART.
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                      //Output type chosen as push-pull.
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;                       // Pull up chosen for USART.
		GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_50MHz;                 //50Mhz speed chosen.
		GPIO_Init(GPIOD, &GPIO_InitStructure);                              //Load the configuration

		GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);           //Alternate function config.
		GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);           //Alternate function config.
	    //**************

        // USART Settings
     	//**************
	    USART_InitStructure.USART_BaudRate = 19200;                          //Baudrate chosen as 19200.
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //We are not using any interrupts or hardware flow.
		USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;      //Receive and transmit mode chosen.
		USART_InitStructure.USART_Parity = USART_Parity_No;                  //No parity bits.
		USART_InitStructure.USART_StopBits = USART_StopBits_1;               //1 stop bit selected.
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;          //Word lenght 8 bits.
		USART_Init(USART3, &USART_InitStructure);                            //Load the configuration

		USART_Cmd(USART3, ENABLE);                                           //USART Start.
}


void EXTI_Config()
{
  GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	//Clock settings
	//**************
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);          //GPIOA clock enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);        //SYSCFG clock enable
	//**************

	//Interrupt Settings -->Button (Gas Button)
	//**************
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;               					//GPIOA Pin2 Select
	GPIO_InitStruct.GPIO_Mode =  GPIO_Mode_IN;					 					//GPIOA Pin2 Input
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;          					//GPIOA Pin2 Push-Pull
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;        					//GPIOA Pin2 Nopull
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;      					//GPIOA Pin2 Very high speed
	GPIO_Init(GPIOA,&GPIO_InitStruct);									 					//Load the configuration

  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,GPIO_PinSource2); 	     //EXTI_Line Config Pin 2
  EXTI_InitStruct.EXTI_Line = EXTI_Line2;                  			//Line 2 selected. In STM32F4 Pin 2 connected to line 2
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;                    		//Line enable selected. (When interrupt comes)
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;							//Mode interrupt selected
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;       		//Trigger rising mode selected
   EXTI_Init(&EXTI_InitStruct);                               		//Load the configuration

	NVIC_InitStruct.NVIC_IRQChannel = EXTI2_IRQn;              		//NVIC EXTI 2 channel selected
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;              		//Channel enable (When interrupt comes)
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;        //Priority set 1
	NVIC_InitStruct.NVIC_IRQChannelSubPriority =0;
    NVIC_Init(&NVIC_InitStruct);                                  //Load the configuration
	//**************
}

void EXTI2_IRQHandler()       //EXTI Interrupt Handler
{
		if(EXTI_GetITStatus(EXTI_Line2) != RESET)
	{
            if(button_counter == 0)
            {
			button_counter = 1;

            }

            else
            {
            	button_counter = 0;
            }

	}
		EXTI_ClearITPendingBit(EXTI_Line2);
}




int main()
{

	ADC_Config();

	counter_TIMER_Init();

 PWM_TIMER_Init();

	EXTI_Config();

	CONFIG_USART3();



	while(1)
	{


	}


}


void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){
  /* TODO, implement your code here */
  return;
}

/*
 * Callback used by stm324xg_eval_audio_codec.c.
 * Refer to stm324xg_eval_audio_codec.h for more info.
 */
uint16_t EVAL_AUDIO_GetSampleCallBack(void){
  /* TODO, implement your code here */
  return -1;
}

