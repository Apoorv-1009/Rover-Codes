/*
  Code implements Motor Code on LEDs thru PWM with Timer4, by reading Joystick values
 from a phone via LAN2UART.
 Phone Joystick->Switcher->(LAN2UART)->STMF1
 Setup:
 PWM LEDs: PB6(Ch1), PB7(Ch2)
 UART: PA9(Tx), PA10(Rx)
       Left           Right
 LED:  PA4            PA5
 PWM:  PB6            PB7
 PB4 By default is HIGH, so we disable it in GPIO_Initialize()
**/

#include "stm32f1xx.h"
#include "stdlib.h"
#include <stdarg.h>
#include <string.h>
#include <stdio.h>

void GPIO_Initialize()
{
	//Enable Clocks:
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;   //Enable Clock for Port A
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;   //Enable Clock for Port B
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;   //Enable Alternate Function
	AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_2;
	AFIO->MAPR &= ~(AFIO_MAPR_SWJ_CFG_1 | AFIO_MAPR_SWJ_CFG_0);   // JTRST & SWJ Disable (100)

	//1)
	//PB3 Setup: (DIR)
	GPIOB->CRL |= GPIO_CRL_MODE3;   //OUTPUT Mode (11)
	GPIOB->CRL &= ~(GPIO_CRL_CNF3);   //Output Push-Pull (00)
	GPIOB->BRR |= 1 << 3;   //Mandatory disable to turn it off initially.
	//Now PB4 can be used as a normal GPIO

	//PA0 Setup: (PWM)
	GPIOA->CRL |= GPIO_CRL_MODE0;   //OUTPUT Mode (11)
	GPIOA->CRL |= GPIO_CRL_CNF0_1;   //AF Output Push-Pull (10)
	GPIOA->CRL &= ~(GPIO_CRL_CNF0_0);

	//2)
	//PB4 Setup: (DIR)
	/*PB4 is by default initialized as a JTRST Pin and thus has to be disabled.
	By default the pin is at a high state and doesnt function as a normal GPIO
	Read AFIO debug configeration in the reference manual for more data on how to disable JTRST*/
	GPIOB->CRL |= GPIO_CRL_MODE4;   //OUTPUT Mode (11)
	GPIOB->CRL &= ~(GPIO_CRL_CNF4);   //Output Push-Pull (00)
	GPIOB->BRR |= 1 << 4;   //Mandatory disable to turn it off initially.
	//Now PB4 can be used as a normal GPIO

	//PA1 Setup: (PWM)
	GPIOA->CRL |= GPIO_CRL_MODE1;   //OUTPUT Mode (11)
	GPIOA->CRL |= GPIO_CRL_CNF1_1;   //AF Output Push-Pull (10)
	GPIOA->CRL &= ~(GPIO_CRL_CNF1_0);

	//3)
	//PB5 Setup: (DIR)
	GPIOB->CRL |= GPIO_CRL_MODE5;   //OUTPUT Mode (11)
	GPIOB->CRL &= ~(GPIO_CRL_CNF5);   //Output Push-Pull (00)

	//PA2 Setup: (PWM)
	GPIOA->CRL |= GPIO_CRL_MODE2;   //OUTPUT Mode (11)
	GPIOA->CRL |= GPIO_CRL_CNF2_1;   //AF Output Push-Pull (10)
	GPIOA->CRL &= ~(GPIO_CRL_CNF2_0);

	//4)
	//PA15 Setup: (DIR)
	GPIOA->CRH |= GPIO_CRH_MODE15;   //OUTPUT Mode (11)
	GPIOA->CRH &= ~(GPIO_CRH_CNF15);   //Output Push-Pull (00)

	//PA3 Setup: (PWM)
	GPIOA->CRL |= GPIO_CRL_MODE3;   //OUTPUT Mode (11)
	GPIOA->CRL |= GPIO_CRL_CNF3_1;   //AF Output Push-Pull (10)
	GPIOA->CRL &= ~(GPIO_CRL_CNF3_0);

	//5)
	//PB2 Setup: (DIR)
	GPIOB->CRL |= GPIO_CRL_MODE2;   //OUTPUT Mode (11)
	GPIOB->CRL &= ~(GPIO_CRL_CNF2);   //Output Push-Pull (00)

	//PB0 Setup: (PWM)
	GPIOB->CRL |= GPIO_CRL_MODE0;   //OUTPUT Mode (11)
	GPIOB->CRL |= GPIO_CRL_CNF0_1;   //AF Output Push-Pull (10)
	GPIOB->CRL &= ~(GPIO_CRL_CNF0_0);

	//6)
	//PB10 Setup: (DIR)
	GPIOB->CRH |= GPIO_CRH_MODE10;   //OUTPUT Mode (11)
	GPIOB->CRH &= ~(GPIO_CRH_CNF10);   //Output Push-Pull (00)

	//PB1 Setup: (PWM)
	GPIOB->CRL |= GPIO_CRL_MODE1;   //OUTPUT Mode (11)
	GPIOB->CRL |= GPIO_CRL_CNF1_1;   //AF Output Push-Pull (10)
	GPIOB->CRL &= ~(GPIO_CRL_CNF1_0);

	//7)
	//PB11 Setup: (DIR)
	GPIOB->CRH |= GPIO_CRH_MODE11;   //OUTPUT Mode (11)
	GPIOB->CRH &= ~(GPIO_CRH_CNF11);   //Output Push-Pull (00)

	//PA7 Setup: (PWM)
	GPIOA->CRL |= GPIO_CRL_MODE7;   //OUTPUT Mode (11)
	GPIOA->CRL |= GPIO_CRL_CNF7_1;   //AF Output Push-Pull (10)
	GPIOA->CRL &= ~(GPIO_CRL_CNF7_0);


}

void Timer_Initialize()
{
	//Timer 2: PA0
	//TIMER 2 SETUP:
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;   //Timer 2 Enable
	TIM2->CCER |= TIM_CCER_CC1E;// | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;   //Enable Channel 1,2,3,4 as OUTPUT
	TIM2->CR1 |= TIM_CR1_ARPE;   //Enable Auto Re-Load Preload (ARPE)

	TIM2->CCMR1 |= TIM_CCMR1_OC1PE;   //Enable Preload for Channel 1
	/*
	TIM2->CCMR1 |= TIM_CCMR1_OC2PE;   //Enable Preload for Channel 2
	TIM2->CCMR2 |= TIM_CCMR2_OC3PE;   //Enable Preload for Channel 3
	TIM2->CCMR2 |= TIM_CCMR2_OC4PE;   //Enable Preload for Channel 4
	/**/

	//PWM Mode 1 for Channel 1:
	TIM2->CCMR1 |= (TIM_CCMR1_OC1M_2) | (TIM_CCMR1_OC1M_1);
	TIM2->CCMR1 &= ~(TIM_CCMR1_OC1M_0);
	/*
	//PWM Mode 1 for Channel 2:
	TIM2->CCMR1 |= (TIM_CCMR1_OC2M_2) | (TIM_CCMR1_OC2M_1);
	TIM2->CCMR1 &= ~(TIM_CCMR1_OC2M_0);
	//PWM Mode 1 for Channel 3:
	TIM2->CCMR2 |= (TIM_CCMR2_OC3M_2) | (TIM_CCMR2_OC3M_1);
	TIM2->CCMR2 &= ~(TIM_CCMR2_OC3M_0);
	//PWM Mode 1 for Channel 4:
	TIM2->CCMR2 |= (TIM_CCMR2_OC4M_2) | (TIM_CCMR2_OC4M_1);
	TIM2->CCMR2 &= ~(TIM_CCMR2_OC4M_0);
	/**/
	TIM2->PSC = 1;   //freq/1 = 72 Mhz
	TIM2->ARR = 4095;   //16 Bit value
	TIM2->CCR1 = 0;
	/*
	TIM2->CCR2 = 0;
	TIM2->CCR3 = 0;
	TIM2->CCR4 = 0;
	/**/
	TIM2->EGR |= TIM_EGR_UG;   //Update Registers
	TIM2->CR1 |= TIM_CR1_CEN;   //Start Counting
}

void UART_Initilaize()
{
    //PA9(Tx) PA10(Rx)
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;   //UART1 Enable, Clk freq = 8Mhz
	//Setting up Baud Rate:
	USART1->BRR |= 4<<4 | 5<<0;   //Gives 115200 Baud Rate(approx.) Register Value = (8MHz)/(16 * Reqd. Baud Rate) = 4.5
	//              Rx Enable      Tx Enable  UART Enable
	USART1->CR1 |= (USART_CR1_RE | USART_CR1_TE | USART_CR1_UE);
}

uint8_t getuval()   //Reads UART Values
{
	uint8_t data;
	while(!(USART1->SR & USART_SR_RXNE));   //Check Status Register if all is Recieved
	data = USART1->DR;
	return data;
}

void Drive(int DL, int DR, int a, int b, int p, int q, int X, int Y, float gear, int mode)
{
	if(mode == 0)
	{
	if (DL == 1)
		GPIOA->BSRR |= 1 << 4;   //Turn on LEFT LED
	else
		GPIOA->BRR |= 1 << 4;   //Turn off LEFT LED

	if (DR == 1)
		GPIOA->BSRR |= 1 << 5;   //Turn on RIGHT LED
	else
		GPIOA->BRR |= 1 << 5;   //Turn off RIGHT LED

	TIM4->CCR1 = (uint32_t) abs(abs(a*X) - abs(b*Y))*(gear*0.1);   //Left PWM
	TIM4->CCR2 = (uint32_t) abs(abs(p*X) - abs(q*Y))*(gear*0.1);   //Right PWM
	}
	else if(mode == 1)
	{


	}
}

/**/

void ArmCode(int x1, int y1, int x2, int y2, int l2, int r2, int dl, int dr, int du, int dd)
{
//void Drive(int DL, int DR, int a, int b, int p, int q, int X, int Y, float gear, int mode)
	if(x1 < 0)
		Drive(0,1,0,0,0,1,x1,0,0,1);   //Swivel Right
	else if(x1 >= 0)
		Drive(1,0,0,0,0,1,x1,0,0,1);   //Swivel Left

	/*
	if(y1 < 0)
		Drive(1,0,0,0,1,0,0,y1,0,1);   //Link 1
	else if(y1 >= 0)
			Drive(0,1,0,0,1,0,0,y1,0,1);

	if(x2 < 0)
		Drive(0,1,0,0,1,1,x2,0,0,1);   //Im thinking of using this for motor control
	else if(x2 >= 0)
		Drive(1,0,0,0,1,1,x2,0,0,1);

	if(y2 < 0)
		Drive(1,0,0,1,0,0,0,y2,0,1);   //Link 2
	else if(y2 >= 0)
		Drive(0,1,0,1,0,0,0,y2,0,1);

	Drive(1,0,0,1,0,1,l2,0,0,1);   //Left Trigger

	Drive(0,1,0,1,1,0,r2,0,0,1);   //Right Trigger

	if(du == 1)
		Drive(1,1,0,1,1,1,1000,0,0,1);   //Pitch Up
	else if(du == 0)
		Drive(1,1,0,1,1,1,0,0,0,1);   //No Pitch

	if(dd == 1)
		Drive(0,0,1,0,0,0,1000,0,0,1);   //Pitch Down
	else if(dd == 0)
		Drive(0,0,1,0,0,0,0,0,0,1);   //No Pitch

	if(dr == 1)
		Drive(1,0,1,0,0,1,1000,0,0,1);   //Roll Left
	else if(dr == 0)
		Drive(1,0,1,0,0,1,0,0,0,1);   //No Roll

	if(dl == 1)
		Drive(0,1,1,0,0,1,1000,0,0,1);   //Roll Right
	else if(dl == 0)
		Drive(0,1,1,0,0,1,0,0,0,1);   // No Roll
	/**/
}

int Adjust(int k)
{
	k = k - 8000;
	if(abs(k) < 500)
	k = 0;
	return k;
}
int main()
{
	GPIO_Initialize();
	Timer_Initialize();
	UART_Initilaize();

	int x1 = 0, y1 = 0;
	int x2 = 0, y2 = 0;
	int dl = 0, dr = 0;
	int du = 0, dd = 0;
	int r2 = 0, l2 = 0;
	int trash = 0;
	//float gear = 1.0;
	while (1)
	{
		//Read LAN2UART Values
		if(getuval() == 'a')
		{
			if(getuval() == 's')
				x1 = (getuval()-'0')*10000 + (getuval()-'0')*1000 + (getuval()-'0')*100 + (getuval()-'0')*10 + (getuval()-'0');   //x1 value

			if(getuval() == 'f')
				y1 = (getuval()-'0')*10000 + (getuval()-'0')*1000 + (getuval()-'0')*100 + (getuval()-'0')*10 + (getuval()-'0');   //y1 value

			if(getuval() == 'i')
				x2 = (getuval()-'0')*10000 + (getuval()-'0')*1000 + (getuval()-'0')*100 + (getuval()-'0')*10 + (getuval()-'0');   //x2 value

			if(getuval() == 'j')
				y2 = (getuval()-'0')*10000 + (getuval()-'0')*1000 + (getuval()-'0')*100 + (getuval()-'0')*10 + (getuval()-'0');   //y2 value

			if(getuval() == 'l')
				l2 = (getuval()-'0')*10000 +(getuval()-'0')*1000 + (getuval()-'0')*100 + (getuval()-'0')*10 + (getuval()-'0');   //Left Trigger

			if(getuval() == 'r')
				r2 = (getuval()-'0')*10000 + (getuval()-'0')*1000 + (getuval()-'0')*100 + (getuval()-'0')*10 + (getuval()-'0');   //Right Trigger

			if(getuval() == 'd')
				dl = (getuval() - '0');

			if(getuval() == 'e')
				dr = (getuval() - '0');

			if(getuval() == 'm')
				du = (getuval() - '0');

			if(getuval() == 'n')
				dd = (getuval() - '0');
		}
		else
		{
			trash = trash + 1 - 1;
		}

		x1 = Adjust(x1);
		y1 = Adjust(y1);
		x2 = Adjust(x2);
		y2 = Adjust(y2);
		if(abs(l2) < 100) l2 = 0;
		if(abs(r2) < 100) r2 = 0;

		ArmCode(x1, y1, x2, y2, l2, r2, dl, dr, du, dd);
	}
}
//Data Packets: a16000s16000f16000i16000j16000l08000r08000d1e1m1n1
//Map l2 & r2 from -1 to 1, to 0 to 8000
