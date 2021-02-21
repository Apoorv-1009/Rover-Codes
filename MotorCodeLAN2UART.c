/*
 Code implements Motor Code on LEDs thru PWM with Timer4, by reading Joystick values
 from a phone via LAN2UART.
 Phone Joystick->Switcher->(LAN2UART)->STMF1
 Setup:
 PWM LEDs: PB6(Ch1), PB7(Ch2)
 UART: PA10(Rx)
       Left           Right
 LED:  PA4            PA5
 PWM:  PB6            PB7
 Same code was later run with our GUI, values sent from right joystick of the DS4
 */

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

	//Setup PA4:
	GPIOA->CRL |= GPIO_CRL_MODE4;   //OUTPUT Mode 50Mhz
	GPIOA->CRL &= ~(GPIO_CRL_CNF4);   //Output Push-Pull

	//Setup PA5:
	GPIOA->CRL |= GPIO_CRL_MODE5;   //OUTPUT Mode 50Mhz
	GPIOA->CRL &= ~(GPIO_CRL_CNF5);   //Output Push-Pull

	//Setup PB6:
	GPIOB->CRL |= GPIO_CRL_MODE6;   //OUTPUT Mode 50Mhz
	//Enable AF Mode:
	GPIOB->CRL |= GPIO_CRL_CNF6_1;
	GPIOB->CRL &= ~(GPIO_CRL_CNF6_0);

	//Setup PB7:
	GPIOB->CRL |= GPIO_CRL_MODE7;   //OUTPUT Mode 50Mhz
	//Enable AF Mode:
	GPIOB->CRL |= GPIO_CRL_CNF7_1;
	GPIOB->CRL &= ~(GPIO_CRL_CNF7_0);

	//PA10 Setup: (UART Rx)
	GPIOA->CRH &= ~(GPIO_CRH_MODE10);   //INPUT Mode (00)
	GPIOA->CRH |= GPIO_CRH_CNF10_1;   //Input with pull-up/pull-down (10)
	GPIOA->CRH &= ~(GPIO_CRH_CNF10_0);
}

void Timer_Initialize()
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;   //Enable Timer4
	TIM4->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E; //Enable Channel 1 and 2 as OUTPUT
	TIM4->CR1 |= TIM_CR1_ARPE;   //Enable Auto Re-Load Preload (ARPE)

	TIM4->CCMR1 |= TIM_CCMR1_OC1PE;   //Enable Preload for Channel 1
	TIM4->CCMR1 |= TIM_CCMR1_OC2PE;   //Enable Preload for Channel 2

	//PWM Mode 1 for Channel 1:
	TIM4->CCMR1 |= (TIM_CCMR1_OC1M_2) | (TIM_CCMR1_OC1M_1);
	TIM4->CCMR1 &= ~(TIM_CCMR1_OC1M_0);
	//PWM Mode 1 for Channel 2:
	TIM4->CCMR1 |= (TIM_CCMR1_OC2M_2) | (TIM_CCMR1_OC2M_1);
	TIM4->CCMR1 &= ~(TIM_CCMR1_OC2M_0);

	TIM4->PSC = 1;   //freq/1 = 8 Mhz
	TIM4->ARR = 8000;
	TIM4->CCR1 = 0;
	TIM4->CCR2 = 0;

	TIM4->EGR |= TIM_EGR_UG;   //Update Registers
	TIM4->CR1 |= TIM_CR1_CEN;   //Start Counting
}

void UART_Initilaize()
{
    	//PA9(Tx) PA10(Rx)
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;   //UART1 Enable, Clk freq = 8Mhz
	//Setting up Baud Rate:
	USART1->BRR |= 4<<4 | 5<<0;   //Gives 115200 Baud Rate(approx.) Register Value = (8MHz)/(16 * Reqd. Baud Rate) = 4.5
	//              Rx Enable      UART Enable
	USART1->CR1 |= (USART_CR1_RE | USART_CR1_UE);
}

uint8_t getuval()   //Reads UART Values
{
	uint8_t data;
	while(!(USART1->SR & USART_SR_RXNE));   //Check Status Register if all is Recieved
	data = USART1->DR;
	return data;
}

//        |Left    Right| |      Left PWM       |  |      Right PWM     | |  Values  |  | Gear |
void Drive(int DL, int DR, int oct0, int a, int b, int oct1, int p, int q,int X, int Y, float gear)
{
	if (DL == 1)
		GPIOA->BSRR |= 1 << 4;   //Turn on LEFT LED
	else
		GPIOA->BRR |= 1 << 4;   //Turn off LEFT LED

	if (DR == 1)
		GPIOA->BSRR |= 1 << 5;   //Turn on RIGHT LED
	else
		GPIOA->BRR |= 1 << 5;   //Turn off RIGHT LED

	TIM4->CCR1 = (uint32_t) (abs(8000 * oct0 - abs(X * a) - abs(Y * b)) * (gear*0.1));   //Left PWM
	TIM4->CCR2 = (uint32_t) (abs(8000 * oct1 - abs(X * p) - abs(Y * q)) * (gear*0.1));   //Right PWM
}

void MotorCode(int x, int y, float g)
{
	if (abs(x) < 100 && abs(y) < 100)   //No Motion
		Drive(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, g);

	else if (abs(x) < 100 && y < 0)   //Full Forward
		Drive(1, 1, 0, 0, 1, 0, 0, 1, x, y, g);

	else if (abs(x) < 100 && y > 0)   //Full Backward
		Drive(0, 0, 0, 0, 1, 0, 0, 1, x, y, g);

	else if (x < 0 && abs(y) <= 100)   //Spot Turn Left
		Drive(0, 1, 0, 1, 0, 0, 1, 0, x, y, g);

	else if (x > 0 && abs(y) <= 100)   //Spot Turn Right
		Drive(1, 0, 0, 1, 0, 0, 1, 0, x, y, g);

	else if (x > 0 && y < 0 && x >= abs(y))   //Octet 1
	{
		if (abs(x) > 8000 - abs(y))
			Drive(1, 0, 0, 1, 0, 1, 0, 1, x, y, g);
		else
			Drive(1, 0, 1, 0, 1, 0, 1, 0, x, y, g);
	}

	else if (x > 0 && y < 0 && x < abs(y))   //Octet 2
	{
		if (abs(y) > 8000 - abs(x))
			Drive(1, 1, 0, 0, 1, 1, 1, 0, x, y, g);
		else
			Drive(1, 1, 1, 1, 0, 0, 0, 1, x, y, g);
	}

	else if (x < 0 && y < 0 && abs(y) > abs(x))   //Octet 3
	{
		if (abs(y) > 8000 - abs(x))
			Drive(1, 1, 1, 1, 0, 0, 0, 1, x, y, g);
		else
			Drive(1, 1, 0, 0, 1, 1, 1, 0, x, y, g);
	}

	else if (x < 0 && y < 0 && abs(x) >= abs(y))   //Octet 4
	{
		if (abs(x) > 8000 - abs(y))
			Drive(0, 1, 1, 0, 1, 0, 1, 0, x, y, g);
		else
			Drive(0, 1, 0, 1, 0, 1, 0, 1, x, y, g);
	}

	else if (x < 0 && y > 0 && abs(x) > abs(y))   //Octet 5
	{
		if (abs(x) > 8000 - abs(y))
			Drive(0, 1, 0, 1, 0, 1, 0, 1, x, y, g);
		else
			Drive(0, 1, 1, 0, 1, 0, 1, 0, x, y, g);
	}

	else if (x < 0 && y > 0 && abs(y) >= abs(x))   //Octet 6
	{
		if (abs(y) > 8000 - abs(x))
			Drive(0, 0, 0, 0, 1, 1, 1, 0, x, y, g);
		else
			Drive(0, 0, 1, 1, 0, 0, 0, 1, x, y, g);
	}

	else if (x > 0 && y > 0 && abs(y) >= abs(x))   //Octet 7
	{
		if (abs(y) > 8000 - abs(x))
			Drive(0, 0, 1, 1, 0, 0, 0, 1, x, y, g);
		else
			Drive(0, 0, 0, 0, 1, 1, 1, 0, x, y, g);
	}

	else if (x > 0 && y > 0 && abs(x) > abs(y))   //Octet 8
	{
		if (abs(x) > 8000 - abs(y))
			Drive(1, 0, 1, 0, 1, 0, 1, 0, x, y, g);
		else
			Drive(1, 0, 0, 1, 0, 1, 0, 1, x, y, g);
	}

	//Test Drive:
	//Drive(1,1,0,1,0,0,0,1,x,y,g);
}

int main()
{
	GPIO_Initialize();
	Timer_Initialize();
    	UART_Initilaize();

	int x = 0, y = 0;
    	int trash = 0;
	float gear = 1.0;
	while (1)
	{
		//Read LAN2UART Values
		if(getuval() == 'm')
		{
			gear = (int) ((getuval() - '0') + 1);   //Get gear value
			if(getuval() == 's')
			{
				x = (getuval()-'0')*10000 + (getuval()-'0')*1000 + (getuval()-'0')*100 + (getuval()-'0')*10 + (getuval()-'0');   //x value
			}
			if(getuval() == 'f')
			{
				y = (getuval()-'0')*10000 + (getuval()-'0')*1000 + (getuval()-'0')*100 + (getuval()-'0')*10 + (getuval()-'0');   //y value
			}
			trash = getuval();   //This is actually Mast CAM values but we're ignoring it for now
		}
		else
		{
			trash = trash + 1 - 1;   //Bakchodi
		 }
		x = x - 8000;  //Adjust x-axis
		y = 8000 - y;  //Adjust y-axis
		if(abs(x) < 1000)
			x = 0;
		if(abs(y) < 1000)
			y = 0;
		MotorCode(x, y, gear);   //Run MotorCode
	}
}


