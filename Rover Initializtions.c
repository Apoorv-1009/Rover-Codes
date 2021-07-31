/*
Official Rover Initialization Code 2021:
CAN pins:   UART Pins:   Wheels Control Pins:   
Tx: PA12    Tx: PA9      PA4   PA5   (DIR)
Rx: PA11    Rx: PA10     PB6   PB7   (PWM)

Mast Cam 1:   Mast Cam 2:   SPI Pins:   ADC Pins:
DIR: PC14     DIR: PC15     MOSI: PB15  ADC1: PA6
PWM: PB8      PWM: PB9      MISO: PB14  
                            SCK: PB13
                            NSS: PB12

Arm Pins:
1) DIR: PB3   2)DIR: PB4   3)DIR: PB5   4)DIR: PA15   5)DIR: PB2   6)DIR: PB10   7)DIR: PB11
   PWM: PA0     PWM: PA1     PWM: PA2     PWM: PA3      PWM: PB0     PWM: PB1      PWM: PA7
	 
	 Timer 2: PA0(C1), PA1(C2), PA2(C3), PA3(C4)
	 Timer 3: PA7(C2), PB0(C3), PB1(C4)
	 Timer 4: PB6(C1), PB7(C2), PB8(C3), PB9(C4)
	 
!NOTE! PA8 is an unused GPIO Pin on the PCB 
GPIO Initialize completed
TIMER Initialize completed
ADC Initialize completed
UART Initialize completed
SPI Initialize compelted

*/

#include "stm32f103xe.h"
#include "stdlib.h"

volatile uint16_t myTicks = 0;

void delay_ms(uint16_t ms)
{
	myTicks = 0;
	while(myTicks<ms);
}

void SysTick_Initialize()   
{
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/1000);
}

void SysTick_Handler(void)
{
	myTicks++;
}

void GPIO_Initialize()
{
	//Enable Clocks:
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN; 
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;   //Enable Alternate Function
	
				//CAN PINS SETUP:
	//PA12 Setup: (Tx)
	GPIOA->CRH |= GPIO_CRH_MODE12;   //OUTPUT Mode (11)
	GPIOA->CRH |= GPIO_CRH_CNF12_1;   //AF Output Push-Pull (10)
	GPIOA->CRH &= ~(GPIO_CRH_CNF12_0); 
	
	//PA11 Setup: (Rx)
	GPIOA->CRH &= ~(GPIO_CRH_MODE11);   //INPUT Mode (00)
	GPIOA->CRH &= ~(GPIO_CRH_CNF11_1);   //Floating Input (reset state) (01)  
	GPIOA->CRH |= (GPIO_CRH_CNF11_0);
	
				//UART PINS SETUP:
	//PA9 Setup: (Tx)
	GPIOA->CRH |= GPIO_CRH_MODE9;   //OUTPUT Mode (11)
	GPIOA->CRH |= GPIO_CRH_CNF9_1;   //AF Output Push-Pull (10)
	GPIOA->CRH &= ~(GPIO_CRH_CNF9_0); 
	
	//PA10 Setup: (Rx)
	GPIOA->CRH &= ~(GPIO_CRH_MODE10);   //INPUT Mode (00)
	GPIOA->CRH &= ~(GPIO_CRH_CNF10_1);   //Floating Input (reset state) (01)  
	GPIOA->CRH |= (GPIO_CRH_CNF10_0);
	
						//WHEEL CONTROL PINS:
	//PA4 Setup: (Left DIR)
	GPIOA->CRL |= GPIO_CRL_MODE4;   //OUTPUT Mode (11)
	GPIOA->CRL &= ~(GPIO_CRL_CNF4);   //Output Push-Pull (00)
	
	//PA5 Setup: (Right DIR)
	GPIOA->CRL |= GPIO_CRL_MODE5;   //OUTPUT Mode (11)
	GPIOA->CRL &= ~(GPIO_CRL_CNF5);   //Output Push-Pull (00)
	
	//PB6 Setup: (Right PWM)
	GPIOB->CRL |= GPIO_CRL_MODE6;   //OUTPUT Mode (11)
	GPIOB->CRL |= GPIO_CRL_CNF6_1;   //AF Output Push-Pull (10)
	GPIOB->CRL &= ~(GPIO_CRL_CNF6_0);
	
	//PB7 Setup: (Left PWM)
	GPIOB->CRL |= GPIO_CRL_MODE7;   //OUTPUT Mode 50Mhz (11)
	GPIOB->CRL |= GPIO_CRL_CNF7_1;   //AF Output Push-Pull (10)
	GPIOB->CRL &= ~(GPIO_CRL_CNF7_0);
	
				//MAST CAM 1 PINS:
	//PC14 Setup: (DIR)
	GPIOA->CRH |= GPIO_CRH_MODE14;   //OUTPUT Mode (11)
	GPIOA->CRH &= ~(GPIO_CRH_CNF14);   //Output Push-Pull (00)
	
	//PB8 Setup: (PWM)
	GPIOB->CRH |= GPIO_CRH_MODE8;   //OUTPUT Mode 50Mhz (11)
	GPIOB->CRH |= GPIO_CRH_CNF8_1;   //AF Output Push-Pull (10)
	GPIOB->CRH &= ~(GPIO_CRH_CNF8_0);
	
						//MAST CAM 2 PINS:
	//PC15 Setup: (DIR)
	GPIOC->CRH |= GPIO_CRH_MODE15;   //OUTPUT Mode (11)
	GPIOC->CRH &= ~(GPIO_CRH_CNF15);   //Output Push-Pull (00)
	
	//PB9 Setup: (PWM)
	GPIOB->CRH |= GPIO_CRH_MODE9;   //OUTPUT Mode (11)
	GPIOB->CRH |= GPIO_CRH_CNF9_1;   //AF Output Push-Pull (10)
	GPIOB->CRH &= ~(GPIO_CRH_CNF9_0);
	
				//ARM PINS:
	//1)
	//PB3 Setup: (DIR)
	GPIOB->CRL |= GPIO_CRL_MODE3;   //OUTPUT Mode (11)
	GPIOB->CRL &= ~(GPIO_CRL_CNF3);   //Output Push-Pull (00)
	
	//PA0 Setup: (PWM)
	GPIOA->CRL |= GPIO_CRL_MODE0;   //OUTPUT Mode (11)
	GPIOA->CRL |= GPIO_CRL_CNF0_1;   //AF Output Push-Pull (10)
	GPIOA->CRL &= ~(GPIO_CRL_CNF0_0);
	
	//2)
	//PB4 Setup: (DIR)
	/*PB4 is by default initialized as a JTRST Pin and thus has to be disabled.
	By default the pin is at a high state and doesnt function as a normal GPIO
	Read AFIO debug configeration in the reference manual for more data on how to disable JTRST*/
	AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_2;
	AFIO->MAPR &= ~(AFIO_MAPR_SWJ_CFG_1 | AFIO_MAPR_SWJ_CFG_0);   // JTRST & SWJ Disable (100)
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
	
						//ADC PINS:
	//PA6 Setup:
	GPIOA->CRL &= ~(GPIO_CRL_MODE6);   //INPUT Mode (00)
	GPIOA->CRL &= ~(GPIO_CRL_CNF6);   //Analog Mode (00)
	
}

void Timer_Initialize()
{
	/*
	Timer 2: PA0(C1), PA1(C2), PA2(C3), PA3(C4)
	Timer 3: PA7(C2), PB0(C3), PB1(C4)
	Timer 4: PB6(C1), PB7(C2), PB8(C3), PB9(C4)	
	*/
	
				//TIMER 2 SETUP:
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;   //Timer 2 Enable
	TIM2->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;   //Enable Channel 1,2,3,4 as OUTPUT
	TIM2->CR1 |= TIM_CR1_ARPE;   //Enable Auto Re-Load Preload (ARPE)
	
	TIM2->CCMR1 |= TIM_CCMR1_OC1PE;   //Enable Preload for Channel 1
	TIM2->CCMR1 |= TIM_CCMR1_OC2PE;   //Enable Preload for Channel 2
	TIM2->CCMR2 |= TIM_CCMR2_OC3PE;   //Enable Preload for Channel 3
	TIM2->CCMR2 |= TIM_CCMR2_OC4PE;   //Enable Preload for Channel 4
	
	//PWM Mode 1 for Channel 1:
	TIM2->CCMR1 |= (TIM_CCMR1_OC1M_2) | (TIM_CCMR1_OC1M_1);
	TIM2->CCMR1 &= ~(TIM_CCMR1_OC1M_0);
	//PWM Mode 1 for Channel 2:
	TIM2->CCMR1 |= (TIM_CCMR1_OC2M_2) | (TIM_CCMR1_OC2M_1);
	TIM2->CCMR1 &= ~(TIM_CCMR1_OC2M_0);
	//PWM Mode 1 for Channel 3:
	TIM2->CCMR2 |= (TIM_CCMR2_OC3M_2) | (TIM_CCMR2_OC3M_1);
	TIM2->CCMR2 &= ~(TIM_CCMR2_OC3M_0);
	//PWM Mode 1 for Channel 4:
	TIM2->CCMR2 |= (TIM_CCMR2_OC4M_2) | (TIM_CCMR2_OC4M_1);
	TIM2->CCMR2 &= ~(TIM_CCMR2_OC4M_0);
	
	TIM2->PSC = 1;   //freq/1 = 72 Mhz
	TIM2->ARR = 4095;   //16 Bit value
	TIM2->CCR1 = 0;
	TIM2->CCR2 = 0;
	TIM2->EGR |= TIM_EGR_UG;   //Update Registers
	TIM2->CR1 |= TIM_CR1_CEN;   //Start Counting
	
	
				//TIMER 3 SETUP:
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;   //Timer 3 Enable
	TIM3->CCER |= TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;   //Enable Channel 1,2,3,4 as OUTPUT
	TIM3->CR1 |= TIM_CR1_ARPE;   //Enable Auto Re-Load Preload (ARPE)
	
	TIM3->CCMR1 |= TIM_CCMR1_OC2PE;   //Enable Preload for Channel 2
	TIM3->CCMR2 |= TIM_CCMR2_OC3PE;   //Enable Preload for Channel 3
	TIM3->CCMR2 |= TIM_CCMR2_OC4PE;   //Enable Preload for Channel 4
	
	//PWM Mode 1 for Channel 2:
	TIM3->CCMR1 |= (TIM_CCMR1_OC2M_2) | (TIM_CCMR1_OC2M_1);
	TIM3->CCMR1 &= ~(TIM_CCMR1_OC2M_0);
	//PWM Mode 1 for Channel 3:
	TIM3->CCMR2 |= (TIM_CCMR2_OC3M_2) | (TIM_CCMR2_OC3M_1);
	TIM3->CCMR2 &= ~(TIM_CCMR2_OC3M_0);
	//PWM Mode 1 for Channel 4:
	TIM3->CCMR2 |= (TIM_CCMR2_OC4M_2) | (TIM_CCMR2_OC4M_1);
	TIM3->CCMR2 &= ~(TIM_CCMR2_OC4M_0);
	
	TIM3->PSC = 1;   //freq/1 = 72 Mhz
	TIM3->ARR = 4095;   //16 Bit value
	TIM3->CCR1 = 0;
	TIM3->CCR2 = 0;
	TIM3->EGR |= TIM_EGR_UG;   //Update Registers
	TIM3->CR1 |= TIM_CR1_CEN;   //Start Counting
	
	
				//TIMER 4 SETUP:
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;   //Timer 4 Enable
	TIM4->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;   //Enable Channel 1,2,3,4 as OUTPUT
	TIM4->CR1 |= TIM_CR1_ARPE;   //Enable Auto Re-Load Preload (ARPE)
	
	TIM4->CCMR1 |= TIM_CCMR1_OC1PE;   //Enable Preload for Channel 1
	TIM4->CCMR1 |= TIM_CCMR1_OC2PE;   //Enable Preload for Channel 2
	TIM4->CCMR2 |= TIM_CCMR2_OC3PE;   //Enable Preload for Channel 3
	TIM4->CCMR2 |= TIM_CCMR2_OC4PE;   //Enable Preload for Channel 4
	
	//PWM Mode 1 for Channel 1:
	TIM4->CCMR1 |= (TIM_CCMR1_OC1M_2) | (TIM_CCMR1_OC1M_1);
	TIM4->CCMR1 &= ~(TIM_CCMR1_OC1M_0);
	//PWM Mode 1 for Channel 2:
	TIM4->CCMR1 |= (TIM_CCMR1_OC2M_2) | (TIM_CCMR1_OC2M_1);
	TIM4->CCMR1 &= ~(TIM_CCMR1_OC2M_0);
	//PWM Mode 1 for Channel 3:
	TIM4->CCMR2 |= (TIM_CCMR2_OC3M_2) | (TIM_CCMR2_OC3M_1);
	TIM4->CCMR2 &= ~(TIM_CCMR2_OC3M_0);
	//PWM Mode 1 for Channel 4:
	TIM4->CCMR2 |= (TIM_CCMR2_OC4M_2) | (TIM_CCMR2_OC4M_1);
	TIM4->CCMR2 &= ~(TIM_CCMR2_OC4M_0);
	
	TIM4->PSC = 1;   //freq/1 = 72 Mhz
	TIM4->ARR = 4095;   //16 Bit value
	TIM4->CCR1 = 0;
	TIM4->CCR2 = 0;
	TIM4->EGR |= TIM_EGR_UG;   //Update Registers
	TIM4->CR1 |= TIM_CR1_CEN;   //Start Counting
}

void ADC_Initialize()
{
    	//PA6(Ch6)
	RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;   //f/6 = 72/6 = 12 Mhz
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;   //Enable ADC1 Clock

	ADC1->SMPR2 |= ADC_SMPR2_SMP6_2 | ADC_SMPR2_SMP6_1 | ADC_SMPR2_SMP6_0;   //601.5 Sampling Rate for Channel 6

	//Set channel you want to convert in the sequence registers:
	ADC1->SQR3 |= ADC_SQR3_SQ1_2 | ADC_SQR3_SQ1_1;   //Channel 6, Sequence 1

	ADC1->CR2 |= ADC_CR2_ADON;   //Enable ADC for the 1st time
	ADC1->CR2 |= ADC_CR2_CONT;   //Set ADC to Continuous Mode
	delay_ms(1);

	//Turn on ADC for the 2nd time to acually turn it on:
	ADC1->CR2 |= ADC_CR2_ADON;
	delay_ms(1);

	//Run Calibration:
	ADC1->CR2 |= ADC_CR2_CAL;
	delay_ms(2);
}

void UART_Initilaize()
{
    	//PA10(Rx)
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;   //UART1 Enable, Clk freq = 72Mhz
	//Setting up Baud Rate:
	USART1->BRR |= 0x0B64; 			//Gives 9600 Baud Rate
	//              Rx enable      UART Enable
	USART1->CR1 |= USART_CR1_RE | USART_CR1_UE;
}

uint8_t getuval()   //Reads UART Values
{
	uint8_t data = 0;
	while(!(USART1 ->SR & USART_SR_RXNE));   //Check Status Register if all is Recieved 
	data = USART1->DR;
	return data;
}

void SPI_Initiliaze()
{
    //PB12(NSS) PB13(SCK) PB14(MISO) PB15(MOSI)
    RCC->APB2ENR |= RCC_APB2ENR_SPI2EN;   //SPI2 Enable
    SPI2->CR1 |= SPI_CR1_BR_0;   //Baud Rate: f/4
    SPI2->CR1 &= ~SPI_CR1_MSTR;   //Slave Mode Select
    SPI2->CR1 |= SPI_CR1_SPE;   //SPI Enable
}

uint8_t getsval()
{
    uint8_t data = 0;
    while(!(SPI2->SR & SPI_SR_RXNE));
    data = SPI2->DR;
    return data;
}

uint8_t CAN_Initialize()
{

}

void Initialize()
{
    SysTick_Initialize();
    GPIO_Initialize();
    Timer_Initialize();
    ADC_Initialize();
    UART_Initilaize();
    SPI_Initiliaze();
    CAN_Initialize();
}

int main()
{
    Initialize();   //Initialize all Parameters
    while(1)
    {
        

    }



}

