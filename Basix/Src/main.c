#include <stm32f407xx.h>
#include <stdint.h>
////////////////////////////////////////////////////
#define PLL_M 4
#define PLL_N 168
#define PLL_P 0
///////////////////////////////////////////////////
void SysClockConfig(void);
void GPIOA_Config(int);
void delay_us(uint16_t time);
void delay_ms(uint16_t time);
void UARTConfig(void);
void UART2SendCh(uint8_t ch);
void TIM6Config(void);
void UART2SendString (char *string);
uint8_t UART2GetCh(void);

void ADC_Disable (void);
uint16_t ADC_GetVal (void);
void ADC_WaitForConv (void);
void ADC_Start (int channel);
void ADC_Enable (void);
void ADC_Init (void);
void itoa(uint16_t number,char* data);
///////////////////////////////////////////////////
int main(void)
{
	SysClockConfig();
	GPIOA_Config(6);
	TIM6Config();
	UARTConfig();
    while(1)
    {
    	GPIOA->BSRR |= (1<<6);

    	UART2SendString("Y ");
    	UART2SendCh('\n');

    	delay_ms(100);

    	GPIOA->BSRR |= ((1<<6)<<16);
    	delay_ms(100);
    }
}
///////////////////////////////////////////////////
void SysClockConfig(void)
{
	RCC->CR |= RCC_CR_HSEON;
	while(!(RCC->CR & RCC_CR_HSERDY));

	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR->CR |= PWR_CR_VOS;

	FLASH->ACR |= FLASH_ACR_ICEN | FLASH_ACR_PRFTEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;

	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;

	RCC->PLLCFGR =  (PLL_M << 0) | (PLL_N << 6) | (PLL_P << 16) | RCC_PLLCFGR_PLLSRC_HSE;
	RCC->CR |= RCC_CR_PLLON;
	while(!(RCC->CR & RCC_CR_PLLRDY));

	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}
/**********************************************************/
void GPIOA_Config(int PIN_NO){
	RCC->AHB1ENR |= (1<<0);
	GPIOA->MODER |= (1<<(2*PIN_NO));
	GPIOA->OTYPER &= ~(1<<(PIN_NO));
	GPIOA->OSPEEDR |= (1<<(2*PIN_NO+1));
	GPIOA->PUPDR &= ~(1<<(2*PIN_NO) | 1<<(2*PIN_NO+1));
}
/**********************************************************/
void TIM6Config(void)
{
	RCC->APB1ENR |= (1<<4);
	TIM6->PSC = 83;
	TIM6->ARR = 0xFFFF;
	TIM6->CR1 |= (1<<0);
	while(!(TIM6->SR & (1<<0)));
}
/**********************************************************/
void delay_us(uint16_t time)
{
	TIM6->CNT = 0;
	while (TIM6->CNT < time);
}
/**********************************************************/
void delay_ms(uint16_t time)
{
	for(uint16_t i=0; i<time; i++)
	{
		delay_us(1000);
	}
}
/**********************************************************/
void UARTConfig(void)
{
	RCC->APB1ENR |= (1<<17);
	RCC->AHB1ENR |= (1<<0);

	GPIOA->MODER |=(2<<4)|(2<<6);
	GPIOA->OSPEEDR |= ((3<<4)|(3<<6));
	GPIOA->AFR[0] |= (7<<8) | (7<<12);

	USART2->CR1 = 0x00;
	USART2->CR1 |= (1<<13);
	//USART2->CR1 |= ~(1<<12);
	USART2->BRR = (13<<0) | (22<<4);

	USART2->CR1 |= (1<<2);
	USART2->CR1 |= (1<<3);
}
/***************************************************************/
void UART2SendCh(uint8_t ch)
{
	USART2->DR = ch;
	while(!(USART2->SR & (1<<6)));
}
/***************************************************************/
void UART2SendString (char *string)
{
	while (*string) UART2SendCh(*string++);
}
/***************************************************************/
uint8_t UART2GetCh(void)
{
	uint8_t Temp;
	while (!(USART2->SR & (1<<5)));
	Temp = USART2->DR;
	return Temp;
}
/***************************************************************/
void ADC_Init(void)
{

	RCC->APB2ENR |= (1<<8);
	RCC->AHB1ENR |= (1<<0);
	ADC->CCR |= 1<<16;

	ADC1->CR1 = (1<<8);    // SCAN mode enabled
	ADC1->CR1 &= ~(1<<24);   // 12 bit RES
	ADC1->CR1 &= ~(1<<25);


	ADC1->CR2 |= (1<<1);     // Enable continuous conversion mode
	ADC1->CR2 |= (1<<10);    // EOC after each conversion
	ADC1->CR2 &= ~(1<<11);   // Data Alignment RIGHT

	ADC1->SMPR2 &= ~((1<<3) | (1<<12));  // Sampling time of 3 cycles for channel 1 and channel 4

	ADC1->SQR1 |= (1<<20);   

	GPIOA->MODER |= (3<<2);  // Analog mode PA 1 (channel 1)
	GPIOA->MODER |= (3<<0);  // Analog mode PA 1 (channel 1)
}

/**********************************************************/
void ADC_Enable (void)
{

	ADC1->CR2 |= 1<<0;   // ADON =1 enable ADC1
	uint32_t delay = 10000;
	while (delay--);
}

/**********************************************************/
void ADC_Start (int channel)
{

	ADC1->SQR3 = 0;
	ADC1->SQR3 |= (channel<<0);    

	ADC1->SR = 0;        // Clear the status register

	ADC1->CR2 |= (1<<30);  // Start the conversion
}
/**********************************************************/

void ADC_WaitForConv (void)
{
	while (!(ADC1->SR & (1<<1)));  // Wait for EOC flag to set
}
/**********************************************************/
uint16_t ADC_GetVal (void)
{
	return ADC1->DR;  // Read the Data Register
}
/**********************************************************/
void ADC_Disable (void)
{
	ADC1->CR2 &= ~(1<<0);  // Disable ADC
}
/**********************************************************/
void itoa(uint16_t number,char* data)
{

	uint16_t temp,i=0;
	*(data) = '0';
	*(data+1) = '0';
	*(data+2) = '0';
	*(data+3) = '0';
	while(number != 0)
	{
		temp = number - (number/10)*10;
		*(data+3-i) = temp + '0';
		number = number/10;
		i++;
	}
}
/**********************************************************/
