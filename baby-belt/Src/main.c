
#include <stdint.h>
// start PS11 @CmpE443’F24
typedef struct{
	volatile uint32_t CR1; //0
	volatile uint32_t CR2; //4
	volatile uint32_t CR3; //8
	volatile uint32_t BRR; //C
	volatile uint32_t GTPR; //10
	volatile uint32_t RTOR; //14
	volatile uint32_t RQR; //18
	volatile uint32_t ISR; //1C
	volatile uint32_t ICR; //20
	volatile uint32_t RDR; //24
	volatile uint32_t TDR; //28
	volatile uint32_t PRESC; //2C
} USARTType;
typedef struct{
	volatile uint32_t ISR; //0
	volatile uint32_t IER; //4
	volatile uint32_t CR; //8
	volatile uint32_t CFGR; //C
	volatile uint32_t CFG2; //10
	volatile uint32_t SMPR1; //14
	volatile uint32_t SMPR2; //18
	uint32_t reserved2; //1C
	volatile uint32_t TR1; //20
	volatile uint32_t TR2; //24
	volatile uint32_t TR3; //28
	uint32_t reserved3; //2C
	volatile uint32_t SQR1; //30
	volatile uint32_t SQR2; //34
	volatile uint32_t SQR3; //38
	volatile uint32_t SQR4; //3C
	volatile uint32_t DR; //40
	uint32_t reserved4[2]; //44 48
	volatile uint32_t JSQR; //4C
	uint32_t reserved5[4]; //50 54 58 5C
	volatile uint32_t OFR1; //60
	volatile uint32_t OFR2; //64
	volatile uint32_t OFR3; //68
	volatile uint32_t OFR4; //6C
	uint32_t reserved6[4]; //70 74 78 7C
	volatile uint32_t JDR1; //80
	volatile uint32_t JDR2; //84
	volatile uint32_t JDR3; //88
	volatile uint32_t JDR4; //8C
	uint32_t reserved7[4]; //90 94 98 9C
	volatile uint32_t AWD2CR; //A0
	volatile uint32_t AWD3CR; //A4
	uint32_t reserved8[2]; //A8 AC
	volatile uint32_t DIFSEL; //B0
	volatile uint32_t CALFACT; //B0
} ADCType;
typedef struct{
	volatile uint32_t CSR; //0
	uint32_t reserved1; //4
	volatile uint32_t CCR; //8
	volatile uint32_t CDR; //C
} ADCCommon;
typedef struct{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFRL;
	volatile uint32_t AFRH;
	volatile uint32_t BRR;
	uint32_t reserved;
	volatile uint32_t SECCFGR;
} GPIO;
#include <stdint.h>
typedef struct {
	volatile uint32_t CR1; //0
	volatile uint32_t CR2; //4
	volatile uint32_t SMCR; //8
	volatile uint32_t DIER; //C
	volatile uint32_t SR; //10
	volatile uint32_t EGR; //14
	volatile uint32_t CCMR1; //18
	uint32_t reserved1; //1C
	volatile uint32_t CCER; //20
	volatile uint32_t CNT; //24
	volatile uint32_t PSC; //28
	volatile uint32_t ARR; //2C
	volatile uint32_t RCR; //30
	volatile uint32_t CCR1; //34
	volatile uint32_t CCR2; //38
	uint32_t reserved2[2]; //3C 40
	volatile uint32_t BDTR; //44
	volatile uint32_t DCR; //48
	volatile uint32_t DMAR; //4C
	volatile uint32_t OR1; //50
	uint32_t reserved3[3]; //54 58 5C
	volatile uint32_t OR2; //60
} TIM15_General_Purpose_Type;
typedef struct{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	uint32_t reserved;
	volatile uint32_t DIER;
	volatile uint32_t SR;
	volatile uint32_t EGR;
	uint32_t reserved1[3];
	volatile uint32_t CNT;
	volatile uint32_t PSC;
	volatile uint32_t ARR;
} TIMxBasicType;
// end PS11 @CmpE443’F24

#define RCC_CCIPR1 *((volatile uint32_t *) 0x40021088)
#define ISER1 *((volatile uint32_t *) 0xE000E104)
#define GPIOA ((GPIO *) 0x42020000)
#define ADC1 ((ADCType *) 0x42028000)
#define ADC2 ((ADCType *) 0x42028100)
#define ADC ((ADCCommon *) 0x42028300)
#define UART4 ((USARTType *) 0x40004C00)
#define GPIOC ((GPIO *) 0x42020800)
#define ISER2 *((volatile uint32_t *) 0xE000E108)
#define RCC_AHB2ENR *((volatile uint32_t *) 0x4002104C)
#define RCC_APB1ENR1 *((volatile uint32_t *) 0x40021058)
#define RCC_APB2ENR *((volatile uint32_t *) 0x40021060)
#define GPIOB ((GPIO *) (0x42020000 + 0x400))
#define TIM15 ((TIM15_General_Purpose_Type *) (0x40014000))
#define TIM6 ((TIMxBasicType *) 0x40001000)

#define CORD_ARR_LENGTH 15

uint32_t state = 0;
uint32_t system_on = 0;
uint32_t overflow_counter = 0;
uint16_t time_start = 0;
uint16_t time_end = 0;

uint16_t rubber_cord_value;
uint32_t count = 0;
uint16_t current_sequence_number_for_cord = 0;
uint16_t cord_value_array[CORD_ARR_LENGTH];
uint32_t elapsed_time;

void send_no_risk_signal();

void turn_off_the_system(void){
	system_on = 0;
	GPIOA->ODR &= ~(1 << 6);
	send_no_risk_signal();
	TIM6->CR1 &= ~(1);
}

void turn_on_the_system(void){
	system_on = 1;
	GPIOA->ODR |= (1 << 6);
	for(int i=0;i<CORD_ARR_LENGTH;i++){
		cord_value_array[i] = 0;
	}
	current_sequence_number_for_cord = 0;
	TIM6->CR1 |= 1;
}

void toggle_the_system(void){
	if (system_on == 0){
		turn_on_the_system();
	}
	else {
		turn_off_the_system();
	}
}

void TIM15_IRQHandler(void){
	// when overflow occurs, PA9 is turned on AND overflow_counter is incremented
	if ((TIM15->SR & (1 << 0)) != 0){
		TIM15->SR &= ~(1 << 0);
		GPIOA->ODR |= (1 << 9);
		overflow_counter++;
	}
	// when overcapture flag is detected, turn on PC7
	if ((TIM15->SR & (1 << 9)) != 0) {
		TIM15->SR &= ~(1 << 9);
		if ((GPIOC->ODR & (1 << 7)) == 0) {
			GPIOC->ODR |= (1 << 7);
		}
		else {
			GPIOC->ODR &= ~(1 << 7);
		}
	}
	// PB7 is turned on when rising edge is detected
	// and turned off when falling edge is detected
	if ((TIM15->SR & (1 << 1)) != 0){
		// no need to clear SR flag because we read CCR1

		if((GPIOA->IDR & (1 << 7)) != 0 ){ // on rising edge
			time_start = TIM15->CCR1;
			overflow_counter = 0;
			GPIOB->ODR |= (1 << 7);
		}
		else{ // on falling edge
			time_end = TIM15->CCR1;
			elapsed_time = overflow_counter * TIM15->ARR - time_start + time_end;
			overflow_counter = 0;
			GPIOB->ODR &= ~(1 << 7);
			if ((elapsed_time > 1498 && elapsed_time < 3500)){
				toggle_the_system();
			}
		}
	}

}

void TIM6_initialization(){
	RCC_APB1ENR1 |= 1 << 4; //TIM6x_CLK is enabled, running at 4MHz
	TIM6->PSC = 3999; //Set Prescaler
	TIM6->ARR = 999; //Set Delay
	TIM6->CR1 &= ~(1<<1); //OVF will generate an event
	// TIM6->CR1 |= (1<<7); //Enable autoreload of ARR register
	TIM6->DIER |= 1; //NEW! enable UIF to generate an interrupt
	ISER1 |= 1 << 17;//NEW! enable global signaling for TIM6 interrupt
	// ISER1 position is inferred from page 530 in rm0438 PDF
	TIM6->CR1 |= 1; //TIM6_CNT is enabled (clocked)
}

void TIM6_IRQHandler(void){
	TIM6->SR=0; // clear UIF bit
	ADC1->CR |= 1<<2; // ADC start
}

void TIM15_initialization(void){
	RCC_AHB2ENR |= 0b111;
	// initialize systemLED in output mode
	GPIOA->MODER &= ~(0b11 << (6 * 2));
	GPIOA->MODER |= (0b01 << (6 * 2));
	// initialize PA7 as input for signal from the button.
	GPIOA->MODER &= ~(0b11 << (7 * 2));
	// initialize GPIOs for debugging
	GPIOA->MODER &= ~(0b11 << (9 * 2));
	GPIOA->MODER |= (0b01 << (9 * 2));
	GPIOB->MODER &= ~(0b11 << (7 * 2));
	GPIOB->MODER |= (0b01 << (7 * 2));
	GPIOC->MODER &= ~(0b11 << (7 * 2));
	GPIOC->MODER |= (0b01 << (7 * 2));
	// load ICOC AF to PA2 and PA3
	GPIOA->MODER &= ~(0b1111 << (2 * 2));
	GPIOA->MODER |= (0b1010 << (2 * 2));
	GPIOA->AFRL &= ~(0b11111111 << (2 * 4));
	GPIOA->AFRL |= (0b11101110 << (2 * 4));
	// initialize timer and configure timer
	RCC_APB2ENR |= (1 << 16);
	TIM15->PSC = 3999; // set frequency to 1KHz
	TIM15->ARR = 19999; // set duration to 20 seconds
	// tie to channels and initialize IC
	TIM15->CCMR1 |= 0b01;
	TIM15->CCMR1 |= (0b0011 << 4); // set input filter N=8
	TIM15->CCER |= 0b1011;

	// enable peripheral level and function level interrupts
	TIM15->DIER |= 0b111;
	TIM15->SR = 0;
	ISER2 |= (1 << 5);
	TIM15->CR1 |= 1;
}



// this code is modified from PS11
// start PS11 @CmpE443’F24
void UART4_initialization(void){
	// Select SYSCLK (=4MHz) for the clock source of UART4.
	RCC_CCIPR1 &= ~(1 << 7);
	RCC_CCIPR1 |= 1 << 6;
	// Enable GPIO C port for Tx and Rx pins.
	RCC_AHB2ENR |= 1 << 2;
	// Set alternate function for PC10 and PC11.
	GPIOC->MODER &= ~(0b0101 << (10 * 2));
	GPIOC->MODER |= 0b1010 << (10 * 2);
	// Connect PC10 to UART4 Tx.
	GPIOC->AFRH &= ~(0b0111 << (2 * 4));
	GPIOC->AFRH |= 0b1000 << (2 * 4);
	// Connect PC11 to UART4 Rx.
	GPIOC->AFRH &= ~(0b0111 << (3 * 4));
	GPIOC->AFRH |= (0b1000 << (3 * 4));
	//Enable Clock for UART4.
	RCC_APB1ENR1 |= 1 << 19;
	// Set UART4 baud rate (BRR) for 115200 baud rate.
	// 4000000 / 115200 for 115200 baud rate.
	UART4->BRR = 34;
	// Enable FIFO.
	UART4->CR1 |= 1 << 29;
	// Enable Transmitter
	UART4->CR1 |= 0b10 << 2;
	// Enable UART4.
	UART4->CR1 |= 1;
}
// end PS11 @CmpE443’F24

void init_risk_signal(void)
{
	//the led emulates the output that the UART functionality will receive
	// start PS10 @CmpE443'F24
	RCC_AHB2ENR |= 1 << 0; //clock for GPIOA is enabled
	GPIOA->MODER &= ~(1 << 11); //PA5 is output
	// end PS10
}
void send_risk_signal(void)
{
	// start PS10 @CmpE443'F24
	GPIOA->ODR |= (1 << 5); //Write 1 to PA5
	UART4->TDR = 0b1;
	// end PS10
}
void send_no_risk_signal(void)
{
	// start PS10 @CmpE443'F24
	GPIOA->ODR &= ~(1 << 5); //Write 0 to PA5
	UART4->TDR = 0;
	// end PS10
}
void init_ADC(void)
{
	// start PS10 @CmpE443'F24
	// Set GPIO pins as analog mode
	RCC_AHB2ENR |= 1 << 1; //clock for GPIOA
	GPIOA->MODER |= 0b11 << 2; //PA1 is analog mode
	RCC_AHB2ENR |= 1 << 13; //enable ADC clock
	ADC1->CR &= ~(1 << 29); //take ADC module from deep-power down
	ADC1->CR |= (1 << 28); //turn on ADC voltage regulator

	RCC_CCIPR1 |= 3 << 28; //main ADC clock is system clock (HCLK = SYSCLK =4MHz out of reset)
	ADC->CCR |= 3 << 16; //ADC_CLK = 1MHz (HCLK/4)
	// end PS10

	//ADC->CCR |= 0b1011 << 18;

	// start PS10 @CmpE443'F24
	ADC1->SMPR1 |= 0b111 << 18; //sampling time= 640.5 ADC12_IN6 clock cycles


	ADC1->SQR1 &= ~(0b1111 << 0); // Set number of conversions to 1
	ADC1->SQR1 |= 6 << 6; // 1st conversion is ADC12_IN6
	ADC1->CR |= (1 << 31); //calibrate ADC
	while((ADC1->CR & (1 << 31)) != 0) {} //wait until calibration is complete
	ADC1->CR |= 1; //turn on ADC module
	while((ADC1->ISR & 1) == 0) {} //wait until ADC is ready
	ADC1->CR |= 1 << 2; //start conversion
	ADC1->IER |= 1 << 2; //turn on EOC interrupts
	ISER1 |= 1 << 5;//enable global signaling for ADC1_2 interrupt
	// end PS10
}

int calculate_risk(void){
	//this was called read cord in the icoc pi but we decided to change its name and it will pass the value to the UART functionality
	int max_diff = 0;
	for(int i = 1;i<15;i++){//this calculates the maximum sequential difference between cord_value_array elements
		int diff = 0;
		if(cord_value_array[i]>cord_value_array[i-1]){
			diff = cord_value_array[i] - cord_value_array[i-1];
		}else{
			diff= cord_value_array[i-1] - cord_value_array[i];
		}
		if(diff > max_diff){
			max_diff =diff;
		}
	}
	int diff = 0;
	if(cord_value_array[0]>cord_value_array[14]){
		diff = cord_value_array[0] - cord_value_array[14];
	}else{
		diff= cord_value_array[14] - cord_value_array[0];
	}
	if(diff > max_diff){
		max_diff =diff;
	}
	if(max_diff < 100){
		return 1;//this means there is a suffocation risk
	}else{
		return 0;//this means there is no risk
	}
}

int adc_read_count = 0;
void ADC1_2_IRQHandler(void)
{
	if((ADC1->ISR & 1<<2) != 0)
	{
		if(adc_read_count != 2){
			rubber_cord_value = ADC1->DR;
			ADC1->CR |= 1<<2;
			adc_read_count++;
		}else{
			rubber_cord_value = ADC1->DR;
			cord_value_array[current_sequence_number_for_cord] = rubber_cord_value;
			current_sequence_number_for_cord = (current_sequence_number_for_cord + 1) % 15;
			if ((calculate_risk()== 1)&& (system_on == 1))
			{
				send_risk_signal();
			}
			else
			{
				send_no_risk_signal();
			}
			//TIM6->CR1 |= 1;
			adc_read_count = 0;
		}
	}
}



// start PS10 @CmpE443'F24
void __enable_irq(void)
{
	__asm volatile(
			"mov r0, #0 \n\t"
			"msr primask, r0 \n\t"
	);
}
// end PS10

int main(void)
{
	init_ADC();
	init_risk_signal();
	__enable_irq();
	UART4_initialization();
	TIM15_initialization();
	TIM6_initialization();
	turn_off_the_system();
	while(1)
	{
		__asm volatile("wfi");
	}
	return(1);
}
