
#include <stdint.h>
#ifndef STRUCT_H_
#define STRUCT_H_

// start PS11 @CmpE443’F24
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

typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	uint32_t reserved1;
	volatile uint32_t DIER;
	volatile uint32_t SR;
	volatile uint32_t EGR;
	uint32_t reserved2;
	uint32_t reserved3;
	uint32_t reserved4;
	volatile uint32_t CNT;
	volatile uint32_t PSC;
	volatile uint32_t ARR;
} TIM_Type;

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
} GPIOType;

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


#endif /* STRUCT_H_ */

#define UART4 ((USARTType *) 0x40004C00)
#define GPIOC ((GPIOType *) 0x42020800)
#define GPIOH ((GPIOType *) 0x42021C00)
#define ISER2 *((volatile uint32_t *) 0xE000E108)
#define ICER2 *((volatile uint32_t *) 0XE000E188)
#define RCC_AHB2ENR *((volatile uint32_t *) 0x4002104C)
#define RCC_APB1ENR1 *((volatile uint32_t *) 0x40021058)
#define RCC_APB1ENR2 *((volatile uint32_t *) 0x4002105C)
#define RCC_APB2ENR *((volatile uint32_t *) 0x40021060)
#define RCC_CCIPR1 *((volatile uint32_t *) 0x40021088)
#define RCC_BASE 0x40021000
#define AHB2ENR *((volatile uint32_t *) (RCC_BASE + 0x04C)) // to allow clock to GPIOA
#define APB1ENR1 *((volatile uint32_t *) (RCC_BASE + 0x058)) // to allow clock to TIM6 TIM7
#define TIM6 ((TIM_Type *) (0x40001000))
#define TIM7 ((TIM_Type *) (0x40001400))
#define NVIC_BASE 0xE000E100
#define ISER1 *((volatile uint32_t *) (NVIC_BASE + 0x004)) // to enable global interrupt for TIM6, TIM7
#define TIM15 ((TIM15_General_Purpose_Type *) (0x40014000))
#define GPIOA ((GPIOType *)(0x42020000))
#define GPIOD ((GPIOType *) (0x42020000 + 0xC00))

volatile uint8_t is_MSB = 0;
volatile uint8_t counter = 0;
volatile uint8_t digit_MSB = 0;
volatile uint8_t digit_LSB = 0;

volatile uint8_t has_risk = 0;

uint32_t overflow_counter = 0;
uint32_t compare_state_debug = 0;

void turn_on_TIM15(void){
	overflow_counter = 0;
	// to turn on alarm, enable OC
	TIM15->CCER |= (1 << 4);
	TIM15->ARR = 399;
	TIM15->CCR2 = (TIM15->ARR >> 1);
}

void turn_off_TIM15(void){
	overflow_counter = 0;
	// to turn off alarm, disable OC
	TIM15->CCER &= ~(1 << 4);
	// make sure to set PA3 to low
	GPIOA->BSRR |= (1 << 19);
	TIM15->CCR2 = TIM15->ARR;
}

// start PS9 @CmpE443’F24
void init_TIM15(void){
	RCC_AHB2ENR |= 0b11;
	// initialize GPIOs for debugging
	GPIOA->MODER &= ~(0b11 << (9 * 2));
	GPIOA->MODER |= (0b01 << (9 * 2));
	// load OC AF to PA3 which is CH2 for TIM15
	GPIOA->MODER &= ~(0b11 << (3 * 2));
	GPIOA->MODER |= (0b10 << (3 * 2));
	GPIOA->AFRL &= ~(0b1111 << (3 * 4));
	GPIOA->AFRL |= (0b1110 << (3 * 4));

	// initialize timer and configure timer
	RCC_APB2ENR |= (1 << 16);
	TIM15->PSC = 3999; // set frequency to 1KHz
	TIM15->ARR = 399; // set duration to 0.4 seconds
	// configure OC
	TIM15->BDTR |= (1 << 15);
	TIM15->CCMR1 &= ~(0b11 << 8);
	TIM15->CCMR1 &= ~((1 << 24) | (0b111 << 12));
	TIM15->CCMR1 |= (0b11 << 13);
	TIM15->CCER &= ~(1 << 5);
	// initially disable OC. when has_risk, enable it.
	// when no has_risk, disable it. therefore, do NOT enable TIM15 right now
	//TIM15->CCER |= (1 << 4);

	// output event time is set to ARR/2
	TIM15->CCR2 = TIM15->ARR >> 1;
	// enable peripheral level and function level interrupts
	TIM15->DIER |= 0b111;
	TIM15->SR = 0;
	ISER2 |= (1 << 5);
	TIM15->CR1 |= 1;
}
// end PS9 @CmpE443’F24

// Pins used are GPIOD, PD0..PD6 as a, b...g; PD7 for common of first 7s, PD8 for common of second 7s
void write_to_7s(uint8_t number) {
	// Values as PD6, ..., PD0
	static const uint8_t seven_segment[10] = { 0x3FU, 0x06U, 0x5BU, 0x4FU,
			0x66U, 0x6DU, 0x7DU, 0x07U, 0x7FU, 0x6FU };
	GPIOD->ODR |= (0b1111111);
	GPIOD->ODR &= ~seven_segment[number]; // write the digit to the pins
}

void increment_digits(void){
	if(digit_LSB == 9){
		if(digit_MSB < 9){
			digit_LSB = 0;
			digit_MSB++;
		}
	}
	else{
		digit_LSB++;
	}
}

void turn_off_7ss() {
	GPIOD->ODR |= (0b1111111);
	GPIOD->ODR &= ~(0b11 << 7);
}

void init_seven_segments() {
	AHB2ENR |= 1 << 3; // Enable GPIO port D
	int mask = ((1 << 18) - 1); // = mask of 18 ones = MASK = 0011 1111 1111 1111 1111
	GPIOD->MODER &= ~(mask); // ~mask = 1100 0000 0000 0000 0000 -> clears last 9 modes to 0.
	GPIOD->MODER |= 0b010101010101010101; // sets 9 pins to output mode
}

// start PS8 @CmpE443’F24
void init_TIM6() {
	APB1ENR1 |= 1 << 4;
	TIM6->PSC = 3999;
	TIM6->ARR = 9;
	TIM6->CR1 &= ~(1 << 1);
	TIM6->DIER |= 1;
	ISER1 |= 1 << 17;
	TIM6->CR1 |= 0; // We default enable to 0, only to set it with the presence of risk.
}
// start PS8 @CmpE443’F24

// this code is modified from PS11
// start PS11 @CmpE443’F24
void init_UART4(void) {
	RCC_CCIPR1 &= ~(1 << 7);
	RCC_CCIPR1 |= 1 << 6;
	RCC_AHB2ENR |= 1 << 2;
	GPIOC->MODER &= ~(0b0101 << (10 * 2));
	GPIOC->MODER |= 0b1010 << (10 * 2);
	GPIOC->AFRH &= ~(0b0111 << (2 * 4));
	GPIOC->AFRH |= 0b1000 << (2 * 4);
	GPIOC->AFRH &= ~(0b0111 << (3 * 4));
	GPIOC->AFRH |= (0b1000 << (3 * 4));
	RCC_APB1ENR1 |= 1 << 19;
	UART4->BRR = 34;
	UART4->CR1 |= 1 << 29;
	UART4->CR1 |= 0b11 << 2;
	UART4->CR1 |= 1 << 5;
	ISER2 |= 1;
	UART4->CR1 |= 1;
}
// end PS11 @CmpE443’F24

void turn_on_alerts(void){
	TIM6->CR1 = 1; // enable TIM6
	turn_on_TIM15();
}

void turn_off_alerts(void){
	TIM6->CNT = 0;
	TIM6->CR1 = 0; //disable TIM6
	counter = 0;
	digit_MSB = 0;
	digit_LSB = 0;
	turn_off_7ss();
	turn_off_TIM15();
}

// start PS11 @CmpE443’F24
void __enable_irq(void) {
	__asm volatile(
			"mov r0, #0 \n\t"
			"msr primask, r0 \n\t"
	);
}
// end PS11 @CmpE443’F24

void TIM15_IRQHandler(void){
	// when overflow occurs, PA9 is turned on AND overflow_counter is incremented
	// if 10 (or 5 for demo purposes) seconds have passed, increase the risk level
	if ((TIM15->SR & (1 << 0)) != 0){
		TIM15->SR &= ~(1 << 0);
		GPIOA->ODR |= (1 << 9);
		overflow_counter++;
		uint32_t duration_of_risk_level = overflow_counter * (TIM15->ARR + 1);
		// if duration is grater than 10 seconds (or 5 for demo purposes)
		if (has_risk == 1 && duration_of_risk_level > 4999 && TIM15->ARR > 75){
			overflow_counter = 0;
			TIM15->ARR = TIM15->ARR >> 1;
			TIM15->CCR2 = TIM15->ARR >> 1;
		}
	}
	// when match occurs for the compare register
	if((TIM15->SR && (1 << 2)) != 0){
		TIM15->SR &= ~(1 << 2);
	}
}

void TIM6_IRQHandler(void) {
	TIM6->SR = 0;
	TIM6->CNT = 0;
	TIM6->ARR = 9;

	counter++;
	if(counter == 100) { // 1 second
		increment_digits();
		counter = 0;
	}

	// Display the digits in a cyclic manner
	if (is_MSB == 1) {
		is_MSB = 0;
		GPIOD->ODR |= 1 << 8;
		GPIOD->ODR &= ~(1 << 7);
		write_to_7s(digit_MSB);
	} else {
		is_MSB = 1;
		GPIOD->ODR |= 1 << 7;
		GPIOD->ODR &= ~(1 << 8);
		write_to_7s(digit_LSB);
	}
}

// Read data from the ESP module.
void UART4_IRQHandler(void) {
	if ((UART4->ISR & (1 << 5)) != 0) {
		uint8_t is_changed = UART4->RDR ^ has_risk;
		if (is_changed) {
			has_risk = UART4->RDR;
			if (has_risk == 0) {
				turn_off_alerts();
			} else {
				turn_on_alerts();
			}
		}
	}
}

int main(void) {
	init_seven_segments();
	//init_GPIOC();
	init_UART4();
	init_TIM6();
	init_TIM15();
	__enable_irq();
	while (1) {
		__asm volatile("wfi");

	}
}
