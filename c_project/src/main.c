#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <murax.h>

static inline uint32_t simd_add(uint32_t input_1, uint32_t input_2){
	uint32_t output;
	asm volatile(".insn r CUSTOM_0, 0x0, 0x0, %0, %1, %2" : "=r" (output) : "r" (input_1), "r" (input_2));
	return output;
}

main() {
	volatile uint32_t a = 11, b = 6, c = 5;
	uint32_t result = 0;
	uint32_t result_2 = 0;

	interruptCtrl_init(TIMER_INTERRUPT);
	prescaler_init(TIMER_PRESCALER);
	timer_init(TIMER_A);

	TIMER_PRESCALER->LIMIT = 12000-1; //1 ms rate

	TIMER_A->LIMIT = 1000-1;  //1 second rate
	TIMER_A->CLEARS_TICKS = 0x00010002;

	TIMER_INTERRUPT->PENDINGS = 0xF;
	TIMER_INTERRUPT->MASKS = 0x1;

	GPIO_A->OUTPUT_ENABLE = 0x000000FF;
	GPIO_A->OUTPUT = 0x00000000;

	UART->STATUS = 2; //Enable RX interrupts
	UART->DATA = 'A';

	result_2 = b;

	while(1){
		result += a;
		result += b + c;
		for(uint32_t idx = 0;idx < 50000;idx++) asm volatile("");
		GPIO_A->OUTPUT = (GPIO_A->OUTPUT & ~0x3F) | ((GPIO_A->OUTPUT + 1) & 0x3F);  //Counter on LED[5:0]
		result_2 = simd_add(b, result_2);
		printf("Result: %d\n", result_2);
	}
}


void irqCallback(){
	if(TIMER_INTERRUPT->PENDINGS & 1){  //Timer A interrupt
		GPIO_A->OUTPUT ^= 0x80; //Toogle led 7
		TIMER_INTERRUPT->PENDINGS = 1;
	}
	while(UART->STATUS & (1 << 9)){ //UART RX interrupt
		UART->DATA = (UART->DATA) & 0xFF;
	}
}



