#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <murax.h>

static inline uint32_t simd_add(uint32_t input_1, uint32_t input_2){
	uint32_t output;
	asm volatile(".insn r CUSTOM_0, 0x0, 0x0, %0, %1, %2" \
		: "=r" (output) : "r" (input_1), "r" (input_2));
	return output;
}

main() {
	volatile uint32_t a = 4;
	uint32_t result = 0;

	while(1){
		result = simd_add(a, result);
		printf("Result: %d\n", result);
	}
}

void irqCallback(){
}



