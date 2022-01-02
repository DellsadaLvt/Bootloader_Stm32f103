#include "stm32f10x.h"                  // Device header

int main(void){
	RCC->APB2ENR |= (uint32_t)(1u << 4u);
	GPIOC->CRH &= ~((uint32_t)(0x0F << 20u));
	GPIOC->CRH |= (uint32_t)(1u << 20u);
	GPIOC->BSRR |= (uint32_t)(1u << 29u);
	
	while(1u){
		GPIOC->BSRR |= (uint32_t)(1u << 29u);
		for( uint32_t i= 0u; i< 1000000u; i+= 1u);
		GPIOC->BSRR |= (uint32_t)(1u << 13u);
		for( uint32_t i= 0u; i< 1000000u; i+= 1u);
	}

}








