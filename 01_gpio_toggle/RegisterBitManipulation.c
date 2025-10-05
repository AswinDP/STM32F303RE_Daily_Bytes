#include <stdint.h>

#define GPIOC_BASE_ADDR 0x48000800UL

#define RCC_BASE_ADDR 0x40021000UL
#define RCC_ABH2_OFF 0x14UL
#define RCC_ABHENR (RCC_BASE_ADDR + RCC_ABH2_OFF)

int main(void)
{
	uint32_t *clockset = (uint32_t*) RCC_ABHENR;
	uint32_t *modeset = (uint32_t*) GPIOC_BASE_ADDR;

    	*clockset |= (1 << 19);		// Enabling Clock for AHB BUS

    	*modeset |=  (1 << 22);		// Set
    	*modeset &= ~(1 << 22);		// Reset
    	*modeset ^= (1 << 22);		// Toggle
   	*modeset ^= (1 << 22);		// Toggle


	for(;;);
}