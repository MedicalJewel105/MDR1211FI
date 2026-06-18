#include "MDR32FxQI_port.h"
#include "MDR32FxQI_rst_clk.h"

int main()
{
	MDR_RST_CLK->HS_CONTROL |= 1;
	
	// zapuskaem oscillator
	while (!(MDR_RST_CLK->CLOCK_STATUS & (1 << 2))) {}
	// init PLL
	MDR_RST_CLK->PLL_CONTROL |= (8 << 8) | (1 << 2);
	while (!(MDR_RST_CLK->CLOCK_STATUS & (1 << 1))) {}
	// taktirovanie
	MDR_RST_CLK->CPU_CLOCK |= (2) | (1 << 2) | (1 << 8);
	// setup port
	MDR_RST_CLK->PER_CLOCK = 0xFFFFFFFF;
	
	MDR_PORTF->ANALOG |= (1 << 6);
	MDR_PORTF->OE |= (1 << 6);
	MDR_PORTF->PWR |= (3 << 12);
		
	MDR_PORTC->ANALOG |= (1 << 0);
	//MDR_PORTC->OE |= (1 << 0);
	//MDR_PORTC->PWR |= (3 << 0);
	
	
	while (1){
		while(!(MDR_PORTC->RXTX &= (1 << 0)))
		{
			MDR_PORTF->RXTX |= (1 << 6);
			for (int i = 0; i < 2000000; i++){
				if ((MDR_PORTC->RXTX &= (1 << 0))){
					break;
				}
			}
			MDR_PORTF->RXTX &= ~(1 << 6);
			for (int i = 0; i < 2000000; i++){
				if ((MDR_PORTC->RXTX &= (1 << 0))){
					break;
				}
			}
		}
			
		MDR_PORTF->RXTX &= ~(1 << 6);
		
	}
	
	return 0;
}