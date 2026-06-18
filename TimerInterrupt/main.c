#include "MDR32FxQI_port.h"
#include "MDR32FxQI_rst_clk.h"

void timer_setup(void);
void Timer1_IRQHandler(void);


int main()
{
	MDR_RST_CLK->HS_CONTROL |= 1;
	
	// запускаем осциллятор
	while (!(MDR_RST_CLK->CLOCK_STATUS & (1 << 2))) {}
		
	// инициализируем PLL
	MDR_RST_CLK->PLL_CONTROL |= (5 << RST_CLK_PLL_CONTROL_PLL_CPU_MUL_Pos); // устанавливаем множитель частоты cpu 6
	MDR_RST_CLK->PLL_CONTROL |= (1 <<  RST_CLK_PLL_CONTROL_PLL_CPU_ON_Pos); // включаем
	while (!(MDR_RST_CLK->CLOCK_STATUS & (1 <<  RST_CLK_CLOCK_STATUS_PLL_CPU_RDY_Pos))) {}

	// тактирование; 48 МГц (изначально 8)
	MDR_RST_CLK->CPU_CLOCK |= (2 << RST_CLK_CPU_CLOCK_CPU_C1_SEL_Pos) | (1 << RST_CLK_CPU_CLOCK_CPU_C2_SEL_Pos); // CPU_C1 = HSE | CPU_C2 = PLLCPUo
	MDR_RST_CLK->CPU_CLOCK |= (1 << RST_CLK_CPU_CLOCK_HCLK_SEL_Pos); // подаём тактирование CPU_C3 на HSE

	// настройка таймера TIMER1
	timer_setup();
	// включение прерываний
	NVIC_EnableIRQ(Timer1_IRQn);
	
	// sнастройка порта
	MDR_RST_CLK->PER_CLOCK = 0xFFFFFFFF;
	
	MDR_PORTF->ANALOG |= (1 << 6);
	MDR_PORTF->OE |= (1 << 6);
	MDR_PORTF->PWR |= (3 << 12);

	while (1){
	}
}

void timer_setup(void){
	// настройка таймера TIMER1
	MDR_RST_CLK->PER_CLOCK |= (1 << RST_CLK_PER_CLOCK_PCLK_EN_TIMER1_Pos); // разрешаем тактирование
	MDR_RST_CLK->TIM_CLOCK |= (4 << RST_CLK_TIM_CLOCK_TIM1_BRG_Pos); // настраиваем делитель частоты 3 МГц = 48 МГц / 16
	MDR_RST_CLK->TIM_CLOCK |= (RST_CLK_TIM_CLOCK_TIM1_CLK_EN); // разрешение тактовой частоты
	
	MDR_TIMER1->PSG = 29; // предделитель, имеем 3 МГц / (29+1) = 100 КГц
	MDR_TIMER1->ARR = 50000; // макс значение, соответствует 0.5 С
	MDR_TIMER1->IE |= (1 << TIMER_IE_CNT_ARR_EVENT_IE_Pos); // включение прерываний при CNT = ARR
	MDR_TIMER1->CNTRL |= (1 << TIMER_CNTRL_CNT_EN_Pos); // включение
}


void Timer1_IRQHandler(void){
	MDR_TIMER1->STATUS &= ~(1 << 1);  // Сброс флага CNT_ARR_EVENT

	if (MDR_PORTF->RXTX &= (1 << 6)){MDR_PORTF->RXTX &= ~(1 << 6);}
	else {MDR_PORTF->RXTX |= (1 << 6);}
}

