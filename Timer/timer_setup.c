#include "MDR32FxQI_port.h"
#include "MDR32FxQI_rst_clk.h"

void timer_setup(void);
void Timer1_IRQHandler(void);

void timer_setup(void){
	MDR_TIMER1->CNTRL = 0x00000000; //Режим инициализации таймера
	//Настраиваем работу основного счетчика
	MDR_TIMER1->CNT = 0x00000000; //Начальное значение счетчика
	MDR_TIMER1->PSG = 0x00000000; //Предделитель частоты TIM_CLK
	MDR_TIMER1->ARR = 0x0000000F; //Основание счета (до 15)
	MDR_TIMER1->IE = 0x00000002; //Разрешение генерировать прерывание при CNT = ARR
	//Разрешение работы таймера
	MDR_TIMER1->CNTRL = 0x00000001; //Счет прямой по TIM_CLK
	
	
	
	// тактирование периферийного блока
	//MDR_RST_CLK->TIM_CLOCK |= (1 << RST_CLK_TIM_CLOCK_TIM1_CLK_EN);
	//MDR_RST_CLK->PER_CLOCK |= (1 << RST_CLK_PER_CLOCK_PCLK_EN_TIMER1);
	//MDR_RST_CLK->PER_CLOCK |= (1 << RST_CLK_PER_CLOCK_PCLK_EN_PORTA);
}

void Timer1_IRQHandler(void){
	MDR_TIMER1->CNT = 0x00000000; //Сброс значения
}