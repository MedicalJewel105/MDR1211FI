#include "MDR32FxQI_port.h"
#include "MDR32FxQI_rst_clk.h"
#include "MDR32FxQI_dac.h"
#include <string.h>
#include <stdlib.h>


// Инициализация ЦАП
void DAC_init(void) {
	MDR_PORTE->ANALOG &= ~(1 << 0); // E0 - OUT
  MDR_PORTE->ANALOG &= ~(1 << 2); // E1 - REF
  
  MDR_DAC->CFG |= DAC_CFG_M_REF1; // DAC2_REF
  MDR_DAC->CFG |= DAC_CFG_ON_DAC1; // включить DAC2
}
// Инициализация АЦП
void ADC_init(void) {
	MDR_PORTD->ANALOG &= ~(1 << 0); // 0 — аналоговый режим
	// Настройка тактирования:
	// Источник → ADC_C1 → ADC_C2 → ADC_C3 → АЦП
	MDR_RST_CLK->ADC_MCO_CLOCK &= ~(1 << RST_CLK_ADC_MCO_CLOCK_ADC_C1_SEL_Pos);// Биты выбора источника для ADC_C1: 00 – CPU_C1
	MDR_RST_CLK->ADC_MCO_CLOCK |= (2 << RST_CLK_ADC_MCO_CLOCK_ADC_C2_SEL_Pos); // Биты выбора источника для ADC_C2: 10 – ADC_C1
	MDR_RST_CLK->ADC_MCO_CLOCK |= (10 << RST_CLK_ADC_MCO_CLOCK_ADC_C3_SEL_Pos); // Биты выбора делителя для ADC_C3: 1010 – ADC_C3 = ADC _C2 / 8
	MDR_RST_CLK->ADC_MCO_CLOCK |= (1 << RST_CLK_ADC_MCO_CLOCK_ADC_CLK_EN_Pos); // включение тактирования
	
	MDR_ADC->ADC1_CFG = 0; // сброс; внутреннее опорное напряжение (от AUCC и AGND)
  MDR_ADC->ADC1_CFG = (1 << 0); // включить АЦП1
}

// Функция чтения значения АЦП
uint32_t ADC_read(uint8_t ch) {
		if (ch > 15) ch = 0; 
		MDR_ADC->ADC1_CFG &= ~(0b11111 << ADC1_CFG_REG_CHS_Pos); // зануляем для выбора
		MDR_ADC->ADC1_CFG |= (ch << ADC1_CFG_REG_CHS_Pos); // Выбор аналогового канала, по которому поступает сигнал для преобразования
    MDR_ADC->ADC1_CFG |= (1 << 1); // начало преобразования
    while (!(MDR_ADC->ADC1_STATUS & (1 << ADC_STATUS_FLG_REG_EOCIF_Pos))); // Ожидание завершения;Флаг выставляется, когда закончено преобразования и данные еще не считаны.
    return (MDR_ADC->ADC1_RESULT & 0x0FFF); // Возврат 12-битного результата
}

uint32_t ADC_result = 0;

int main()
{
    // включаем HSE осциллятор
    MDR_RST_CLK->HS_CONTROL |= 1;

    // запускаем осциллятор
    while (!(MDR_RST_CLK->CLOCK_STATUS & (1 << 2))) {}

    // инициализируем PLL
    MDR_RST_CLK->PLL_CONTROL |= (5 << RST_CLK_PLL_CONTROL_PLL_CPU_MUL_Pos); // устанавливаем множитель частоты cpu 6
    MDR_RST_CLK->PLL_CONTROL |= (1 << RST_CLK_PLL_CONTROL_PLL_CPU_ON_Pos);  // включаем
    while (!(MDR_RST_CLK->CLOCK_STATUS & (1 << RST_CLK_CLOCK_STATUS_PLL_CPU_RDY_Pos))) {}

    // тактирование; 48 МГц (изначально 8)
    MDR_RST_CLK->CPU_CLOCK |= (2 << RST_CLK_CPU_CLOCK_CPU_C1_SEL_Pos) | (1 << RST_CLK_CPU_CLOCK_CPU_C2_SEL_Pos); // CPU_C1 = HSE | CPU_C2 = PLLCPUo
    MDR_RST_CLK->CPU_CLOCK |= (1 << RST_CLK_CPU_CLOCK_HCLK_SEL_Pos); // подаём тактирование CPU_C3 на HSE

    // разрешение тактирования всей периферии
    MDR_RST_CLK->PER_CLOCK = 0xFFFFFFFF; 

    // настройка периферии
	  ADC_init();
	  DAC_init();
   
    
	while (1)
	{
			ADC_result = ADC_read(0); // Читаем значение с АЦП (D0)
			//MDR_DAC->DAC2_DATA = 2048;
      MDR_DAC->DAC2_DATA = ADC_result; // Выводим значение АЦП напрямую в ЦАП

			for (int i=0; i<10000; i++){__NOP();}
	}
}