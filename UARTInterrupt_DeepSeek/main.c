#include "MDR32FxQI_port.h"
#include "MDR32FxQI_rst_clk.h"
#include <string.h>

void timer_setup(void);
void Timer1_IRQHandler(void);
void UART1_IRQHandler(void);
void setup_UART1(void);

int main()
{
  // включаем HSE осциллятор
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
  
  // разрешение тактирования периферии
  MDR_RST_CLK->PER_CLOCK = 0xFFFFFFFF;
	
  // настройка UART1
  setup_UART1();
  
  // включение прерываний
  NVIC_EnableIRQ(Timer1_IRQn);
  NVIC_EnableIRQ(UART1_IRQn);
  
  while (1){
    // Пустой цикл, всё делается в прерываниях
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

  if (MDR_PORTF->RXTX & (1 << 6)){
    MDR_PORTF->RXTX &= ~(1 << 6);
  }
  else {
    MDR_PORTF->RXTX |= (1 << 6);
  }
}

void setup_UART1(void){
  // настройка портов
  MDR_PORTA->ANALOG |= (1 << 6); // RX
  MDR_PORTA->ANALOG |= (1 << 7); // TX
  MDR_PORTA->FUNC |= (3 << 12);  // PA6 как UART1_RX
  MDR_PORTA->FUNC |= (3 << 14);  // PA7 как UART1_TX
  MDR_PORTA->OE &= ~(1 << 6);    // RX - вход
  MDR_PORTA->OE |= (1 << 7);     // TX - выход
  MDR_PORTA->PWR |= (3 << 12);   // мощность для PA6
  MDR_PORTA->PWR |= (3 << 14);   // мощность для PA7
  // настройка тактирования для BR (Baud Rate) 115200
  MDR_RST_CLK->PER_CLOCK |= (1 << RST_CLK_PER_CLOCK_PCLK_EN_UART1_Pos); // разрешаем тактирование периферии
  MDR_RST_CLK->UART_CLOCK |= (1 << RST_CLK_UART_CLOCK_UART1_CLK_EN_Pos); // разрешаем тактирование
  MDR_RST_CLK->UART_CLOCK |= (4 << RST_CLK_UART_CLOCK_UART1_BRG_Pos); // делитель 16 (48 МГц / 16 = 3 МГц)
  // настройка BR 115200
  // BAUDDIV = FUARTCLK/(16*BR) = 3 МГц/(16*115200) = 1,628
  // IBRD = 1
  // FBRD = int(0.628*64 + 0.5) = 41
  MDR_UART1->IBRD = 1;
  MDR_UART1->FBRD = 41;
  // количество бит для передачи
  MDR_UART1->LCR_H = 0x60; // Передавать будем 8 бит
  // регистр управления
  MDR_UART1->CR |= UART_CR_RXE; // разрешить чтение данных
  MDR_UART1->CR |= UART_CR_TXE; // разрешить отправку данных
  MDR_UART1->CR |= UART_CR_UARTEN; // разрешить приёмо-передачу
  // настройка прерываний
  MDR_UART1->IMSC |= UART_IMSC_RXIM; // прерывание по приёму данных
}

void UART1_IRQHandler(void){
  unsigned char temp = 0;
  
  // Проверяем, что прерывание действительно от приёмника
  if (MDR_UART1->MIS & UART_MIS_RXMIS) {
    // принимаем и отсылаем обратно
    temp = MDR_UART1->DR;
    MDR_UART1->DR = temp;
    // ждем, пока отправится (можно и без ожидания, но для надежности оставим)
    while (!(MDR_UART1->FR & UART_FR_TXFE)) {}
    
    // Сбрасываем флаг прерывания (чтение DR автоматически сбрасывает RX прерывание)
  }
}