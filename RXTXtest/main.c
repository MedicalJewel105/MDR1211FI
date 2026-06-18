#include "MDR32FxQI_port.h"
#include "MDR32FxQI_rst_clk.h"
#include <string.h>


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
  
  // настройка порта
  MDR_RST_CLK->PER_CLOCK = 0xFFFFFFFF;
  
  MDR_PORTF->ANALOG |= (1 << 6);
  MDR_PORTF->OE |= (1 << 6);
  MDR_PORTF->PWR |= (3 << 12);

  // настройка UART1
  setup_UART1();
  // включение прерываний
  NVIC_EnableIRQ(UART1_IRQn);
  
  while (1){
    if (!(MDR_UART1->FR & UART_FR_RXFE)){ // буфер приемника заполнен
      unsigned int temp=0;
      temp=MDR_UART1->DR;
      MDR_UART1->DR=temp;
    }     
  }
}

void setup_UART1(void){
	// ???
  // настройка портов
  MDR_PORTA->ANALOG |= (1 << 6); // RX
  MDR_PORTA->ANALOG |= (1 << 7); // TX
  MDR_PORTA->FUNC |= (3 << 12);
  MDR_PORTA->FUNC |= (3 << 14);
  MDR_PORTA->OE |= (0 << 6);
  MDR_PORTA->OE |= (1 << 7);
  MDR_PORTA->PWR |= (3 << 12);
  MDR_PORTA->PWR |= (3 << 14);
  // настройка дополнительных портов
  MDR_PORTB->ANALOG |= (1 << 6); // RX
  MDR_PORTB->ANALOG |= (1 << 5); // TX
  MDR_PORTB->FUNC |= (2 << 12);
  MDR_PORTB->FUNC |= (2 << 10);
  MDR_PORTB->OE |= (0 << 6);
  MDR_PORTB->OE |= (1 << 5);
  MDR_PORTB->PWR |= (3 << 12);
  MDR_PORTB->PWR |= (3 << 10);
  // настройка тактирования для BR (Baud Rate) 115200
  MDR_RST_CLK->PER_CLOCK |= (1 << RST_CLK_PER_CLOCK_PCLK_EN_UART1_Pos); // разрешаем тактирование периферии
  MDR_RST_CLK->UART_CLOCK |= (1 << RST_CLK_UART_CLOCK_UART1_CLK_EN_Pos); // разрешаем тактирование
  MDR_RST_CLK->UART_CLOCK |= (4 << RST_CLK_UART_CLOCK_UART1_BRG_Pos); // делитель 16 (48 МГц / 16 = 3 МГц) (можно и без этого)
  // настройка BR 115200
  // BAUDDIV = FUARTCLK/(16*BR) = 3 МГц/(16*115200) = 1,628
  // IBRD = 1
  // FBRD = int(0.628*64 + 0.5) = 41
  MDR_UART1->IBRD = 1;
  MDR_UART1->FBRD = 41;
  // количество бит для передачи
  MDR_UART1->LCR_H  =0x60;//Передавать будем 8 бит
  // регистр управления
  MDR_UART1->CR |= UART_CR_RXE; // разрешить чтение данных
  MDR_UART1->CR |= UART_CR_TXE; // разрешить отправку данных
  MDR_UART1->CR |= UART_CR_UARTEN; // разрешить приёмо-передачу
  // настройка прерываний
  MDR_UART1->IMSC |= UART_IMSC_RXIM;
  // отключение буфера FIFO для эхо
  MDR_UART1->LCR_H &= ~(UART_LCR_H_FEN);  
  
}

void UART1_IRQHandler(void){
  unsigned char temp=0;
  // принимаем и отсылаем обратно
  temp=MDR_UART1->DR;
  MDR_UART1->DR=temp;
  while (!(MDR_UART1->FR & UART_FR_TXFE)){}; // ждем, пока не отправит
}