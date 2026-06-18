#include "MDR32FxQI_port.h"
#include "MDR32FxQI_rst_clk.h"
#include "MDR32FxQI_i2c.h"
#include <string.h>
#include <stdlib.h>

void I2C_setup(void);
void I2C_IRQHandler(void);
// void I2C_IRQHandler(void)
void UART1_IRQHandler(void);
void setup_UART1(void);

void i2c_write_reg(const uint8_t device_addr, uint8_t reg_addr, uint8_t data);
void i2c_read_reg(const uint8_t addr, uint8_t reg, uint8_t *data_ptr);
void uart_send_str(char *str);
void uart_send_int(int16_t num);
void uart_send_float(float num);

void MPU_read_ACCEL(const uint8_t addr, int axis, uint8_t *data_h_ptr, uint8_t *data_l_ptr); // TODO: перевести addr в const ссылки (?) (подумать)
float convert_to_ms2(const uint8_t *data_h, const uint8_t *data_l);

void command_handler(void);

struct MPU_REGISTERS
{
	const uint8_t ACCEL_XOUT_H;
	const uint8_t ACCEL_XOUT_L;
	const uint8_t ACCEL_YOUT_H;
	const uint8_t ACCEL_YOUT_L;
	const uint8_t ACCEL_ZOUT_H;
	const uint8_t ACCEL_ZOUT_L;
	const uint8_t ACCEL_CONFIG;
	const uint8_t XA_OFFS_H;
	const uint8_t XA_OFFS_L; // [7:1]
	const uint8_t YA_OFFS_H;
	const uint8_t YA_OFFS_L; // [7:1]
	const uint8_t ZA_OFFS_H;
	const uint8_t ZA_OFFS_L; // [7:1]
	const uint8_t PWR_MGMT_1;
	const uint8_t PWR_MGMT_2;
	const uint8_t DIS_XA_Pos;
	const uint8_t DIS_YA_Pos;
	const uint8_t DIS_ZA_Pos;
};
static struct MPU_REGISTERS MPU_REG = {59, 60, 61, 62, 63, 64, 28, 119, 120, 121, 122, 123, 124, 107, 108, 5, 4, 3};
static uint8_t slave_addr = 0x68; // b110100X; AD0 не подключён => x = 0

// UART буфер
#define RX_BUFFER_SIZE 32
char rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t rx_index = 0;
volatile uint8_t string_received = 0;  // флаг готовности строки

int main()
{
	// включаем HSE осциллятор
	MDR_RST_CLK->HS_CONTROL |= 1;

	// запускаем осциллятор
	while (!(MDR_RST_CLK->CLOCK_STATUS & (1 << 2))) {}

	// инициализируем PLL
	MDR_RST_CLK->PLL_CONTROL |= (5 << RST_CLK_PLL_CONTROL_PLL_CPU_MUL_Pos); // устанавливаем множитель частоты cpu 6
	MDR_RST_CLK->PLL_CONTROL |= (1 << RST_CLK_PLL_CONTROL_PLL_CPU_ON_Pos);	// включаем
	while (!(MDR_RST_CLK->CLOCK_STATUS & (1 << RST_CLK_CLOCK_STATUS_PLL_CPU_RDY_Pos))) {}

	// тактирование; 48 МГц (изначально 8)
	MDR_RST_CLK->CPU_CLOCK |= (2 << RST_CLK_CPU_CLOCK_CPU_C1_SEL_Pos) | (1 << RST_CLK_CPU_CLOCK_CPU_C2_SEL_Pos); // CPU_C1 = HSE | CPU_C2 = PLLCPUo
	MDR_RST_CLK->CPU_CLOCK |= (1 << RST_CLK_CPU_CLOCK_HCLK_SEL_Pos);											 // подаём тактирование CPU_C3 на HSE

	// разрешение тактирования периферии
	MDR_RST_CLK->PER_CLOCK = 0xFFFFFFFF;

	// настройка периферии
	I2C_setup();
	setup_UART1();

	// включение прерываний
	// NVIC_EnableIRQ(I2C_IRQn);
	NVIC_EnableIRQ(UART1_IRQn);

//	uint8_t data_h = 0;
//	uint8_t data_l = 0;
//	int axis = 0;

	// Включение MPU9250
	i2c_write_reg(slave_addr, MPU_REG.PWR_MGMT_1, 1 << 7); // reset
	for (int i = 0; i < 100000; i++)
		__NOP();
	i2c_write_reg(slave_addr, MPU_REG.PWR_MGMT_2, 0); // включение гиро и акселерометра
	for (int i = 0; i < 100000; i++)
		__NOP();

	while (1)
	{
		command_handler();
//			for (axis = 2; axis < 3; axis++) // ИЗМЕНЕНО
//			{
//					data_h = 0;
//					data_l = 0;

//					MPU_read_ACCEL(slave_addr, axis, &data_h, &data_l);

//					// объединяем в 16-битное знаковое число
//					int16_t accel_raw = ((int16_t)data_h << 8) | data_l;

//					if (axis == 0)
//							uart_send_str("X: ");
//					else if (axis == 1)
//							uart_send_str("Y: ");
//					else if (axis == 2)
//							uart_send_str("Z: ");
//					
//					// преобразование в м/с²
//					uart_send_float((float)accel_raw * 9.81f / 16384.0f); // для текущего диапазона измерений +- 2g; не откалиброван!!!
//					uart_send_str("\n\r");
//					
//					for (int i = 0; i < 5000; i++)
//							__NOP();
//			}
//			uart_send_str("========\n\r");
//			for (int i = 0; i < 5000000; i++)
//					__NOP();
	}
}

// =============== настройка I2C ===============
void I2C_setup(void)
{

	MDR_RST_CLK->PER_CLOCK |= (1 << RST_CLK_PER_CLOCK_PCLK_EN_I2C_Pos); // разрешаем тактирование, идёт 48 МГц
	MDR_I2C->PRH = 23;													// делитель частоты (Fscl = HCLK/[5(x+1)] = 400к)
	MDR_I2C->PRL = 0;

	// настройка выводов PC0 (SCL) и PC1 (SDA)
	MDR_PORTC->ANALOG |= (1 << 0) | (1 << 1); // цифровой режим
	MDR_PORTC->FUNC |= (2 << 0);			  // PC0 = SCL
	MDR_PORTC->FUNC |= (2 << 2);			  // PC1 = SDA
	MDR_PORTC->OE |= (1 << 0) | (1 << 1);
	MDR_PORTC->PWR |= (3 << 0) | (3 << 2); // максимально быстрый фронт
	MDR_PORTC->PD |= (1 << 0) | (1 << 1);  // режим открытого стока

	MDR_I2C->CTR |= (1 << I2C_CTR_EN_I2C_Pos); // разрешение работы контроллера I2C
	// MDR_I2C->CTR |= (1 << I2C_CTR_EN_INT_Pos); // разрешение прерываний I2C
}

// =============== запись в регистр по I2C ===============
void i2c_write_reg(const uint8_t addr, uint8_t reg_addr, uint8_t data)
{
	while (MDR_I2C->STA & I2C_STA_BUSY)
		; // ждем шину

	// 1. Отправляем адрес устройства с битом записи
	MDR_I2C->TXD = (addr << 1);
	MDR_I2C->CMD = I2C_CMD_START | I2C_CMD_WR;
	while (MDR_I2C->STA & I2C_STA_TR_PROG)
		;

	// 2. Отправляем адрес регистра
	MDR_I2C->TXD = reg_addr;
	MDR_I2C->CMD = I2C_CMD_WR;
	while (MDR_I2C->STA & I2C_STA_TR_PROG)
		;

	// 3. Отправляем само значение
	MDR_I2C->TXD = data;
	MDR_I2C->CMD = I2C_CMD_WR | I2C_CMD_STOP; // данные + СТОП
	while (MDR_I2C->STA & I2C_STA_TR_PROG)
		;
}

// =============== чтение из регистра по I2C ===============
void i2c_read_reg(const uint8_t addr, uint8_t reg, uint8_t *data_ptr)
{
	// ожидаем, пока шина освободится
	while (MDR_I2C->STA & I2C_STA_BUSY)
		;

	// ========== передача адреса ==========
	uint8_t addr_with_rw = (addr << 1) | 0; // бит записи
	MDR_I2C->TXD = addr_with_rw;
	MDR_I2C->CMD = I2C_CMD_START | I2C_CMD_WR;
	while (MDR_I2C->STA & I2C_STA_TR_PROG)
		; // ожидаем завершения передачи адреса
	// проверяем, был ли получен ACK
	if (MDR_I2C->STA & I2C_STA_RX_ACK)
	{
		// получен NACK
		MDR_I2C->CMD = I2C_CMD_STOP;
		return;
	}

	// ========== передача регистра ==========
	MDR_I2C->CMD = I2C_CMD_CLRINT; // сброс прерывания
	MDR_I2C->TXD = reg;
	MDR_I2C->CMD = I2C_CMD_WR;
	while (MDR_I2C->STA & I2C_STA_TR_PROG)
		; // ожидаем завершения передачи адреса
	// проверяем, был ли получен ACK
	if (MDR_I2C->STA & I2C_STA_RX_ACK)
	{
		// получен NACK
		MDR_I2C->CMD = I2C_CMD_STOP;
		return;
	}

	// ========== запрос на чтение ==========
	MDR_I2C->CMD = I2C_CMD_START; // повторный START без STOP
	while (MDR_I2C->STA & I2C_STA_TR_PROG)
		;

	addr_with_rw = (addr << 1) | 1; // адрес с битом чтения
	MDR_I2C->TXD = addr_with_rw;
	MDR_I2C->CMD = I2C_CMD_WR | I2C_CMD_START;
	while (MDR_I2C->STA & I2C_STA_TR_PROG)
		;
	if (MDR_I2C->STA & I2C_STA_RX_ACK)
	{
		MDR_I2C->CMD = I2C_CMD_STOP;
		return;
	}

	// ========== чтение ==========
	// MDR_I2C->CMD = I2C_CMD_CLRINT;  // сброс прерывания
	MDR_I2C->CMD = I2C_CMD_RD;

	while (MDR_I2C->STA & I2C_STA_TR_PROG);	// ожидаем завершения чтения
	*data_ptr = MDR_I2C->RXD;	// читаем принятые данные
	MDR_I2C->CMD = I2C_CMD_RD | I2C_CMD_ACK | I2C_CMD_STOP; // считать байт и завершить сессию: подтверждение (NACK) + СТОП
}
// =============== чтение с осей акселерометра ===============
void MPU_read_ACCEL(const uint8_t addr, int axis, uint8_t *data_h_ptr, uint8_t *data_l_ptr)
{
	// 0 X
	// 1 Y
	// 2 Z
	switch (axis)
	{
	case 0:
		i2c_read_reg(addr, MPU_REG.ACCEL_XOUT_H, data_h_ptr);
		i2c_read_reg(addr, MPU_REG.ACCEL_XOUT_L, data_l_ptr);
		break;
	case 1:
		i2c_read_reg(addr, MPU_REG.ACCEL_YOUT_H, data_h_ptr);
		i2c_read_reg(addr, MPU_REG.ACCEL_YOUT_L, data_l_ptr);
		break;
	case 2:
		i2c_read_reg(addr, MPU_REG.ACCEL_ZOUT_H, data_h_ptr);
		i2c_read_reg(addr, MPU_REG.ACCEL_ZOUT_L, data_l_ptr);
		break;
	default:
		i2c_read_reg(addr, MPU_REG.ACCEL_XOUT_H, data_h_ptr);
		break;
	}
}

// =============== настройка UART ===============
void setup_UART1(void)
{
	// настройка портов
	MDR_PORTA->ANALOG |= (1 << 6); // RX
	MDR_PORTA->ANALOG |= (1 << 7); // TX
	MDR_PORTA->FUNC |= (3 << 12);  // PA6 как UART1_RX
	MDR_PORTA->FUNC |= (3 << 14);  // PA7 как UART1_TX
	MDR_PORTA->OE &= ~(1 << 6);	   // RX - вход
	MDR_PORTA->OE |= (1 << 7);	   // TX - выход
	MDR_PORTA->PWR |= (3 << 12);   // мощность для PA6
	MDR_PORTA->PWR |= (3 << 14);   // мощность для PA7
	// настройка тактирования для BR (Baud Rate) 115200
	MDR_RST_CLK->PER_CLOCK |= (1 << RST_CLK_PER_CLOCK_PCLK_EN_UART1_Pos);  // разрешаем тактирование периферии
	MDR_RST_CLK->UART_CLOCK |= (1 << RST_CLK_UART_CLOCK_UART1_CLK_EN_Pos); // разрешаем тактирование
	MDR_RST_CLK->UART_CLOCK |= (4 << RST_CLK_UART_CLOCK_UART1_BRG_Pos);	   // делитель 16 (48 МГц / 16 = 3 МГц)
  // настройка BR 115200
  // BAUDDIV = FUARTCLK/(16*BR) = 3 МГц/(16*115200) = 1,628
  // IBRD = 1
  // FBRD = int(0.628*64 + 0.5) = 41
	MDR_UART1->IBRD = 1;
	MDR_UART1->FBRD = 41;
	// количество бит для передачи
	MDR_UART1->LCR_H = 0x60; // Передавать будем 8 бит
	// регистр управления
	MDR_UART1->CR |= UART_CR_RXE;	 // разрешить чтение данных
	MDR_UART1->CR |= UART_CR_TXE;	 // разрешить отправку данных
	MDR_UART1->CR |= UART_CR_UARTEN; // разрешить приёмо-передачу
	// настройка прерываний
	MDR_UART1->IMSC |= UART_IMSC_RXIM; // прерывание по приёму данных
}

// =============== отправка строки по UART ===============
void uart_send_str(char *str)
{
	while (*str)
	{
		MDR_UART1->DR = *str++;
		while (!(MDR_UART1->FR & UART_FR_TXFE))
			;
	}
}

// =============== отправка int (DEC) по UART ===============
void uart_send_int(int16_t num)
{
	char buffer[10];
	int i = 0;

	// обработка отрицательных чисел
	if (num < 0)
	{
		MDR_UART1->DR = '-';
		while (!(MDR_UART1->FR & UART_FR_TXFE))
			;
		num = -num;
	}
	if (num == 0)
	{
		buffer[i++] = '0';
	}
	else
	{
		while (num > 0)
		{
			buffer[i++] = (num % 10) + '0';
			num /= 10;
		}
	}
	// вывод цифр в обратном порядке
	while (i > 0)
	{
		MDR_UART1->DR = buffer[--i];
		while (!(MDR_UART1->FR & UART_FR_TXFE))
			;
	}
}

// =============== отправка float (DEC) по UART ===============
void uart_send_float(float num)
{
    int16_t int_part = (int16_t)num;
    int16_t frac_part = (int16_t)((num - int_part) * 100); // 2 знака после запятой
    
    // отправляем целую часть
    uart_send_int(int_part);
    
    // отправляем точку
    MDR_UART1->DR = '.';
    while (!(MDR_UART1->FR & UART_FR_TXFE));
    
    // Отправляем дробную часть (всегда 2 цифры)
    if (frac_part < 0) frac_part = -frac_part;
    if (frac_part < 10) {
        MDR_UART1->DR = '0';
        while (!(MDR_UART1->FR & UART_FR_TXFE));
    }
    uart_send_int(frac_part);
}
// =============== обработчик прерываний UART ===============
void UART1_IRQHandler(void)
{
	char c = 0;
	// проверяем, что прерывание действительно от приёмника
	if (MDR_UART1->MIS & UART_MIS_RXMIS)
	{
		c = MDR_UART1->DR;
		if (c == '\n' || c == '\r')
			{
					if (rx_index > 0)
					{
							// добавляем завершающий ноль для строки
							if (rx_index < RX_BUFFER_SIZE - 1)
									rx_buffer[rx_index] = '\0';
							else
									rx_buffer[RX_BUFFER_SIZE - 1] = '\0';
							
							string_received = 1;  // устанавливаем флаг готовности
							rx_index = 0;         // сбрасываем индекс для следующей строки
					}
			}
		// обычный символ (не конец строки)
		else if (rx_index < RX_BUFFER_SIZE - 1)
		{
				rx_buffer[rx_index++] = c;
		}
		else
		{
				// переполнение буфера - сбрасываем
				rx_index = 0;
		}
	}
	MDR_UART1->ICR |= UART_ICR_RXIC; // сброс флага прерывания
}

// =============== обработчик UART-команд ===============
void command_handler(void){
	if (!string_received)
		return;
	string_received = 0;
	
	// парсинг команды
	char buffer[RX_BUFFER_SIZE];
	strcpy(buffer, rx_buffer);
	
	char *cmd = strtok(buffer, " ");
	char *arg_str = strtok(NULL, " ");
	if (cmd == NULL)
		return;
	
	if (strcmp(cmd, "acc_offset_x") == 0){
		if (arg_str != NULL){
			float arg = (float)atof(arg_str);
			uint8_t cfg;
			i2c_read_reg(slave_addr, MPU_REG.ACCEL_CONFIG, &cfg);
			uint8_t range_bits = (cfg & 0x18) >> 3; // Маска 0b00011000
			float sensitivity = 16384.0f / (float)(1 << range_bits);
			
			// расчет смещения (важно: для акселерометра MPU это обычно 15-битное число)
			int16_t offset_val = (int16_t)(arg / 9.81f * sensitivity);
			
			uint8_t arg_h = (uint8_t)((offset_val >> 8) & 0xFF);
			// сохраняем бит 0 (он часто зарезервирован или OTP), меняем только [7:1]
			uint8_t current_l;
			i2c_read_reg(slave_addr, MPU_REG.XA_OFFS_L, &current_l); 
			uint8_t arg_l = (uint8_t)(offset_val & 0xFE) | (current_l & 0x01);
			
			i2c_write_reg(slave_addr, MPU_REG.XA_OFFS_H, arg_h);
			i2c_write_reg(slave_addr, MPU_REG.XA_OFFS_L, arg_l);
		}
	}
	
	// ========== ACC_OFFSET_X ==========
	// РАБОТАЕТ, НО КАКАЯ-ТО НАСТРОЙКА НЕКОРРЕКТНАЯ
	if (strcmp(cmd, "acc_offset_x") == 0){
		if (arg_str != NULL){
			float arg = (float)atof(arg_str);
			// преобразование: g -> единицы МЗР (LSB)
			uint8_t cfg;
			i2c_read_reg(slave_addr, MPU_REG.ACCEL_CONFIG, &cfg); // считываем диапазон с датчика 2g(00) , 4g(01), 8g(10), 16g(11)
			uint8_t range_bits = (cfg & 0b00011000) >> 3 ;
			float sensitivity = 16384.0f / (float)(1 << range_bits);
			int16_t offset_val = (int16_t)(arg / 9.81f * sensitivity);
			
			uint8_t arg_h = (uint8_t)((offset_val >> 8) & 0xFF);
			uint8_t arg_l = (uint8_t)(offset_val & 0xFE);
			
			i2c_write_reg(slave_addr, MPU_REG.XA_OFFS_H, arg_h);
			i2c_write_reg(slave_addr, MPU_REG.XA_OFFS_L, arg_l);
		}
	}
	// ========== ACC_OFFSET_Y ==========
	else if (strcmp(cmd, "acc_offset_y") == 0){
	if (arg_str != NULL){
			double arg = atof(arg_str);
			return; //ДОДЕЛАТЬ для остальных функций
		}
	}
	// ========== ACC_OFFSET_Z ==========
	else if (strcmp(cmd, "acc_offset_z") == 0){
	if (arg_str != NULL){
			double arg = atof(arg_str);
			return; //ДОДЕЛАТЬ для остальных функций
		}
	}
	// ========== ASK_ACC ==========
	else if (strcmp(cmd, "ask_acc") == 0){
		uint8_t data_h, data_l;
		for (int axis = 0; axis < 3; axis++)
		{
				data_h = 0;
				data_l = 0;
				MPU_read_ACCEL(slave_addr, axis, &data_h, &data_l);

				if (axis == 0)
						uart_send_str("X: ");
				else if (axis == 1)
						uart_send_str("Y: ");
				else if (axis == 2)
						uart_send_str("Z: ");
				
				uart_send_float(convert_to_ms2(&data_h, &data_l)); // для текущего диапазона измерений +- 2g; не откалиброван!!!
				uart_send_str("\n\r");
				
				for (int i = 0; i < 5000; i++)
						__NOP();
		}
	}
}

// =============== перевод ускорения с акселерометра ===============
float convert_to_ms2(const uint8_t *data_h, const uint8_t *data_l){
	int16_t data_raw = ((int16_t)(*data_h) << 8) | (*data_l);
	// преобразование: g -> единицы МЗР (LSB)
	uint8_t cfg;
	i2c_read_reg(slave_addr, MPU_REG.ACCEL_CONFIG, &cfg); // считываем диапазон с датчика 2g(00) , 4g(01), 8g(10), 16g(11)
	uint8_t range_bits = (cfg & 0b00011000) >> 3 ;
	float sensitivity = 16384.0f / (float)(1 << range_bits);
	
	return (float)data_raw * 9.81f / sensitivity;
}
