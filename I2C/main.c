#include "MDR32FxQI_port.h"
#include "MDR32FxQI_rst_clk.h"
#include "MDR32FxQI_i2c.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

#define M_PI 3.1415926535f //Никита добавил
#define DT 0.02f      // Время одного цикла (подбери под свои задержки)//Никита добавил хер знает надо ли
#define GYRO_SENS 131.0f // Чувствительность для режима по умолчанию //Никита добавил

float current_yaw = 0.0f; //Никита добавил

void I2C_setup(void);
void I2C_IRQHandler(void);
// void I2C_IRQHandler(void)
void UART1_IRQHandler(void);
void setup_UART1(void);

void i2c_write_reg(const uint8_t device_addr, uint8_t reg_addr, uint8_t data);
void i2c_read_reg(const uint8_t addr, uint8_t reg, uint8_t *data_ptr);
void i2c_write_2reg(const uint8_t addr, uint8_t reg_addr, uint8_t h, uint8_t l);
void uart_send_str(char *str);
void uart_send_int(int16_t num);
void uart_send_float(float num);

void MPU_read_ACCEL(const uint8_t addr, int axis, uint8_t *data_h_ptr, uint8_t *data_l_ptr);
void MPU_read_GYRO (const uint8_t addr, int axis, uint8_t *data_h_ptr, uint8_t *data_l_ptr); //Никита добавил
float convert_to_ms2(const uint8_t *data_h, const uint8_t *data_l);
void transform_to_global(int16_t ax, int16_t ay, int16_t az, int16_t gz_raw, float *gx, float *gy, float *gz);


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
	const uint8_t GYRO_CONFIG;
	const uint8_t GYRO_XOUT_H; //Никита добавил + регистры от 67 до 72
  const uint8_t GYRO_XOUT_L; //Никита добавил + регистры от 67 до 72
  const uint8_t GYRO_YOUT_H; //Никита добавил + регистры от 67 до 72
	const uint8_t GYRO_YOUT_L; //Никита добавил + регистры от 67 до 72
  const uint8_t GYRO_ZOUT_H; //Никита добавил + регистры от 67 до 72
  const uint8_t GYRO_ZOUT_L; //Никита добавил + регистры от 67 до 72
};
//                                                                 119 120
//                                                                 0x06 0x07
static struct MPU_REGISTERS MPU_REG = {59, 60, 61, 62, 63, 64, 28, 119, 120, 121, 122, 123, 124, 107, 108, 5, 4, 3, 27, 67, 68, 69, 70, 71, 72};
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

	// Включение MPU9250
	i2c_write_reg(slave_addr, MPU_REG.PWR_MGMT_1, 1 << 7); // reset
	for (int i = 0; i < 100000; i++)
		__NOP();
	i2c_write_reg(slave_addr, MPU_REG.PWR_MGMT_2, 0); // включение гиро и акселерометра
	for (int i = 0; i < 100000; i++)
		__NOP();
	i2c_write_reg(slave_addr, MPU_REG.GYRO_CONFIG, 0);

	uint8_t data;
	i2c_read_reg(slave_addr, 0x75, &data);
	uart_send_int(data);
	while (1)
	{
		command_handler();

    uint8_t data_h, data_l;
    int16_t ax, ay, az, gz_raw;

    // Читаем акселерометр по всем осям
    MPU_read_ACCEL(slave_addr, 0, &data_h, &data_l);
		ax = (int16_t)((data_h << 8) | data_l);
		
    MPU_read_ACCEL(slave_addr, 1, &data_h, &data_l);
		ay = (int16_t)((data_h << 8) | data_l);
		
    MPU_read_ACCEL(slave_addr, 2, &data_h, &data_l);
		az = (int16_t)((data_h << 8) | data_l);

    // Читаем гироскоп по оси Z
    MPU_read_GYRO(slave_addr, 2, &data_h, &data_l);
		gz_raw = (int16_t)((data_h << 8) | data_l);

    float global_x, global_y, global_z;

    // Используем матрицу перехода
    transform_to_global(ax, ay, az, gz_raw, &global_x, &global_y, &global_z);

    // Выводим результат
//    uart_send_str("G_X: "); uart_send_float(global_x * 9.81f / 16384.0f);
//    uart_send_str(" G_Y: "); uart_send_float(global_y * 9.81f / 16384.0f);
//    uart_send_str(" G_Z: "); uart_send_float(global_z * 9.81f / 16384.0f);
//    uart_send_str("\r\n");

    // Задержка (важно, чтобы совпадала с DT из шага 1)
    for (int i = 0; i < 200000; i++) {
			__NOP(); }
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
	while (MDR_I2C->STA & I2C_STA_BUSY);

	// ========== передача адреса ==========
	uint8_t addr_with_rw = (addr << 1) | 0; // бит записи
	MDR_I2C->TXD = addr_with_rw;
	MDR_I2C->CMD = I2C_CMD_START | I2C_CMD_WR;
	while (MDR_I2C->STA & I2C_STA_TR_PROG); // ожидаем завершения передачи адреса
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
	while (MDR_I2C->STA & I2C_STA_TR_PROG); // ожидаем завершения передачи адреса
	// проверяем, был ли получен ACK
	if (MDR_I2C->STA & I2C_STA_RX_ACK)
	{
		// получен NACK
		MDR_I2C->CMD = I2C_CMD_STOP;
		return;
	}

	// ========== запрос на чтение ==========
	MDR_I2C->CMD = I2C_CMD_START; // повторный START без STOP
	while (MDR_I2C->STA & I2C_STA_TR_PROG);

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
	// Мы уже отправили адрес устройства с битом READ и получили ACK.
	MDR_I2C->CMD = I2C_CMD_RD | I2C_CMD_ACK | I2C_CMD_STOP;	// ЧИТАТЬ + НЕ ДАВАТЬ ACK + СТОП

	// Ждем окончания физического процесса чтения байта
	while (MDR_I2C->STA & I2C_STA_TR_PROG);

	// Только после этого забираем данные из регистра
	*data_ptr = (uint8_t)(MDR_I2C->RXD);
}
// =============== запись в 2 регистра по I2C ===============
void i2c_write_2reg(const uint8_t addr, uint8_t reg_addr, uint8_t h, uint8_t l)
{
    while (MDR_I2C->STA & I2C_STA_BUSY);

    MDR_I2C->TXD = (addr << 1);
    MDR_I2C->CMD = I2C_CMD_START | I2C_CMD_WR;
    while (MDR_I2C->STA & I2C_STA_TR_PROG);

    MDR_I2C->TXD = reg_addr;
    MDR_I2C->CMD = I2C_CMD_WR;
    while (MDR_I2C->STA & I2C_STA_TR_PROG);

    MDR_I2C->TXD = h; // Записываем High
    MDR_I2C->CMD = I2C_CMD_WR;
    while (MDR_I2C->STA & I2C_STA_TR_PROG);

    MDR_I2C->TXD = l; // Записываем Low + STOP
    MDR_I2C->CMD = I2C_CMD_WR | I2C_CMD_STOP;
    while (MDR_I2C->STA & I2C_STA_TR_PROG);
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

// =============== чтение с осей гироскопа ===============  //Никита добавил
void MPU_read_GYRO(const uint8_t addr, int axis, uint8_t *data_h_ptr, uint8_t *data_l_ptr)
{
	// 0 X
	// 1 Y
	// 2 Z
	switch (axis)
	{
	case 0:
		i2c_read_reg(addr, MPU_REG.GYRO_XOUT_H, data_h_ptr);
		i2c_read_reg(addr, MPU_REG.GYRO_XOUT_L, data_l_ptr);
		break;
	case 1:
		i2c_read_reg(addr, MPU_REG.GYRO_YOUT_H, data_h_ptr);
		i2c_read_reg(addr, MPU_REG.GYRO_YOUT_L, data_l_ptr);
		break;
	case 2:
		i2c_read_reg(addr, MPU_REG.GYRO_ZOUT_H, data_h_ptr);
		i2c_read_reg(addr, MPU_REG.GYRO_ZOUT_L, data_l_ptr);
		break;
	default:
		i2c_read_reg(addr, MPU_REG.GYRO_XOUT_H, data_h_ptr);
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
		while (!(MDR_UART1->FR & UART_FR_TXFE));
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
	
	// ========== ACC_OFFSET_X ==========
if (strcmp(cmd, "acc_offset_x") == 0) {
// АППАРАТНАЯ НАСТРОЙКА НЕ РАБОТАЕТ
//    if (arg_str != NULL) {
//        float arg_ms2 = (float)atof(arg_str);
//        // Используем 2048 LSB/g. Пока БЕЗ минуса, проверим реакцию.
//        int16_t offset_lsb = (int16_t)((arg_ms2 / 9.81f) * 2048.0f); 

//        uint8_t cur_h, cur_l;
//        i2c_read_reg(slave_addr, MPU_REG.XA_OFFS_H, &cur_h);
//        i2c_read_reg(slave_addr, MPU_REG.XA_OFFS_L, &cur_l);

//        // Формируем 15-битное значение вручную
//        // Нам нужно: [Bit 15...Bit 1] -> данные, [Bit 0] -> Reserved
//        uint16_t val_to_send = (uint16_t)(offset_lsb << 1); 
//        
//        uint8_t new_h = (uint8_t)((val_to_send >> 8) & 0xFF);
//        uint8_t new_l = (uint8_t)(val_to_send & 0xFE) | (cur_l & 0x01);

//        // ВАЖНО: Записываем сначала H, потом L
//        i2c_write_reg(slave_addr, MPU_REG.XA_OFFS_H, new_h);
//        i2c_write_reg(slave_addr, MPU_REG.XA_OFFS_L, new_l);

//        // ОТЛАДКА: выведи в терминал, что реально улетело в чип
//        uart_send_str("Sent H:"); uart_send_int(new_h);
//        uart_send_str(" L:"); uart_send_int(new_l);
//        uart_send_str("\r\n");
//    }
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
				
				uart_send_float(convert_to_ms2(&data_h, &data_l));
				uart_send_str("\n\r");
				
				for (int i = 0; i < 5000; i++)
						__NOP();
		}
	}
	// ========== ACC_RESET ==========
	else if (strcmp(cmd, "acc_reset") == 0) {
// АППАРАТНАЯ НАСТРОЙКА НЕ РАБОТАЕТ
//    // Пишем во все регистры нули (сохраняя бит Reserved в L)
//    uint8_t cl;
//    i2c_read_reg(slave_addr, 120, &cl);
//    i2c_write_2reg(slave_addr, 119, 0, (cl & 0x01)); // X
//    
//    i2c_read_reg(slave_addr, 122, &cl);
//    i2c_write_2reg(slave_addr, 121, 0, (cl & 0x01)); // Y
//    
//    i2c_read_reg(slave_addr, 124, &cl);
//    i2c_write_2reg(slave_addr, 123, 0, (cl & 0x01)); // Z
//    
//    uart_send_str("All offsets cleared\r\n");
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

// =============== Переход в глобальные оси ===============
void transform_to_global(int16_t ax, int16_t ay, int16_t az, int16_t gz_raw, float *gx, float *gy, float *gz) {
  // ax - значение акселерометра по x, ay - по y, az - по z (без обработки)
	// gz_raw - значение гироскопа по z (без обработки)
	// 1. Считаем углы наклона по акселерометру
    float roll = atan2f((float)ay, (float)az);
    float pitch = atan2f(-(float)ax, sqrtf((float)ay * ay + (float)az * az));
	
    // 2. Интегрируем скорость гироскопа в угол Yaw
    float gz_dps = (float)gz_raw / GYRO_SENS;
    if (fabs(gz_dps) > 0.5f) { // чтобы не плыло в покое
        current_yaw += (gz_dps * (M_PI / 180.0f)) * DT;
    }

    // 3. Матрица поворота (перевод из локальных осей в глобальные)
    float sr = sinf(roll);  float cr = cosf(roll);
    float sp = sinf(pitch); float cp = cosf(pitch);
    float sy = sinf(current_yaw); float cy = cosf(current_yaw);

    float lax = (float)ax; float lay = (float)ay; float laz = (float)az;

    *gx = (cp * cy) * lax + (sr * sp * cy - cr * sy) * lay + (cr * sp * cy + sr * sy) * laz;
    *gy = (cp * sy) * lax + (sr * sp * sy + cr * cy) * lay + (cr * sp * sy - sr * cy) * laz;
    *gz = (-sp) * lax + (sr * cp) * lay + (cr * cp) * laz;
}