#include "lcd.h"

// Инициализация дисплея
void LCD_init(void)
{
	HAL_Delay(40);		// задержка необходимая для инициализации микросхемы HD44780 после подачи питания
	rs0;
	// Первая запись после включения питания воспринимается микросхемой как 8-битная,
	// то есть биты D3,D2,D1,D0 могут принять случайный мусор, т.к. висят в воздухе.
	// ТОЛЬКО на этой первой передаче будет прочитан бит DB4==0, и далее уже все
	// команды будут восприниматься дисплеем как 4-битные.
	// Для дисплея эта команда будет равнозначна 0011ХХХХ (DB7..DB0)
	
	// Далее нужна именно такая последовательность команд, иначе инициализация дисплея проходит некорректно
	LCD_WriteData(3);
	E_Impulse();
	HAL_Delay(5);

	LCD_WriteData(3);
	E_Impulse();
	HAL_Delay(5);

	LCD_WriteData(3);
	E_Impulse();
	HAL_Delay(1);
	
	LCD_WriteData(2);
	E_Impulse();
	HAL_Delay(1);
	
	// С этого момента все команды уже будут считываться дисплеем за 2 такта
	LCD_Command(0x28);	//режим 4 бит, 2 линии, шрифт 5х8
	HAL_Delay(1);
	LCD_Command(0x0C);	// включаем дисплей (D=1), без курсора и моргания над ним
	HAL_Delay(1);
	LCD_Command(0x01);	// очистка дисплея
	HAL_Delay(2);
	LCD_Command(0x06);	// курсор будет двигаться вправо на 1 символ после очередной записи данных
	HAL_Delay(1);
	LCD_Command(0x02);	// возврат курсора в нулевое положение
	HAL_Delay(2);	// ждём 2 мс, т.к. эта операция по документации выполняется 1.5мс
}


// Вывод младших 4 бит 8-битных данных на шину D4..D7
void LCD_WriteData(uint8_t dt)
{
	if(dt & (1<<3)) d7_set(); else d7_reset();
	if(dt & (1<<2)) d6_set(); else d6_reset();
	if(dt & (1<<1)) d5_set(); else d5_reset();
	if(dt & (1<<0)) d4_set(); else d4_reset();
}


// Генерация импульса на выводе Е
void E_Impulse(void)
{
	e1;
	delay();
	e0;
	delay();
}


// Отправка команды дисплею
void LCD_Command(uint8_t dt)
{
	rs0;
	LCD_WriteData(dt>>4);
	E_Impulse();
	LCD_WriteData(dt);
	E_Impulse();
}


// Запись данных в дисплей
void LCD_Data(uint8_t dt)
{
	rs1;
	LCD_WriteData(dt>>4);
	E_Impulse();
	LCD_WriteData(dt);
	E_Impulse();
}


// Команда очистки дисплея
void LCD_Clear(void)
{
	LCD_Command(0x01);
	HAL_Delay(2);
}


// Вывод символа на дисплей
void LCD_SendChar(char ch)
{
	LCD_Data((uint8_t) ch);
	delay();
}


// Вывод строки на дисплей
void LCD_String(char* st)
{
	uint8_t i=0;
	// Ограничение в 16 выводимых символов чтобы не выходило за рамки строки
	while((st[i]!=0) && (i < 16))
	{
		LCD_Data(st[i]);
		delay();
		i++;
	}
}


// Функция позиционирования по столбцу и строке
void LCD_SetPos(uint8_t x, uint8_t y)
{
	switch(y)
	{
		case 0:
			LCD_Command(x|0x80);
			delay();
			break;
		case 1:
			LCD_Command((0x40+x)|0x80);
			delay();
			break;
	}
}
