/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t pause = 100;

#include "main.h"
#include "n1202.h"
#include "stdlib.h"

char str1[100];

void FloatToStringNew(char *str, float f, char size)

{

char pos;  // position in string

    char len;  // length of decimal part of result

    char* curr;  // temp holder for next digit

    int value;  // decimal digit(s) to convert

    pos = 0;  // initialize pos, just to be sure

    value = (int)f;  // truncate the floating point number
    itoa(value,str, 10);  // this is kinda dangerous depending on the length of str
    // now str array has the digits before the decimal

    if (f < 0 )  // handle negative numbers
    {
        f *= -1;
        value *= -1;
    }

     len = strlen(str);  // find out how big the integer part was
    pos = len;  // position the pointer to the end of the integer part
    str[pos++] = '.';  // add decimal point to string

    while(pos < (size + len + 1) )  // process remaining digits
    {
        f = f - (float)value;  // hack off the whole part of the number
        f *= 10;  // move next digit over
        value = (int)f;  // get next digit
        itoa(value, curr, 10); // convert digit to string
        str[pos++] = *curr; // add digit to result string and increment pointer
    }
 }

//***********************
//Display
//***********************


//#define soft_SPI //реализация софтверного SPI если ваш контроллер не поддерживает работу в 9 битнои режиме
#define HAL	//если используем HAL раскомментируйте эту строку
#define RESET_Pin Reset_Pin


#define CS_GPIO_Port GPIOA
#define RESET_GPIO_Port GPIOA

#define MOSI_GPIO_Port GPIOB
#define SCK_GPIO_Port GPIOA


uint8_t _LCD_RAM[LCD_X*9]; // Память нашего LCD

void delay_us ( uint32_t us )
{
volatile uint32_t delay = (us * (SystemCoreClock / 1000000)/8);
while (delay--);
}

void delay_ms ( uint32_t ms )
{
volatile uint32_t delay = (ms * (SystemCoreClock / 1000)/8);
while (delay--);
}


void LCD_SendByte(uint8_t mode, uint8_t c)
	{
		uint8_t SPI_Data[2];


		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
		SPI_Data[0] = c;
		SPI_Data[1] = mode;
		HAL_SPI_Transmit(&hspi1, SPI_Data, 1, 5);
		//delay_us(1);
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	}

	// Очистка памяти дисплея
void LCD_Clear(void) {
	 for (int index = 0; index < 864 ; index++){
	_LCD_RAM[index] = (0x00);
	 }
}

// Обновляем данные на экране
void LCD_Update(void) {
  for(uint8_t p = 0; p < 9; p++) {
    LCD_SendByte(LCD_C, SetYAddr | p);
    LCD_SendByte(LCD_C, SetXAddr4);
    LCD_SendByte(LCD_C, SetXAddr3);
    for(uint8_t col=0; col < LCD_X; col++){
      LCD_SendByte(LCD_D, _LCD_RAM[(LCD_X * p) + col]);
    }
  }
}

// Рисование пикселя по координатам и цвету
void LCD_DrawPixel (uint8_t x, uint8_t y, uint8_t color) {
  if ((x < 0) || (x >= LCD_X) || (y < 0) || (y >= LCD_Y)) return;

  if (color) _LCD_RAM[x+ (y/8)*LCD_X] |= 1<<(y%8);
  else       _LCD_RAM[x+ (y/8)*LCD_X] &= ~(1<<(y%8));
}

// Рисование линии
void LCD_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color) {
  int steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    swap(x0, y0);
    swap(x1, y1);
  }
  if (x0 > x1) {
    swap(x0, x1);
    swap(y0, y1);
  }
  int dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);
  int err = dx / 2;
  int ystep;
  if (y0 < y1) {ystep = 1;}
  else {ystep = -1;};
  for ( ; x0 <= x1; x0++) {
    if (steep) {LCD_DrawPixel(y0, x0, color);}
    else {LCD_DrawPixel(x0, y0, color);};
		err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}

// Рисование вертикальной линии
void LCD_DrawFastVLine(uint8_t x, uint8_t y, uint8_t h, uint8_t color) {
  LCD_DrawLine(x, y, x, y+h-1, color);
}

// Рисование горизонтальной линии
void LCD_DrawFastHLine(uint8_t x, uint8_t y, uint8_t w, uint8_t color) {
  LCD_DrawLine(x, y, x+w-1, y, color);
}

// Рисование прямоугольника
void LCD_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color) {
  LCD_DrawFastHLine(x, y, w, color);
  LCD_DrawFastHLine(x, y+h-1, w, color);
  LCD_DrawFastVLine(x, y, h, color);
  LCD_DrawFastVLine(x+w-1, y, h, color);
}

// Рисование залитый прямоугольник
void LCD_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color) {
  for (int16_t i=x; i<x+w; i++) {
    LCD_DrawFastVLine(i, y, h, color);
  }
}

// Заливка экрана
void LCD_FillScreen(uint8_t color) {
  LCD_FillRect(0, 0, LCD_X, LCD_Y, color);
}

// Нарисовать букву
void LCD_DrawChar(uint8_t x, uint8_t y, uint8_t color, unsigned char c) {
  if((x >= LCD_X) ||(y >= LCD_Y) || ((x + 4) < 0) || ((y + 7) < 0)) return;
  if(c<128)            c = c-32;
  if(c>=144 && c<=175) c = c-48;
  if(c>=128 && c<=143) c = c+16;
  if(c>=176 && c<=191) c = c-48;
  if(c>191)  return;
  for (uint8_t i=0; i<6; i++ ) {
    uint8_t line;
    if (i == 5) {line = 0x00;}
    else {line = font[(c*5)+i];
    for (uint8_t j = 0; j<8; j++)
			{
				if (line & 0x01) {LCD_DrawPixel(x+i, y+j, color);}
				else {LCD_DrawPixel(x+i, y+j, !color);};
				line >>= 1;
			}
		}
  }
}

// Вывод строки
void LCD_print(uint8_t x, uint8_t y, uint8_t color, char *str) {
  unsigned char type = *str;
  if(type>=128) x=x-3;
  while(*str){
    LCD_DrawChar(x, y, color, *str++);
    unsigned char type = *str;
    if (type>=128) {x=x+3;}
    else {x=x+6;};
  }
}

/*
// Вывод числовых значений
// закомментировал ибо  много памяти сжирает
void LCD_write(uint8_t x, uint8_t y, uint8_t color, float num){
  char c[10];
//	sprintf(c, "text %f\n", num);
	sprintf(c, "%5.2f", num);
  LCD_print(x, y, color, c);
}
*/
static const char fillDrop[] = {
	0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,
	0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,
	0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,
	0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00,
};
static const char unFillDrop[] = {
	0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
	0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
	0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
	0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00,
};

static const char halfFillDrop[] = {
	0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00,
	0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00,
	0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00,
	0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00,
	0x00, 0x00, 0x01, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00,
};



// Вывод картинки
void LCD_DrawBitmap(uint8_t x, uint8_t y, const char *bitmap, uint8_t w, uint8_t h, uint8_t color) {
	for (int16_t i=0; i < h; i++)
    {
		for (int16_t j=0; j < w; j++ )
		{
			if (bitmap[i*w + j] >= 0x01)
			{
				LCD_DrawPixel(x+j, y+i, color);
			}
		}
    }
}

// Очистка области
void LCD_ClearBitmap(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color) {
	for (int16_t i=0; i < h; i++)
    {
		for (int16_t j=0; j < w; j++ )
		{
			LCD_DrawPixel(x+j, y+i, color);
		}
    }
}
// �?нициализируем дисплей
void LCD_Init(void) {
  // �?нициализация дисплея
	HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET); // Активируем ресет
	delay_ms(10);
	HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);   // Деактивируем ресет

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);			 // Выбираем дисплей
	// Задержка
  	delay_ms(5);
  	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);			 // Выбираем дисплей
	delay_ms(1);
	LCD_SendByte(LCD_C, 0xE2);  // Сброс чипа
	LCD_SendByte(LCD_C, 0xE2);  // Сброс чипа ещё раз, глюк аппаратного spi

	delay_ms(5);
  // Устанавливаем энергию заряда сегмента
	LCD_SendByte(LCD_C, 0x3D);  // Умножитель энергии заряда
	LCD_SendByte(LCD_C, 0x02); 	// Не понятное значение умножителя
  // Команда и следом данные по контрастности
	LCD_SendByte(LCD_C, 0xE1);  // Additional VOP for contrast increase
 	LCD_SendByte(LCD_C, 0x90);  // from -127 to +127
  // Устанавливаем режим работы Normal
 	LCD_SendByte(LCD_C, 0xA4);  // Power saver off

	LCD_SendByte(LCD_C, 0x2F);  // Booster ON Voltage regulator ON Voltage follover ON
	LCD_SendByte(LCD_C, 0xA0);  // Segment driver direction select: Normal
	LCD_SendByte(LCD_C, 0xAF);  // Включение дисплея
	//LCD_SendByte(LCD_C, 0xAD);
	delay_ms(10);
  // Очищаем, обновляем

  LCD_Clear();
  LCD_Update();
}

void drawWiFi(int countDiv)
{
	if(countDiv > 4)
	{
#ifdef MY_DEBUG
		assert(!(countDiv > 4));
#else
		countDiv = 4;
#endif
	}
	else if(countDiv < 0)
	{
#ifdef MY_DEBUG
		assert(!(countDiv < 0));
#else
		countDiv = 0;
#endif
	}

	LCD_DrawFastVLine(5, 1, 9, 1);
	LCD_DrawFastHLine(1, 1, 9, 1);

	LCD_DrawPixel(2, 2, 1);
	LCD_DrawPixel(3, 3, 1);
	LCD_DrawPixel(4, 4, 1);

	LCD_DrawPixel(6, 4, 1);
	LCD_DrawPixel(7, 3, 1);
	LCD_DrawPixel(8, 2, 1);

	if(countDiv != 0)
	{
		int startX = 8;
		int stepX = 3;
		int startY = 7;
		int stepY = 2;
		int endY = 3;
		for(int i = 0; i < countDiv; i++)
		{
			int step = i * stepY;
			LCD_DrawFastVLine(startX + i*stepX, startY - step, endY + step, 1);
		}
	}
	else
	{
		int size = 7;
		for(int i = 0; i < size; i++)
		{
			for(int j = 0; j < size; j++)
			{
				if(i == j)
					LCD_DrawPixel(10 + i, 3 + j, 1);
				if(i + j == size - 1)
					LCD_DrawPixel(10 + i, 3 + j, 1);
			}
		}
	}
}

void LCD_ClearWifi() {
	int x = 1;
	int y = 1;
	int w = 18;
	int h = 11;
	int color = 0;
	for (int16_t i=0; i < h; i++)
    {
		for (int16_t j=0; j < w; j++ )
		{
			LCD_DrawPixel(x+j, y+i, color);
		}
    }
}

void drawStream()
{
  	for(int i = 0; i < 6; i++)
  	{
  		int stepY = 10;
  		for(int j = 0; j < 4; j++)
  		{
  			LCD_DrawFastVLine(j*2 ,2 + j*2 + i*stepY, 4 + j*2, 1);
  			LCD_Update();
  		}

  		for(int j = 3; j >= 0; j--)
  		{
  			LCD_DrawFastVLine(16 - j*2 , 2 + j*2 + i*stepY, 4 + j*2, 1);
  			LCD_Update();
  		}


  	}
  	return;
}


void LCD_Battery(int countDiv)
{
	if(countDiv > 4)
	{
#ifdef MY_DEBUG
		assert(!(countDiv > 4));
#else
		countDiv = 4;
#endif
	}
	else if(countDiv < 0)
	{
#ifdef MY_DEBUG
		assert(!(countDiv < 0));
#else
		countDiv = 1;
#endif
	}

	LCD_DrawRect(96-18, 2, 15, 8, 1);
	LCD_DrawPixel(96-2, 3, 1);
	LCD_DrawPixel(96-2, 8, 1);
	LCD_DrawPixel(96-3, 3, 1);
	LCD_DrawPixel(96-3, 8, 1);

	LCD_DrawPixel(96-2, 3, 1);
	LCD_DrawPixel(96-2, 4, 1);
	LCD_DrawPixel(96-2, 5, 1);
	LCD_DrawPixel(96-2, 6, 1);
	LCD_DrawPixel(96-2, 7, 1);
	LCD_DrawPixel(96-2, 8, 1);

	int offsetX = 16;
	for(int i = 0; i < countDiv; i++)
	{
		LCD_FillRect(LCD_X - offsetX + i*3, 4, 2, 4, 1);
	}
}

void LCD_ClearBattery()
{
	int offsetX = 18;
	int x = LCD_X - offsetX;
	int y = 1;
	int w = 17;
	int h = 9;
	int color = 0;
	for (int16_t i=0; i < h; i++)
    {
		for (int16_t j=0; j < w; j++ )
		{
			LCD_DrawPixel(x+j, y+i, color);
		}
    }
}

int flag = 0;

void testDisplay()
{
	//LCD_Clear();
	LCD_print(0, 30, 1, "17:34 24/03/2021");
	LCD_Update();

	LCD_ClearWifi();
	drawWiFi(4);

	LCD_DrawBitmap(19, 1, fillDrop, 9, 9, 1);
	LCD_DrawBitmap(27, 1, unFillDrop, 9, 9, 1);
	LCD_DrawBitmap(36, 1, halfFillDrop, 9, 9, 1);

	LCD_ClearBattery();
	LCD_Battery(4);

	LCD_Update();
	/*if(flag == 0)
	{
		LCD_DrawBitmap(19, 1, fillDrop, 9, 9, 1);
		LCD_DrawBitmap(27, 1, unFillDrop, 9, 9, 1);
		LCD_DrawBitmap(36, 1, halfFillDrop, 9, 9, 1);
		flag = 1;
	}
	else
	{
		LCD_ClearBitmap(19, 1, 7, 7, 0);
		LCD_ClearBitmap(27, 1, 7, 7, 0);
		LCD_ClearBitmap(36, 1, 7, 7, 0);
		flag = 0;
	}
	LCD_Update();
	*/
	HAL_Delay(1000);
	//Test Wifi
	LCD_ClearWifi();
	drawWiFi(4);
	LCD_print(0, 30, 1, "4 division");
	LCD_Update();
	HAL_Delay(1000);
	LCD_ClearWifi();
	drawWiFi(3);
	LCD_print(0, 30, 1, "3 division");
	LCD_Update();
	HAL_Delay(1000);
	LCD_ClearWifi();
	drawWiFi(2);
	LCD_print(0, 30, 1, "2 division");
	LCD_Update();
	HAL_Delay(1000);
	LCD_ClearWifi();
	drawWiFi(1);
	LCD_print(0, 30, 1, "1 division");
	LCD_Update();
	HAL_Delay(1000);
	LCD_ClearWifi();
	drawWiFi(0);
	LCD_print(0, 30, 1, "not signal");
	LCD_Update();
	HAL_Delay(1000);
	// Test Battery
	LCD_ClearBattery();
	LCD_Battery(4);
	LCD_print(0, 30, 1, "4 division");
	LCD_Update();
	HAL_Delay(1000);
	LCD_ClearBattery();
	LCD_Battery(3);
	LCD_print(0, 30, 1, "3 division");
	LCD_Update();
	HAL_Delay(1000);
	LCD_ClearBattery();
	LCD_Battery(2);
	LCD_print(0, 30, 1, "2 division");
	LCD_Update();
	HAL_Delay(1000);

	LCD_ClearBattery();
	LCD_Battery(1);
	LCD_print(0, 30, 1, "1 division");
	LCD_Update();
	HAL_Delay(1000);

	for(int i = 0; i < 3; i++)
	{
		LCD_ClearBattery();
		LCD_Update();
		HAL_Delay(500);
		LCD_Battery(1);
		LCD_Update();
		HAL_Delay(500);
	}

	HAL_Delay(1000);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_I2C3_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6);

 // HAL_UART_Transmit(&huart1, (uint8_t*)"Hello\n", 6, 1000 );
  //	HAL_Delay(1000);

  	//LCD_Init();
  	//HAL_Delay(500);

/*
  	RTC_DateTime.RTC_Date = 22;
  	RTC_DateTime.RTC_Month = 9;
  	RTC_DateTime.RTC_Year = 2016;

  	RTC_DateTime.RTC_Hours = 14;
  	RTC_DateTime.RTC_Minutes = 30;
  	RTC_DateTime.RTC_Seconds = 00;

  			//После инициализации требуется задержка. Без нее время не устанавливается.
  	HAL_Delay(500);*/
   // RTC_SetCounter(RTC_GetRTC_Counter(&RTC_DateTime));



  	float tf = 0.0f, pf = 0.0f, af = 0.0f, hf = 0.0f;
  	BME280_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);







	  tf = BME280_ReadTemperature(&huart2);

	  		int si = sprintf((char*)str1, "Temperature: %d", (int)tf);
	  		HAL_UART_Transmit(&huart2,(uint8_t*)str1, si, 0x1000);
	  		//LCD_SetPos(0, 0);
	  		sprintf(str1, "%11.3f *C", tf);
	  		HAL_UART_Transmit(&huart2,(uint8_t*)str1,strlen(str1),0x1000);
	  		//LCD_String(str1);
	  		pf = BME280_ReadPressure();
	  		sprintf(str1, "Pressure: %.3f Pa; %.3f hPa; %.3f mmHg\r\n", pf, pf/1000.0f, pf * 0.000750061683f); // @suppress("Float formatting support")
	  		HAL_UART_Transmit(&huart2,(uint8_t*)str1,strlen(str1),0x1000);
	  		//LCD_SetPos(0, 1);
	  		sprintf(str1, "%11.3f hPa", pf/1000.0f); // @suppress("Float formatting support")
	  		//LCD_String(str1);
	  		//LCD_SetPos(0, 2);
	  		sprintf(str1, "%11.3f mmHg", pf * 0.000750061683f); // @suppress("Float formatting support")
	  		//LCD_String(str1);
	  		//af = BME280_ReadAltitude(SEALEVELPRESSURE_PA);
	  		//sprintf(str1, "Altitude: %.3f m\r\n", af); // @suppress("Float formatting support")
	  		//HAL_UART_Transmit(&huart2,(uint8_t*)str1,strlen(str1),0x1000);
	  		hf = BME280_ReadHumidity();
	  		sprintf(str1, "Humidity: %.3f %%\r\n", hf); // @suppress("Float formatting support")
	  		HAL_UART_Transmit(&huart2,(uint8_t*)str1,strlen(str1),0x1000);
	  		//LCD_SetPos(0, 3);
	  		sprintf(str1, "%7.3f %% %4.1f m", hf, af); // @suppress("Float formatting support")
	  		//LCD_String(str1);
	  		HAL_Delay(1000);




	// LCD_DrawRect(10, 10, 20, 20, 1);

	  //HAL_UART_Transmit(&huart1, (uint8_t*)"Hello\n", 6, 1000 );
	  //HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str), 1000);
	 // HAL_Delay(1000);
	 // HAL_GPIO_TooglePin()

	  testDisplay();

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C3;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00100B3D;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_9BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 47999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CS_Pin|Reset_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Btn_Pin */
  GPIO_InitStruct.Pin = Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Reset_Pin */
  GPIO_InitStruct.Pin = Reset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Reset_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PH3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
  int ttt = 20;
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
