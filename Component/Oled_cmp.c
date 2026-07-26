#include "Oled_cmp.h"
#include "Oled_Font.h"

//控制PB3（SCL）电平
#define OLED_W_SCL(x)		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, (GPIO_PinState)(x))
//控制PB5（SDA）电平
#define OLED_W_SDA(x)		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, (GPIO_PinState)(x))

// 全局变量：SysTick每微秒的计数值
static uint32_t us_ticks = 0;
extern const uint8_t OLED_F8x16[][16];

/**
 * @brief  初始化微秒延时函数（必须在HAL初始化后调用）
 * @param  无
 * @retval 无
 */
void Delay_Init(void)
{
    // SystemCoreClock是HAL库定义的系统主频（Hz），除以1000000得到每微秒的计数值
    us_ticks = SystemCoreClock / 1000000;
}

/**
 * @brief  HAL库版微秒延时函数
 * @param  us: 要延时的微秒数（范围：1 ~ 0xFFFFFF/us_ticks，一般足够用）
 * @retval 无
 */
void delay_us(uint32_t us)
{
    uint32_t start_tick = SysTick->VAL;  // 记录当前SysTick计数值
    uint32_t ticks = us * us_ticks;      // 需要递减的总计数值
    uint32_t now_tick, elapsed_tick;

    // 循环等待，直到累计递减的计数值达到ticks
    while (1)
    {
        now_tick = SysTick->VAL;
        // 处理SysTick溢出（递减到0后会重新加载）
        if (now_tick < start_tick)
        {
            elapsed_tick = start_tick - now_tick;
        }
        else
        {
            elapsed_tick = (SysTick->LOAD + 1 - now_tick) + start_tick;
        }

        if (elapsed_tick >= ticks)
        {
            break;
        }
    }
}

void OLED_I2C_Init(void)
{
	OLED_W_SCL(1);
	OLED_W_SDA(1);
	delay_us(1);
}

/**
  * @brief  I2C开始
  * @param  无
  * @retval 无
  */
void OLED_I2C_Start(void)
{
	OLED_W_SDA(1);
	OLED_W_SCL(1);
	delay_us(1);
	OLED_W_SDA(0);
	OLED_W_SCL(0);
	delay_us(1);
}

/**
  * @brief  I2C停止
  * @param  无
  * @retval 无
  */
void OLED_I2C_Stop(void)
{
	OLED_W_SDA(0);
	delay_us(1);

	OLED_W_SCL(1);
	delay_us(1);
	OLED_W_SDA(1);
	delay_us(1);
}

/**
  * @brief  I2C发送一个字节
  * @param  Byte 要发送的一个字节
  * @retval 无
  */
void OLED_I2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		OLED_W_SDA(Byte & (0x80 >> i));
		delay_us(1);

		OLED_W_SCL(1);
		delay_us(1);
		OLED_W_SCL(0);
		delay_us(1);
	}
	OLED_W_SCL(1);	//额外的一个时钟，不处理应答信号
	OLED_W_SCL(0);
	delay_us(1);
}

/**
  * @brief  OLED写命令
  * @param  Command 要写入的命令
  * @retval 无
  */
void OLED_WriteCommand(uint8_t Command)
{
	OLED_I2C_Start();
	OLED_I2C_SendByte(0x78);		//从机地址
	OLED_I2C_SendByte(0x00);		//写命令
	OLED_I2C_SendByte(Command); 
	OLED_I2C_Stop();
}

/**
  * @brief  OLED写数据
  * @param  Data 要写入的数据
  * @retval 无
  */
void OLED_WriteData(uint8_t Data)
{
	OLED_I2C_Start();
	OLED_I2C_SendByte(0x78);		//从机地址
	OLED_I2C_SendByte(0x40);		//写数据
	OLED_I2C_SendByte(Data);
	OLED_I2C_Stop();
}

/**
  * @brief  OLED设置光标位置
  * @param  Y 以左上角为原点，向下方向的坐标，范围：0~7
  * @param  X 以左上角为原点，向右方向的坐标，范围：0~127
  * @retval 无
  */
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
	OLED_WriteCommand(0xB0 | Y);					//设置Y位置
	OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));	//设置X位置高4位
	OLED_WriteCommand(0x00 | (X & 0x0F));			//设置X位置低4位
}

/**
  * @brief  OLED清屏
  * @param  无
  * @retval 无
  */
void OLED_Clear(void)
{  
	uint8_t i, j;
	for (j = 0; j < 8; j++)
	{
		OLED_SetCursor(j, 0);

		for(i = 0; i < 128; i++)
		{
			OLED_WriteData(0x00);
		}
	}
}

/**
  * @brief  OLED显示一个字符
  * @param  Line 行位置，范围：1~4
  * @param  Column 列位置，范围：1~16
  * @param  Char 要显示的一个字符，范围：ASCII可见字符
  * @retval 无
  */
void OLED_ShowChar(uint8_t Column, uint8_t Line, char Char)
{      	
	uint8_t i;
	OLED_SetCursor((Line ) * 2, (Column ) * 8);		//设置光标位置在上半部分
	for (i = 0; i < 8; i++)
	{
		OLED_WriteData(OLED_F8x16[Char - ' '][i]);			//显示上半部分内容
	}
	OLED_SetCursor((Line ) * 2 + 1, (Column ) * 8);	//设置光标位置在下半部分
	for (i = 0; i < 8; i++)
	{
		OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]);		//显示下半部分内容
	}
}

/**
  * @brief  OLED显示字符串
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  String 要显示的字符串，范围：ASCII可见字符
  * @retval 无
  */
void OLED_ShowString(uint8_t Column, uint8_t Line, char *String,	...)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i++)
	{
		OLED_ShowChar(Column + i, Line, String[i]);
	}
}

/**
  * @brief  OLED次方函数
  * @retval 返回值等于X的Y次方
  */
uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y--)
	{
		Result *= X;
	}
	return Result;
}

/**
  * @brief  OLED显示数字（十进制，正数）
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  Number 要显示的数字，范围：0~4294967295
  * @param  Length 要显示数字的长度，范围：1~10
  * @retval 无
  */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar(Column + i, Line, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}

/**
  * @brief  OLED显示数字（十进制，带符号数）
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  Number 要显示的数字，范围：-2147483648~2147483647
  * @param  Length 要显示数字的长度，范围：1~10
  * @retval 无
  */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
	uint8_t i;
	uint32_t Number1;
	if (Number >= 0)
	{
		OLED_ShowChar(Line, Column, '+');
		Number1 = Number;
	}
	else
	{
		OLED_ShowChar(Line, Column, '-');
		Number1 = -Number;
	}
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar(Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}

/**
  * @brief  OLED显示数字（十六进制，正数）
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  Number 要显示的数字，范围：0~0xFFFFFFFF
  * @param  Length 要显示数字的长度，范围：1~8
  * @retval 无
  */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i, SingleNumber;
	for (i = 0; i < Length; i++)							
	{
		SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
		if (SingleNumber < 10)
		{
			OLED_ShowChar(Line, Column + i, SingleNumber + '0');
		}
		else
		{
			OLED_ShowChar(Line, Column + i, SingleNumber - 10 + 'A');
		}
	}
}

/**
  * @brief  OLED显示浮点数字
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  Number 要显示的数字
  * @param  digit 要显示的小数位数，范围：1~4
  * @retval 无
  */
void OLED_Float(uint8_t Line, uint8_t Column, double Number, uint8_t digit)
{
	unsigned char   Count=1;
	int   		 	Num_int=0,j=1;
	double 			decimal=0;
	unsigned int 	Num_decimal=0;
	char 		    n[15]={0};
	if(Number < 0)
	{
		Num_int=(int)(-Number);
		decimal=-Number-Num_int;
	}
	else 
	{
		Num_int=(int)(Number);
		decimal=Number-Num_int;
	}

    while(Num_int/10/j!=0)
    {
       j=j*10;Count++;
    }

	Num_decimal=(int)(decimal*10000);

   n[0]=(Num_int/10000)%10;
   n[1]=(Num_int/1000)%10;
   n[2]=(Num_int/100)%10;
   n[3]=(Num_int/10)%10;
   n[4]=(Num_int/1)%10;
//   n[5]='.';
   n[6]=(Num_decimal/1000)%10;
   n[7]=(Num_decimal/100)%10;
   n[8]=(Num_decimal/10)%10;
   n[9]=Num_decimal%10;
//   n[6+digit]='\0';

	for(j=0;j<10;j++) n[j]=n[j]+16+32;

   if(Number<0)
   {
      Count++;
      n[5-Count]='-';
   }
	n[5]='.';
    n[6+digit]='\0';

	OLED_ShowString(Line,Column,&n[5-Count]);

}

/**
  * @brief  OLED显示数字（二进制，正数）
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  Number 要显示的数字，范围：0~1111 1111 1111 1111
  * @param  Length 要显示数字的长度，范围：1~16
  * @retval 无
  */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
	}
}

/**
  * @brief  OLED初始化
  * @param  无
  * @retval 无
  */
void OLED_Init(void)
{
//	uint32_t i, j;
	
	HAL_Delay(1000);
	
	OLED_I2C_Init();			//端口初始化
	
	OLED_WriteCommand(0xAE);	//关闭显示
	
	OLED_WriteCommand(0xD5);	//设置显示时钟分频比/振荡器频率
	OLED_WriteCommand(0x80);
	
	OLED_WriteCommand(0xA8);	//设置多路复用率
	OLED_WriteCommand(0x3F);
	
	OLED_WriteCommand(0xD3);	//设置显示偏移
	OLED_WriteCommand(0x00);
	
	OLED_WriteCommand(0x40);	//设置显示开始行
	
	OLED_WriteCommand(0xA1);	//设置左右方向，0xA1正常 0xA0左右反置
	
	OLED_WriteCommand(0xC8);	//设置上下方向，0xC8正常 0xC0上下反置

	OLED_WriteCommand(0xDA);	//设置COM引脚硬件配置
	OLED_WriteCommand(0x12);
	
	OLED_WriteCommand(0x81);	//设置对比度控制
	OLED_WriteCommand(0xCF);

	OLED_WriteCommand(0xD9);	//设置预充电周期
	OLED_WriteCommand(0xF1);

	OLED_WriteCommand(0xDB);	//设置VCOMH取消选择级别
	OLED_WriteCommand(0x30);

	OLED_WriteCommand(0xA4);	//设置整个显示打开/关闭

	OLED_WriteCommand(0xA6);	//设置正常/倒转显示

	OLED_WriteCommand(0x8D);	//设置充电泵
	OLED_WriteCommand(0x14);

	OLED_WriteCommand(0xAF);	//开启显示
		
	OLED_Clear();				//OLED清屏

}

