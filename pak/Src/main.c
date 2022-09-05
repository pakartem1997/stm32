/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//????? ???????
#define Start_T1  HAL_TIM_Base_Start(&htim1);
#define Start_T2  HAL_TIM_Base_Start(&htim2);
#define Start_T3  HAL_TIM_Base_Start(&htim3);
#define Start_T4  HAL_TIM_Base_Start(&htim4);
#define Start_T5  HAL_TIM_Base_Start(&htim5);
//???? ???????
#define Stop_T1   HAL_TIM_Base_Stop(&htim1);
#define Stop_T2   HAL_TIM_Base_Stop(&htim2);
#define Stop_T3   HAL_TIM_Base_Stop(&htim3);
#define Stop_T4   HAL_TIM_Base_Stop(&htim4);
#define Stop_T5   HAL_TIM_Base_Stop(&htim5);

#define Inhibit_set			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);  
#define stosc_set				HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
#define ledpin29_set    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);  
#define stp_set         HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);  
#define charp_set       HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);
#define modulatoroff_set HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
#define dischp_set      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
#define rechp_set       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);

#define Inhibit_reset   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);  
#define stosc_reset	    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);  
#define ledpin29_reset  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
#define stp_reset  	    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);  
#define charp_reset   	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);  
#define modulatoroff_reset HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
#define dischp_reset    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
#define rechp_reset     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

bool instrok;
bool rs232 = 1;
bool bum14_mode = 0;
bool storen = 1; //0
bool bur_go_mark = 0;

unsigned int faza = 100;
unsigned int fazatemp = 100; //faza defoult=5ms

unsigned int adr485 = 71;

unsigned int protection_code = 32;
//unsigned int fazatemp = 201;
unsigned int Shift6_6Temp = 205;
unsigned int frset = 2;
unsigned int StorDelH = 0xB7, StorDelL = 0x19;
unsigned int StorageH = 100, StorageL = 0x18;
unsigned int StorDelH_Temp = 0xB7, StorDelL_Temp = 0x19;
unsigned int Magic_H = 500, Magic_L = 0xCB;
unsigned int PulseTime_H = 0xEA, PulseTime_L = 0x66;
unsigned int BUR_Signal_H_Temp = 0xFA, BUR_Signal_L_Temp = 0x98;
unsigned int StorageH_Temp = 0xED, StorageL_Temp = 0x18;
unsigned int PulseTime_H_noStor = 0xF3;
unsigned char PulseTime_L_noStor = 0x08;
unsigned int PulseTime_H_Stor = 0xF3;
unsigned char PulseTime_L_Stor = 0x08;

unsigned int bufer;
unsigned int Pulse_Number = 1; 
unsigned int Fr = 2;
unsigned int Counter = 1; 
unsigned int Shift6_6 = 205;
unsigned int t3mode = 0;
unsigned int t2mode = 0;
unsigned int t1mode = 0;
unsigned int t0mode = 0;

bool StorageNew_Mark = 0;

bool rectif_protect_OFF = 0;
bool slow_up_OFF = 0;
bool rectifok2;

bool ReverseV_markON = 0;
#define rectifofftoupu_set   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
#define rectifofftoupu_reset HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); 
#define urectifpin18         HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10)
bool rectifok;
#define rectifok2simul       HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11)
unsigned int vline1 = 255;
unsigned int ADC_W;
unsigned int protlevel = 0x66;
bool Reverse_V_sim = 0;
bool reverseV_protect_OFF = 0;
bool ReverseV_start_ON = 1;
unsigned int slow_up_step = 3;
unsigned int Charge_Off_Counter = 0;
bool VLineOK = 0;
bool StorageOn = 1;
#define tumbstoreon          HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15)
unsigned int protlevel_noStor = 0x19;
bool First_Store_off_mark = 1;
#define storeofftobum23_set   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
#define storeofftobum23_reset HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);


int instr;
unsigned int ident = 1208;
unsigned int softversion = 314; // soft version instr=253
int User_Data[256];
uint8_t ui;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
__STATIC_INLINE void DelayMicro(__IO uint32_t micros)
{
	micros = (SystemCoreClock / 41999) * micros;
	while (micros--);
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
void adcprepare(unsigned char a)  // a adr ADC
{
	unsigned char temp = 8;

	ADDR = 1;           //ADC Va2, Vb2 channal
	convstart = 0;      //ADC start 
	//__no_operation();
	convstart = 1;
	while (Busy == 1) {}
	//delaysec(2,2);
	//PORTC_Bit7=0;       //CS ADC
	adc_cs = 0;
	//adc_rd=0;     	    //RD=0
	while (temp > 0)
	{
		adc_sclk = 0;
		if (DoutA == 1) ADC_W = ADC_W + 1;
		ADC_W = ADC_W << 1;
		//ADC_W=PINA;
		//ADC_W=0;
		temp = temp - 1;
		adc_sclk = 1;
	}
	adc_cs = 1;
	if (ADC_W >= 128) ADC_W = 0;
	//adc_rd=1;     	    //RD=1
	//PORTC_Bit7=1;       //CS ADC
	//if (ADC_W >= 0x80) ADC_W=0;
}     // endvoid adcprepare
*/
//******************************************************
//
//******************************************************
void  checkpulseenable()
{

	if (ReverseV_markON == 1) 
	{ 
	rectifofftoupu_set; 
	ReverseV_markON = 0; 
	}

	if (urectifpin18 == 0)
	{
		rectifok = 1;
	}//
	else
	{
		rectifok = 0;
	}

	if ((rectifok2simul == 0) || (rectif_protect_OFF == 1)) 
	{
	rectifok = 1;
	}

  if (urectifpin18 == 1) 
  {
  protection_code = protection_code | 4;
  }     //protection code U Modulator to comp 00000100B// 
	//l=ADCL;
	//h=ADCH;                     
	vline1 = ADC_W;

	if (((vline1 > protlevel) || (Reverse_V_sim == 1)) && (reverseV_protect_OFF == 0))
	{
		if (ReverseV_start_ON == 0 && slow_up_step > 0)
		{
		}
		else
		{
			rectifofftoupu_reset;
			ReverseV_markON = 1;
			if (Charge_Off_Counter == 0) 
			{
			Charge_Off_Counter = 53;
			}
		}
	} //go to BBZ reverse V
	if (vline1 < protlevel)
	{	
	VLineOK = 1;
	}
	if (reverseV_protect_OFF == 1) VLineOK = 1;
	if ((StorageOn == 1) && (VLineOK == 1) && (rectifok == 1) && (tumbstoreon == 1) && (slow_up_step < 3) && (Reverse_V_sim == 0) && (Charge_Off_Counter == 0) && (vline1 > protlevel_noStor))
	{
		if (First_Store_off_mark == 1)
		{
			storen = 0;
			storeofftobum23_reset;
			First_Store_off_mark = 0;
		}
		else
		{
			storen = 1;
			storeofftobum23_set;
		}
	}
	//if ((tumbstoreon==1) && (slow_up_step<3))storen=1;
	else
	{
		Reverse_V_sim = 0;
		First_Store_off_mark = 1;
		storen = 0;
		storeofftobum23_reset;
		slow_up_step = 3;   //??????? ????? ?? 3 ????
	}
	if (vline1 > protlevel) protection_code = protection_code | 1;     //protection code U obratnoe to comp    000001B
} // endvoid  checkpulseenable()
//******************************************************
//
//******************************************************
void delaysec(unsigned char cin, unsigned char  cex)
{
	unsigned char i, e;
	for (e = 0; e < cex; e++)
	{
		for (i = 0; i < cin; i++) i = i;
	}
}
//******************************************************
//
//******************************************************
uint8_t readbyte(void)
{
	while (sep() == 0)
	{

	}
	ser(0);
	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
	ui = readbyte1();
	return ui;
}
//******************************************************
//
//******************************************************
unsigned int readword()       // from PC
{
	unsigned char bufL, bufH, err;
	unsigned int res;

	bufL = readbyte();
	writebyte(bufL);
	bufH = readbyte();
	writebyte(bufH);
	err = readbyte();
//	if (err != 0xBB)  return (1);
	res = bufH << 8; 
	res = res | bufL;
	return res;
}
//******************************************************
//
//******************************************************
unsigned int readfrompc()       // from PC
{
	unsigned int res1;

	if (rs232)
	{
		res1 = readword();  
		return res1;
	}
	return 0;
}
//******************************************************
//
//******************************************************
char writeword(unsigned int w)
{
	unsigned char bufH, bufL, err, temp;
	bufL = w & 255;
	bufH = w >> 8;
	temp = 0xBB;
	HAL_Delay(500);
	writebyte(bufL);
	err = readbyte();
	if (bufL != err) return(1);
	//buf=w>>8;
	writebyte(bufH);
	err = readbyte();
	if (bufH != err) return(1);
	writebyte(temp);
	return(0);
}
//******************************************************
//
//******************************************************
unsigned char writetopc(unsigned int w)
{
	unsigned char  er;
	er = 0;
	if (rs232)
	{
	 er = writeword(w); return er;
	}
}

//******************************************************
//
//******************************************************


//int  rt;
//int foo()
//{
//
//	for (size_t i = 0; ; i++)
//	{
//		if ( sep() == 1)
//		{
//		ser(0);
//		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
//		return 0;
//		}
//		
//	}
//	
//}
//

//******************************************************
//
//******************************************************
void readinstr()
{

	int startbyte;
	startbyte = readbyte();
	if (rs232)
	{
		if (startbyte != adr485)
		{
			instrok = 0;
		}
		else
		{
			// HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
			writebyte(0xF);
			instr = readbyte();

			writebyte(instr);
			instrok = 1;
		}
	}
}
//******************************************************
//
//******************************************************
void wordtopc()
{
	unsigned char in;
	unsigned int tmpw;
	if (instr < 128) return;
	in = instr - 128;
	if (!instrok) return;
	//127 
	if (127 == in) { writetopc(ident); return; }
	
	//126
	if (126 == in) { writetopc(adr485); return; }
	
	//125 -
	if (125 == in) { writetopc(softversion); return; }
	//30
	
	if (30 == in) { writetopc(protlevel); return; }
	//29
	if (29 == in) { writetopc(protection_code); return; }
	//28
	if (28 == in) { writetopc(vline1); return; }
	//27
	if (27 == in) { writetopc(fazatemp); return; }
	//26
	if (26 == in) { writetopc(Shift6_6Temp); return; }
	//25
	if (25 == in) { writetopc(frset); return; }
	//24
	if (24 == in)
	{
		tmpw = StorDelH_Temp;
	//	tmpw = (tmpw << 8) | StorDelL_Temp;
		writetopc(tmpw);
	}
	//23
	if (23 == in)
	{
		tmpw = StorageH_Temp;
		tmpw = (tmpw << 8) | StorageL_Temp;
		writetopc(tmpw);
		return;
	}
	//22
	if (22 == in)
	{
		tmpw = Magic_H;
	//	tmpw = (tmpw << 8) | Magic_L;
		writetopc(tmpw);
		return;
	}
	//21
	if (21 == in)
	{
		tmpw = PulseTime_H;
		tmpw = (tmpw << 8) | PulseTime_L;
		writetopc(tmpw);
		return;
	}
	//20
	if (20 == in)
	{
		if (StorageOn == 0) tmpw = 35;
		if (StorageOn == 1) tmpw = 155;
		writetopc(tmpw);
		return;
	}
	//19
	if (19 == in)
	{
		tmpw = BUR_Signal_H_Temp;
		tmpw = (tmpw << 8) | BUR_Signal_L_Temp;
		writetopc(tmpw);
		return;
	}
	//17
	if (17 == in)
	{
		if (ReverseV_start_ON == 1) tmpw = 39;
		if (ReverseV_start_ON == 0) tmpw = 159;
		writetopc(tmpw);
		return;
	}
	return;
}
//******************************************************
//
//******************************************************
void wordfrompc()
{ //unsigned char i;
	unsigned int tmpw;
	int pt;
	if (instr > 127) return;
	if (!instrok) return;


	//if (65 == instr) { tmpw = readfrompc(); rectifofftoupu = tmpw & 1; rectifofftoupu = !rectifofftoupu; return; }// soft store enable //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	if (0 == instr) { ident = readfrompc(); return; }
	if (1 == instr)
	{
		fazatemp = readfrompc();
		/*
		if (fazatemp > 254)
		{
			fazatemp = 254;
		//  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
		}

		if (fazatemp < 40)
		{
			fazatemp = 40;
		//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		}
		*/
		return;
	}
	if (2 == instr)
	{
		Shift6_6Temp = readfrompc();
		if (Shift6_6Temp > 254) { Shift6_6Temp = 254; }
		if (Shift6_6Temp < 39) { Shift6_6Temp = 39; }
		return;
	}
	
	if (3 == instr)
	{
		frset = readfrompc();
//		if (frset > 50)
//		{
//			frset = 50;
//			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
//		}
//		if (frset < 1)
//		{
//			frset = 1;
//			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
//		}
		return;
	}
	
	if (4 == instr)
	{
		tmpw = readfrompc();
		if (tmpw < 46182) tmpw = 46182; //47564=3000ms(16ms) 46182=3000(17ms)  44799=3000(18ms)
		StorDelH_Temp = tmpw;
    //	StorDelH_Temp = tmpw >> 8;
	//	StorDelL_Temp = tmpw & 255;
		StorageNew_Mark = 0;
		return;
	}
	
	if (5 == instr)
	{
		tmpw = readfrompc();
		if (tmpw > 61389) tmpw = 61389; //3000 ms
		if (tmpw < 42035) tmpw = 42035; //43417=16ms  42035=17ms 40652=18ms
		StorageH_Temp = tmpw >> 8;
		StorageL_Temp = tmpw & 255;
		StorageNew_Mark = 1;
		return;
	}
	
	if (6 == instr)
	{
		tmpw = readfrompc();
		if (tmpw > 62770) tmpw = 62770;
		if (tmpw < 37890) tmpw = 37890;
		Magic_H = tmpw;
	//	Magic_H = tmpw >> 8;
	//	Magic_L = tmpw & 255;
		return;
	}
	
	if (7 == instr)
	{
		tmpw = readfrompc();
		if (tmpw > 62216) tmpw = 62216;
		if (tmpw < 54473) tmpw = 54473;
		PulseTime_H = tmpw >> 8;
	//	PulseTime_L = tmpw & 255;
		return;
	}
	
	if (8 == instr) 
	{ 
		tmpw = readfrompc(); 
		if (tmpw == 35)  StorageOn = 0; 
		if (tmpw == 155) StorageOn = 1; 
		return; 
	}
	if (9 == instr)
	{ 
		tmpw = readfrompc(); 
		if (tmpw == 159) protection_code = 0; 
		return; 
	}
	//if (10==instr) {Madgic_adj_Temp=readfrompc();return;}
	if (11 == instr)
	{ 
		tmpw = readfrompc(); 
		if (tmpw == 199) Reverse_V_sim = 1; 
		return; 
	}
	if (12 == instr)
	{
		tmpw = readfrompc();
		if (tmpw > 65500) tmpw = 65500;
		if (tmpw < 60070) tmpw = 60070;
		BUR_Signal_H_Temp = tmpw >> 8;
		BUR_Signal_L_Temp = tmpw & 255;
		//StorageNew_Mark=1; 
		return;
	}
		
	if (13 == instr) 
	{ 
		tmpw = readfrompc(); 
		if (tmpw == 36)  rectif_protect_OFF = 0; 
		if (tmpw == 156) rectif_protect_OFF = 1; 
		return; 
	}
	if (14 == instr) 
	{ 
		tmpw = readfrompc(); 
		if (tmpw == 37)  reverseV_protect_OFF = 0;
		if (tmpw == 157) reverseV_protect_OFF = 1;
		return; 
	}
	if (15 == instr) 
	{ 
		tmpw = readfrompc(); 
		if (tmpw == 38)  slow_up_OFF = 0; 
		if (tmpw == 158) slow_up_OFF = 1; 
		return; 
	}
	if (16 == instr) 
	{ 
		tmpw = readfrompc(); 
		if (tmpw == 39)  ReverseV_start_ON = 1;
		if (tmpw == 159) ReverseV_start_ON = 0; 
		return; 
	}
		
	if (17 == instr)
	{
		tmpw = readfrompc();
		if (tmpw > 62216) tmpw = 62216;
		if (tmpw < 54473) tmpw = 54473;
		PulseTime_H_noStor = tmpw;
	//	PulseTime_L_noStor = tmpw & 255;
		return;
	}
	
	if (18 == instr)
	{
		tmpw = readfrompc();
		if (tmpw > 62216) tmpw = 62216;
		if (tmpw < 54473) tmpw = 54473;
		PulseTime_H_Stor = tmpw >> 8;
		PulseTime_L_Stor = tmpw & 255;
		return;
	}
	
	if (50 == instr)
	{
		protlevel = readfrompc();
		if (protlevel > 0x7F) protlevel = 0x7F;
		return;
	}
	
	if (51 == instr)
	{
		protlevel_noStor = readfrompc();
		if (protlevel_noStor > 0x7F) protlevel = 0x7F;
		return;
	}
	/*
	if (18 == instr) 
	{ 
		tmpw = readfrompc();  
		rectifok2 = tmpw & 1; 
		return; 
	}
	*/
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  rectifok2 = rectifok2simul;
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
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
 // HAL_TIM_Base_Start_IT(&htim3);
 // HAL_TIM_Base_Start(&htim3);
//  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	readinstr();
	wordtopc();
	wordfrompc();
	/*
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	int i = (uint32_t)HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	*/
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//	foo();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV8;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void EXTI1_IRQHandler(void)
{
	/* USER CODE BEGIN EXTI1_IRQn 0 */
  //  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
//	__HAL_TIM_SET_COMPARE(&htim2, 0);
	//HAL_TIM_Base_Stop(&htim2);

	faza = fazatemp;

//	a = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13);
  __HAL_TIM_SET_AUTORELOAD(&htim2, faza); //?????? ????? ???????
	//if(faza = 500)

	//TIM2->ARR = faza;
	//else
	//TIM2->ARR = 1000;

    Start_T2;
	
	// __HAL_TIM_SET_AUTORELOAD(&htim2, 3000);

	 /* USER CODE END EXTI1_IRQn 0 */
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
	/* USER CODE BEGIN EXTI1_IRQn 1 */

	/* USER CODE END EXTI1_IRQn 1 */
}
//**********************************************************************************************************************
/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
	/* USER CODE BEGIN TIM2_IRQn 0 */
//	HAL_TIM_Base_Start(&htim3);
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);

	if (Fr < 1)
	{
		Fr = 1;
	}

	bufer = 50 * Pulse_Number / Fr;

	if (bufer == Counter)
	{
	    Inhibit_reset;    //PORTE_Bit2 //pin 29 inhibit to CCM
	    stosc_reset;    // go generator pin23 PORTB_Bit5 // pin 22  0- store pulse for oscil
		//delaysec(3000, 1000000);
		DelayMicro(100);
		stosc_set;
		Inhibit_set;
		Shift6_6 = Shift6_6Temp;

		if (bum14_mode == 1)
		{
			Shift6_6 = 0xFF;
		}

		//   SFIOR_Bit1  = 1; //Preddelitel reset
	  __HAL_TIM_SET_AUTORELOAD(&htim3, Shift6_6); //?????? ????? ???????
	  //  Start_T3;
		//????????? ??????????
	 //   ET2         = 1;  // enable T0 interupt  TIMSK_Bit2  //ET2 interrupt on timer2 overflow
		t2mode = 1;
		Pulse_Number++;
	}

	Counter++;

	if (adr485 != 71)
	{
		frset = 50;
	}

	if (Counter == 51)
	{
		Counter = 1;
		Pulse_Number = 1;
		Fr = frset;
	}
	
	/* USER CODE END TIM2_IRQn 0 */
	HAL_TIM_IRQHandler(&htim2);
	/* USER CODE BEGIN TIM2_IRQn 1 */

	/* USER CODE END TIM2_IRQn 1 */
}
//**********************************************************************************************************************
/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
	/* USER CODE BEGIN TIM3_IRQn 0 */
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);

	if (t2mode == 1)
	{
		if (StorageNew_Mark == 1)
		{
			StorDelH = StorDelH_Temp;
		//	StorDelL = StorDelL_Temp;
			StorageH = StorageH_Temp;
		//	StorageL = StorageL_Temp;
			StorageNew_Mark = 0;
		}

		if (bum14_mode == 1)
		{
			////   heckpulseenable();
		}

		if (slow_up_step <= 1)
		{
			__HAL_TIM_SET_AUTORELOAD(&htim4, StorDelH); //?????? ????? ???????	
		}

		if (slow_up_step == 2 || slow_up_step == 3)
		{
			if (storen == 1)
			{
				// defolt 13.5 ms for 3.5ms storage (17ms)
				__HAL_TIM_SET_AUTORELOAD(&htim4, 135); //?????? ????? ???????	
			}
			else
			{
				// defolt 10 ms for 3ms storage (17ms)
				__HAL_TIM_SET_AUTORELOAD(&htim4, 100); //?????? ????? ???????	
			}
		}

		if (slow_up_OFF == 1)
		{
			__HAL_TIM_SET_AUTORELOAD(&htim4, StorageH); //?????? ????? ???????	
		}

		t1mode = 2;
	  Start_T4; //????? ??????? 4
	  return;
	}
	/* USER CODE END TIM3_IRQn 0 */
	HAL_TIM_IRQHandler(&htim3);
	/* USER CODE BEGIN TIM3_IRQn 1 */

	/* USER CODE END TIM3_IRQn 1 */
}
//**********************************************************************************************************************
/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
	/* USER CODE BEGIN TIM4_IRQn 0 */
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
	unsigned int back_int_temp = 0;

	if (t1mode == 2)    //  make store pulse
	{
		if (storen == 1)
		{
			stp_reset;  //storage go
		 //   SET_D12;      
			ledpin29_reset;
		}
		t1mode = 3;   // back front store pulse
	  __HAL_TIM_SET_AUTORELOAD(&htim4, 1); //14 ?? 1 write  to high byte TCNT1 205 then write to low byte  
		Start_T4;//run timer T1  tic-0.71us CK/8  40 us
		return;
	} //endif(1==t1mode)    // make  store pulse

	if (t1mode == 3)  // back front store pulse
	{
	//	Stop_T4;
		stp_set;// back front store pulse 
	  //  RES_D12;
		t1mode = 4; // back front store pulse

		if (slow_up_step <= 1)
		{
			__HAL_TIM_SET_AUTORELOAD(&htim4, StorageH);
		}

		if (slow_up_step == 2 || slow_up_step == 3)
		{
			__HAL_TIM_SET_AUTORELOAD(&htim4, 100);    // defolt 3.5ms storage (16ms)(17ms)
		}

		if ((slow_up_step >= 1) && (storen == 1))
		{
			slow_up_step = slow_up_step - 1;
		}
		if (slow_up_step == 3) slow_up_step = 2;
		if (slow_up_OFF == 1)
		{
			__HAL_TIM_SET_AUTORELOAD(&htim4, StorageH);
		}
		Start_T4;//run timer T1  tic-0.71us CK/8
		return;
	}//endif (2==t1mode)  // back front store pulse

	if (t1mode == 4) // front zarad
	{
	//	Stop_T4;
	//	adcprepare();
	////	if (bum14_mode == 0) checkpulseenable();
		charp_reset;
		//  SET_D13;
		if (bum14_mode == 1) charp_reset;   //Classic mode
		ledpin29_set;
		t1mode = 5; // begin 2-d piece 1.5 ms
		__HAL_TIM_SET_AUTORELOAD(&htim4, 217);//217 ?? 1 write  to high byte TCNT1 then write to low byte   
		Start_T4; //run timer T1  tic-0.71us CK/1  1350 us  
		return;
	}//endif (3==t1mode)  

	if (t1mode == 5)    //Back zarad
	{
	//	Stop_T4;
		t3mode = 1;
		t1mode = 0;
		//  ET3    = 1;
		charp_set;
		//  RES_D13;
		if (storen == 1)
		{
			__HAL_TIM_SET_AUTORELOAD(&htim5, Magic_H); // 1 write  to high byte TCNT1 205 then write to low byte 
		}
		else
		{
			__HAL_TIM_SET_AUTORELOAD(&htim5, Magic_H - 11);  // no store, 6 ms(17ms)
		}
	//	BUR_Signal = BUR_Signal_Temp;
		//  OCR3BH       = BUR_Signal;
	//	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, BUR_Signal);
		//  ETIMSK_Bit3  = 1;    //OCIE3B: Timer/Counter3, Output Compare A Match Interrupt Enable
		bur_go_mark = 1;
		Start_T5;
		return;
	}
	
	/* USER CODE END TIM4_IRQn 0 */
	HAL_TIM_IRQHandler(&htim4);
	/* USER CODE BEGIN TIM4_IRQn 1 */
	/*
	if (bur_go_mark == 1)
	{
		unsigned int i = 10;
		if (bur_go_mark == 0)
		{
			return;
		}
		while (i > 0)
		{
			tobur_set;
			i = i - 1;
		}
		tobur_reset;
	}*/
	/* USER CODE END TIM4_IRQn 0 */
	HAL_TIM_IRQHandler(&htim4);
	/* USER CODE BEGIN TIM4_IRQn 1 */

	/* USER CODE END TIM4_IRQn 1 */
}
//**********************************************************************************************************************
/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
	/* USER CODE BEGIN TIM5_IRQn 0 */
	modulatoroff_set; //no modulator off
	if (t3mode == 1)
	{
		bur_go_mark = 0;
	//	Stop_T5;
		t3mode = 2;
		dischp_reset;
		//  SET_D14;
		__HAL_TIM_SET_AUTORELOAD(&htim5, 400); //40 mkbur_go_marks
		Start_T5; //timer3=clk/1
		return;
	}

	if (t3mode == 2)   //Back Discharge
	{
	//	Stop_T5;
		t3mode = 3;
		dischp_set;
		//  RES_D14;
		if (storen == 0)
		{
			__HAL_TIM_SET_AUTORELOAD(&htim5, PulseTime_H_noStor);
		}
		else
		{
			if (slow_up_step >= 1)
			{
				__HAL_TIM_SET_AUTORELOAD(&htim5, PulseTime_H_noStor);
			}
			else
			{
				__HAL_TIM_SET_AUTORELOAD(&htim5, PulseTime_H);
			}
		}
		Start_T5;//timer3=clk/1
		return;
	}

	if (t3mode == 3)  //Front Recharge
	{
	//	Stop_T5;
		t3mode = 4;
		rechp_reset;
		//  SET_D15;
		__HAL_TIM_SET_AUTORELOAD(&htim5, 40); //40 mks
		Start_T5;  //timer3=clk/1
		return;
	}//

	if (t3mode == 4)  //Back Recharge
	{
		//	Stop_T5;
		t3mode = 0;
		rechp_set;
		return;
	}
	/* USER CODE END TIM5_IRQn 0 */
	HAL_TIM_IRQHandler(&htim5);
	/* USER CODE BEGIN TIM5_IRQn 1 */

	/* USER CODE END TIM5_IRQn 1 */
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
