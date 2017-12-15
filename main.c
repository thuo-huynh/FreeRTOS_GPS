
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#define GPS_BUFFER_SIZE 200
#define GPS_VTG_SIZE 50
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);
/*_____________FreeRTOS_Queue___________________*/
osMessageQDef(MsgBox, 16, float); 
osMessageQId MsgBox;

float lat, lon, velocity;
int Nmea_Line1 = 1;
int Nmea_Line2 = 0;
char ch;
/*_______________________________GPS_Struct_____________________________________*/
struct nmeaMessage_t{
	//Raw Data GPS
	char GPS_RX_byte[2];
	char GPS_Transfer_cplt;
	char GPS_RX_Buffer[GPS_BUFFER_SIZE];
	char GPS_VTG_Buffer[GPS_VTG_SIZE];
	uint8_t GPS_Counter;
	uint8_t GPS_Counter_Tmp;
	uint8_t GPS_Flag;
	
	uint8_t GPS_SCounter;
	uint8_t GPS_SCounter_Tmp;
	uint8_t GPS_SFlag;
	
	

};
struct dataGps_t{
		//Data GPS
	char Time[20];
	char Status[2];
	char Latitude[9];
	char S_N[2];
	char Longtitude[11];
	char E_W[2];
	char Speed[20];
	char Dir[20];
	char Date[20];
};
struct statusGps_t{
	unsigned char GPS_ans_stt;
	unsigned char GPS_send_error;
	unsigned char GPS_receive_error;
};
struct point_t {
  double X;       // kinh do
  double Y;       // vi do
};
//Struct
struct nmeaMessage_t 	nmeaMessage;
struct dataGps_t 			dataGps;
struct statusGps_t		statusGps;
/*________________________________GPS_Prototype_________________________________*/
int Search_Char(unsigned char Char, char *Str, unsigned char Time, int Len);
unsigned char GPS_DeviceInfo(char* time, char* status, char* latitude, char* S_N, 
														 char* longitude, char* E_W, char* speed, char* dir, char* date);
void CLEAR_GPS_RX_Buffer(void);
void Processing_$GPGRMC(void);
void Processing_$GPGVTG(void);
/*________________________________GPS_Function__________________________________*/
// Ham xoa ki tu trong chuoi
void Delete_Char(char s[], int pos)
{
	int n = strlen(s); 
	for(int i = pos + 1; i < n; i++)
	{
		s[i - 1] = s[i];
	}
	s[strlen(s) - 1] = '\0'; // Ki tu ket thuc
}
// Ham tim kiem ki tu trong chuoi
int Search_Char(unsigned char Char, char *Str, unsigned char Time, int Len)
{
    int   i=0;
    int  j=0;
    while((j<Time)&&(i<=Len))
    {
        if(Str[i] == Char)    j++;  
        i++;  
    }
    return i;
}
// Ham clear bo nho dem rx
void CLEAR_GPS_RX_Buffer() 
{
	for (int j=0; j<GPS_BUFFER_SIZE; j++)
	nmeaMessage.GPS_RX_Buffer[j] = 0; //clear Rx_Buffer before receiving new data
	for(int j=0; j< 10; j++)
	dataGps.Latitude[j] = 0;					//clear latitude buffer
	for(int j=0; j< 11; j++)
	dataGps.Longtitude[j] = 0;				//clear longtitude buffer
}
// Ham lay vi tri chuoi $GPRMC
int GPS_GetGPRMC()
{
	int k = 0;
	for(int k=0; k < (GPS_BUFFER_SIZE); k++)
	{
		if( (nmeaMessage.GPS_RX_Buffer[k] == 'M') && (nmeaMessage.GPS_RX_Buffer[k] == 'C') )
		{
			k = k - 5; // get the $
			return k;
		}
	}
	return k;
}
//Ham lay thong tin gps
unsigned char GPS_DeviceInfo(char* time, char* status, char* latitude, char* S_N, char* longitude, char* E_W, char* speed, char* dir, char* date)
{
  int i = 0;
	int k = 0;
	int Temp1, Temp2;
	Temp1 = Search_Char(',',nmeaMessage.GPS_RX_Buffer,1,GPS_BUFFER_SIZE);	//Tim vi tri co ',' lan 2
  // printf("%d\n",Temp1);
	Temp2 = Search_Char(',',nmeaMessage.GPS_RX_Buffer,2,GPS_BUFFER_SIZE);	//Tim vi tri co ',' lan 2
  // printf("%d\n",Temp2);
	if(nmeaMessage.GPS_RX_Buffer[Temp2] == 'V'){
		return 0;
	}
	else{
//-------------------------------------------------------------------------------------------------		
    //LAY VI DO:
		Temp1 = Search_Char(',',nmeaMessage.GPS_RX_Buffer,3,GPS_BUFFER_SIZE);	 //Tim vi tri co ',' lan 3
		//printf("%d\n",Temp1);
		Temp2 = Search_Char(',',nmeaMessage.GPS_RX_Buffer,4,GPS_BUFFER_SIZE);	//Tim vi tri co ',' lan 4
	  //	printf("%d\n",Temp2);
		//Tach chuoi vi do
		k = 0;
		for(i = Temp1; i < Temp2-1; i++){
			dataGps.Latitude[k] = nmeaMessage.GPS_RX_Buffer[i];
			k++;	
		}
    dataGps.Latitude[i] = 0;
		lat = atof(dataGps.Latitude);
//-------------------------------------------------------------------------------------------------				
    //LAY KINH DO:
		Temp1 = Search_Char(',',nmeaMessage.GPS_RX_Buffer,5,GPS_BUFFER_SIZE);	 //Tim vi tri co ',' lan 5
		//printf("%d\n",Temp1);
		Temp2 = Search_Char(',',nmeaMessage.GPS_RX_Buffer,6,GPS_BUFFER_SIZE);	//Tim vi tri co ',' lan 6
		//printf("%d\n",Temp2);
		k = 0;
		for(i = Temp1 ; i < Temp2-1; i++){
			dataGps.Longtitude[k] = nmeaMessage.GPS_RX_Buffer[i];
			k++;	
		}
   dataGps.Longtitude[i] = 0;
	  lon = atof(dataGps.Longtitude);
		printf("%f",lon);
//-------------------------------------------------------------------------------------------------			
		//LAY VAN TOC:
		Temp1 = Search_Char(',',nmeaMessage.GPS_VTG_Buffer,7,GPS_VTG_SIZE);	 //Tim vi tri co ',' lan 3
		//printf("%d\n",Temp1);
		Temp2 = Search_Char(',',nmeaMessage.GPS_VTG_Buffer,8,GPS_VTG_SIZE);	//Tim vi tri co ',' lan 4
	  //	printf("%d\n",Temp2);
		//Tach chuoi van toc
		k = 0;
		for(i = Temp1; i < Temp2-1; i++){
			dataGps.Speed[k] = nmeaMessage.GPS_VTG_Buffer[i];
			k++;	
		}
    dataGps.Speed[i] = 0;
		velocity = atof(dataGps.Speed);
		
	return 1;	
	}
}
// Tim kiem dau , trong chuoi GPS
void Scan_for_dots(){
	uint8_t i = 0;
	while (i!=26){
		if (nmeaMessage.GPS_RX_Buffer[i] == ','){
			nmeaMessage.GPS_RX_Buffer[i] = 'y';
		}
		i++;
	}
}
/*_____________________________________________________________________________________________*/

void Processing_$GPGRMC(){
	if(Nmea_Line1 == 1){
	if (ch == '$'){
		nmeaMessage.GPS_Counter = 1;
		nmeaMessage.GPS_RX_Buffer[nmeaMessage.GPS_Counter-1] = '$';
	}
	if (nmeaMessage.GPS_Counter == 2){
		if (ch == 'G'){
		nmeaMessage.GPS_RX_Buffer[nmeaMessage.GPS_Counter-1] = 'G';
		}
	}
	if (nmeaMessage.GPS_Counter == 3){
		if (ch == 'P'){
		nmeaMessage.GPS_RX_Buffer[nmeaMessage.GPS_Counter-1] = 'P';
		}
	}
	if (nmeaMessage.GPS_Counter == 4){
		if (ch == 'R'){
		nmeaMessage.GPS_RX_Buffer[nmeaMessage.GPS_Counter-1] = 'R';
		nmeaMessage.GPS_Flag++;
		}
	}
	if (nmeaMessage.GPS_Counter == 5){
		if (ch == 'M'){
		nmeaMessage.GPS_RX_Buffer[nmeaMessage.GPS_Counter-1] = 'M';
		nmeaMessage.GPS_Flag++;
			
		}
	}
	if (nmeaMessage.GPS_Counter == 6){
		if (ch == 'C'){
		nmeaMessage.GPS_RX_Buffer[nmeaMessage.GPS_Counter-1] = 'C';
		nmeaMessage.GPS_Flag++;
		}
	}
	if (ch == '*'){
		if (nmeaMessage.GPS_Flag == 3){
			statusGps.GPS_ans_stt = GPS_DeviceInfo(dataGps.Time, dataGps.Status, dataGps.Latitude, dataGps.S_N, 
																				 dataGps.Longtitude, dataGps.E_W, dataGps.Speed, dataGps.Dir, dataGps.Date);
			if(statusGps.GPS_ans_stt){
				//printf("Kinh do: %s\n", dataGps.Latitude);
				//printf("Vi do: %s\n", dataGps.Longtitude);
				//printf("Toc do: %s\n",dataGps.Speed);
				osMessagePut(MsgBox, lon, osWaitForever);
				osDelay(100);
			}
		//	printf("%s\n", nmeaMessage.GPS_RX_Buffer);
			Nmea_Line1 = 0; 
			Nmea_Line2 = 1;
			}
		nmeaMessage.GPS_Flag = 0;
		nmeaMessage.GPS_Counter = 0;
	}

	if (0 < nmeaMessage.GPS_Counter && nmeaMessage.GPS_Counter < 200){
			if (nmeaMessage.GPS_Flag == 3){
				nmeaMessage.GPS_Counter_Tmp = nmeaMessage.GPS_Counter-1;
				nmeaMessage.GPS_RX_Buffer[nmeaMessage.GPS_Counter_Tmp] = ch;
				nmeaMessage.GPS_Counter++;
			}
			else {
				nmeaMessage.GPS_Counter++;
		}
	}
	

	
/*-------------------------------------------------------------------*/	
}
}


void Processing_$GPVTG(){
	if(Nmea_Line2 == 1){
	if (ch == '$'){
		nmeaMessage.GPS_SCounter = 1;
		nmeaMessage.GPS_VTG_Buffer[nmeaMessage.GPS_SCounter-1] = '$';
	}
	if (nmeaMessage.GPS_SCounter == 2){
		if (ch == 'G'){
		nmeaMessage.GPS_VTG_Buffer[nmeaMessage.GPS_SCounter-1] = 'G';
		}
	}
	if (nmeaMessage.GPS_SCounter == 3){
		if (ch == 'P'){
		nmeaMessage.GPS_VTG_Buffer[nmeaMessage.GPS_SCounter-1] = 'P';
		}
	}
	if (nmeaMessage.GPS_SCounter == 4){
		if (ch == 'V'){
		nmeaMessage.GPS_VTG_Buffer[nmeaMessage.GPS_SCounter-1] = 'V';
		nmeaMessage.GPS_SFlag++;
		}
	}
	if (nmeaMessage.GPS_SCounter == 5){
		if (ch == 'T'){
		nmeaMessage.GPS_VTG_Buffer[nmeaMessage.GPS_SCounter-1] = 'T';
		nmeaMessage.GPS_SFlag++;	
		}
	}
	if (nmeaMessage.GPS_SCounter == 6){
		if (ch == 'G'){
		nmeaMessage.GPS_VTG_Buffer[nmeaMessage.GPS_SCounter-1] = 'G';
		nmeaMessage.GPS_SFlag++;
		}
	}
	if (ch == '*'){
		if (nmeaMessage.GPS_SFlag == 3){
			//printf("%s\n", nmeaMessage.GPS_VTG_Buffer);
			Nmea_Line1 = 1;
			Nmea_Line2 = 0;
		}
		nmeaMessage.GPS_SFlag = 0;
		nmeaMessage.GPS_SCounter = 0;
	}
	if (0 < nmeaMessage.GPS_SCounter && nmeaMessage.GPS_SCounter < 50){
			if (nmeaMessage.GPS_SFlag == 3){
				nmeaMessage.GPS_SCounter_Tmp = nmeaMessage.GPS_SCounter-1;
				nmeaMessage.GPS_VTG_Buffer[nmeaMessage.GPS_SCounter_Tmp] = ch;
				nmeaMessage.GPS_SCounter++;
			}
			else {
				nmeaMessage.GPS_SCounter++;
		}
	}
}
	}

	//Ham ngat Uart1 de nhan gia tri GPS
void GPS_USART_RX_ISR(){
	if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET){
		ch = (uint8_t)((&huart1)->Instance->DR & (uint8_t)0x00FF);
		Processing_$GPVTG();
		Processing_$GPGRMC();
	}
}


int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityNormal, 0, 128);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, StartTask03, osPriorityLow, 0, 128);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

	MsgBox = osMessageCreate(osMessageQ(MsgBox), NULL);
  /* Start scheduler */
  osKernelStart();
  
  while (1)
  { 

  }

}

/** System Clock Configuration
*/

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

}


void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pins : PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}


void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

void StartTask02(void const * argument)
{

  for(;;)
  {
		GPS_USART_RX_ISR();
  }
}


void StartTask03(void const * argument)
{
	float delay = 500; /* Default delay */
	osEvent evt;
  for(;;)
  {
			evt = osMessageGet(MsgBox, 1);
	if(evt.status == osEventMessage){
		delay = evt.value.v;
		}
		printf("Nhan duoc la: %f\n",delay);
		osDelay(100);
  }
	osThreadTerminate(NULL);
}

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
	



/* USER CODE END PFP */

PUTCHAR_PROTOTYPE  
{

    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
    return ch;
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
