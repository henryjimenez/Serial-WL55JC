/* USER CODE BEGIN Header */
/*FIRMWARE DE PRUEBA, COMUNICACION SERIAL
MICROCONTROLADOR WC55JL
ESTA TRAMA SE USA PARA LA TRANSMISION SERIAL HACIA LORA Y HACIA EPORT E10
DISEÑADO POR HENRY JIMENEZ ROSERO
TECNOPARQUE NODO CALI 2022

  ******************************************************************************
  */

#include "main.h"


#if (USE_TIMEOUT == 1)
#define USART_SEND_TIMEOUT_TXE_MS 10
#define USART_SEND_TIMEOUT_TC_MS  20
#endif 


#if (USE_TIMEOUT == 1)
uint32_t Timeout = 0; 
#endif 
__IO uint8_t ubButtonPress = 0;
uint8_t ubSend = 0;
const uint8_t aStringToSend[] = "STM32WLxx USART LL API Example : TX in Polling mode\r\nConfiguration UART 115200 bps,"
		" 8 data bit/1 stop bit/No parity/No HW flow control\r\nTECNOPARQUE NODO CALI 2022 Powered By HJR.";


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void     LED_On(void);
void     LED_Off(void);
#if (USE_TIMEOUT == 1)
void     LED_Blinking(uint32_t Period);
#endif 
void     WaitForUserButtonPress(void);
void     BufferTransfer(void);

int main(void)
{
  

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);


  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART1_UART_Init();

  LED_Off();

  while (1)
  {
    WaitForUserButtonPress();
    BufferTransfer();
  }

}

/*
*seleccion de fuente del reloj

*/
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
  {
  }

  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_MSI_Enable();

  while(LL_RCC_MSI_IsReady() != 1)
  {
  }

  LL_RCC_MSI_EnableRangeSelection();
  LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_11);
  LL_RCC_MSI_SetCalibTrimming(0);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_MSI);

   
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_MSI)
  {
  }

  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAHB3Prescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  LL_Init1msTick(48000000);

 
  LL_SetSystemCoreClock(48000000);
}

/**
  * @Se inicializa la USART 1 conectada a los pines RTX y TX del conector de arduinto
  * @
  * @
  */
static void MX_USART1_UART_Init(void)
{



  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK2);

  
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  /**USART1 cofiguracion del gpio
  PB7   ------> USART1_RX
  PB6   ------> USART1_TX
  debe verificarse este tipo de cambio, las pruebas actuales se hicieron en hardware
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7|LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 9600;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART1);
  LL_USART_ConfigAsyncMode(USART1);


  LL_USART_Enable(USART1);

  while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))))
  {
  }
 

}

/**
  * @brief Inicializa el GPIO
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);


  LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);


  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);


  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);


  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_0;
  EXTI_InitStruct.Line_32_63 = LL_EXTI_LINE_NONE;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);


  LL_GPIO_SetPinPull(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin, LL_GPIO_PULL_UP);


  LL_GPIO_SetPinMode(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin, LL_GPIO_MODE_INPUT);

  NVIC_SetPriority(EXTI0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(EXTI0_IRQn);

}


void LED_On(void)
{
  
  LL_GPIO_SetOutputPin(LED2_GPIO_Port, LED2_Pin);
}


void LED_Off(void)
{

  LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);
}

#if (USE_TIMEOUT == 1)

void LED_Blinking(uint32_t Period)
{
  
  while (1)
  {
    LL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    LL_mDelay(Period);
  }
}
#endif 



void WaitForUserButtonPress(void)
{
  while (ubButtonPress == 0)
  {
  }
  ubSend = 0;
}


void BufferTransfer(void)
{

  
  while (ubSend < sizeof(aStringToSend))
  {
#if (USE_TIMEOUT == 1)
    Timeout = USART_SEND_TIMEOUT_TXE_MS;
#endif 

    
    while (!LL_USART_IsActiveFlag_TXE(USART1))
    {
#if (USE_TIMEOUT == 1)
      
      if (LL_SYSTICK_IsActiveCounterFlag())
      {
        if (Timeout-- == 0)
        {
         
          LED_Blinking(LED_BLINK_SLOW);
        }
      }
#endif 
    }

   
    if (ubSend == (sizeof(aStringToSend) - 1))
    {
      LL_USART_ClearFlag_TC(USART1);
    }

    
    LL_USART_TransmitData8(USART1, aStringToSend[ubSend++]);
  }

#if (USE_TIMEOUT == 1)
  Timeout = USART_SEND_TIMEOUT_TC_MS;
#endif 

  /* Wait for TC flag to be raised for last char */
  while (!LL_USART_IsActiveFlag_TC(USART1))
  {
#if (USE_TIMEOUT == 1)
   
    if (LL_SYSTICK_IsActiveCounterFlag())
    {
      if (Timeout-- == 0)
      {
        
        LED_Blinking(LED_BLINK_SLOW);
      }
    }
#endif 
  }

  ubButtonPress = 0;


  LED_On();
}


void UserButton_Callback(void)
{
  
  ubButtonPress = 1;
}

void Error_Handler(void)
{
  
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{
  
}
#endif /

/************************ LUXOFT SCREEN LAB 2022 *****END OF FILE****/
