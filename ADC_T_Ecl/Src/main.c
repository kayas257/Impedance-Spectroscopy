
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * Author          : Kayas Ahmed
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "arm_math.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define U0 3
#define ARM_MATH_CM4
#define DACSAMPLE 32
#define ADCSAMPLE 320
#define FREQ 100000 //Frequency for signal
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void Set_DAC_Freq(uint32_t );
void Sin_Gen(void);
void calc();
void DFT();
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint16_t NUM_SAMPLES_DAC =DACSAMPLE;
uint16_t NUM_SAMPLES_ADC =ADCSAMPLE;
uint8_t no=0;
uint32_t frequen[5]={10000,25000,50000,75000,100000};
uint32_t ADC1ConvertedValues[ADCSAMPLE];
uint16_t Sine_Lut[DACSAMPLE];
uint16_t  Voltage_val[ADCSAMPLE], Current_val[ADCSAMPLE];
uint32_t  FFT_Data[ADCSAMPLE];
float X_Real_V , X_Imag_V,X_Real_C , X_Imag_C,Phase_V,Abs_C,Abs_V;
float Imp, Phase_V, Phase_C, Phase, impe_freq[5];
volatile int busy=0,flag=0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
//Generates Sine wave

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();

  /* USER CODE BEGIN 2 */

  /* SETTING UP THE CONDITIONS */
  Sin_Gen();
  Set_DAC_Freq(FREQ);

  /* STARTING HARDWARE COMPONENTS */
  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)Sine_Lut, NUM_SAMPLES_DAC, DAC_ALIGN_12B_R);
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_ADCEx_MultiModeStart_DMA(&hadc1, ADC1ConvertedValues, (sizeof(ADC1ConvertedValues)));
  HAL_TIM_Base_Start(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
/*
 * This Function Generates Sine Signal
 *
 *
 */
void Sin_Gen()
{
	//gernerate LUT
  	for (int i=0; i< NUM_SAMPLES_DAC; i++)
  	{
  		Sine_Lut[i] = (0.8*cosf(((2*PI)/NUM_SAMPLES_DAC)*i)+1)*682;
  	}
}

/*
 * This Function sets DAC signal frequency by setting appropriate value in counter
 *
 *
 */
void Set_DAC_Freq(uint32_t freq)
{
	uint32_t	CP;

	//calculating the Counter Period for specific frequency
	CP=(RCC_FREQ/(NUM_SAMPLES_DAC*freq))-1;

	//setting Counter Period
	MX_TIM6_Init();
	htim6.Init.Period=CP;

	if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}


int h=0;
/*
 * Callback function after finishing sampling of ADC
 *  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

  if(hadc->Instance==ADC1)
   {


        if(h==5)
        {
        	calc();
	        HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_8);	//LED shows calculation are done
         	h=0;
         }
   h++;
   }
}

  /* NOTE : This function should not be modified. When the callback is needed,
            function HAL_ADC_ConvCpltCallback must be implemented in the user file.
   */



/*
 * This function separates ADC1 and ADC2 values
 */
void calc(){
    
    //separating values of the two ADCs
    for(int i=0;i<ADCSAMPLE;i++)
    {
	Voltage_val[i]=ADC1ConvertedValues[i]&0xffff;
        Current_val[i]=(ADC1ConvertedValues[i]&0xffff0000)>>16;

    }
    DFT();
    }

uint32_t N=ADCSAMPLE;

/*
 * Only calculating DFT for 10th BIN ie k=10, as it is our desired frequency
 */
void DFT()
{

	    X_Real_V = X_Imag_V=0.0;
	    X_Real_C = X_Imag_C=0.0;

	    //calculating DFT
	    for(int n=0;n<ADCSAMPLE;n++)
	     {
	       X_Real_V+=Voltage_val[n]*cos((2*PI*10*(n))/ADCSAMPLE);
	       X_Imag_V+=Voltage_val[n]*sin((2*PI*10*(n))/ADCSAMPLE);
	       X_Real_C+=-(Current_val[n]*cos((2*PI*10*(n))/ADCSAMPLE));
	       X_Imag_C+=-(Current_val[n]*sin((2*PI*10*(n))/ADCSAMPLE));

	     }

	    //getting average imaginary and real Voltage
	    X_Imag_V=(2*X_Imag_V/ADCSAMPLE)*U0/4095;
	    X_Real_V=(2*X_Real_V/ADCSAMPLE)*U0/4095;

	    //getting average real Voltage from Current to Voltage conv.
	    X_Imag_C=(2*X_Imag_C/ADCSAMPLE)*U0/4095;
	    X_Real_C=(2*X_Real_C/ADCSAMPLE)*U0/4095;

	    X_Imag_V=X_Imag_V*(-1.0);		//imaginary values are negative
	    X_Imag_C=X_Imag_C*(-1.0);

	    //getting magnitudes
	    Abs_V=sqrt((X_Imag_V*X_Imag_V)+(X_Real_V*X_Real_V));
	    Abs_C=sqrt((X_Imag_C*X_Imag_C)+(X_Real_C*X_Real_C));
            Imp=Abs_V/(Abs_C/910);			//Abs_C Value divided by Shunt-Resistor = I

	    //phase calculations
	    Phase_V = atan(X_Imag_V/X_Real_V)*180/PI;
		Phase_C = atan(X_Imag_C/X_Real_C)*180/PI;
		Phase = (Phase_V-Phase_C);

        busy=0;
        flag=0;

}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
