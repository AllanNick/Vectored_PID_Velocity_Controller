/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define GblIttLength 20 //Global Interger Length
#define PI 3.1415926535
#define SysDt 20;

#define Default_KP 1;
#define Default_KI 0.2;
#define Default_KD 0.01;
typedef struct PID_Vector{
    float parameters[3];
    //[0]->P, [1]->I, [2]->D
    int16_t history_errs[GblIttLength]; 
    uint16_t pointer;
    uint16_t this_container_len;
    //For interger process
    uint16_t Access_tag_ID;
}PIDVexHandleTypedef;

typedef struct Velocies_Target_Vector{
    float velocies_of_whls[4];
    float velocies_of_targs[4];
    float velocies_of_deltas[4];
}VelcVexTypedef;

typedef struct Differential_Control_Interface{
    int16_t Differential_Layer[4];
    int16_t Velocity_Restriction[4];
    int16_t Targ_Velocity_Layer[4];
    int16_t Velocity_Targ;
    int16_t Rotation_Targ;
}DCIHandleTypedef;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
PIDVexHandleTypedef PID0;
PIDVexHandleTypedef PID1;
PIDVexHandleTypedef PID2;
PIDVexHandleTypedef PID3;

VelcVexTypedef Wheels_Velocities;

DCIHandleTypedef DCIController;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int16_t EC_Vloc[4] = {0,0,0,0};
TIM_HandleTypeDef *Timer_Handlers[4] = {&htim1,&htim3,&htim4,&htim8};
PIDVexHandleTypedef *PID_Devices_Handlers[4] = {&PID0, &PID1, &PID2, &PID3};
float pid_0o;
float vlc_0o;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Fetch_Velocity(void);

void PID_CON_Init(PIDVexHandleTypedef *PIDController, uint16_t Access_Tag_ID);
void VLC_CON_Init(void);
void DCI_CON_Init(DCIHandleTypedef *DCIC);
void DCI_Targ_Modify_Overwrite(DCIHandleTypedef *DCIC, int16_t Targ_V, int16_t Targ_R);
void DCI_Targ_Modify_Offset(DCIHandleTypedef *DCIC, int16_t Targ_V, int16_t Targ_R);
void DCI_Update_Calc(DCIHandleTypedef *DCIC);
void DCIC_To_VLC_Update(DCIHandleTypedef *DCIC, VelcVexTypedef *VLCC);

void error_update(PIDVexHandleTypedef *TargPIDController);
float error_calc(PIDVexHandleTypedef *PIDController);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM7_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start_IT(&htim7);

  for(int i =0; i <4; i++){
      PID_CON_Init(PID_Devices_Handlers[i],i);
  }
  VLC_CON_Init();
  DCI_CON_Init(&DCIController);

  DCI_Targ_Modify_Offset(&DCIController, 40, 0);
  DCI_Update_Calc(&DCIController);
  DCIC_To_VLC_Update(&DCIController, &Wheels_Velocities);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_Delay(20);
    Fetch_Velocity();
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Fetch_Velocity(void){
  HAL_TIM_Base_Init(&htim7);
  for(int i=0;i<4;i++){
    __HAL_TIM_SetCounter(Timer_Handlers[i], 0);
  }
  HAL_TIM_Base_Start_IT(&htim7);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  __disable_irq();
  if(htim -> Instance == TIM7){
    for(int i=0;i<4;i++){
      Wheels_Velocities.velocies_of_whls[i] = (int16_t)__HAL_TIM_GetCounter(Timer_Handlers[i]);
      //PID_Devices_Handlers[i]->history_errs[PID_Devices_Handlers[i]->pointer] = (int16_t)__HAL_TIM_GetCounter(Timer_Handlers[i]) + Wheels_Velocities.velocies_of_deltas[i];      
      error_calc(PID_Devices_Handlers[i]);
      error_update(PID_Devices_Handlers[i]);
    }
    //ec_vloc = __HAL_TIM_GetCounter(&htim1);
    }
  __enable_irq();
}


void PID_CON_Init(PIDVexHandleTypedef *PIDController, uint16_t Access_Tag_ID){
    PIDController->parameters[0] = Default_KP;
    PIDController->parameters[1] = Default_KI;
    PIDController->parameters[2] = Default_KD;
    for(int i =0; i < GblIttLength; i++){
        PIDController->history_errs[i] = 0;
    }
    PIDController->this_container_len = GblIttLength-1;
    PIDController->Access_tag_ID = Access_Tag_ID;
}

void VLC_CON_Init(void){
    for(int i =0; i <4; i++){
        Wheels_Velocities.velocies_of_deltas[i] = 0;
        Wheels_Velocities.velocies_of_targs[i] = 0;
        Wheels_Velocities.velocies_of_whls[i] = 0;
    }
}

void DCI_CON_Init(DCIHandleTypedef *DCIC){
    for(int i = 0; i<4 ;i++){
        DCIC->Differential_Layer[i] = 0;
        DCIC->Targ_Velocity_Layer[i] =0;
        DCIC->Velocity_Restriction[i] = 100;
    }
    DCIC->Rotation_Targ = 0;
    DCIC->Velocity_Targ = 0;
}

//xxxxxxxBUG CODExxxxxxxxxx
void DCIC_To_VLC_Update(DCIHandleTypedef *DCIC, VelcVexTypedef *VLCC){
    for(int i =0; i<4; i++){
        VLCC->velocies_of_targs[i] = DCIC->Targ_Velocity_Layer[i];
    }
}

void DCI_Targ_Modify_Overwrite(DCIHandleTypedef *DCIC, int16_t Targ_V, int16_t Targ_R){
    DCIC->Velocity_Targ = Targ_V;
    DCIC->Rotation_Targ = Targ_R;
}

void DCI_Targ_Modify_Offset(DCIHandleTypedef *DCIC, int16_t Targ_V, int16_t Targ_R){
    DCIC->Velocity_Targ = DCIC->Velocity_Targ + Targ_V;
    DCIC->Rotation_Targ = DCIC->Rotation_Targ + Targ_R;
}

void DCI_Update_Calc(DCIHandleTypedef *DCIC){
    //DIC LOCIC PART
    //PREVILIAGE OF 'R' HIGHER THAN 'V'
    if(abs(DCIC->Velocity_Targ)>100){
        if(DCIC->Velocity_Targ>100){
            DCIC->Velocity_Targ = 100;
        }else{
            DCIC->Velocity_Targ = -100;
        }
    }
    for(int i =0;i <4; i++){
        DCIC->Targ_Velocity_Layer[i] = DCIC->Velocity_Targ;
    }
}

void error_update(PIDVexHandleTypedef *TargPIDController){
    uint16_t Data_Index = TargPIDController->Access_tag_ID;
    if(TargPIDController->pointer >= GblIttLength-1){
        TargPIDController->pointer = 0;
    }else{
        TargPIDController->pointer ++;
    }
    
    //calculate velocities then transmit to PID Controller history
    float delta = Wheels_Velocities.velocies_of_targs[Data_Index] - Wheels_Velocities.velocies_of_whls[Data_Index]; 
    Wheels_Velocities.velocies_of_deltas[Data_Index] = delta;
        
    //to access PID's velocity: Wheels_Velocities.velocies_of_whls[Data_Index];
    TargPIDController->history_errs[TargPIDController->pointer] = delta;
    
}

float error_calc(PIDVexHandleTypedef *PIDController){
    float result = 0;
    float itteration_val = 0;
    for(int i =0; i < PIDController->this_container_len; i++){
        itteration_val += PIDController->history_errs[i];
    }
    float this_error = PIDController->history_errs[PIDController->pointer];
    result = PIDController->parameters[0] * this_error
            +PIDController->parameters[1] * itteration_val
            +PIDController->parameters[2] * this_error/SysDt;
    if( PIDController->Access_tag_ID ==0){
      pid_0o = result;
      vlc_0o = Wheels_Velocities.velocies_of_whls[0];
    }
    return result;
}
//global functions declearations:

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
