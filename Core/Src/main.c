/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "dma.h"
#include "fdcan.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dm_motor_ctrl.h"
#include "motor.h"
#include "tim.h"
#include "remote_control.h"
#include "vofa.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 电机参数,必须和达妙调试助手里面的设置保持一致 */

/* 底板可控电源输出(对应官方原理图 P7/P8 两路 XT30(2+2) 可控输出):
     PC14 = Power_OUT1_EN → PMOS Q3 → VCC_OUT1 → P7, 该口旁边 LED1 亮
     PC13 = Power_OUT2_EN → PMOS Q4 → VCC_OUT2 → P8, 该口旁边 LED2 亮
   两个都拉高, 两个口就都有24V, 电机插哪个口都能转.
   PC15 = Power_5V_EN 控制的是"对外5V", 板上 D-BUS/SBUS 三针口的 5V 就是这一路,
   遥控接收机靠这个口供电。这里不开, 接收机不上电, UART5(PD2)就一直是空闲高电平,
   一个字节都收不到 → 遥控一直显示离线 */
#define DM_POWER_OUT2_ENABLE   1
#define DM_POWER_5V_ENABLE     1

#define DM_POWER_PORT   GPIOC
#define DM_POWER_PIN0   GPIO_PIN_14     /* 24V输出1使能 Power_OUT1_EN */
#define DM_POWER_PIN1   GPIO_PIN_13     /* 24V输出2使能 Power_OUT2_EN */
#define DM_POWER_PIN2   GPIO_PIN_15     /* 对外5V使能 Power_5V_EN(给遥控接收机供电) */

/* 状态指示灯: DM-H7 板上没有确认的用户LED(只有WS2812彩灯), 就先关掉 */
#define DM_STATUS_LED_ENABLE   0
#define DM_STATUS_LED_PORT     GPIOC
#define DM_STATUS_LED_PIN      GPIO_PIN_00

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
static void LedTestInit(void);
static void CanStartup(void);
static void MotorKeepAlive(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* 打开底板可控电源输出, 见文档9.1
   达妙底板把电机的24V供电做成了可控输出, 使能脚不拉高电机就不上电(DM-H7 上是 PC14) */
void power(uint8_t enable)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  uint32_t pins = DM_POWER_PIN0;

  __HAL_RCC_GPIOC_CLK_ENABLE();

#if DM_POWER_OUT2_ENABLE
  pins |= DM_POWER_PIN1;          /* 第二路24V输出(PC13)一起打开 */
#endif
#if DM_POWER_5V_ENABLE
  pins |= DM_POWER_PIN2;          /* 对外5V(PC15): 给遥控接收机供电 */
#endif

  GPIO_InitStruct.Pin = pins;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DM_POWER_PORT, &GPIO_InitStruct);

  HAL_GPIO_WritePin(DM_POWER_PORT, pins, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* 状态灯引脚初始化(默认关闭, 见 DM_STATUS_LED_ENABLE) */
static void LedTestInit(void)
{
#if DM_STATUS_LED_ENABLE
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitStruct.Pin = DM_STATUS_LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DM_STATUS_LED_PORT, &GPIO_InitStruct);
#endif
}

/* 状态灯指示,主循环里反复调用(不阻塞),用闪烁快慢报出卡在哪一环(period=亮/灭各自的ms数):
   慢闪 500ms = 总线上一帧都没收到 → 查CAN物理层(收发器/接线/共地/终端电阻)、波特率、24V供电
   中闪 300ms = 收到总线上的帧,但帧id不是反馈id → 电机CAN ID / Master ID 填错了
   快闪 150ms = 收到反馈但电机没使能
   极快 50ms  = 电机已经使能(这时还不转就是控制模式或参数问题)
   15ms       = 电机报了错误代码(过压/欠压/过流/过温/通信丢失/过载) */
static void LedIndicator(void)
{
#if DM_STATUS_LED_ENABLE
  static uint32_t tick;
  uint32_t period;

  if (dm_motor_rx_count() == 0)
    period = (dm_motor_rx_any() == 0) ? 500 : 300;
  else if (motor[Motor1].para.state == 0)
    period = 150;
  else if (motor[Motor1].para.state == 1)
    period = 50;
  else
    period = 15;

  if (HAL_GetTick() - tick >= period)
  {
    tick = HAL_GetTick();
    HAL_GPIO_TogglePin(DM_STATUS_LED_PORT, DM_STATUS_LED_PIN);
  }
#endif
}

/* 文档9.1的电机初始化流程(每换一次CAN引脚都要重来一遍, 否则命令发在了错误的引脚上):
   写控制模式寄存器 → 读回寄存器 → 失能 → 保存 → 使能 → 再读回寄存器
   注意: 这里写的必须是 motor_control.c 里配的那个模式(当前是 mit_mode=1 MIT模式).
         以前这里固定写 DM_CTRL_MODE_REG_MIT(1), 而 motor_control() 发的是速度帧,
         模式对不上电机就不理这帧 → 电机不转. 现在两边都从 ctrl.mode 取值 */
static void CanStartup(void)
{
  dm_log("\r\n--- CanStartup on %s ---\r\n", dm_can_get_pins() ? "PD0/PD1" : "PA11/PA12");

  /* 把 motor_control.c 里配置的控制模式写进电机寄存器10(CTRL_MODE) */
  write_motor_data(motor[Motor1].id, RID_CMODE, (uint8_t)motor[Motor1].ctrl.mode, 0, 0, 0);
  HAL_Delay(100);

  /* 先读回一次: 确认电机CAN ID / Master ID / 控制模式 / 波特率是不是和代码里一致 */
  dm_motor_check_regs(&motor[Motor1]);
  HAL_Delay(100);
  dm_motor_disable(&hfdcan1, &motor[Motor1]);   // 保存参数只在失能状态下生效
  HAL_Delay(100);
  save_motor_data(motor[Motor1].id, RID_CMODE); // 保存电机参数(只保存控制模式这一个寄存器)
  HAL_Delay(100);
  dm_motor_init();                              // 失能会清掉给定值, 这里按配置重新填回
  dm_motor_enable(&hfdcan1, &motor[Motor1]);    // 使能电机
  HAL_Delay(50);
  dm_motor_check_regs(&motor[Motor1]);
  motor_control_softstart();                    // 使能后速度从0爬坡, 避免"抽动"
}

/* 一直收不到反馈时的保活: 重新发一遍控制模式+清错+使能
   (这些都是RAM寄存器/控制帧, 不会写flash, 可以放心重发) */
static void MotorKeepAlive(void)
{
  static uint32_t tick;

  /* 收到过任意帧(或发出去的帧被ACK)就说明链路是活的, 不要插一脚,
     否则每隔2s重新使能一次, 电机会卡顿一下 */
  if ((dm_motor_rx_count() != 0) || (dm_motor_rx_any() != 0) || dm_can_link_ok())
    return;
  if ((HAL_GetTick() - tick) < 2000u)
    return;
  tick = HAL_GetTick();

  /* 一个反馈帧都没收到: 可能只是还没使能, 也可能寄存器10的取值对不上.
     这里固定按 ctrl.mode 重发, 不再来回切模式(速度模式下切模式会让电机乱转) */
  write_motor_data(motor[Motor1].id, RID_CMODE, (uint8_t)motor[Motor1].ctrl.mode, 0, 0, 0);
  dm_motor_clear_err(&hfdcan1, &motor[Motor1]);
  dm_motor_enable(&hfdcan1, &motor[Motor1]);
  motor_control_softstart();                    // 重新使能后从0速爬坡
  dm_log("keepalive: no feedback yet, re-send CTRL_MODE=%u/clear-err/enable on %s\r\n",
         (unsigned)motor[Motor1].ctrl.mode, dm_can_get_pins() ? "PD0/PD1" : "PA11/PA12");
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint8_t can_pins = 0xFF;   /* CanStartup() 最近一次是在哪组CAN引脚上执行的 */
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_DMA_Init();              /* 必须最先: UART5 的收发 DMA 流要用 */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_USART1_UART_Init();
  MX_UART5_Init();            /* 遥控接收 DT7/D-BUS: PD2 + DMA1_Stream0 */
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  LedTestInit();              /* 状态灯引脚(默认关闭, 见 DM_STATUS_LED_ENABLE) */
  power(1);                   /* 打开底板可控24V + 对外5V(遥控接收机供电) */
  HAL_Delay(1000);            /* 等电机上电自检通过 */

  bsp_fdcan_set_baud(&hfdcan1, CAN_CLASS, CAN_BR_1M);   /* FDCAN1 经典模式 1Mbps */
  bsp_can_init();
  dm_motor_init();            /* 电机参数: CAN_ID=0x01, 速度模式 */
  HAL_Delay(10);
  dm_log("\r\n=== DM4310 + DT7: FDCAN1 classic 1Mbps, TIM3 1kHz, CAN_ID=0x%02X ===\r\n",
         (unsigned)motor[Motor1].id);

  /* TIM3 是 1kHz 控制节拍的来源(在 HAL_TIM_PeriodElapsedCallback 里发控制帧).
     TIM3 挂在 APB1 上: 本工程 APB1 不分频时 定时器时钟 = PCLK1 = 72MHz,
     而 tim.c 里写的预分频是按 192MHz 算的, 差了三倍多(变成375Hz).
     这里按实际时钟重算一次, 保证真的是 1kHz */
  {
    uint32_t pclk1  = HAL_RCC_GetPCLK1Freq();
    uint32_t timclk = (pclk1 == HAL_RCC_GetHCLKFreq()) ? pclk1 : (pclk1 * 2u);
    htim3.Init.Prescaler = (timclk / 1000000u) - 1u;   /* 计数时钟 1MHz */
    htim3.Init.Period    = 1000u - 1u;                 /* 1MHz/1000 = 1kHz */
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
    {
      Error_Handler();
    }
  }
  HAL_TIM_Base_Start_IT(&htim3);

  CanStartup();               /* 写控制模式 → 读回 → 失能 → 保存 → 使能 */
  can_pins = dm_can_get_pins();
  HAL_Delay(200);

  /* 遥控接收: UART5(PD2, 板上的 D-BUS/SBUS 口) + DMA双缓冲 + 空闲中断,
     底层见 application/bsp_rc.c, 协议解析见 application/remote_control.c。
     左摇杆竖直通道(RC_CH_LEFT_VERTICAL)给速度给定, 见 motor_control.c */
  remote_control_init();

  vofa_init();                /* VOFA+波形(USART1), 会关掉dm_log文本, 见vofa.h */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE BEGIN 3 */
    LedIndicator();   /* 状态灯(默认关闭) */

    /* CAN引脚自动探测: 达妙底板的收发器在PD0/PD1上, 而CubeMX给FDCAN1的默认引脚是PA11/PA12.
       探测锁定到新引脚后, 之前发到错误引脚上的初始化命令要重发一遍 */
    if (dm_can_probe_poll() && (dm_can_get_pins() != can_pins))
    {
      can_pins = dm_can_get_pins();
      CanStartup();
    }

    MotorKeepAlive();   /* 收不到反馈时定期重发 控制模式/清错/使能 */
    dm_log_status();    /* 1Hz状态日志(USART1: TX=PA9, 115200, 板上那排4pin的 USART1 座) */
    vofa_send();        /* VOFA+波形, 按 VOFA_PERIOD_MS 限速(默认50Hz) */
    /* USER CODE END 3 */
  }
}

/* 1ms定时中断(1kHz): 周期发送控制帧; 接收回调在 dm_motor_ctrl.c 里, 见文档9.1 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    motor_control();   /* 里面读遥控值 + 速度爬坡 + 发速度模式帧 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 13;
  RCC_OscInitStruct.PLL.PLLP = 3;
  /* PLL1Q同时作为FDCAN内核时钟: 216MHz / 4 = 54MHz,
     与fdcan.c中的(预分频3, TSEG1=14, TSEG2=3)配合恰好得到1Mbps,
     达妙电机的CAN波特率默认为1Mbps,改成其他值会导致电机无响应 */
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 4096;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
#ifdef USE_FULL_ASSERT
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