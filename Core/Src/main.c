/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Catcher game with PB5 short press (decrease delay)
  *                   and long press (game over) functionality.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// (Any custom typedefs can be placed here)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC

#define LONG_PRESS_THRESHOLD 1000  // Long press threshold in ms
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// (Any custom macros)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Global variables for joystick readings */
uint16_t Xaxis = 0;
uint16_t Yaxis = 0;
uint16_t rawValues[2];
char msg[40];

bool beepActive = false;
uint32_t beepStartTick = 0;
const uint32_t beepDuration = 100; // Duration in ms for the nonblocking beep

/* Global variables for LED matrix */
uint8_t screenstatus[11] = {0};
bool paused = false;  // Global pause flag, initialized to false

/* Game speed variables moved to global scope so they are accessible in callbacks */
uint32_t gameDelay = 150; // Global frame delay in ms (initial game speed)
const uint32_t minDelay = 50; // Minimum delay (max game speed)

/* LED matrix constants */
static const uint8_t LEDMAT_ADD = 0x75 << 1;
static const uint8_t PAGE_1 = 0x00;
static const uint8_t FUN_REG = 0x0B;
static const uint8_t COM_REG = 0xFD;
static const uint8_t MAT_ROW[11] = {0x00, 0x02, 0x04, 0x06, 0x08, 0x0A,
                                    0x01, 0x03, 0x05, 0x07, 0x09};
static const uint8_t MAT_COL[7] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40};

/* Digit patterns for score display (5 rows x 3 columns per digit) */
const uint8_t digitPatterns[10][5][3] = {
  { // 0
    {1,1,1},
    {1,0,1},
    {1,0,1},
    {1,0,1},
    {1,1,1}
  },
  { // 1
    {0,1,0},
    {1,1,0},
    {0,1,0},
    {0,1,0},
    {1,1,1}
  },
  { // 2
    {1,1,1},
    {0,0,1},
    {1,1,1},
    {1,0,0},
    {1,1,1}
  },
  { // 3
    {1,1,1},
    {0,0,1},
    {1,1,1},
    {0,0,1},
    {1,1,1}
  },
  { // 4
    {1,0,1},
    {1,0,1},
    {1,1,1},
    {0,0,1},
    {0,0,1}
  },
  { // 5
    {1,1,1},
    {1,0,0},
    {1,1,1},
    {0,0,1},
    {1,1,1}
  },
  { // 6
    {1,1,1},
    {1,0,0},
    {1,1,1},
    {1,0,1},
    {1,1,1}
  },
  { // 7
    {1,1,1},
    {0,0,1},
    {0,1,0},
    {0,1,0},
    {0,1,0}
  },
  { // 8
    {1,1,1},
    {1,0,1},
    {1,1,1},
    {1,0,1},
    {1,1,1}
  },
  { // 9
    {1,1,1},
    {1,0,1},
    {1,1,1},
    {0,0,1},
    {1,1,1}
  }
};

/* Variables for PB5 press detection */
uint32_t pb5_press_time = 0;
bool pb5_pressed_flag = false;
bool pb5_long_press_executed = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
/* Custom function prototypes */
static void clearscreen(void);
static void turnoffscreen(void);
static void turnonscreen(void);
static void addpixel(uint8_t r, uint8_t c);
static void removepixel(uint8_t r, uint8_t c);
static void drawPaddle(uint8_t centerCol);
static void drawDigit(uint8_t digit, uint8_t topRow, uint8_t leftCol);
static void displayScore(uint32_t score);
static void flashScreen(uint8_t times);
void beep(uint32_t score);
void beepBlocking(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void beep(uint32_t score)
{
  if (!beepActive)
  {
    beepActive = true;
    beepStartTick = HAL_GetTick();

    // Calculate tone period based on score (lower period means higher pitch)
    uint32_t basePeriod = 3000; // Start with a low tone
    uint32_t decrement  = 50;
    uint32_t minPeriod  = 300;  // Clamp to a high tone at minimum period
    uint32_t tonePeriod;

    if (score * decrement < (basePeriod - minPeriod))
    {
      tonePeriod = basePeriod - score * decrement;
    }
    else
    {
      tonePeriod = minPeriod;
    }

    __HAL_TIM_SET_AUTORELOAD(&htim3, tonePeriod);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, tonePeriod / 2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  }
}

void beepBlocking(void)
{
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, htim3.Init.Period / 2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_Delay(300);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == B1_Pin)
  {
    paused = !paused;
  }
  // Removed PB5 interrupt-based code.
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    Xaxis = rawValues[0];
    Yaxis = rawValues[1];
    // sprintf(msg, "X: %hu, Y: %hu\r\n", Xaxis, Yaxis);
    // HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
  }
}

static void clearscreen(void)
{
  uint8_t data[2];
  for (uint8_t r = 0; r < 11; r++)
  {
    data[0] = COM_REG;
    data[1] = PAGE_1;
    HAL_I2C_Master_Transmit(&hi2c1, LEDMAT_ADD, data, 2, HAL_MAX_DELAY);
    data[0] = MAT_ROW[r];
    data[1] = 0x00;
    HAL_I2C_Master_Transmit(&hi2c1, LEDMAT_ADD, data, 2, HAL_MAX_DELAY);
    screenstatus[r] = 0;
  }
}

static void turnoffscreen(void)
{
  uint8_t data[2];
  data[0] = COM_REG;
  data[1] = FUN_REG;
  HAL_I2C_Master_Transmit(&hi2c1, LEDMAT_ADD, data, 2, HAL_MAX_DELAY);

  data[0] = 0x0A;
  data[1] = 0x00;
  HAL_I2C_Master_Transmit(&hi2c1, LEDMAT_ADD, data, 2, HAL_MAX_DELAY);
}

static void turnonscreen(void)
{
  uint8_t data[2];
  data[0] = COM_REG;
  data[1] = FUN_REG;
  HAL_I2C_Master_Transmit(&hi2c1, LEDMAT_ADD, data, 2, HAL_MAX_DELAY);

  data[0] = 0x0A;
  data[1] = 0x01;
  HAL_I2C_Master_Transmit(&hi2c1, LEDMAT_ADD, data, 2, HAL_MAX_DELAY);
}

static void addpixel(uint8_t r, uint8_t c)
{
  uint8_t data[2];
  screenstatus[r] |= MAT_COL[c];
  data[0] = COM_REG;
  data[1] = PAGE_1;
  HAL_I2C_Master_Transmit(&hi2c1, LEDMAT_ADD, data, 2, HAL_MAX_DELAY);
  data[0] = MAT_ROW[r];
  data[1] = screenstatus[r];
  HAL_I2C_Master_Transmit(&hi2c1, LEDMAT_ADD, data, 2, HAL_MAX_DELAY);
}

static void removepixel(uint8_t r, uint8_t c)
{
  uint8_t data[2];
  screenstatus[r] &= ~MAT_COL[c];
  data[0] = COM_REG;
  data[1] = PAGE_1;
  HAL_I2C_Master_Transmit(&hi2c1, LEDMAT_ADD, data, 2, HAL_MAX_DELAY);
  data[0] = MAT_ROW[r];
  data[1] = screenstatus[r];
  HAL_I2C_Master_Transmit(&hi2c1, LEDMAT_ADD, data, 2, HAL_MAX_DELAY);
}

static void drawPaddle(uint8_t centerCol)
{
  if (centerCol == 0)
  {
    addpixel(10, 0);
    addpixel(10, 1);
    addpixel(10, 2);
  }
  else if (centerCol >= 6)
  {
    addpixel(10, 4);
    addpixel(10, 5);
    addpixel(10, 6);
  }
  else
  {
    addpixel(10, centerCol - 1);
    addpixel(10, centerCol);
    addpixel(10, centerCol + 1);
  }
}

static void drawDigit(uint8_t digit, uint8_t topRow, uint8_t leftCol)
{
  if (digit > 9) return;
  for (uint8_t i = 0; i < 5; i++)
  {
    for (uint8_t j = 0; j < 3; j++)
    {
      if (digitPatterns[digit][i][j])
      {
        addpixel(topRow + i, leftCol + j);
      }
    }
  }
}

static void displayScore(uint32_t score)
{
  clearscreen();
  uint8_t topRow = 3;
  if (score < 10)
  {
    drawDigit((uint8_t)score, topRow, 2);
  }
  else
  {
    uint8_t tens = (score / 10) % 10;
    uint8_t ones = score % 10;
    drawDigit(tens, topRow, 0);
    drawDigit(ones, topRow, 4);
  }
}

static void flashScreen(uint8_t times)
{
  uint8_t data[2];
  uint32_t defaultPeriod = htim3.Init.Period;
  uint32_t notePeriods[3] = {999, 1249, 1666};
  uint32_t noteDurations[3] = {150, 150, 300};

  for (uint8_t t = 0; t < times; t++)
  {
    for (uint8_t r = 0; r < 11; r++)
    {
      data[0] = COM_REG;
      data[1] = PAGE_1;
      HAL_I2C_Master_Transmit(&hi2c1, LEDMAT_ADD, data, 2, HAL_MAX_DELAY);
      data[0] = MAT_ROW[r];
      data[1] = 0x7F;
      HAL_I2C_Master_Transmit(&hi2c1, LEDMAT_ADD, data, 2, HAL_MAX_DELAY);
      screenstatus[r] = 0x7F;
    }
    if (t < 3)
    {
      __HAL_TIM_SET_AUTORELOAD(&htim3, notePeriods[t]);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, notePeriods[t] / 2);
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
      HAL_Delay(noteDurations[t]);
      HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    }
    else
    {
      beepBlocking();
    }
    HAL_Delay(200);
    clearscreen();
    HAL_Delay(200);
  }
  __HAL_TIM_SET_AUTORELOAD(&htim3, defaultPeriod);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  // (Custom pre-initialization code can be added here)
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  // (Additional initialization before system clock config)
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  // (Additional system initialization)
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)rawValues, 2);
  HAL_TIM_Base_Start_IT(&htim2);
  turnonscreen();

  /* Game variables for paddle and falling object */
  uint8_t paddlePos = 3;
  uint8_t fallRow   = 0;
  uint8_t fallCol   = 3;
  uint32_t scoreLocal = 0;

  /* Joystick thresholds */
  const uint16_t thresholdRight = 2500;
  const uint16_t thresholdLeft  = 1500;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* --- PB5 Button Polling --- */
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET)
    {
      if (!pb5_pressed_flag)
      {
        // Button pressed down for the first time.
        pb5_press_time = HAL_GetTick();
        pb5_pressed_flag = true;
        pb5_long_press_executed = false;
      }
      else
      {
        // Button is still pressed.
        if (!pb5_long_press_executed &&
            (HAL_GetTick() - pb5_press_time >= LONG_PRESS_THRESHOLD))
        {
          // Long press detected: run game-over sequence.
          sprintf(msg, "Game Over! Final Score: %lu\r\n", scoreLocal);
          HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
          flashScreen(3);
          displayScore(scoreLocal);
          HAL_Delay(2000);

          // Reset game variables.
          scoreLocal = 0;
          gameDelay = 150;
          fallRow = 0;
          fallCol = (fallCol + 2) % 7;
          pb5_long_press_executed = true;
        }
      }
    }
    else
    {
      // Button released.
      if (pb5_pressed_flag)
      {
        // If button was released before the long-press threshold, it's a short press.
        if (!pb5_long_press_executed)
        {
          if (gameDelay > minDelay)
          {
            gameDelay -= 10;
          }
        }
        pb5_pressed_flag = false;
        pb5_long_press_executed = false;
      }
    }

    if (!paused)
    {
      clearscreen();

      if (Yaxis > thresholdRight && paddlePos > 0)
      {
        paddlePos--;
      }
      else if (Yaxis < thresholdLeft && paddlePos < 6)
      {
        paddlePos++;
      }
      drawPaddle(paddlePos);
      addpixel(fallRow, fallCol);
      HAL_Delay(gameDelay);

      if (fallRow < 10)
      {
        fallRow++;
      }
      else
      {
        uint8_t catchStart, catchEnd;
        if (paddlePos == 0)
        {
          catchStart = 0;
          catchEnd   = 2;
        }
        else if (paddlePos >= 6)
        {
          catchStart = 4;
          catchEnd   = 6;
        }
        else
        {
          catchStart = paddlePos - 1;
          catchEnd   = paddlePos + 1;
        }

        if (fallCol >= catchStart && fallCol <= catchEnd)
        {
          beep(scoreLocal);
          scoreLocal++;
          sprintf(msg, "Score: %lu\r\n", scoreLocal);
          HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
          if (gameDelay > minDelay)
          {
            gameDelay -= 10;
          }
        }
        else
        {
          sprintf(msg, "Game Over! Final Score: %lu\r\n", scoreLocal);
          HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
          flashScreen(3);
          displayScore(scoreLocal);
          HAL_Delay(2000);
          scoreLocal = 0;
          gameDelay = 150;
        }
        fallRow = 0;
        fallCol = (fallCol + 2) % 7;
      }
    }
    else
    {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      HAL_Delay(200);
    }

    if (beepActive && (HAL_GetTick() - beepStartTick >= beepDuration))
    {
      HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
      beepActive = false;
    }
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  // (Additional code if needed)
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 234;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 10;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 84000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 500;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* Additional user code can be added here */
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
  /* User can add his own implementation to report the file name and line number */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
