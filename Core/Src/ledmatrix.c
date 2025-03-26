#include "ledmatrix.h"

/* Extern declarations for required peripheral handles */
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim3;

/* Global variable definitions */
uint8_t screenstatus[11] = {0};

/* Constant arrays for the LED matrix */
const uint8_t MAT_ROW[11] = {0x00, 0x02, 0x04, 0x06, 0x08, 0x0A,
                             0x01, 0x03, 0x05, 0x07, 0x09};

const uint8_t MAT_COL[7] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40};

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

/* LED Matrix Functions Implementation */

void LEDMatrix_ClearScreen(void)
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

void LEDMatrix_TurnOff(void)
{
  uint8_t data[2];
  data[0] = COM_REG;
  data[1] = FUN_REG;
  HAL_I2C_Master_Transmit(&hi2c1, LEDMAT_ADD, data, 2, HAL_MAX_DELAY);

  data[0] = 0x0A;
  data[1] = 0x00;
  HAL_I2C_Master_Transmit(&hi2c1, LEDMAT_ADD, data, 2, HAL_MAX_DELAY);
}

void LEDMatrix_TurnOn(void)
{
  uint8_t data[2];
  data[0] = COM_REG;
  data[1] = FUN_REG;
  HAL_I2C_Master_Transmit(&hi2c1, LEDMAT_ADD, data, 2, HAL_MAX_DELAY);

  data[0] = 0x0A;
  data[1] = 0x01;
  HAL_I2C_Master_Transmit(&hi2c1, LEDMAT_ADD, data, 2, HAL_MAX_DELAY);
}

void LEDMatrix_AddPixel(uint8_t r, uint8_t c)
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

void LEDMatrix_RemovePixel(uint8_t r, uint8_t c)
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

void LEDMatrix_DrawPaddle(uint8_t centerCol)
{
  if (centerCol == 0)
  {
    LEDMatrix_AddPixel(10, 0);
    LEDMatrix_AddPixel(10, 1);
    LEDMatrix_AddPixel(10, 2);
  }
  else if (centerCol >= 6)
  {
    LEDMatrix_AddPixel(10, 4);
    LEDMatrix_AddPixel(10, 5);
    LEDMatrix_AddPixel(10, 6);
  }
  else
  {
    LEDMatrix_AddPixel(10, centerCol - 1);
    LEDMatrix_AddPixel(10, centerCol);
    LEDMatrix_AddPixel(10, centerCol + 1);
  }
}

void LEDMatrix_DrawDigit(uint8_t digit, uint8_t topRow, uint8_t leftCol)
{
  if (digit > 9) return;
  for (uint8_t i = 0; i < 5; i++)
  {
    for (uint8_t j = 0; j < 3; j++)
    {
      if (digitPatterns[digit][i][j])
      {
        LEDMatrix_AddPixel(topRow + i, leftCol + j);
      }
    }
  }
}

void LEDMatrix_DisplayScore(uint32_t score)
{
  LEDMatrix_ClearScreen();
  uint8_t topRow = 3;
  if (score < 10)
  {
    LEDMatrix_DrawDigit((uint8_t)score, topRow, 2);
  }
  else
  {
    uint8_t tens = (score / 10) % 10;
    uint8_t ones = score % 10;
    LEDMatrix_DrawDigit(tens, topRow, 0);
    LEDMatrix_DrawDigit(ones, topRow, 4);
  }
}

void LEDMatrix_FlashScreen(uint8_t times)
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
      /*
       * If no LED-matrix–related beep is desired here,
       * you can either remove this call or implement beepBlocking()
       * in the appropriate module.
       */
      beepBlocking();
    }
    HAL_Delay(200);
    LEDMatrix_ClearScreen();
    HAL_Delay(200);
  }
  __HAL_TIM_SET_AUTORELOAD(&htim3, defaultPeriod);
}
