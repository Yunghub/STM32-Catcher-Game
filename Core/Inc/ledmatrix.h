#ifndef LEDMATRIX_H
#define LEDMATRIX_H

#include "main.h"  // Ensure this provides required HAL definitions

/* LED Matrix Constants */
#define LEDMAT_ADD  (0x75 << 1)
#define PAGE_1      0x00
#define FUN_REG     0x0B
#define COM_REG     0xFD

/* LED matrix row and column mapping arrays */
extern const uint8_t MAT_ROW[11];
extern const uint8_t MAT_COL[7];

/* Digit patterns for score display (5 rows x 3 columns per digit) */
extern const uint8_t digitPatterns[10][5][3];

/* Global variable holding the current screen state */
extern uint8_t screenstatus[11];

/* Function prototypes */
void LEDMatrix_ClearScreen(void);
void LEDMatrix_TurnOff(void);
void LEDMatrix_TurnOn(void);
void LEDMatrix_AddPixel(uint8_t row, uint8_t col);
void LEDMatrix_RemovePixel(uint8_t row, uint8_t col);
void LEDMatrix_DrawPaddle(uint8_t centerCol);
void LEDMatrix_DrawDigit(uint8_t digit, uint8_t topRow, uint8_t leftCol);
void LEDMatrix_DisplayScore(uint32_t score);
void LEDMatrix_FlashScreen(uint8_t times);

/*
 * Note: LEDMatrix_FlashScreen uses beepBlocking() (a blocking sound routine)
 * which is assumed to be defined elsewhere. If you decide to move the beep
 * functionality into its own module, update this dependency accordingly.
 */
extern void beepBlocking(void);

#endif /* LEDMATRIX_H */
