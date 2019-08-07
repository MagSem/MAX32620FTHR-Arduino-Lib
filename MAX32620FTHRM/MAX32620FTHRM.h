//=============================================================================>
// 1. Project - start

/*
  MAX32620FTHRM.h - Header File for MAX32620FTHR
  MagSem, 2019
*/

// 1. Project - finish
//=============================================================================<

#ifndef MAX32620FTHRM_H_
#define MAX32620FTHRM_H_

//=============================================================================>
// 2. Includes - start

// Maxim Integrated Package
#include <Arduino.h>          // Maxim mod          
#include "tmr_utils.h"        // Timer utility functions
#include "wdt2.h"             // WDT2 peripheral module API
#include "lp.h"               // High level API for the Lower Power
#include "clkman.h"           // Clock management driver

// 2. Includes - finish
//=============================================================================<

//=============================================================================>
// 3. Definitions - start
#define MAX32620FTHR_PHLD 18	// MAX77650 Power Hold input pin	
#define MAX32620FTHR_INT  19	// MAX77650 IRQ
#define MAX32620FTHR_RLED 20	// User LED RED
#define MAX32620FTHR_GLED 21	// User LED GREEN
#define MAX32620FTHR_BLED 22	// User LED BLUE
#define MAX32620FTHR_PBUT 23	// User PUSH BUTTON

// 3. Definitions - finish
//=============================================================================<

//=============================================================================>
// 4. Globals - start

// MAX32620 Globals
extern uint8_t LED_Status[4];
// MAX77650 Globals
extern boolean PMIC_INT;
extern boolean LED_setup;
extern int MAX77650_GLBL_INT_REG;
extern int MAX77650_CHG_INT_REG;

// 4. Globals - finish
//=============================================================================<

//=============================================================================>
// 5. Functions - start

//----------------------------------------------------------------------------->
// Led
void LED_On(uint8_t i);
void LED_Off(uint8_t i);
void LED_Toggle(uint8_t i);

//----------------------------------------------------------------------------->
// Push Button
uint8_t PB_Get(uint8_t PB, boolean PS);

//----------------------------------------------------------------------------->
// Delay
void delayMaxim(uint32_t ms);

//----------------------------------------------------------------------------->
// Nanoring watchdog timer (WDT2) uses 8KHz NanoRing clock
void WDT2_Setup_Mod(int WakeUp, int WakeUp_Clks, int Reset, int Reset_Clks);
void WDT2_WakupFromLP1();

//----------------------------------------------------------------------------->
// Working Frequency
void Freq_04MHz_Nms(int Nms);
void Freq_96MHz(void);

//----------------------------------------------------------------------------->
// MAX77650 Power Management
void PMIC_IRQ_handler(void);
void PMIC_LED_CH(void);
void MAX77650_Global_Setup(uint8_t Acc_Current);

//----------------------------------------------------------------------------->
// Some Sounds
void Sounds_0(uint8_t pin_tone);
void Sounds_1(uint8_t pin_tone);

// 5. Functions - finish
//=============================================================================<

#endif /*MAX32620FTHRM_H_*/
