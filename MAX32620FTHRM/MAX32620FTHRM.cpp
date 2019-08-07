/*
  MAX32620FTHRM.cpp - Lib for MAX32620FTHR
  MagSem, 2019
*/

#include <Arduino.h>                  
#include "MAX32620FTHRM.h"
#include <MAX77650-Arduino-Library.h> // PMIC MAX77650

//----------------------------------------------------------------------------->
// User Leds Functions (Maxim Int Compat)
// 0 - White, 1 - Red, 2 - Green, 3 - Blue
void LED_On(uint8_t i)
{
  switch (i)
  {
    case 0:
      digitalWrite(MAX32620FTHR_RLED, LOW);
      digitalWrite(MAX32620FTHR_GLED, LOW);
      digitalWrite(MAX32620FTHR_BLED, LOW);
      LED_Status[0] = 1; LED_Status[1] = 1;
      LED_Status[2] = 1; LED_Status[3] = 1;
      break;
    case 1:
      digitalWrite(MAX32620FTHR_RLED, LOW);
      LED_Status[1] = 1;
      break;
    case 2:
      digitalWrite(MAX32620FTHR_GLED, LOW);
      LED_Status[2] = 1;
      break;
    case 3:
      digitalWrite(MAX32620FTHR_BLED, LOW);
      LED_Status[3] = 1;
      break;
    default:
      ; // do something
  }
}

void LED_Off(uint8_t i)
{
  switch (i)
  {
    case 0:
      digitalWrite(MAX32620FTHR_RLED, HIGH);
      digitalWrite(MAX32620FTHR_GLED, HIGH);
      digitalWrite(MAX32620FTHR_BLED, HIGH);
      LED_Status[0] = 0; LED_Status[1] = 0;
      LED_Status[2] = 0; LED_Status[3] = 0;
      break;
    case 1:
      digitalWrite(MAX32620FTHR_RLED, HIGH);
      LED_Status[1] = 0;
      break;
    case 2:
      digitalWrite(MAX32620FTHR_GLED, HIGH);
      LED_Status[2] = 0;
      break;
    case 3:
      digitalWrite(MAX32620FTHR_BLED, HIGH);
      LED_Status[3] = 0;
      break;
    default:
      ; // do something
  }
}

void LED_Toggle(uint8_t i)
{
  if (LED_Status[i]) LED_Off(i); else LED_On(i);
}
//-----------------------------------------------------------------------------<

//----------------------------------------------------------------------------->
// Switches SW1, SW2, SW3 emulation from One PUSH BUTTON
// PB - Push Button, PS - Press Status
uint8_t PB_Get(uint8_t PB, boolean PS)
{
  uint8_t SW = 0;
  int t_but = 0; int t_but1 = 500; int t_but2 = 1000; int t_but3 = 3000;
  int mils = 0;

  // Check PUSH BUTTON
  if (digitalRead(PB) == PS)
  {
    mils = millis();
    while (digitalRead(PB) == PS)
    {
      t_but = millis() - mils;
    }
  }  

  if ((t_but > 0) and (t_but < t_but1))
  {
    SW = 1;
    goto end;
  }
  if ((t_but > t_but1) and (t_but < t_but2))
  {
    SW = 2;
    goto end;
  }
  if ((t_but > t_but2) and (t_but < t_but3))
  {
    SW = 3;
    goto end;
  }
  ;
end:
  return SW;
}
//-----------------------------------------------------------------------------<

//----------------------------------------------------------------------------->
// Delay Function (Maxim Int Compat)
void delayMaxim(uint32_t ms)
{
  int i;

  for(i = 0; i < ms; i++)
    SYS_SysTick_Delay(94600);
}
//-----------------------------------------------------------------------------<

//----------------------------------------------------------------------------->
// Initialize and setup watchdog wake-up and reset times
// WDT2_Setup_Mod(int WakeUp, int WakeUp_Clks, int Reset, int Reset_Clks)
// WakeUp {0,1}, WakeUp_Clks {0,..., 15}, Reset {0,1}, Reset_Clks {0,..., 15}
// Example:
// WDT2_Setup_Mod(1, 10, 0, 0); // SetUp WakeUp Time 4.096s without Reset
void WDT2_Setup_Mod(int WakeUp, int WakeUp_Clks, int Reset, int Reset_Clks)
{
  int enableInSleep = 1;
  // Initialize watchdog clock for run and sleep mode
  // WDT2 uses 8KHz NanoRing clock  
  WDT2_Init(enableInSleep, MXC_V_WDT2_UNLOCK_KEY);

  // Setup wake-up time
  if (WakeUp)
  {
    switch (WakeUp_Clks)
    {
      case  0: // 2^25/8KHz = 4194.304s = 70min
        WDT2_EnableWakeUp(WDT2_PERIOD_2_25_CLKS, MXC_V_WDT2_UNLOCK_KEY);
        break;
      case  1: // 2^24/8KHz = 2097.152s = 35min
        WDT2_EnableWakeUp(WDT2_PERIOD_2_24_CLKS, MXC_V_WDT2_UNLOCK_KEY);
        break;
      case  2: // 2^23/8KHz = 1048.576s
        WDT2_EnableWakeUp(WDT2_PERIOD_2_23_CLKS, MXC_V_WDT2_UNLOCK_KEY);
        break;
      case  3: // 2^22/8KHz =  524.288s
        WDT2_EnableWakeUp(WDT2_PERIOD_2_22_CLKS, MXC_V_WDT2_UNLOCK_KEY);
        break;
      case  4: // 2^21/8KHz =  262.144s
        WDT2_EnableWakeUp(WDT2_PERIOD_2_21_CLKS, MXC_V_WDT2_UNLOCK_KEY);
        break;
      case  5: // 2^20/8KHz =  131.072s
        WDT2_EnableWakeUp(WDT2_PERIOD_2_20_CLKS, MXC_V_WDT2_UNLOCK_KEY);
        break;
      case  6: // 2^19/8KHz =   65.536s
        WDT2_EnableWakeUp(WDT2_PERIOD_2_19_CLKS, MXC_V_WDT2_UNLOCK_KEY);
        break;
      case  7: // 2^18/8KHz =   32.768s
        WDT2_EnableWakeUp(WDT2_PERIOD_2_18_CLKS, MXC_V_WDT2_UNLOCK_KEY);
        break;
      case  8: // 2^17/8KHz =   16.384s
        WDT2_EnableWakeUp(WDT2_PERIOD_2_17_CLKS, MXC_V_WDT2_UNLOCK_KEY);
        break;
      case  9: // 2^16/8KHz =    8.192s
        WDT2_EnableWakeUp(WDT2_PERIOD_2_16_CLKS, MXC_V_WDT2_UNLOCK_KEY);
        break;
      case 10: // 2^15/8KHz =    4.096s
        WDT2_EnableWakeUp(WDT2_PERIOD_2_15_CLKS, MXC_V_WDT2_UNLOCK_KEY);
        break;
      case 11: // 2^14/8KHz =    2.048s
        WDT2_EnableWakeUp(WDT2_PERIOD_2_14_CLKS, MXC_V_WDT2_UNLOCK_KEY);
        break;
      case 12: // 2^13/8KHz =    1.024s
        WDT2_EnableWakeUp(WDT2_PERIOD_2_13_CLKS, MXC_V_WDT2_UNLOCK_KEY);
        break;
      case 13: // 2^12/8KHz =    0.512s
        WDT2_EnableWakeUp(WDT2_PERIOD_2_12_CLKS, MXC_V_WDT2_UNLOCK_KEY);
        break;
      case 14: // 2^11/8KHz =    0.256s
        WDT2_EnableWakeUp(WDT2_PERIOD_2_11_CLKS, MXC_V_WDT2_UNLOCK_KEY);
        break;
      case 15: // 2^10/8KHz =    0.128s
        WDT2_EnableWakeUp(WDT2_PERIOD_2_10_CLKS, MXC_V_WDT2_UNLOCK_KEY);
        break;
      default:
        ; // do something
    }
  }

  // Setup reset time
  if (Reset)
  {
    switch (Reset_Clks)
    {
      case  0: // 2^25/8KHz = 4194.304s = 70min
        WDT2_EnableReset(WDT2_PERIOD_2_25_CLKS, MXC_V_WDT2_UNLOCK_KEY);
        break;
      case  1: // 2^24/8KHz = 2097.152s = 35min
        WDT2_EnableReset(WDT2_PERIOD_2_24_CLKS, MXC_V_WDT2_UNLOCK_KEY);
        break;
      case  2: // 2^23/8KHz = 1048.576s
        WDT2_EnableReset(WDT2_PERIOD_2_23_CLKS, MXC_V_WDT2_UNLOCK_KEY);
        break;
      case  3: // 2^22/8KHz =  524.288s
        WDT2_EnableReset(WDT2_PERIOD_2_22_CLKS, MXC_V_WDT2_UNLOCK_KEY);
        break;
      case  4: // 2^21/8KHz =  262.144s
        WDT2_EnableReset(WDT2_PERIOD_2_21_CLKS, MXC_V_WDT2_UNLOCK_KEY);
        break;
      case  5: // 2^20/8KHz =  131.072s
        WDT2_EnableReset(WDT2_PERIOD_2_20_CLKS, MXC_V_WDT2_UNLOCK_KEY);
        break;
      case  6: // 2^19/8KHz =   65.536s
        WDT2_EnableReset(WDT2_PERIOD_2_19_CLKS, MXC_V_WDT2_UNLOCK_KEY);
        break;
      case  7: // 2^18/8KHz =   32.768s
        WDT2_EnableReset(WDT2_PERIOD_2_18_CLKS, MXC_V_WDT2_UNLOCK_KEY);
        break;
      case  8: // 2^17/8KHz =   16.384s
        WDT2_EnableReset(WDT2_PERIOD_2_17_CLKS, MXC_V_WDT2_UNLOCK_KEY);
        break;
      case  9: // 2^16/8KHz =    8.192s
        WDT2_EnableReset(WDT2_PERIOD_2_16_CLKS, MXC_V_WDT2_UNLOCK_KEY);
        break;
      case 10: // 2^15/8KHz =    4.096s
        WDT2_EnableReset(WDT2_PERIOD_2_15_CLKS, MXC_V_WDT2_UNLOCK_KEY);
        break;
      case 11: // 2^14/8KHz =    2.048s
        WDT2_EnableReset(WDT2_PERIOD_2_14_CLKS, MXC_V_WDT2_UNLOCK_KEY);
        break;
      case 12: // 2^13/8KHz =    1.024s
        WDT2_EnableReset(WDT2_PERIOD_2_13_CLKS, MXC_V_WDT2_UNLOCK_KEY);
        break;
      case 13: // 2^12/8KHz =    0.512s
        WDT2_EnableReset(WDT2_PERIOD_2_12_CLKS, MXC_V_WDT2_UNLOCK_KEY);
        break;
      case 14: // 2^11/8KHz =    0.256s
        WDT2_EnableReset(WDT2_PERIOD_2_11_CLKS, MXC_V_WDT2_UNLOCK_KEY);
        break;
      case 15: // 2^10/8KHz =    0.128s
        WDT2_EnableReset(WDT2_PERIOD_2_10_CLKS, MXC_V_WDT2_UNLOCK_KEY);
        break;
      default:
        ; // do something
    }
  }

  //start watchdog timer
  WDT2_Start(MXC_V_WDT2_UNLOCK_KEY);

}

void WDT2_WakupFromLP1()
{
  // Clear existing wake-up config
  LP_ClearWakeUpConfig();

  // Clear any event flags
  LP_ClearWakeUpFlags();

  // Reset watchdog before sleep
  WDT2_Reset();

  // Global disable interrupt
  __disable_irq();

  // Enter sleep
  LP_EnterLP1();

  // Global enable interrupt
  __enable_irq();

  // Clear all wake-up flags
  LP_ClearWakeUpFlags();
}
//-----------------------------------------------------------------------------<

//----------------------------------------------------------------------------->
void Freq_04MHz_Nms(int Nms)
{
  // Set the System clock to the 4MHz oscillator
  CLKMAN_SetSystemClock(CLKMAN_SYSTEM_SOURCE_4MHZ, CLKMAN_SYSTEM_SCALE_DIV_1);
  TMR_Delay(MXC_TMR0, MSEC(Nms));
}

void Freq_96MHz(void)
{
  // Set the System clock to the 96MHz oscillator
  CLKMAN_SetSystemClock(CLKMAN_SYSTEM_SOURCE_96MHZ, CLKMAN_SYSTEM_SCALE_DIV_1);
}
//-----------------------------------------------------------------------------<

//----------------------------------------------------------------------------->
void PMIC_IRQ_handler(void)
{
  PMIC_INT = true;
}

void PMIC_LED_CH(void)
{
  if (PMIC_INT)
  {
    PMIC_INT = false;

    //handling the interrupts from the system
    MAX77650_GLBL_INT_REG = MAX77650_getINT_GLBL();  //read global interrupt register to clear it

    //handling the interrupts from the charger
    MAX77650_CHG_INT_REG = MAX77650_getINT_CHG();    //read charger interrupt register to clear it

    if ((MAX77650_CHG_INT_REG) && 0b00000010 || LED_setup)
    {      //only execute if bit 1 in charger interrupt register is set: CHG_DTLS have changed
      LED_setup = false;
      //per default alle LEDs ausschalten
      MAX77650_setLED_FS0(0);
      MAX77650_setLED_FS1(0);
      MAX77650_setLED_FS2(0);
      switch (MAX77650_getCHG_DTLS())
      {
        case 0x0: //Charger is off
          if ((MAX77650_getCHG_EN() && (MAX77650_getCHGIN_DTLS() == 0b11)))  //switch on green LED only if power supply attached and charger is in off state and charger is enabled
           MAX77650_setLED_FS1(0x1);
          break;
        case 0x1: //Charger is in prequalification mode
          MAX77650_setLED_FS0(0x1);
          break;
        case 0x2: //Charger is in fast-charge constant-current (CC) mode
          MAX77650_setLED_FS0(0x1);
          break;
        case 0x3: //Charger is in JEITA modified fast-charge constant-current mode
          MAX77650_setLED_FS0(0x1);
          break;
        case 0x4: //Charger is in fast-charge constant-voltage (CV) mode
          MAX77650_setLED_FS0(0x1);
          break;
        case 0x5: //Charger is in JEITA modified fast-charge constant-voltage mode
          MAX77650_setLED_FS0(0x1);
          break;
        case 0x6: //Charger is in  top-off mode
          MAX77650_setLED_FS0(0x1);
          break;
        case 0x7: //Charger is in JEITA modified top-off mode
          MAX77650_setLED_FS0(0x1);
          break;
        case 0x8: //Charger is done
          MAX77650_setLED_FS1(0x1);
          break;
        case 0x9: //Charger is JEITA modified done (done was entered through the JEITAmodified fast-charge states)
          MAX77650_setLED_FS1(0x1);
          break;
        case 0xA: //Charger is suspended due to a prequalification timer fault
          break;
        case 0xB: //Charger is suspended due to a fast-charge timer fault
          break;
        case 0xC: //Charger is suspended due to a battery temperature fault
          break;
        default:
          break;
      }
    }
  }
}

// Configure the MAX77650 Power-Management:
// Pins, PMIC Leds, Charger and others   
void MAX77650_Global_Setup(uint8_t Acc_Current)
{
  // Configure pin as output
  pinMode(MAX32620FTHR_PHLD, OUTPUT);
  // Set output to HIGH to hold the power-on state
  digitalWrite(MAX32620FTHR_PHLD, HIGH);

  // Init Wire channel for MAX32620FTHR: Wire2.begin()
  MAX77650_init();

  // Baseline Initialization following rules printed
  // in MAX77650 Programmres Guide Chapter 4 Page 5
  // Set Main Bias to normal Mode
  MAX77650_setSBIA_LPM(false);
  // Set On/Off-Button to push-button-mode
  MAX77650_setnEN_MODE(false);
  // Set nEN input debounce time to 30ms
  MAX77650_setDBEN_nEN(true);
  // Comparing part-numbers
  //MAX77650_getDIDM() == PMIC_partnumber
  // Checking OTP options
  //MAX77650_getCID() != MAX77650_CID
  // Values for NTC beta=3800K; Battery-values are for 1s 303759 with 600mAh
  // Set the VCOLD JEITA Temperature Threshold to 0°C
  MAX77650_setTHM_COLD(2);
  // Set the VCOOL JEITA Temperature Threshold to 15°C
  MAX77650_setTHM_COOL(3);
  // Set the VWARM JEITA Temperature Threshold to 45°C
  MAX77650_setTHM_WARM(2);
  // Set the VHOT JEITA Temperature Threshold to 60°C
  MAX77650_setTHM_HOT(3);
  // Set CHGIN regulation voltage to 4.00V
  MAX77650_setVCHGIN_MIN(0);
  // Set CHGIN Input Current Limit to 380mA
  MAX77650_setICHGIN_LIM(3);
  // Set the prequalification charge current to 10%:
  MAX77650_setI_PQ(false);
  // Set Battery prequalification voltage threshold to 3.0V
  MAX77650_setCHG_PQ(7);
  // Set Charger Termination Current to 15% of of fast charge current
  MAX77650_setI_TERM(3);
  // Set Topoff timer value to 0 minutes
  MAX77650_setT_TOPOFF(0);
  // Set the die junction temperature regulation point to 60°C
  MAX77650_setTJ_REG(0);
  // Set System voltage regulation to 4.50V
  MAX77650_setVSYS_REG(0x10);
  // Set the fast-charge constant current value to 300mA
  MAX77650_setCHG_CC(Acc_Current); // looking value at #define
  // Set the fast-charge safety timer to 5h
  MAX77650_setT_FAST_CHG(2);
  // Set IFAST-CHG_JEITA to 300mA
  MAX77650_setCHG_CC_JEITA(0x3f);
  // Set Thermistor enable bit
  MAX77650_setTHM_EN(false);
  // Set fast-charge battery regulation voltage to 4.20V
  MAX77650_setCHG_CV(0x18);
  // Set USB not in power down
  MAX77650_setUSBS(false);
  // Set the modified VFAST-CHG to 4.00V
  MAX77650_setCHG_CV_JEITA(0x10);
  // Selects the battery discharge current full-scale current value to 300mA
  MAX77650_setIMON_DISCHG_SCALE(0x0A);
  // Disable the analog MUX output
  MAX77650_setMUX_SEL(0);
  // Set the Charger to Enable
  MAX77650_setCHG_EN(true);
  // Disable SIMO Buck-Boost Channel 0 Active-Discharge
  MAX77650_setADE_SBB0(false);
  // Disable SIMO Buck-Boost Channel 1 Active-Discharge
  MAX77650_setADE_SBB1(false);
  // Disable SIMO Buck-Boost Channel 2 Active-Discharge
  MAX77650_setADE_SBB2(false);
  // Set SIMO Buck-Boost to maximum drive strength
  MAX77650_setDRV_SBB(0b00);
  // Set SIMO Buck-Boost Channel 0 Peak Current Limit to 500mA
  MAX77650_setIP_SBB0(0b00);
  // Set SIMO Buck-Boost Channel 1 Peak Current Limit to 500mA
  MAX77650_setIP_SBB1(0b00);
  // Set SIMO Buck-Boost Channel 2 Peak Current Limit to 500mA
  MAX77650_setIP_SBB2(0b00);
  // Set SIMO Buck-Boost Channel 2 to on while in stand-by-mode
  MAX77650_setEN_SBB2(0b110);
  // Set SIMO Buck-Boost Channel 0 to on while in stand-by-mode
  //MAX77650_setEN_SBB0(0b100);
  // Initialize Global Interrupt Mask Register
  MAX77650_setINT_M_GLBL(0b00000000);
  // Initialize Charger Interrupt Mask Register
  MAX77650_setINT_M_CHG(0b00000000);

  // Initialization of PMIC-LEDs
  MAX77650_setLED_FS0(0b00);
  MAX77650_setINV_LED0(false);    //LED red: phase operation
  MAX77650_setBRT_LED0(0b00000);  //LED red: brightness
  MAX77650_setP_LED0(0b1111);     //LED red: LED period
  MAX77650_setD_LED0(0b1111);     //LED red: LED duty-cycle
  MAX77650_setLED_FS1(0b00);
  MAX77650_setINV_LED1(false);    //LED green: phase operation
  MAX77650_setBRT_LED1(0b00000);  //LED green: brightness
  MAX77650_setP_LED1(0b1111);     //LED green: LED period
  MAX77650_setD_LED1(0b1111);     //LED green: LED duty-cycle
  MAX77650_setLED_FS2(0b00);
  MAX77650_setINV_LED2(false);    //LED blue: phase operation
  MAX77650_setBRT_LED2(0b11111);  //LED blue: brightness
  MAX77650_setP_LED2(0b1111);     //LED blue: LED period
  MAX77650_setD_LED2(0b1111);     //LED blue: LED duty-cycle
  MAX77650_setEN_LED_MSTR(true);  //LEDs Master enable

  // Setup GPIO
  pinMode(MAX32620FTHR_INT, INPUT_PULLUP);

  // Setup Interrupts
  attachInterrupt(MAX32620FTHR_INT, PMIC_IRQ_handler, FALLING);    //enable interrupt
  MAX77650_getINT_GLBL();  //read global interrupt register to clear it
  MAX77650_getINT_CHG();   //read charger interrupt register to clear it

  // Run the PMIC LED set only once after start-up
  PMIC_INT = true;
  LED_setup = true;

  // Set init output Voltage
  MAX77650_setTV_SBB2(0b110010);    //Set output Voltage of SBB2 to 3.3V
  //MAX77650_setTV_SBB2(0b101100);    //Set output Voltage of SBB2 to 3.0V
  //MAX77650_setTV_SBB2(0b100110);    //Set output Voltage of SBB2 to 2.7V
  //MAX77650_setTV_SBB2(0b100010);    //Set output Voltage of SBB2 to 2.5V
  //MAX77650_setTV_SBB2(0b011110);    //Set output Voltage of SBB2 to 2.3V

  //MAX77650_setTV_SBB0(0b111111);    //Set output Voltage of SBB0 to 2.35V
  //MAX77650_setTV_SBB0(0b101000);    //Set output Voltage of SBB0 to 1.8V
}
//-----------------------------------------------------------------------------<

//----------------------------------------------------------------------------->
void Sounds_0(uint8_t pin_tone)
{
  tone(pin_tone, 600, 500);
  delay(200);
  tone(pin_tone, 600, 500);
  delay(200);
  tone(pin_tone, 600, 500);
  delay(200);
  tone(pin_tone, 500, 2000);
  delay(1000);
}

void Sounds_1(uint8_t pin_tone)
{
  tone(pin_tone, 700, 300);
  delay(600);
  tone(pin_tone, 700, 300);
  delay(600);
  tone(pin_tone, 780, 150);
  delay(300);
  tone(pin_tone, 700, 150);
  delay(300);
  tone(pin_tone, 625, 450);
  delay(600);
  tone(pin_tone, 590, 150);
  delay(300);
  tone(pin_tone, 520, 150);
  delay(300);
  tone(pin_tone, 460, 450);
  delay(600);
  tone(pin_tone, 350, 450);
  delay(600);
  delay(600);
  tone(pin_tone, 350, 450);
  delay(600);
  tone(pin_tone, 460, 450);
  delay(600);
  tone(pin_tone, 520, 150);
  delay(300);
  tone(pin_tone, 590, 150);
  delay(300);
  tone(pin_tone, 625, 450);
  delay(600);
  tone(pin_tone, 590, 150);
  delay(300);
  tone(pin_tone, 520, 150);
  delay(300);
  tone(pin_tone, 700, 1350);
  delay(1800);
  tone(pin_tone, 700, 300);
  delay(600);
  tone(pin_tone, 700, 300);
  delay(600);
  tone(pin_tone, 780, 150);
  delay(300);
  tone(pin_tone, 700, 150);
  delay(300);
  tone(pin_tone, 625, 450);
  delay(600);
  tone(pin_tone, 590, 150);
  delay(300);
  tone(pin_tone, 520, 150);
  delay(300);
  tone(pin_tone, 460, 450);
  delay(600);
  tone(pin_tone, 350, 450);
  delay(600);
  delay(600);
  tone(pin_tone, 350, 450);
  delay(600);
  tone(pin_tone, 625, 450);
  delay(600);
  tone(pin_tone, 590, 150);
  delay(300);
  tone(pin_tone, 520, 150);
  delay(300);
  tone(pin_tone, 700, 450);
  delay(600);
  tone(pin_tone, 590, 150);
  delay(300);
  tone(pin_tone, 520, 150);
  delay(300);
  tone(pin_tone, 460, 1350);
  delay(5000);
}
//-----------------------------------------------------------------------------<
