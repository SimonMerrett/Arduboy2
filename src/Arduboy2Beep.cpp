/**
 * @file Arduboy2Beep.cpp
 * \brief
 * Classes to generate simple square wave tones on the Arduboy speaker pins.
 */

#include <Arduino.h>
#include "Arduboy2.h" // included to try and define ARDUBOY_SAMD where needed
#include "Arduboy2Beep.h"

#ifdef ARDUBOY_SAMD

uint8_t BeepPin1::duration = 0;

void BeepPin1::begin() // TODO: implement if needed
{
  initSAMD21timer(); // set up the timer
  REG_PORT_OUTCLR0 = PINPORT_SPEAKER_1; // Set the output of TONE_PIN to LOW
  REG_PORT_DIRSET0 = PINPORT_SPEAKER_1; // Set the direction of TONE_PIN to an output
}


void BeepPin1::initSAMD21timer(){ // copyright Shawn Hymel - see http://shawnhymel.com/1710/arduino-zero-samd21-raw-pwm-using-cmsis/
  uint32_t period = 0xFFFF;	 // TODO: give this an initial value of some meaning
  // Enable and configure generic clock generator 4
  GCLK->GENCTRL.reg = GCLK_GENCTRL_IDC |          // Improve duty cycle
                      GCLK_GENCTRL_GENEN |        // Enable generic clock gen
                      GCLK_GENCTRL_SRC_DFLL48M |  // Select 48MHz as source
                      GCLK_GENCTRL_ID(4);         // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Set clock divider of 1 to generic clock generator 4
  GCLK->GENDIV.reg = GCLK_GENDIV_DIV(1) |         // Divide 48 MHz by 1
                     GCLK_GENDIV_ID(4);           // Apply to GCLK4 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
  
  // Enable GCLK4 and connect it to TCC0 and TCC1
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |        // Enable generic clock
                      GCLK_CLKCTRL_GEN_GCLK4 |    // Select GCLK4
                      GCLK_CLKCTRL_ID_TCC0_TCC1;  // Feed GCLK4 to TCC0/1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Divide counter by 256 giving 187.5kHz (5.33 us) on each TCC0 tick
  TCC0->CTRLA.reg |= TCC_CTRLA_PRESCALER(TCC_CTRLA_PRESCALER_DIV256_Val);

  // Use "Normal PWM" (single-slope PWM): count up to PER, match on CC[n]
  TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;         // Select NPWM as waveform
  while (TCC0->SYNCBUSY.bit.WAVE);                // Wait for synchronization

  
/*
  // Set the period (the number to count to (TOP) before resetting timer)
  TCC0->PER.reg = period;  // TODO: consider deleting the setting of these registers in initSAMD21timer() and leaving for tones()
  while (TCC0->SYNCBUSY.bit.PER);

  // Set PWM signal to output 50% duty cycle
  // n for CC[n] is determined by n = x % 4 where x is from WO[x]
  TCC0->CC[1].reg = period / 2; // TODO: consider deleting the setting of these registers in initSAMD21timer() and leaving for tones()
  while (TCC0->SYNCBUSY.bit.CC2);
  */
  
  //TODO: check the following pin configuration can be deleted as taken care of in BeepPin1::begin()
  // Configure PA15 (D5 on Arduino Zero) to be output
  //PORT->Group[PORTA].DIRSET.reg = PINPORT_SPEAKER_1;      // Set pin as output
  //PORT->Group[PORTA].OUTCLR.reg = PINPORT_SPEAKER_1;      // Set pin to low

  // Enable the port multiplexer for PA15
  PORT->Group[PORTA].PINCFG[15].reg |= PORT_PINCFG_PMUXEN; //TODO add a reference to a #define for pin port etc

  // Connect TCC0 timer to PA15. Function F is TCC0/WO[5] for PA15.
  // Odd pin num (2*n + 1): use PMUXO
  // Even pin num (2*n): use PMUXE
  PORT->Group[PORTA].PMUX[7].reg = PORT_PMUX_PMUXO_F;
}

void BeepPin1::tone(uint16_t count) // TODO: implement if needed
{
  tone(count, 0);
}

void BeepPin1::tone(uint16_t count, uint8_t dur) // TODO: implement if needed
{
  duration = dur;
  // Enable output (start PWM)
  TCC0->CTRLA.reg |= (TCC_CTRLA_ENABLE);
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
  
  // Set the period (the number to count to (TOP) before resetting timer)
  TCC0->PER.reg = count * 2; // TODO: check that this is doubled due to AVR toggling
  while (TCC0->SYNCBUSY.bit.PER);

  // Set PWM signal to output 50% duty cycle
  // n for CC[n] is determined by n = x % 4 where x is from WO[x]
  TCC0->CC[1].reg = count;  // TODO: check that this is whole count due to AVR toggling
  while (TCC0->SYNCBUSY.bit.CC2);
}

void BeepPin1::timer() // TODO: implement if needed
{
  if (duration && (--duration == 0)) {
    // Disable output (stop PWM)
    TCC0->CTRLA.reg &= ~(TCC_CTRLA_ENABLE);
    while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
	}
}

void BeepPin1::noTone() // TODO: implement if needed
{
  duration = 0;
  TCC0->CTRLA.reg &= ~(TCC_CTRLA_ENABLE);
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization 
}

uint8_t BeepPin2::duration = 0;

void BeepPin2::begin() // TODO: implement if needed
{
}

void BeepPin2::tone(uint16_t count) // TODO: implement if needed
{
}

void BeepPin2::tone(uint16_t count, uint8_t dur) // TODO: implement if needed
{
}

void BeepPin2::timer() // TODO: implement if needed
{
}

void BeepPin2::noTone() // TODO: implement if needed
{
}


#endif

#ifndef AB_DEVKIT 
#ifndef ARDUBOY_SAMD
// Speaker pin 1, Timer 3A, Port C bit 6, Arduino pin 5

uint8_t BeepPin1::duration = 0;

void BeepPin1::begin()
{
  TCCR3A = 0;
  TCCR3B = (bit(WGM32) | bit(CS31)); // CTC mode. Divide by 8 clock prescale
}

void BeepPin1::tone(uint16_t count)
{
  tone(count, 0);
}

void BeepPin1::tone(uint16_t count, uint8_t dur)
{
  duration = dur;
  TCCR3A = bit(COM3A0); // set toggle on compare mode (which connects the pin)
  OCR3A = count; // load the count (16 bits), which determines the frequency
}

void BeepPin1::timer()
{
  if (duration && (--duration == 0)) {
    TCCR3A = 0; // set normal mode (which disconnects the pin)
  }
}

void BeepPin1::noTone()
{
  duration = 0;
  TCCR3A = 0; // set normal mode (which disconnects the pin)
}


// Speaker pin 2, Timer 4A, Port C bit 7, Arduino pin 13

uint8_t BeepPin2::duration = 0;

void BeepPin2::begin()
{
  TCCR4A = 0; // normal mode. Disable PWM
  TCCR4B = bit(CS43); // divide by 128 clock prescale
  TCCR4D = 0; // normal mode
  TC4H = 0;  // toggle pin at count = 0
  OCR4A = 0; //  "
}

void BeepPin2::tone(uint16_t count)
{
  tone(count, 0);
}

void BeepPin2::tone(uint16_t count, uint8_t dur)
{
  duration = dur;
  TCCR4A = bit(COM4A0); // set toggle on compare mode (which connects the pin)
  TC4H = highByte(count); // load the count (10 bits),
  OCR4C = lowByte(count); //  which determines the frequency
}

void BeepPin2::timer()
{
  if (duration && (--duration == 0)) {
    TCCR4A = 0; // set normal mode (which disconnects the pin)
  }
}

void BeepPin2::noTone()
{
  duration = 0;
  TCCR4A = 0; // set normal mode (which disconnects the pin)
}

#else






#endif //ndef ARDUBOY_SAMD
#else /* AB_DEVKIT */

// *** The pins used for the speaker on the DevKit cannot be directly
// controlled by a timer/counter. The following "dummy" functions will
// compile and operate properly but no sound will be produced

uint8_t BeepPin1::duration = 0;

void BeepPin1::begin()
{
}

void BeepPin1::tone(uint16_t count)
{
  tone(count, 0);
}

void BeepPin1::tone(uint16_t count, uint8_t dur)
{
  (void) count; // parameter not used

  duration = dur;
}

void BeepPin1::timer()
{
  if (duration) {
    --duration;
  }
}

void BeepPin1::noTone()
{
  duration = 0;
}


uint8_t BeepPin2::duration = 0;

void BeepPin2::begin()
{
}

void BeepPin2::tone(uint16_t count)
{
  tone(count, 0);
}

void BeepPin2::tone(uint16_t count, uint8_t dur)
{
  (void) count; // parameter not used

  duration = dur;
}

void BeepPin2::timer()
{
  if (duration) {
    --duration;
  }
}

void BeepPin2::noTone()
{
  duration = 0;
}

#endif /* AB_DEVKIT */
