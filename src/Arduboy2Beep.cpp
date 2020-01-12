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


void TC4_Handler() //TODO: work out how to handle SpeakerPin2. Will ISR tolerate being class-specific? Or use conditionals like in ArduboyTones.cpp?                             // Interrupt Service Routine (ISR) for timer TC4
{     
  // Check for overflow (OVF) interrupt
  if (TC4->COUNT16.INTFLAG.bit.OVF && TC4->COUNT16.INTENSET.bit.OVF)             
  {
    REG_PORT_OUTTGL0 = PINPORT_SPEAKER_1; // should toggle the speaker pin

    REG_TC4_INTFLAG = TC_INTFLAG_OVF;         // Clear the OVF interrupt flag
  }
}

void BeepPin1::initSAMD21timer(){ // copyright Martin L
	
	  // Set up the generic clock (GCLK4) used to clock timers
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor 1: 48MHz/1=48MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Feed GCLK4 to TC4 and TC5
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TC4 and TC5
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TC4_TC5;     // Feed the GCLK4 to TC4 and TC5
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
 //TODO: change this to another variable, rather than a magic number
  REG_TC4_COUNT16_CC0 = 0xB71A;                   // Set the TC4 CC0 register as the TOP value in match frequency mode
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);       // Wait for synchronization

  //NVIC_DisableIRQ(TC4_IRQn);
  //NVIC_ClearPendingIRQ(TC4_IRQn);
  NVIC_SetPriority(TC4_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TC4 to 0 (highest)
  NVIC_EnableIRQ(TC4_IRQn);         // Connect TC4 to Nested Vector Interrupt Controller (NVIC)

  REG_TC4_INTFLAG |= TC_INTFLAG_OVF;              // Clear the interrupt flags
  REG_TC4_INTENSET = TC_INTENSET_OVF;             // Enable TC4 interrupts
  // REG_TC4_INTENCLR = TC_INTENCLR_OVF;          // Disable TC4 interrupts
 
  REG_TC4_CTRLA |= TC_CTRLA_PRESCALER_DIV1024 |   // Set prescaler to 1024, 48MHz/1024 = 46.875kHz
                   TC_CTRLA_WAVEGEN_MFRQ /*| */       // Put the timer TC4 into match frequency (MFRQ) mode
                   /*TC_CTRLA_ENABLE*/;  // enable in tone function               // Enable TC4
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);       // Wait for synchronization
}

void BeepPin1::tone(uint16_t count) // TODO: implement if needed
{
  tone(count, 0);
}

void BeepPin1::tone(uint16_t count, uint8_t dur) // TODO: implement if needed
{
  REG_TC4_CTRLA |= TC_CTRLA_ENABLE;
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);
  REG_TC4_COUNT16_CC0 = count;                   // Set the TC4 CC0 register as the TOP value in match frequency mode
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);       // Wait for synchronization
}

void BeepPin1::timer() // TODO: implement if needed
{
  if (duration && (--duration == 0)) {
    REG_TC4_CTRLA &= ~TC_CTRLA_ENABLE; // TODO: check disable is recognised, may need to NAND with a bitmask
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY);  
	}
}

void BeepPin1::noTone() // TODO: implement if needed
{
  duration = 0;
  REG_TC4_CTRLA &= ~TC_CTRLA_ENABLE; // TODO: check disable is recognised, may need to NAND with a bitmask
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);  
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
