/**
 * @file Arduboy2Core.cpp
 * \brief
 * The Arduboy2Core class for Arduboy hardware initilization and control.
 */

#include "Arduboy2Core.h"

#ifdef _SAMD21_
// TODO: Implement this
#else
#include <avr/wdt.h> 
#endif

// #if defined (ARDUBOY_SAMD)
// THIS CODE INITIALISES THE 128*64 LCD DISPLAY WTIH UC1701 CONTROLLER WITH SPI INTERFACE - NOT FOR NORMAL ARDUBOYS. TAKEN FROM U8G2LIB
// const uint8_t PROGMEM displayBootProgram[] = {  // all data verified with o'scope using u8g2 example of ug2.begin()
// 0xE2, /* soft reset 0x0e2*/
// 0xAE, /* display off 0x0ae*/
// 0x40, /* set display start line to 0   0x040*/
// 0xA0, /* ADC set to reverse (was originally set to reverse 0x0a0)*/
// 0xC8, /* common output mode (was originally set to 0x0c8)*/
// 0xA6, /* display normal, not inverted, bit val 0: LCD pixel off. 0x0a6*/
// 0xA2, /* LCD bias 1/9 (was originally 1/9 0x0a2)*/
// 0x2F, /* all power  control circuits on 0x02f*/
// 0xF8, /* set booster ratio to... 0x0f8*/
// 0x00, /* (was originally 4x 0x000)*/
// 0x24, /* set V0 voltage resistor ratio to default (was originally 0x023)*/
// 0x81, /* set contrast 0x081*/
// 0x20, /* contrast value (originally was 0x027)*/
// 0xAC, /* indicator 0x0ac*/
// 0x00, /* disable 0x000*/
// 0xAF,  /* display on 0x0af*/
// //0xAE,  /* display off 0x0ae*/
// //0xA5,  /* enter powersafe: all pixel on, issue 142 0x0a5*/
// };
// break into two arrays so delays can be added
const uint8_t PROGMEM displayBootProgram2[] = {
		
};
// #else
	
const uint8_t PROGMEM displayBootProgram[] = {

   0xAE, // Display OFF/ON: off (POR = 0xae)
   0xA4, // Set Entire Display OFF/ON: off (POR = 0xa4)

   0xD5, 0x50, // Divide Ratio/Oscillator FrequencyData Set: divide ratio = 1 (POR = 1), Oscillator Frequency = +/- 0% (POR = +/- 0%)

  // Set Multiplex Ratio v = 0x3F
   0xA8, 0x3F,  // Multiplex Ratio Data Set: 64 (POR = 0x3f, 64)

  // Set Display Offset v = 0
   0xD3, 0x00,  // Display OffsetData Set: 0 (POR = 0x00)

  // Set Start Line (0)
   0x40, // Set Display Start Line: 0  (was 0x40)

  0x010,		/* TEST FOR CENTRING set upper 4 bit of the col adr to 0 */
  0x002,		/* TEST FOR CENTRING set lower 4 bit of the col adr to 2 (centered display with sh1106)  */

  // Charge Pump Setting v = enable (0x14)
  // default is disabled
  0xAD, 0x8B, // DC-DC ON/OFF Mode Set: Built-in DC-DC is used, Normal Display (POR = 0x8b)

  0xD9, 0x22, // Dis-charge/Pre-charge PeriodData Set: pre-charge 2 DCLKs, dis-charge 2 DCLKs (POR = 0x22, pre-charge 2 DCLKs, dis-charge 2 DCLKs)

  0xDB, 0x35, // VCOM Deselect LevelData Set: 0,770V (POR = 0x35, 0,770 V)

  0x32,  // Set Pump voltage value: 8,0 V (POR = 0x32, 8,0 V)
  
  0x81, 0xFF, // Contrast Data Register Set: 255 (large) (POR = 0x80)
  
  0xA6,  // Set Normal/Reverse Display: normal (POR = 0xa6)
   
  0xDA, 0x12, // com pin HW config, sequential com pin config (bit 4), disable left/right remap (bit 5) 

  // Set Segment Re-map (A0) | (b0001)
  // default is (b0000)
  //0xA1,

  // Set COM Output Scan Direction
 // 0xC8,

  // Set COM Pins v
  // 0xDA, 0x12,

  // Set Contrast v = 0xCF
  //0x81, 0xFF,

  // Set Precharge = 0xF1
  //0xD9, 0x1F,

  // Set VCom Detect
  // 0xDB, 0x40,
   
  // Set VPP
  //0x33,

  // Entire Display ON
   //0xA4,

  // Set normal/inverse display
  // 0xA6,

  // Display On
  0xAF,

  // set display mode = horizontal addressing mode (0x00)
   0x20, 0x00,

  // set col address range
   0x21, 0x00, COLUMN_ADDRESS_END,

  // set page address range
   0x22, 0x00, PAGE_ADDRESS_END
};

//#endif //ndef ARDUINO_ARCH_AVR

Arduboy2Core::Arduboy2Core() { }

void Arduboy2Core::boot()
{
  #ifdef ARDUBOY_SET_CPU_8MHZ
  // ARDUBOY_SET_CPU_8MHZ will be set by the IDE using boards.txt
  setCPUSpeed8MHz();
  #endif

  // Select the ADC input here so a delay isn't required in initRandomSeed()
#ifndef ARDUBOY_SAMD  
  ADMUX = RAND_SEED_IN_ADMUX;
#endif // ARDUBOY_SAMD  
  bootPins(); // TODO: now working 
  // DEBUG - have commented out boot functions to isolate those not working
  bootSPI(); // TODO: test individually
  bootOLED(); // TODO: test individually
 // bootPowerSaving(); // TODO: test individually
}

#ifdef ARDUBOY_SET_CPU_8MHZ
// If we're compiling for 8MHz we need to slow the CPU down because the
// hardware clock on the Arduboy is 16MHz.
// We also need to readjust the PLL prescaler because the Arduino USB code
// likely will have incorrectly set it for an 8MHz hardware clock.
void Arduboy2Core::setCPUSpeed8MHz()
{
  uint8_t oldSREG = SREG;
  cli();                // suspend interrupts
  PLLCSR = _BV(PINDIV); // dissable the PLL and set prescale for 16MHz)
  CLKPR = _BV(CLKPCE);  // allow reprogramming clock
  CLKPR = 1;            // set clock divisor to 2 (0b0001)
  PLLCSR = _BV(PLLE) | _BV(PINDIV); // enable the PLL (with 16MHz prescale)
  SREG = oldSREG;       // restore interrupts
}
#endif

// Pins are set to the proper modes and levels for the specific hardware.
// This routine must be modified if any pins are moved to a different port
void Arduboy2Core::bootPins()
{
#ifdef ARDUBOY_10

  // Port B INPUT_PULLUP or HIGH
  PORTB |= _BV(RED_LED_BIT) | _BV(GREEN_LED_BIT) | _BV(BLUE_LED_BIT) |
           _BV(B_BUTTON_BIT);
  // Port B INPUT or LOW (none)
  // Port B inputs
  DDRB &= ~(_BV(B_BUTTON_BIT) | _BV(SPI_MISO_BIT));
  // Port B outputs
  DDRB |= _BV(RED_LED_BIT) | _BV(GREEN_LED_BIT) | _BV(BLUE_LED_BIT) |
          _BV(SPI_MOSI_BIT) | _BV(SPI_SCK_BIT) | _BV(SPI_SS_BIT);

  // Port C
  // Speaker: Not set here. Controlled by audio class

  // Port D INPUT_PULLUP or HIGH
  PORTD |= _BV(CS_BIT);
  // Port D INPUT or LOW
  PORTD &= ~(_BV(RST_BIT));
  // Port D inputs (none)
  // Port D outputs
  DDRD |= _BV(RST_BIT) | _BV(CS_BIT) | _BV(DC_BIT);

  // Port E INPUT_PULLUP or HIGH
  PORTE |= _BV(A_BUTTON_BIT);
  // Port E INPUT or LOW (none)
  // Port E inputs
  DDRE &= ~(_BV(A_BUTTON_BIT));
  // Port E outputs (none)

  // Port F INPUT_PULLUP or HIGH
  PORTF |= _BV(LEFT_BUTTON_BIT) | _BV(RIGHT_BUTTON_BIT) |
           _BV(UP_BUTTON_BIT) | _BV(DOWN_BUTTON_BIT);
  // Port F INPUT or LOW
  PORTF &= ~(_BV(RAND_SEED_IN_BIT));
  // Port F inputs
  DDRF &= ~(_BV(LEFT_BUTTON_BIT) | _BV(RIGHT_BUTTON_BIT) |
            _BV(UP_BUTTON_BIT) | _BV(DOWN_BUTTON_BIT) |
            _BV(RAND_SEED_IN_BIT));
  // Port F outputs (none)

#elif defined(AB_DEVKIT)

  // Port B INPUT_PULLUP or HIGH
  PORTB |= _BV(LEFT_BUTTON_BIT) | _BV(UP_BUTTON_BIT) | _BV(DOWN_BUTTON_BIT) |
           _BV(BLUE_LED_BIT);
  // Port B INPUT or LOW (none)
  // Port B inputs
  DDRB &= ~(_BV(LEFT_BUTTON_BIT) | _BV(UP_BUTTON_BIT) | _BV(DOWN_BUTTON_BIT) |
            _BV(SPI_MISO_BIT));
  // Port B outputs
  DDRB |= _BV(SPI_MOSI_BIT) | _BV(SPI_SCK_BIT) | _BV(SPI_SS_BIT) |
          _BV(BLUE_LED_BIT);

  // Port C INPUT_PULLUP or HIGH
  PORTC |= _BV(RIGHT_BUTTON_BIT);
  // Port C INPUT or LOW (none)
  // Port C inputs
  DDRC &= ~(_BV(RIGHT_BUTTON_BIT));
  // Port C outputs (none)

  // Port D INPUT_PULLUP or HIGH
  PORTD |= _BV(CS_BIT);
  // Port D INPUT or LOW
  PORTD &= ~(_BV(RST_BIT));
  // Port D inputs (none)
  // Port D outputs
  DDRD |= _BV(RST_BIT) | _BV(CS_BIT) | _BV(DC_BIT);

  // Port E (none)

  // Port F INPUT_PULLUP or HIGH
  PORTF |= _BV(A_BUTTON_BIT) | _BV(B_BUTTON_BIT);
  // Port F INPUT or LOW
  PORTF &= ~(_BV(RAND_SEED_IN_BIT));
  // Port F inputs
  DDRF &= ~(_BV(A_BUTTON_BIT) | _BV(B_BUTTON_BIT) | _BV(RAND_SEED_IN_BIT));
  // Port F outputs (none)
  // Speaker: Not set here. Controlled by audio class

#elif defined(ARDUBOY_SAMD)

	// TODO: Implement this
  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_DC, OUTPUT);
  pinMode(PIN_RST, OUTPUT);
  
  digitalWrite(PIN_CS, HIGH);
  digitalWrite(PIN_RST, LOW);
  // SPI hardware kludge until SAMD port manipulation is sorted
 // pinMode(11, OUTPUT); // MOSI
 // pinMode(13, OUTPUT); // SCK
  // do we need SPI_SS here to get LCD working if we have PIN_CS above?
  
  pinMode(PIN_LEFT_BUTTON, INPUT_PULLUP);
  pinMode(PIN_RIGHT_BUTTON, INPUT_PULLUP);
  pinMode(PIN_UP_BUTTON, INPUT_PULLUP);
  pinMode(PIN_DOWN_BUTTON, INPUT_PULLUP);
  
  pinMode(PIN_A_BUTTON, INPUT_PULLUP);
  pinMode(PIN_B_BUTTON, INPUT_PULLUP);
  
  pinMode(RAND_SEED_IN, INPUT);
#endif
}

void Arduboy2Core::bootOLED()
{
  // reset the display
  delayShort(5); // reset pin should be low here. let it stay low a while
#ifdef ARDUBOY_SAMD
  digitalWrite(PIN_RST, HIGH); // set high to come out of reset
  delayShort(5); // wait a while
    // select the display (permanently, since nothing else is using SPI)
  digitalWrite(PIN_CS, LOW); // TODO: deconflict so that SD card and flash can use SPI bus
#else 
  bitSet(RST_PORT, RST_BIT); // set high to come out of reset
  delayShort(5); // wait a while
  // select the display (permanently, since nothing else is using SPI)
  bitClear(CS_PORT, CS_BIT);
#endif
  // run our customized boot-up command sequence against the
  // OLED to initialize it properly for Arduboy
  LCDCommandMode();
  for (uint8_t i = 0; i < sizeof(displayBootProgram); i++) {
    SPItransfer(pgm_read_byte(displayBootProgram + i));
  }
  delayShort(100); // copied from u8glib
 // SPItransfer(0xA5); /* display all points, ST7565 */ // copied from u8glib
  delayShort(100); // copied from u8glib
  delayShort(100); // copied from u8glib
 // SPItransfer(0xA4); /* normal display */ // copied from u8glib
/*
  for (uint8_t i = 0; i < sizeof(displayBootProgram); i++) {
    SPItransfer(pgm_read_byte(displayBootProgram2 + i));
  }
  */
  LCDDataMode();
}

void Arduboy2Core::LCDDataMode()
{
#ifdef ARDUBOY_SAMD
  digitalWrite(PIN_DC, HIGH);
#else	
  bitSet(DC_PORT, DC_BIT);
#endif
}

void Arduboy2Core::LCDCommandMode()
{
#ifdef ARDUBOY_SAMD
  digitalWrite(PIN_DC, LOW);
#else	
  bitClear(DC_PORT, DC_BIT);
#endif
}

// Initialize the SPI interface for the display
void Arduboy2Core::bootSPI()
{
#ifdef ARDUBOY_SAMD
  SPI.begin(); // TODO: experimental to address issue of no spi signal detected on M0 port
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0)); // was 8000000

  LCDDataMode(); // should be the equivalent function from ArduboyCore2 to the one from ArduboyCore_Z
#else
// master, mode 0, MSB first, CPU clock / 2 (8MHz)
  SPCR = _BV(SPE) | _BV(MSTR);
  SPSR = _BV(SPI2X);
#endif  
}

// Write to the SPI bus (MOSI pin)
void Arduboy2Core::SPItransfer(uint8_t data)
{
#ifdef ARDUBOY_SAMD	
  SPI.transfer(data);
#else	
  SPDR = data;
  /*
   * The following NOP introduces a small delay that can prevent the wait
   * loop form iterating when running at the maximum speed. This gives
   * about 10% more speed, even if it seems counter-intuitive. At lower
   * speeds it is unnoticed.
   */
  asm volatile("nop");
  while (!(SPSR & _BV(SPIF))) { } // wait
#endif  
}

void Arduboy2Core::safeMode()
{
  if (buttonsState() == UP_BUTTON)
  {
    digitalWriteRGB(RED_LED, RGB_ON);

#if !defined ARDUBOY_CORE && !defined ARDUBOY_SAMD // for Arduboy core timer 0 should remain enabled
    // prevent the bootloader magic number from being overwritten by timer 0
    // when a timer variable overlaps the magic number location
    power_timer0_disable();
#endif

    while (true) { }
  }
}


/* Power Management */

void Arduboy2Core::idle()
{
// TODO: Implement this

}

void Arduboy2Core::bootPowerSaving()
{
// TODO: Implement this

}

// Shut down the display
void Arduboy2Core::displayOff()
{
  LCDCommandMode();
  #ifndef ARDUBOY_SAMD 
  SPItransfer(0xAE); // display off
  SPItransfer(0x8D); // charge pump:
  SPItransfer(0x10); //   disable
  delayShort(250);
 
  bitClear(RST_PORT, RST_BIT); // set display reset pin low (reset state)
#else
	//TODO: implement a suitable screen shutdown instruction
  digitalWrite(PIN_RST, LOW); // set display reset pin low (reset state)
#endif	
}

// Restart the display after a displayOff()
void Arduboy2Core::displayOn()
{
  bootOLED();
}


/* Drawing */

void Arduboy2Core::paint8Pixels(uint8_t pixels)
{
  SPItransfer(pixels);
}

void Arduboy2Core::paintScreen(const uint8_t *image)
{
  for (int i = 0; i < (HEIGHT*WIDTH)/8; i++)
  {
    SPItransfer(pgm_read_byte(image + i));
  }
}

// paint from a memory buffer, this should be FAST as it's likely what
// will be used by any buffer based subclass
//
// The following assembly code runs "open loop". It relies on instruction
// execution times to allow time for each byte of data to be clocked out.
// It is specifically tuned for a 16MHz CPU clock and SPI clocking at 8MHz.
void Arduboy2Core::paintScreen(uint8_t image[], bool clear)
{
 // TODO: Implement this
#ifdef ARDUBOY_SH1106
  // TODO optimise like AVR version
  LCDCommandMode();
 // SPItransfer(0x81); 							// set contrast (command)
 // SPItransfer(0x20); 	/* routinely 0x32 */	// 32 = 50, 3E = 62 (out of max 63 64?)
 // delayShort(5); 								// copy the apparent 5-6ms delay in the u8glib example from oscope
  uint8_t pageNum = 0; 							// the page number of the LCD
  constexpr size_t size = ((HEIGHT*WIDTH)/8); 	// size of the pages
  for (size_t index = 0; index < size; index++)
  {
    if (index == 0 || (index % 128 == 0)) {
	  LCDCommandMode();
	  SPItransfer(0x10);
	  SPItransfer(0x00);
	  if ( index  == 0) pageNum = 0xB0;   		// the page number is 0 at the beginning
	  else pageNum = 0xB0 | (index >> 7); 		// get the page number by dividing index by 128
      SPItransfer(pageNum);
	  LCDDataMode();
	}	
    SPItransfer(image[index]); 					// send the data to the display controller
	if(clear){
		image[index] = 0; 						// clear the buffer if told to do so
	}	
  }
#else
//TODO: reinsert the AVR code here and swap round the #ifdef with ifndef etc to make AVR the special case
#endif
}
#if 0
// For reference, this is the "closed loop" C++ version of paintScreen()
// used prior to the above version.
void Arduboy2Core::paintScreen(uint8_t image[], bool clear)
{
  uint8_t c;
  int i = 0;

  if (clear)
  {
    SPDR = image[i]; // set the first SPI data byte to get things started
    image[i++] = 0;  // clear the first image byte
  }
  else
    SPDR = image[i++];

  // the code to iterate the loop and get the next byte from the buffer is
  // executed while the previous byte is being sent out by the SPI controller
  while (i < (HEIGHT * WIDTH) / 8)
  {
    // get the next byte. It's put in a local variable so it can be sent as
    // as soon as possible after the sending of the previous byte has completed
    if (clear)
    {
      c = image[i];
      // clear the byte in the image buffer
      image[i++] = 0;
    }
    else
      c = image[i++];

    while (!(SPSR & _BV(SPIF))) { } // wait for the previous byte to be sent

    // put the next byte in the SPI data register. The SPI controller will
    // clock it out while the loop continues and gets the next byte ready
    SPDR = c;
  }
  while (!(SPSR & _BV(SPIF))) { } // wait for the last byte to be sent
}
#endif

void Arduboy2Core::blank()
{
  for (int i = 0; i < (HEIGHT*WIDTH)/8; i++)
	  // this probably needs a helper function or two to send the required data to the LCD with appropriate page wrappers and commands
    SPItransfer(0x00);
}

void Arduboy2Core::sendLCDCommand(uint8_t command)
{
  LCDCommandMode();
  SPItransfer(command);
  LCDDataMode();
}

// invert the display or set to normal
// when inverted, a pixel set to 0 will be on
void Arduboy2Core::invert(bool inverse)
{
  sendLCDCommand(inverse ? OLED_PIXELS_INVERTED : OLED_PIXELS_NORMAL);
}

// turn all display pixels on, ignoring buffer contents
// or set to normal buffer display
void Arduboy2Core::allPixelsOn(bool on)
{
  sendLCDCommand(on ? OLED_ALL_PIXELS_ON : OLED_PIXELS_FROM_RAM);
}

// flip the display vertically or set to normal
void Arduboy2Core::flipVertical(bool flipped)
{
  sendLCDCommand(flipped ? OLED_VERTICAL_FLIPPED : OLED_VERTICAL_NORMAL);
}

// flip the display horizontally or set to normal
void Arduboy2Core::flipHorizontal(bool flipped)
{
  sendLCDCommand(flipped ? OLED_HORIZ_FLIPPED : OLED_HORIZ_NORMAL);
}

/* RGB LED */

void Arduboy2Core::setRGBled(uint8_t red, uint8_t green, uint8_t blue)
{
// TODO: Implement this

}

void Arduboy2Core::setRGBled(uint8_t color, uint8_t val)
{
// TODO: Implement this

}

void Arduboy2Core::freeRGBled()
{
// TODO: Implement this

}

void Arduboy2Core::digitalWriteRGB(uint8_t red, uint8_t green, uint8_t blue)
{
// TODO: Implement this

}

void Arduboy2Core::digitalWriteRGB(uint8_t color, uint8_t val)
{
// TODO: Implement this

}

/* Buttons */

uint8_t Arduboy2Core::buttonsState() // TODO: reintroduce original arduboy HW
{
  uint8_t buttons;
/* TODO: probably delete this section as the code immediately below this commented block is from ARDUBOY_Z and works!
  buttons = ((digitalRead(PIN_LEFT_BUTTON) << 5) | 
			(digitalRead(PIN_RIGHT_BUTTON) << 6) | 
			(digitalRead(PIN_UP_BUTTON) << 7) | 
			(digitalRead(PIN_DOWN_BUTTON) << 4));
  if(digitalRead(PIN_A_BUTTON == 0)) { buttons |= (1 << 3); }			
  if(digitalRead(PIN_B_BUTTON == 0)) { buttons |= (1 << 2); }
 */ 
    // buttons: L R x U x B A D
  // PORT A bits: left 11, right 10, up 8, down 4
  buttons = (~PORT->Group[PORTA].IN.reg & (bit(11) | bit(10) | bit(8) | bit(4)))
            >> 4;
  // PORT B bits: B 9, A 8
  buttons |= (~PORT->Group[PORTB].IN.reg & (bit(9) | bit(8))) >> 7;
  
  return buttons;
}

// delay in ms with 16 bit duration
void Arduboy2Core::delayShort(uint16_t ms)
{
  delay((unsigned long) ms);
}

void Arduboy2Core::exitToBootloader()
{
// TODO: Implement this
}

// Replacement main() that eliminates the USB stack code.
// Used by the ARDUBOY_NO_USB macro. This should not be called
// directly from a sketch.

void Arduboy2Core::mainNoUSB()
{
  // Intentionally omitted for SAMD21
}

