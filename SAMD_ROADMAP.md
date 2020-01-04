## ROADMAP for SAMD port of the Arduboy2 Library

This file is intended to keep track of what the higher level TODOs are if the library is to be made suitable for (a) merging with the MLXXXp original branch and (b) compile for a range of processors (original AVR, SAMD21, perhaps SAMD51 and others).

- Consider removing pin mappings from Arduboy2Core.h and into separate header files that can be conditionally included at compilation
- Timers for audio generation (and other needs?) - lower priority than DAC for audio
- DAC for audio?
- DMA for...? Display, audio?
- Port manipulation for optimised DC and RESET pins on display SPI bus?
- Wireless comms link - long term, IR comms for e.g. multiplayer has been mooted and is desirable.
- additional sensor input interface - not currently planned but others have discussed.