# Smart Sensor Simulator 2 Hardware Documentation
## Manual Fixes for the SSS2 Rev 2
The SSS2 rev 2 boards need the following fixes for the system to work:
  1. Connect the Teensy 3.6 D41, D42, D53 and D55 with 30# wire.
  2. Connect the Teensy 3.6 USB D+ and D- lines with 30# wire.
  3. Connect D50 and D51 to 3.3V on the PCB.
  4. Solder all pin for the Teensy in place. 
  5. Remove U49 and C55. 
  6. Bridge the pads of C55.
  7. Lift pins 15, 16, and 17  of U37 and tie them to 3.3V.
  8. Install U45.
  9. Solder 30# wire connecting TX of ESP8266 to pad 47 (next to 3.3V).
  10. Solder 30# wire connecting RX of ESP8266 to pad 48.
  11. Install fuses.
  12. Connect pin 2 of U38 to a 3.3V source. 
