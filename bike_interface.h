/*********************************************************************
   Keiser M3 interface through the RJ9 jack (standard colors)
      Green  - CRANK_PIN - crank switch to ground (dry switch, not Hall effect)
               To a digital input. Transient protection is recommended.
      Black  - RESISTANCE_TOP - 10K magnetic brake position sense pot high side
               Use a digital output so it can be turned off to save power
      Red    - RESISTANCE_PIN - magnet brake position sense pot wiper
               To an analog input channel
      Yellow - GROUND - magnetic brake position sense pot low / crank switch low side
 *********************************************************************/

//#define CRANK_PIN  7  // Pushbutton on the Adafruit nrf52840 Express, for debugging
#define CRANK_PIN 9
#define RESISTANCE_PIN A1
#define RESISTANCE_TOP 10  // Should move this to A2 so that the voltage at the top can be measured
#define BATTERY_PIN A6