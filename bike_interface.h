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
/* Original prototype connections
#define CRANK_PIN 9
#define RESISTANCE_PIN A1
#define RESISTANCE_TOP 10  
#define BATTERY_PIN A6
*/

// Connections for tidy ribbon cable. Using an analog input pin for RESISTANCE_TOP also allows
// checking of Vdd as the battery gets low.
#define CRANK_PIN A1
//#define CRANK_PIN  7  // Pushbutton on the Adafruit nrf52840 Express, for debugging
#define RESISTANCE_PIN A2
#define RESISTANCE_TOP A3

#define BATTERY_PIN A6

#define ANALOG_SAMPLE_TIME 20 // The SAADC sampling time in us. 10 us is probably enough for reading
                              // from the 20K resistance sense potentiometer