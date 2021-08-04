// Options of most interest to the end user

#ifndef OPTIONS_
#define OPTIONS_

/* Filters on the resistance value, to avoid flickering digits. ---------------------------------------

   Filtered value is (FILTER * last + 2 * new)/(FILTER + 2)
            where FILTER = the #defined value below
                    last = last value
                     new = new instantaneous value
   Setting filter to...
       0 means no filtering (instantaneous measurements pass through).
       1 means the new value is 2/3 of the new measurement plus 1/3 of the old.
       2 means new = 1/2 of the new measurement + 1/2 of the old 
         ...etc.

   Separate filters are provided for the display and for the power measurements.
   Excessive filtering of the display can make it harder for the user to crisply
   adjust the resistance. 

   Values between 0 and 2 are recommended. Most BLE software will have its own filtering.
   Some software may capture data such as max 5-second sustained power that could be 
   compromised by excessive filtering. 
   
   If resistance values are really noisy, check electrical connections to the bike.
   Prototyping connectors can be unreliable, and dirt can get into the connector
   on the bike. Measured resistance changing when the display dims is a sign of
   a poor Ground connection to the controller board.

   NOTE: When displaying the Keiser gear number instead of resistance %, the
         resistance used in the power calculation is used to determine the gear.
*/

#define RESISTANCE_FILTER 0          // Resistance used in power calculation and gear determination
#define RESISTANCE_DISPLAY_FILTER 0  // Resistance used in the resistance display

/* Power savings settings. ----------------------------------------------------------------------------

   When the user isn't pedaling, the system will
      - dim the display, then
      - blank the display, then
      - disconnect from Bluetooth and optionally go to suspend/powerdown mode.

   POWERSAVE determines whether and how the system saves power when not in use
      - 0      - Nothing beyond blanking the display and shutting down Bluetooth.
      - 1      - Shut down the system. Pedaling will then cause a full restart.
      - Other  - Power savings without shutdown.  
                 between checks for pedal movement.
   
   Because FreeRTOS does a good job of going into a low-power state when there's nothing to do, there's
   little difference between POWERSAVE = 1 (shutdown) and POWERSAVE = 2 (just idle all tasks).

   Times are in approximate seconds. For convenience, use N * 60 to specify N minutes.
*/

#define POWERSAVE  2           // Defines power save mode; 0 = none, 1 = shutdown, other = no shutdown
#define DIM_TIME        1 * 60 // Dim the display (effect depends upon the display)
#define BLANK_TIME      3 * 60 // Blank the display
#define NO_BLE_PS_TIME  5 * 60 // In absence of Bluetooth connection and POWERSAVE defined, enter power save mode
#define BLE_PS_TIME    15 * 60 // Disconnect Bluetooth and, if POWERSAVE defined, enter power save mode

/* Display contrast. ----------------------------------------------------------------------------------
   For OLED displays, this sets the pixel current and therefore the brightness.
   Different OLED displays are affected differently by the contrast settings 
   Defaults are for a generic SH1106, which doesn't provide a very wide brightness range. Higher values
   can involve significantly higher current (battery usage) without much difference in brightness or clarity.
*/

#define CONTRAST_FULL 128   // Full and reduced display brightness. These depend upon the display used.
#define CONTRAST_DIM 0

/* Low battery indicator - flashes when below this percentage. -----------------------------------------
   This ought to be determined by the level at which the proper ADC reference is lost. For the
   Adfruit Feather nrf52840 Express, things should work down to about 10%.
*/
#define BATT_LOW 20

/* Display the Keiser gear number, or % resistance, at startup? ---------------------------------------
   The alternative can always be chosen by moving the resistance lever to the top.
*/

#define GEAR_DISPLAY true   // true = gear, false = res %

/* Bluetooth options. ---------------------------------------------------------------------------------
   The device connecting via Bluetooth is usually very close to the bike, so the power can be reduced.
   Set the transmit power to something that works reliably without wasting power.
   The blue LED blink rate can be changed as well but is probably not significant.
*/
#define BLE_TX_POWER -12 // BLE transmit power in dBm
// Valid options for nRF52840: -40, -20, -16, -12, -8, -4, 0, +2, +3, +4, +5, +6, +7, or +8
#define BLE_LED_INTERVAL 1000 // ms

/* Console (command line interface) options. ----------------------------------------------------------
*/
#define CONFIRMATION_TIMEOUT 15; // Approx seconds to timeout of commands requiring confirmation

/* Optional features and functions.
   The console interface allows the user to, among other things, calibrate the computer to the individual
   bike, so enabling it is recommended. Presently, the BLEUart interface is fully developed, while the
   serial interface is not.
*/
#define USE_SERIAL  // Incorporate USB serial functions including the console and any debugging.
#define BLEUART     // Activates serial over BLE
#define BLEBAS      // Activate BLE battery service

/* The following depend upon functions provided by the Adafruit nRF52 core that are not present
 * in v0.24. 
 */
//#define SAADC_TACQ  // The Adafruit nRF52 core includes analogSampleTime().
                    // This will reduce variability in resistance measurements but the system will function without it.
//#define SAADC_CALIBRATE_OFFSET // The Adafruit core includes analogCalibrateOffset().
                               // If not defined here, one will be compiled in from the .ino.
//#define READ_CPU_TEMP  // The Adafruit nRF52 core includes readCPUTemperature().
                       // If note defined here, one will be compiled in from the .ino. 

#endif
