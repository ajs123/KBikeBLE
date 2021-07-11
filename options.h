// Options of most interest to the end user

/* Filters on the resistance value. 
   Bike resistance measurements can be noisy.
   Filtered value is (FILTER * last + new)/(FILTER + 1)
            where FILTER = the #defined value below
                    last = last value
                     new = new instantaneous value
   Setting filter to 0 means no filtering (instantaneous measurements pass through).
   Separate filters are provided for the display and for the power measurements.
   NOTE: When displaying the Keiser gear number instead of resistance %, the
         resistance used in the power calculation is used to determine the gear.
*/

#define RESISTANCE_FILTER 1          // Resistance used in power calculation and gear determination
#define RESISTANCE_DISPLAY_FILTER 1  // Resistance used in the resistance display

/* Power savings settings.
   When the user isn't pedaling, the system will
      - dim the display, then
      - blank the display, then
      - disconnect from Bluetooth and optionally go to suspend/powerdown mode.

   POWERSAVE determines whether and how the system saves power when not in use
      - 0      - Nothing beyond blanking the display and shutting down Bluetooth.
      - 1      - Shut down the system. Pedaling will then cause a full restart.
      - Other  - Power savings without shutdown. Presently, this involves lengthening the delay 
                 between checks for pedal movement.
   
   Times are in approximate seconds. For convenience, use N * 60 to specify N minutes.
*/

#define POWERSAVE  2           // Defines power save mode; 0 = none, 1 = shutdown, other = no shutdown
#define DIM_TIME        1 * 60 // Dim the display (effect depends upon the display)
#define BLANK_TIME      3 * 60 // Blank the display
#define NO_BLE_PS_TIME  5 * 60 // In absence of Bluetooth connection and POWERSAVE defined, enter power save mode
#define BLE_PS_TIME    15 * 60 // Disconnect Bluetooth and, if POWERSAVE defined, enter power save mode

/* Display contrast.  For OLED displays, this sets the pixel current and therefore the brightness.
   Different OLED displays are affected differently by the contrast settings 
   Defaults are for a generic SH1106, which doesn't provide a very wide brightness range.
*/

#define CONTRAST_FULL 127   // Full and reduced display brightness. These depend upon the display used.
#define CONTRAST_DIM 0

/* Low battery indicator - flashes when below this percentage.
   This ought to be determined by the level at which the proper ADC reference is lost.
*/
#define BATT_LOW 20

/* Display the Keiser gear number, or % resistance, at startup? 
   The alternative can always be chosen by moving the resistance lever to the top.
*/

#define GEAR_DISPLAY true   // true = gear, false = res %

/* Bluetooth options.
   The device connecting via Bluetooth is usually very close to the bike, so the power can be reduced.
   Set the transmit power to something that works reliably without wasting power.
   The blue LED blink rate can be changed as well but is probably not significant.
*/

#define BLE_TX_POWER -12 // BLE transmit power in dBm
// Valid options for nRF52840: -40, -20, -16, -12, -8, -4, 0, +2, +3, +4, +5, +6, +7, or +8
#define BLE_LED_INTERVAL 1000 // ms
