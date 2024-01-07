# KBikeBLE
Arduino-based replacement computer for a Keiser M3 spin bike with Bluetooth services and display
 ![Photo](docs/Cover_photo.jpeg)
## What it does
Replaces the computer on a Keiser M3 spin bike and adds Bluetooth Low Energy (BLE) Cycling Power Service (CPS) and Fitness Machine Service (FTMS). Since it replaces rather than augments the original computer, a simple display is provided as well.

### Capabilities
* Displays cadence (RPM), gear or % resistance per the user's preference, and estimated power. Power estimates replicate those from Keiser's V4.06 computer.
* Battery level, with low battery indicator
* Bluetooth services
  * CPS (typical for power meters) - power and cadence
  * FTMS (typical for ergometers and trainers) - power, resistance setting, and cadence (though no apps I have tried to date read the resistance, and all I have tried want to set the resistance (resistance servo on the "some day" list!))
  * Battery level
* Command line "console," accessible via USB or Bluetooth, used mostly for one-time calibration but also providing some debugging functions.
* Calibration to an individual bike using a procedure similar the Keiser's, using the Keiser calibration tool.
* If used about an hour per day, runs for a month or two on an 1800 mAHr LiPo battery

### NOTES: 
* Important changes starting with v1.0: Default pin assignments in bike_interface.h have changed, and the default ADC reference is now Vdd. See Calibration_change.md for details.
* Use Released v1.1 if intending to enable the FTMS service, to avoid an issue at startup with FTMS.
* Note to Apple watch users: Once paired, your Apple watch may connect whenever KBikeBLE is started up with the watch within its Bluetooth range, preventing individual apps from connecting. Unless and until the apps you use acquire data via the watch, you may need to forget KBikeBLE in your Apple watch Bluetooth settings.

### Computer hardware
This code is for an Adafruit nrf52840 Express microcontroller and a generic 128x64 OLED display (SH1106 driver in the development system). Modestly experienced programmers should be able to adapt the code pretty readily to any board with a Nordic nrf52840. The U8G2 display library will accommodate other 128x64 displays, with a change in just one line in the code.

### Bike hardware
Connection to the bike is through the RJ9 (phone handset) connector used for the stock Keiser computer. The sensor board has a 10K pot that's linked to the magnet assembly, and a magnetic reed switch (not a Hall sensor) that closes once per crank revolution. Using the standard RJ9 lead colors, the leads are
* Green  - The crank switch. This is pulled to ground when a magnet on the crank assembly passes by.
* Black  - "Top" of the resistance magnet position sense pot
* Red    - Wiper of the pot
* Yellow - Ground - other end of the pot, and of the crank switch

Any replacement RJ9 cord provides access, or you connect to the stock Keiser connector at the handlebar end. Connectors to mate with the stock cable are easily obtained.

### Dependencies
* Adafruit nRF52 Arduino core https://github.com/adafruit/Adafruit_nRF52_Arduino, version 1.0 or greater. The code makes some use of FreeRTOS and the Nordic nrf libraries that are included in the Adafruit core for the board. 
* U8G2 display library https://github.com/olikraus/u8g2

### Calibration
The software provides for calibration using Keiser's calibration tool. Calibration is reatined in flash memory, so this is a one-time process when the computer is installed on a new bike or the bike's magnetic brake assembly is serviced. 

Calibrations were obtained by comparison with a Keiser computer, V4.06 software. A published Keiser chart showing power vs. resistance magnet position, along with data from a Keiser computer on power vs. speed at fixed resistance, are used to provide the power estimates. 

See https://user-images.githubusercontent.com/68538658/113517760-e75ac880-9579-11eb-968e-854193421594.jpeg for info from Keiser on gear number vs. magnet assembly position, as well as power vs. magnet position at 90 RPM. Calibration replicates the scale on that figure, with 0 to 100% representing the full extent of the X axis. The Gear display can be the unevenly spaced gears shown on the figure, or a more uniform but very similar set. Users may prefer one over the other, or prefer the simple 0-100% display that's provided as well.

See the comments in the code for details.

## Inspiration and thanks
https://github.com/Tschucker/ArduinoBLE-Cycle-Power-Service

https://github.com/turbodonkey/bike_power_meter

The very nice people over at https://github.com/ptx2/gymnasticon
