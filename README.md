# KBikeBLE
Arduino-based replacement computer for a Keiser M3 spin bike with Bluetooth services and display

## What this does
Replaces the computer on a Keiser M3 spin bike and adds Bluetooth Low Energy (BLE) Cycling Power Service (CPS) and Fitness Machine Service (FTMS). Since it replaces rather than augments the original computer, a simple display is provided as well.

### Capabilities
* Displays cadence (RPM), gear/resistance, and estimated power. The user can choose between an approximation of the gear displayed by the Keiser computer, or resistance from 0 to 100%, by moving the gear lever to the very top (with or without pedaling).
* Power estimates replicate those from Keiser V4.06 software
* Bluetooth services
  * CPS (typical for power meters) - power and cadence
  * FTMS (typical for ergometers and trainers) - power, resistance setting, and cadence (though no apps I have tried to date read the resistance, and all I have tried want to set the resistance (resistance servo on the "some day" list!))
* Power savings (runs for several days on a little 350 mAHr Li-poly battery)
  * Display dim (if supported by the display) after 60 (or #defined) seconds without pedaling
  * Display blank and de-energize the resistance sense pot after 300 (or #defined) seconds without pedaling
  * Bluetooth shutdown after 30 minutes without pedaling; restart on resumption of pedaling


### Bike hardware
The M3 computer connects to a sensor/pickup board near the resistance magnet assembly using an RJ9 (phone handset) connector. The sensor board has a 10K pot that's linked to the magnet assembly, and a magnetic reed switch (not a Hall sensor) that closes once per crank revolution. Using the standard RJ9 lead colors, the leads are
* Green  - The crank switch. This is pulled to ground when a magnet on the crank assembly passes by.
* Black  - "Top" of the resistance magnet position sense pot
* Red    - Wiper of the pot
* Yellow - Ground - other end of the pot, and of the crank switch

Any replacement RJ9 cord provides access, or you could tap into the Keiser computer connector at the handlebar end. Retaining the ability to use the Keiser computer is recommended, as it can be helpful in calibration.

KBikeBLE uses a digital output to apply voltage to the pot (Black wire) and reads the voltage at the wiper. Disabling resistance measurements and blanking the display provides good (days) battery life. The original Keiser computer appears to operate differently, perhaps for futher power savings. The ability to leave both connected at the same time is TBD.

### Computer hardware
This code is for an Adafruit nrf52840 Express microcontroller and a generic 128x64 OLED display (SH1106 driver). The code will be the same or very similar for any board with a Nordic nrf52840, and the U8G2 display library will accommodate other displays.

### Dependencies
* Adafruit BLE (Bluefruit) https://github.com/adafruit/Adafruit_nRF52_Arduino
* U8G2 display https://github.com/olikraus/u8g2

### Calibration
Calibrations were obtained by comparison with a Keiser computer, V4.06 software. A published Keiser chart showing power vs. resistance magnet position, along with data from the Keiser computer on power vs. speed at fixed resistance, are used to provide the power estimates. Comparison with a calibrated Keiser computer or Keiser's calibration tool (documentation forthcoming) can be used to calibrate to an individual bike.

See https://user-images.githubusercontent.com/68538658/113517760-e75ac880-9579-11eb-968e-854193421594.jpeg for info from Keiser on gear number vs. magnet assembly position, as well as power vs. magnet position at 90 RPM. Calibration replicates the scale on that figure, with 0 to 100% representing the full extent of the X axis. The Gear display can be the unevenly spaced gears shown on the figure, or a more uniform but very similar set. Users may prefer one over the other, or prefer the simple 0-100% display that's provided as well.

See the comments in the code for details.

## Inspiration and thanks
https://github.com/Tschucker/ArduinoBLE-Cycle-Power-Service

https://github.com/turbodonkey/bike_power_meter

The very nice people over at https://github.com/ptx2/gymnasticon
