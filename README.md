# KBikeBLE
Replacement computer for a Keiser M3 spin bike with Bluetooth services and display

## What this does
Replaces the computer on a Keiser M3 spin bike and adds Bluetooth Low Energy (BLE) Cycling Power Service (CPS) and Fitness Machine Service (FTMS). Since it replaces rather than augments the original computer, a simple display is provided as well.

### Capabilities
* Displays cadence (RPM), gear/resistance, and estimated power. The user can choose between an approximation of the gear displayed by the Keiser computer, or resistance from 0 to 100%, by moving the gear lever to the very top (with or without pedaling).
* Bluetooth services
  * CPS (typical for power meters) - power and cadence
  * FTMS (typical for ergometers and trainers) - power, resistance setting, and cadence (though no apps I have tried to date read the resistance, and all I have tried want to set the resistance (resistance servo on the "some day" list!))
* Power savings (runs for several days on a little 350 mAHr Li-poly battery)
  * Display dim (if supported by the display) after 60 (or #defined) seconds without pedaling
  * Display blank and de-energize the resistance sense pot after 300 (or #defined) seconds without pedaling
  * More possible with Bluetooth power adjustment, reduced advertising intervals, etc.

#### Cautions
* Once connected to a client via CPS or FTMS, the code stops advertising and stops updating (notify() calls) the other service. Therefore, a device that remains in range with an app that doesn't disconnect at the end of a session can tie up the service, preventing the next user (or different app) from connecting. Release it by quitting the app, or moving far enough away. Once disconnected, advertising of both services will resume.

## A few details

A bit more, as well as my To Do list, are in the comments in the code.

### Bike hardware
The M3 computer connects to a sensor/pickup board near the resistance magnet assembly using an RJ9 (phone handset) connector. The sensor board has a 10K pot that's linked to the magnet assembly, and a magnetic reed switch (not a Hall sensor) that closes once per crank revolution. Using the standard RJ9 lead colors, the leads are
* Green  - The crank switch. This is pulled to ground when a magnet on the crank assembly passes by.
* Black  - "Top" of the resistance magnet position sense pot
* Red    - Wiper of the pot
* Yellow - Ground - other end of the pot, and of the crank switch

Any replacement RJ9 cord provides access, or you could tap into the Keiser computer connector at the handlebar end. Retaining the ability to use the Keiser computer is recommended. In the absence of power pedals, calibration is done by matching the Keiser power estimates.

KBikeBLE uses a digital output to apply voltage to the pot (Black wire) and reads the voltage at the wiper. Disabling resistance measurements and blanking the display provides good (days) battery life. The original Keiser computer appears to operate differently, perhaps for futher power savings. The ability to leave both connected at the same time is TBD.

### Computer hardware
This code is for an Adafruit nrf52840 Express microcontroller and a generic 128x64 OLED display (SH1106 driver). The code will be the same or very similar for any board with a Nordic nrf52840, and the U8G2 display library will accommodate other displays.

### Dependencies
* Adafruit BLE (Bluefruit) https://github.com/adafruit/Adafruit_nRF52_Arduino
* U8G2 display https://github.com/olikraus/u8g2

### Calibration
Calibrations were obtained by switching back and forth between the Keiser computer and KBikeBLE. 

Finding "gear" positions at which the Keiser is right at the edge between gears and recording the resistance readings provided gear calibration. Gear vs. resistance is nicely represented by a second order polynomial, even though it's not clear that the Keiser gear numbers are intended to be evenly spaced. See https://user-images.githubusercontent.com/68538658/113517760-e75ac880-9579-11eb-968e-854193421594.jpeg for info from Keiser on gear position and power. 

Data were also collected for the Keiser power estimate vs. gear (or resistance) at a constant cadence, and for power vs. cadence at a couple of fixed resistances. For an eddy current brake, it should be possible https://link.springer.com/article/10.1007/s00202-017-0636-x to express power as the product of a power vs. speed relationship and a power vs. resistance relationship, and that seems to hold up. Power vs. speed, power vs. resistance both fit nicely (r^2 around .98) to second order polynomials (though see the subtle flattening at the highest resistance setting in the above-referenced graph). With all that said, calibrations in the code were done pretty quickly.

## Inspiration and thanks
https://github.com/Tschucker/ArduinoBLE-Cycle-Power-Service

https://github.com/turbodonkey/bike_power_meter

The very nice people over at https://github.com/ptx2/gymnasticon
