# KBikeBLE
Replacement computer for a Keiser M3 spin bike with Bluetooth services and display

## What this does
Replaces the computer on a Keiser M3 spin bike and adds Bluetooth Cycling Power Service (CPS) and Fitness Machine Service (FTMS). Since it replaces the original computer, a display is provided as well.

### Capabilities
* Provides cadence (RPM), gear/resistance, and estimated power. The user can choose between an approximation of the gear displayed by the Keiser computer, or resistance from 0 to 100%, by moving the gear lever to the very top (with or without pedaling).
* Bluetooth services
  * CPS (typical for power meters) - power and cadence
  * FTMS (typical for ergometers, etc.) - power, resistance setting, and cadence (though no apps I have tried to date read the resistance, and all I have tried want to set the resistance (resistance servo on the "some day" list!)
* Power savings (runs for TBD days on a little 350 mAHr Li-poly battery)
  * Display dim (if supported by the display) after 60 (or whatever) seconds without pedaling
  * Display blank and de-energize the resistance sense pot after 300 (or whatever) seconds without pedaling

#### Cautions
* Once connected to a client via CPS or FTMS, the code stops advertising and stops updating (notify() calls) the other service. Therefore, a device that remains in range with an app that doesn't disconnect at the end of a session can tie up the service, preventing the next user (or different app) from connecting. Release it by quitting the app, or moving far enough away. Once disconnected, advertising of both services will resume.

## A few details
### Bike hardware
The M3 computer connects to a sensor/pickup board near the resistance magnet assembly using an RJ9 (phone handset) connector. The sensor board has a 10K pot that's linked to the magnet assembly, and a magnetic reed switch (not a Hall sensor) that closes once per crank revolution. Using the standard RJ9 lead colors, the leads are
* Green  - The crank switch. This is pulled to ground when a magnet on the crank assembly passes by.
* Black  - "Top" of the resistance magnet position sense pot
* Red    - Wiper of the pot
* Yellow - Ground - other end of the pot, and of the crank switch
Any replacement RJ9 cord provides access, or you could tap into the Keiser computer connector at the handlebar end. Retaining the ability to use the Keiser computer is recommended. In the absence of power pedals, calibration is done by matching the Keiser power estimates.
KBikeBLE uses a digital output to apply voltage to the pot (Black wire) and reads the voltage at the wiper. The original Keiser computer appears to operate differently. The ability to leave both connected at the same time is TBD.

### Computer hardware
This code is for an Adafruit nrf52840 Express microcontroller and a generic 128x64 OLED display. The code will be the same or very similar for any board with a Nordic nrf52840.

### Calibration
Calibrations were obtained by switching back and forth between the Keiser computer and KBikeBLE. 

Finding "gear" positions at which the Keiser is right at the edge between gears and recording the resistance readings provided gear calibration. Gear vs. resistance is nicely represented by a second order polynomial, even though it's not clear that the Keiser gear numbers are intended to be evenly spaced.

Data were also collected for the Keiser power estimate vs. gear (or resistance) at a constant cadence, and for power vs. cadence at a couple of fixed resistances. For an eddy current brake, it should be possible to express power as the product of a power vs. speed relationship and a power vs. resistance relationship, and that seems to hold up. Power vs. speed, power vs. resistance - fit nicely (r^2 around .98) to second order polynomials. 

With all that said, calibrations in the code were done pretty quickly.
