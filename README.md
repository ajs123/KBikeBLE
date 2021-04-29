# KBikeBLE
Computer for a Keiser M3 spin bike with Bluetooth services and display

## What this does
Replaces the computer on a Keiser M3 spin bike and adds Bluetooth Cycling Power Service (CPS) and Fitness Machine Service (FTMS). Since it replaces the original computer, a display is provided as well.

### Capabilities
* Provides cadence (RPM), gear/resistance, and estimated power. The user can choose between an approximation of the gear displayed by the Keiser computer, or resistance from 0 to 100%.
* Bluetooth services
** CPS (typical for power meters) - power and cadence
** FTMS (typical for ergometers, etc.) - power, resistance setting, and cadence (though no apps I have tried to date read the resistance, and all I have tried want to set the resistance (resistance servo on the "some day" list!)
* Power savings
** Display dim (if supported by the display) after 60 (or whatever) seconds without pedaling
** Display blank and de-energize the resistance sense pot after 300 (or whatever) seconds without pedaling

## A few details
### Bike hardware
The M3 computer connects to a sensor/pickup board near the resistance magnet assembly using an RJ9 (phone handset) connector. The sensor board has a 10K pot that's linked to the magnet assembly, and a magnetic reed switch (not a Hall sensor) that closes once per crank revolution. Using the standard RJ9 lead colors, the leads are
* Green  - The crank switch. This is pulled to ground when a magnet on the crank assembly passes by.
* Black  - "Top" of the resistance magnet position sense pot
* Red    - Wiper of the pot
* Yellow - Ground - other end of the pot, and of the crank switch
Any replacement RJ9 cord provides access, or you could tap into the Keiser computer connector at the handlebar end. Retaining the ability to use the Keiser computer is recommended. In the absence of power pedals, calibration is done by matching the Keiser power estimates.

### Computer hardware
This code is for an Adafruit nrf52840 Express microcontroller and a generic 128x64 OLED display. The code will be the same or very similar for any board with a Nordic nrf52840.
