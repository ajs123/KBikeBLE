# How to build, install, and operate
## Parts needed
1. Adafruit Feather nRF52840 Express
2. A generic 128x64 pixel monochrome OLED display. These displays most commonly incorporate SH1106, SD1306, or similar display driver chips. To use the KBikeBLE code essentially as-is, choose one that comes configured for an I2C interface. 
2. Probably, pullup resistors for the I2C clock and data lines. Many (most?) of these displays will require you to add pullup resistors - something in the 10K-20K range - from the SCL and SDA pins to Vcc. According to your skills and how you plan to mount things, you can do that right on the back of the display board or in the wiring from the Feather to the display.
3. A suitable Lithium-Polymer battery, if you want the computer to work untethered. Note that the current code expects a battery and may endlessly flash the low battery indicator if one isn't there.  A little 375 mAHr battery will power the device for a week or so. An 1800 mAHr battery will keep it going for a couple of months.
4. A cable to connect to the bike, via the RJ9 connector on the resistance magnet assembly (right behind the crank shaft on the left side). RJ9 is the standard for handsets on landline phones, so these are easy to get. Some choices are
   - An RJ9 cable, with the connector cut off of one end, conductors stripped, and connected to the Feather by your preferred means. These cables are made to be mechanically terminated with insulation displacement connectors, so conductors are sometimes hard to strip cleanly.
   - An intact RJ9 cable, and an RJ9 receptacle to provide easily soldered wires for connection to the Feather.
   - Pull the existing cable from the Keiser computer and tap into that connector. Note that presently it may be helpful to retain the ability to compare Keiser computer readings with KBikeBLE readings, so destruction of the Keiser cable or making it hard to switch between the two isn't recommended.
5. Transient protection for the crank (pedal) rotation input to the Feather. While this may be unnecessary since the sensor assembly on the bike has a diode for the same purpose, the prototype used a 2.2K ohm resistor between the crank switch and the digital input to the Feather. 
6. Some sort of case. The prototype used a nice paper box with a piece of an old CD case as a window in the lid, and pieces of foam to take up unused space inside. Note that the OLED display is configured for *portrait* (vertical) orientation.

  ## Connecting the parts
  Part | Pin/conductor | Where | Notes
  ---- | ------------- | ----- | -----
  OLED display | Vcc | 3V
  || SCL | SCL
  || SDA | SDA
  || Gnd | Gnd
  Battery || JST connector | Be sure that the polarity is correct!
  RJ9 (bike) | Green* | Pin 9** | Crank (pedal rotation) switch***
  || Black* | Pin 10* | Resistance sensor excitation
  || Red* | Pin A1 | Resistance magnet position
  || Green* | Gnd | Resistance sensor / crank switch common

  \* Colors refer to the standard RJ9 conductors on most cables
  
  \** If changing any of these, edit bike_interface.h accordingly

  \*** Include a 2.2K Ohm resistor between the green wire and the pin on the Feather.

## Installing the software
### Determining the display configuration

* Install the Arduino IDE if you haven't already, and follow Adafruit's instructions for installing the core for the Feather nRF52840 Express. 
* Install the required libraries via the Arduino Library Manager:
  * Adafruit Bluefruit
  * U8G2 display
* Configure the U8G2 library for your display.
  * Choose an example from those that come with the U8G2 library, such as full_buffer/GraphicsTest or full_buffer/FontUsage.  
  * Near the top, look for a "constructor" line that matches your display. These are organized by driver chip (SD1306, SH1106, etc.) and interface. If you've followed the  advice above, you'll want one that matches the driver chip on your display and uses hardware I2C, so it will end with _HW_I2C, and you want one that provides Full Buffer capabilities, so it will include _F. Un-comment an appropriate line.
  * Download and run the example. If it works - great! If not, look again for an appropriate line, or check online. More than one may work, but the display might look better with certain choices. 
  * Copy the correct constructor line to globals.h in the indicated spot.

### Choosing options

* Look over options.h. The file includes options for 
  * power-saving timeouts, and whether power savings between cycling sessions is by full power-down or just idling all tasks (in practice, the difference is minimal).
  * filtering the keep the resistance display and power readings from flipping around too much (note: if electrical connections are secure, you shouldn't need much filtering).
  * whether you want to default to showing the "gear" like the Keiser computer, or the resistance in %. You can change what's displayed while using the bike.
  * Bluetooth power. Generally, the bike computer will be very close to whatever device it connects with. Choose something that works reliably.
  * whether you want to connect to KBikeBLE's command line interface via the USB (Serial) port and/or BluetoothLE (using Adafruit's app). 
* Pay attention to the settings at the very bottom of options.h. The software makes use of some functions not included in the Adafruit core for the Feather as of version 0.24. You must leave the #defines commented out or un-commented, according to whether the indicated functions are included in the release that you're using.
* Now download the KBikeBLE to the Feather.

## Getting started

At reset, KBikeBLE will briefly show a little startup log. When run for the first time, it should indicate that it's written default calibration values ("Factor" and "Offset") to flash memory. It will also show a current ADC (resistance sense) reading, indicate that it's run an ADC calibration, and show a second reading. On subsequent resets, it should show that's it's *read* calibration from flash, and if you haven't moved the bike from a very cool spot to a very hot spot the two ADC readings will probably be almost the same.

The display should then switch to the normal contents: 

* Battery indicator at the top right
* RPM (cadence)
* Resistance % or Gear
* Power

When first connected to a new bike, the resistance or gear display will almost certainly be wrong. But you should be able to pedal the bike an see the cadence. The blue Bluetooth LED will be flashing (rapidly for 30 seconds, then more slowly) and you should be able to connect.

Open the Adafruit Bluefruit Connect app on your phone, tablet, or laptop. If the blue LED has gone away by the time you've downloaded it, reset the feather.

KBikeBLE should show up in the list of nearby devices. Hit Connect. The app should connect to KBikeBLE. The blue LED should go from flashing to on, and the app should show the current battery charge along with a list of services. If you enabled BLEUart in options.h, one of the listed functions will be UART. Choose UART, type "batt" (without the quotes) and hit Send. Or type "help" then hit Send for a list of commands. You can do the same things through the Serial Monitor in the Arduino IDE, under Tools, as long as the USB cable remains connected.
