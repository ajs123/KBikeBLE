# How to build, install, and operate
## Parts needed
1. [Adafruit Feather nRF52840 Express]
(https://www.adafruit.com/product/4062)
![Feather](docs/nRF52840FE.jpg)
1. A generic 128x64 pixel monochrome OLED display. These displays most commonly incorporate SH1106, SD1306, or similar display driver chips. Be sure that it's supported by the [U8G2 library](https://github.com/olikraus/u8g2). To use the KBikeBLE code essentially as-is, choose one that comes configured for an I2C interface. 
1. Pullup resistors for the I2C clock and data lines. Many (most?) of these displays will require you to add pullup resistors - something in the 10K-20K range - from the SCL and SDA pins to Vcc. According to your skills and how you plan to mount things, you can do that right on the back of the display board or in the wiring from the Feather to the display.
1. A suitable Lithium-Polymer battery, if you want the computer to work untethered. Note that the current code expects a battery and may endlessly flash the low battery indicator if one isn't there.  A little 350 mAHr battery will power the device for a week or so. An 1800 mAHr battery will keep it going for a couple of months.
1. A cable to connect to the bike, via the RJ9 connector on the resistance magnet assembly (right behind the crank shaft on the left side). RJ9 is the standard for handsets on landline phones, so these are easy to get. Some choices are
   - An RJ9 cable, with the connector cut off of one end, conductors stripped, and connected to the Feather by your preferred means. These cables are made to be mechanically terminated with insulation displacement connectors, so conductors are sometimes hard to strip cleanly.
   - An intact RJ9 cable, and an RJ9 receptacle to provide easily soldered wires for connection to the Feather.
   - Pull the existing cable from the Keiser computer and tap into that connector. Note that presently it may be helpful to retain the ability to compare Keiser computer readings with KBikeBLE readings, so destruction of the Keiser cable or making it hard to switch between the two isn't recommended.
1. Transient protection for the crank (pedal) rotation input to the Feather. While this may be unnecessary since the sensor assembly on the bike has a diode for the same purpose, the prototype used a 2.2K ohm resistor between the crank switch and the digital input to the Feather. 
1. Some sort of case. The prototype used a nice paper box with a piece of an old CD case as a window in the lid, and pieces of foam to take up unused space inside. Note that the OLED display is configured for *portrait* (vertical) orientation.
1. A Keiser calibration tool.

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
  || Yellow* | Gnd | Resistance sensor / crank switch common

  \* Colors refer to the standard RJ9 conductors on most cables
  
  \** If changing any of these, edit bike_interface.h accordingly

  \*** Include a 2.2K Ohm resistor between the green wire and the pin on the Feather.

## Installing the software
* Install the Arduino IDE if you haven't already, and follow Adafruit's instructions for installing the core for the Feather nRF52840 Express. 
* Install the required libraries via the Arduino Library Manager:
  * Adafruit Bluefruit
  * U8G2 display
* You'll probably want to run some of the examples to ensure that the Arduino installation is working.

### Configure the U8G2 library for your display.
* Choose an example from those that come with the U8G2 library, such as full_buffer/GraphicsTest or full_buffer/HelloWorld.  
* Near the top, look for a "constructor" line that matches your display. You'll specify what display you have by un-commenting the appropriate line. These are organized by driver chip (SD1306, SH1106, etc.) and interface. Taking apart an example...
    ```C
    U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R1, /* reset=*/U8X8_PIN_NONE);
    ```
  All of the constructor lines begin with `U8G2_`

    ... followed by the driver chip, e.g., `_SH1106`

    ... then the dimensions in pixels, e.g., `_128x64`

    ... and the vendor, in this case `_NONAME` for a generic display

    ... `_F` for full buffer operation which is required for KBikeBLE

    ... and finally `_HW_I2C` which is correct for an I2C interface display connected to the SCL and SDA pins on the Feather.

  The name, `display` in this example, is the name used to refer to the display elsewhere in the code. Many of the U8G2 examples use `u8g2` here. Later, when you copy the constructor line to globals.h in the KBikeBLE code, you'll use `display`.
  
  Finally, notice the `U8G2_R1` in the example above. The `R1` specifies portrait (128 pixels vertical, 64 horizontal) orientation. When experimenting with the U8G2 examples, you can simply un-comment a line that seems right and leave the default `R0`. But later, in globals.h, you need to use `R1`.

* Run the example. If it works - great! If not, look again for an another appropriate line, or check online. Often, reviews from other buyers will mention what worked for them with U8G2. More than one may work, but the display might look better with certain choices, or the effect of a call to setContrast() (see below) may be different. 
* Copy the correct constructor to globals.h in the indicated spot, being sure to leave the name as `display` and to keep the `U8G2_R1`. If you've chosen an I2C interface display, only the driver and vendor will change.

### Choosing options

Look over options.h. The file includes a number of options, with some guidance for each: 
* Display contrast (brightness). 
* Power-saving timeouts, and whether power savings between cycling sessions is by full power-down or just idling all tasks.
* Filtering the keep the resistance display and power readings from flipping around too much (note: if electrical connections are secure, you shouldn't need much filtering).
* Whether you want to default to showing the "gear" like the Keiser computer, or the resistance in %. 
* Bluetooth power. 
* Whether you want to connect to KBikeBLE's command line interface via the USB (Serial) port and/or BluetoothLE (using Adafruit's app). 

Pay attention to the settings at the very bottom of options.h. The software makes use of some functions not included in the Adafruit core for the Feather, as of version 0.24. You must leave the #defines commented out or un-commented, according to whether the indicated functions are included in the release that you're using.

## Download and first run

Now use the Arduino IDE to download the KBikeBLE to the Feather.

At reset, KBikeBLE will briefly show a little startup log. When run for the first time, it should indicate that it's written default calibration values ("Factor" and "Offset") to flash memory. It will also show a current ADC (resistance sense) reading, indicate that it's run an ADC calibration, and show a second reading. On subsequent resets, it should show that's it's *read* calibration from flash, and if you haven't moved the bike from a very cool spot to a very hot spot the two ADC readings will probably be almost the same.

After a short delay, you should see the normal cadence/resistance (or gear)/power display: 

![display](docs/std_disp.jpeg)

When first connected to a new bike, the resistance or gear display will almost certainly be wrong. But you should be able to pedal the bike an see the cadence. The blue Bluetooth LED will be flashing (rapidly for 30 seconds, then more slowly) and you should be able to connect.

Open the Adafruit Bluefruit Connect app on your phone, tablet, or laptop. If the blue LED has gone away by the time you've downloaded it, reset the feather.

KBikeBLE should show up in the list of nearby devices. Hit Connect. The app should connect to KBikeBLE. The blue LED should go from flashing to on, and the app should show the current battery charge (if you included BLEBAS in options.h) along with a list of services ("modules"). If you enabled BLEUart in options.h, one of the listed modules will be UART. Choose UART. Check that the app is set to include an end-of-line.  Then type "batt" (without the quotes) and hit Send. Or type "help" then hit Send for a list of commands. 

You can do the same things through the Serial Monitor in the Arduino IDE, under Tools, as long as you included USE_SERIAL in options.h and the USB cable remains connected.

## Calibrating to your bike

The gearshift lever, through a cable, moves the magnetic brake so that more or less of the flywheel is under the influence of the magnets. Behind the cone-shaped magnet assembly, there's a position sensing potentiometer that's linked the the magnet assembly through gear teeth that mesh when the magnet assembly is placed on the bike. The rotation of both parts during assembly can vary. Calibration basically involves correcting for the variation.

As with the Keiser computer, the present calibration procedure requires Keiser's red calibration tool.  The tool fits over the flywheel and provides  "pockets" on both sides of the flywheel to receive the magnets. Holding the magnets against the tool provides the needed index position.

With the tool in hand, proceed as follows:

1. Be sure that KBikeBLE is awake. Turn the pedals so that the display comes on and the blue LED is flashing.
1. Connect to the bike either by ensuring that the USB port is connected and opening the Serial Monitor in the Arduino IDE, or by connecting through the Adafruit Bluefruit Connect app (iPhone/iPad, Android, or Mac) and choosing UART. If you've left USE_SERIAL and BLEUART defined in options.h, either will work.
1. Type *calibrate* and tap Send (phone or tablet) or Return.  (Note: all commands are case-insensitive.)
1. KBikeBLE will ask you to confirm that you want to calibrate. Type Y and Send or Return. From here, you'll get further prompts and countdowns, but no further typing is required, so your hands are free to hold the calibration tool and move the magnet assembly.  Be sure to abide by the instruction to leave the gearshift lever at the bottom and move the magnet assembly by hand. Any tension in the cable can affect the measurement. When prompted, gently holding the magnets in the pockets of the tool is all you need to do.
1. KBikeBLE will walk you through the procedure. 
    * At two steps - placing the tool on the flywheel and moving it up to the magnet assembly, and moving the magnets against the pockets in the tool - it will prompt and then show a 5-second count as you follow the instructions.  
    * KBikeBLE will show you the ten magnet position mesurements it has taken. These should be pretty consistent. Check that they vary by no more than a few single digits. Check that the first few and last few are similar to the middle measurements, indicating that you got the magnets settled into the pockets before measurements begain and held them there throughout. If in doubt, you can always repeat the procedure.
1. KBikeBLE will tell you what offset and factor it has determined. The offset is actually what has been determined, since (as far as we know) the factor shouldn't change. *These values are not yet active.*
1. Type *activate* and hit Send or Return. This makes the newly determined calibration active. 
1. Be sure to remove the tool, and move the gearshift lever up and down. You should see the gear number (1 to 24) or the % resistance change accordingly.
    * NOTE: Moving the lever beyond 100% (up to where the mechanical brake touches the flywheel) will switch between the gear and % resistance displays.
    * You'll probably find that setting the gearshift lever in a vertical position gives a resistance reading of about 40% and a gear number of 13 or 14. This might vary, but if the cable was adjusted to allow the magnet assembly to be as far away from the flywheel as it can go at the bottom, and the mechanical brake in firm contact at the top, a vertical lever will be at about 40% or around the border between gears 13 and 14. 
    * At the bottom (minimum), the gear number should be 1 and the resistance at least in the neighborhood of 0 (depends upon cable adjustment). At the top (mechanical brake) the gear will be 24 and the resistance will be well over 100%. Remember that each time you go beyond 100%, the display will switch between gear number and % resistance.
1. KBikeBLE is now using the calibration, but it hasn't been saved. If you'd reset the Feather now, it would go back to the defaults that were saved to flash memory.  Type *write* and Send or Return.  KBikeBLE will ask for confirmation. Use y to confirm, and the calibration will be saved.  The next time you reset the Feather, the startup log will show the values being read from flash.

### Alternative calibration against the Keiser computer

If you don't have the red Keiser calibration tool but do have the original Keiser computer and are confident that it was properly calibrated to your bike, you can proceed as follows:

1. Disconnect the KBikeBLE cable from the bike and plug in the Keiser computer in its place.
1. Turn the crank to wake up the Keiser computer. Wait for it to complete its startup and to show the gear indicator.
1. Adjust the gearshift lever so that the gear is right at the edge between gear 13 and gear 14. Flickering between the two, about equal time for each, is perfect. The Keiser computers shut down quickly when the crank isn't turning, so you may need to turn it occasionally. 
1. Without moving the gearshift lever or disturbing the magnet assembly, remove the Keiser cable and plug in the KBikeBLE cable.
1. Follow the calibration procedure above, but when prompted to place the tool and move the magnet assembly, just leave the bike untouched.
1. Use *activate* and *write* as described above to use the new settings and write them to flash memory.

## Final notes

### Gear vs. resistance display
The gear display replicated Keiser's gear numbers. These are not evenly spaced, and there's a big difference between the resistances at the "bottom" and "top" of each, but perhaps you participate in spin classes that refer to them.

The % resistance display is an approximately linear indication of the rotation of the magnet assembly, i.e., of the proportion of the magnets that is interacting with the flywheel. While there's a big difference between the "bottom" and "top" of a gear, the resistance at a given two-digit resistance is pretty repeatable.

To switch what is indicated, move the gearshift lever to the very top for about a second (The display needs to be on, but you do not need to be pedaling!).

If you chose POWERSAVE = 1 in options.h, KBikeBLE will power down between sessions and when pedal movement causes it to restart the display will go back to the default.  Otherwise, the display will remain in its current mode until you again move the lever to the top.

If you want to change the default at reset, you can do so in options.h.

### Stability and filtering of resistance measurements
The resistance measurements are pretty sensitive (a few digits translates to a 1% change) and the nRF52840 analog to digital converter has some variability from measurement to measurement. Accordingly, KBikeBLE provides some ability to filter measurements to provide a steadier display. 

An important note at the outset: Measurements should be *fairly* consistent. Unfiltered, % resistance might flicker by one digit, or maybe two. Using a release of the Adafruit nRF52 core beyond version 0.24 should provide cleaner readings because it provides an advantageous ADC setting (at this writing, the current Master on github provides this, and you then want to uncomment #define SAADC_TACQ in options.h). 

If the resistance display jumps around a lot, check that your wiring is secure. Check in particular whether the indicated resistance varies with whether the display is at full brightness, or dimmed. Through the command line interface, you can also check the resistance when the display is off but Bluetooth hasn't shut down yet. The resistance measurements should not depend upon the display. If they do, this is a specific indication that your ground connection at the Feather isn't secure.

If you wish, you can set filter values in options.h.  You shouldn't need values more than 2. Note that there are separete settings for the resistnce display and for the power estimate that's displayed and provided via Bluetooth. This is because training software may have its own filtering. Naturally, there is some tradeoff between a steady display and responsiveness.  Again, if the display is annoyingly variable, check your electrical connections.

## Training software

KBikeBLE provides the BLE Cycling Power Service and should be compatible with a wide range of training software. The author has most experience with excellent, open source, Golden Cheetah. Be patient: While (for example) the Adafruit Bluefruit Connect app will connect instantly, Golden Cheetah (for example) can take half a minute or more.

KBikeBLE also provides the BLE Fitness Machine Service. That service is incomplete - speed in mph is basically faked by providing a multiple of pedal cadence - and might be abandoned if it proves to be extraneous.  It appears that most software that connects with FTMS wants to set the resistance (providing a resistance servo in place of the lever is at the bottom of the TO DO list).