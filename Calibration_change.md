## Migrating from fixed to Vdd ADC reference
If you installed KBikeBLE prior to v1.1, the default ADC reference for reading the bike resistance setting was the fixed 3.6V reference. Starting with v1.1, the reference is the nominal 3.3V supply, i.e., Vdd. This was done because the Vdd reference should be more reliable. Resistance sensing is by the rotation of a potentiometer in the resistance magnet assembly, and the Vdd reference will provide a reading that's independent of the precise Vdd value, which can vary a bit from unit to unit. Using the Vdd reference also keeps the measurements table as the battery gets low and, potentially, Vdd begins to sag.

Since the calibration values saved in nonvolatile memory are used without regard to changes in the reference, you'll need to adjust those values. There are two straightforward options:

- Returning to default values and re-calibrating the bike. This is the most straightforward if you have the Keiser cal tool on hand. It should also be the most accurate.
- Adjusting the calibration parameters you already have. 

### Re-calibrating the bike
The ideal approach is to re-calibrate the bike using Keiser's tool. This is because the default scale factor has been re-calculated against a Keiser computer reference. If you have the Keiser tool, this is quick and easy:

1. Have the tool ready. Move the gearshift lever to the bottom position.
1. Connect with a USB cable and the Arduino serial monitor, or by Bluetooth using the Adafruit Bluefruit Connect app and its UART funciton.
1. Return to default calibration values:
    - Enter the command *defaults*. The system should respond with 

      Defaults...

      followed by the default values. These defaults include the correct scale factor.

    - Enter the command *activate* and confirm with *Y*.

1. Calibrate the bike:
    - Enter the command *calibrate* and follow the prompts.
    - Enter *activate* to make the new calibration active.
    - Enter *write* to save the new calibration.

### Migrating the existing calibration
    
1. Enter the command *adcref*. The system should respond with 
    
    ADC reference is now internal 3.6V.

1. Enter the command *read*. The system will read the calibration values that were used, prior to v1.1, with the fixed 3.6V reference.
1. Enter the command *activate* to make those values active.
1. Enter *adcref* to return to the Vdd reference. When switching references, the system will adjust the active calibration values.
1. Enter *write*, and confirm with *Y*, to save the new values.