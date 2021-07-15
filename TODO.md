## TODO
- Calibration items
  - Save calibration in littleFS
- Console access
  - Option to use conventional USB serial instead of the spiffier BLE Uart service
- Documentation
  - More comprehensive writeup of calibration
  - Tutorial how-to-build for less experienced folks
  - Writeup on power consumption
- Consider using a different reference, e.g., Vdd ref for the pot to maintain cal near battery end of charge
- Keeping more parameters or options in the filesystem- 
- BLE Services
  - If continuing to support FTMS, implement a real model-based calc for speed (mph/kph) as a function of power and cadence. 
  - Either way, use globals rather than passed parameters for the data.
- Lots of functions have void argument lists and are there just for organizational purposes. Should they be marked as inline?
- Clean up code in connect callbacks
- Improve the display: larger size or double area
  - Things maybe to add
    - Accumulated  data
      - Miles - requires a model
      - Calories - requires a model
      - Elapsed time
- Other uses for swings of the resistance lever as a signal
  - e.g., reset ride between people, if providing accumulated data

ONE DAY?
- Servo on the resistance for full FTMS function!
