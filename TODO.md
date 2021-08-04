## TODO
- Documentation
  - More comprehensive writeup of calibration
  - Tutorial how-to-build for less experienced folks
  - Writeup on power consumption
  - Better writeup on calibration
- Calibration
  - Consider using a different reference, e.g., Vdd ref for the pot to maintain cal near battery end of charge
  - Check for temperature dependence or other sources of variation - is SAADC offset calibration needed?
- Keeping more parameters or options in the filesystem
  - Gear vs. Res% display
- BLE Services
  - If continuing to support FTMS, implement a real model-based calc for speed (mph/kph) as a function of power and cadence. 
- Improve the display: larger size with more data
  - Things maybe to add
    - Accumulated  data
      - Miles - requires a model
      - Calories - requires a model
      - Elapsed time
- Other uses for swings of the resistance lever as a signal
  - e.g., reset ride between people, if providing accumulated data
- More comprehensive code cleanup
  - Split code across multiple source files
  - Clean up code in connect callbacks
  - Review for functions that should be inline
  - C++ style for C-style #defines and macros where appropriate

ONE DAY?
- Servo on the resistance for full FTMS function!
