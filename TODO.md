###TODO
- Consider using a different reference, e.g., Vdd ref for the pot to maintain cal near battery end of charge
- Consider closer look at what needs to be recalculated under different circumstances - e.g., no need to re-do resistance calc if the value hasn't changed at all
- Waking up the display after it's blanked should cause the battery to be checked (set ticker appropriately)
- Resistance sensor calibration, equivalent to Keiser's procedure
- Consider a #define DEBUG that turns on Serial and the LED heartbeat
- If continuing to implement FTMS, implement a real calc for speed (mph/kph) as a function of power and cadence
- Clean up code in connect callbacks
- Real speed calibration
- Look at ways to further save power, especially when not connected
  - CPU sleep, and pedal to wake
  - Stop bluetooth advertising under appropriate circumstances
  - Only energize the pot when needing a resistance reading (significant savings?)
  - Bluetooth parameters
- Consider more careful calibration to the bike, as well as the eddy current brake model used
- Use a proper (pixel-based) right-justification on the display
- Something more flexible and elegant than the current hard-coded display routine
  - In any event, choose between globals and arguments on the display routine(s)
- Improve the display: larger size or double area
  - Things maybe to add
    - Resistance % (with Keiser gear, or option based on #define or other)
    - Accumulated  data
      - Miles - how to determine??
      - Calories
      - Elapsed time
- Other uses for swings of the resistance lever as a signal
  - e.g., reset ride between people

ONE DAY?
- Servo on the resistance for full FTMS function!