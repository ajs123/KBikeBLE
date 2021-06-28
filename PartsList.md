## Parts list

The code assumes a Nordic nrf52 system-on-chip and I2C display. 
It uses Adafruit's Bluetooth LE library, which is a wrapper around Nordic's code, and the U8G2 library for the display. 

Code as provided works with...
- Adafruit nRF52840 Feather Express https://www.adafruit.com/product/4062
- Generic 64 x 128 pixel SH1106 OLED display with I2C interface. 
  - The prototype uses https://www.amazon.com/gp/product/B08V97FYD2. 
  - The U8G2 library supports a wide range of displays. Using a display with a controller other than the SH1106 involves 
    - Choosing a different constructor line from the U8G2 examples
    - Possibly choosing a different value for CONTRAST_DIM
- An RJ9 cable (landline phone handset cable) to plug into the connector on the bike. 
- A battery if you want the computer to work without power from the USB connector. Testing is with an Adafruit 4237 https://www.adafruit.com/product/4237. At 350 mAHr, this will power the computer for *about* a week with daily use (testing is incomplete!).

Electrical connections...
- Bike - RJ9 conductors to 3.3V, ground, GPIO, and analog input - see comments in the code
- Display - 3.3V, ground, SCD, SCL (standard I2C connection)
- Battery - with the Adafruit board and battery, plug it in.
