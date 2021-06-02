 /* Bluetooth console for Keiser M3 **********************************************************************************************
   V0.13
********************************************************************************************************************************/

/*********************************************************************
  Development started from the Adafruit HRM BLE example.
**********************************************************************
  This is an example for our nRF52 based Bluefruit LE modules

  Pick one up today in the adafruit shop!

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  MIT license, check LICENSE for more information
  All text above, and the splash screen below must be included in
  any redistribution
*********************************************************************/

#include <bluefruit.h>   // nrf52 built-in bluetooth
#include <U8g2lib.h>     // OLED library
  #ifdef U8X8_HAVE_HW_SPI
  #include <SPI.h>
  #endif
  #ifdef U8X8_HAVE_HW_I2C  // For this hardware (nrf52840) should just be able to include Wire.h
  #include <Wire.h>
  #endif

#include "power_gear_tables.h"
#include "calibration.h"

#define POWERSAVE        // Incorporate power-saving shutdown
//#define USE_SERIAL           // Incorporate serial functions. Will attempt serial connection at startup.
#define BLEUART          // Activates serial over BLE
//#define DEBUG            // Activates debug code. Requires USE_SERIAL for any serial console bits.

#if defined(USE_SERIAL) && defined(DEBUG)
   #define DEBUG(x,l) Serial.print(x); Serial.print(l);
#else
   #define DEBUG(x,l)
#endif

//#define LEVER_EXTRAS stopBLE(); // Anything here is inserted into lever_check(), usually to
                                  // avoid having to wait for timeouts.

/*********************************************************************
   Keiser M3 interface through the RJ9 jack (standard colors)
      Green  - CRANK_PIN - crank switch to ground (dry switch, not Hall effect)
               To a digital input. Transient protection is recommended.
      Black  - RESISTANCE_TOP - 10K magnet position sense pot high
               Use a digital output so it can be turned off to save power
      Red    - RESISTANCE_PIN - magnet position sense pot wiper
               To an analog input channel
      Yellow - GROUND - resistance sense pot low / crank switch low side
 *********************************************************************/
//#define CRANK_PIN  7  // Pushbutton on the Adafruit nrf52840 Express, for debugging
#define CRANK_PIN 9
#define RESISTANCE_PIN A1
#define RESISTANCE_TOP 10
#define BATTERY_PIN    A6

// The battery is measured through a divider providing half the voltage
#define VBAT_MV_PER_LSB 7.03125  // 3600 mV ref / 1024 steps * 2.0 divider ratio

/********************************************************************************
   Options

 ********************************************************************************/
#define DIM_TIME 60         // Duration (sec) of no pedaling to dim display (if supported)
#define BLANK_TIME 180      // Duration (sec) to blank display
#define BLE_OFF_TIME 900    // Duration (sec) to turn off Bluetooth to save power. Will disconnect from Central after this time.
#define POWERDOWN_TIME 1200 // Duration (sec) to suspend the main loop pending a crank interrupt
#define CONTRAST_FULL 255
#define CONTRAST_DIM 0

#define ANALOG_OVERSAMPLE 4 // Resistance measurements can be noisy!

#define BLE_TX_POWER -12       // BLE transmit power in dBm
// - nRF52840: -40, -20, -16, -12, -8, -4, 0, +2, +3, +4, +5, +6, +7, or +8
#define BLE_LED_INTERVAL 1000  // ms

// Pushing the gearshift lever to the top switches between display of Keiser gear number and display of % resistance
uint8_t lever_state = 0b00000000;
bool gear_display = true;
#define BRAKE 100       // Edge of the valid range, less than the max reading

/********************************************************************************
   Globals
 ********************************************************************************/
// BLE data blocks addressable as bytes for flags and words for data
union ble_data_block {  // Used to set flag bytes (0, 1) and data uint16's (words 1, ...)
  uint8_t bytes[2];
  uint16_t words[16];
};
union ble_data_block bike_data;  // For the Bike Data Characteristic (FiTness Machine Service)
union ble_data_block power_data; // For the Cycling Power Measurement Characteristic (Cycling Power Service)

// Globals. Variable accessed in multiple places are declared here.
// Those used only in specific functions are declared within or nearby.

float cadence;                   // Pedal cadence, determined from crank event timing
float power;                     // Power, calculated from resistance and cadence to match Keiser's estimates
float bspeed;                    // Bike speed, required by FTMS, to be estimated from power and cadence. A real estimate is TO DO
float resistance;                // Normalized resistance, determined from the eddy current brake magnet position
float raw_resistance;            // Raw resistance measurement from the ADC. Global because it's reported in the serial monitor
float res_offset;                // Cal fators - raw_resistance to normalized resistance
float res_factor;
bool serial_present = false;
float resistance_sq;             // Set when checking resistance, used in both gear and power calcs
uint8_t inst_gear;               // Gear number: Index into power tables and, optionally, displayed
#ifdef POWERSAVE
bool suspended;                  // Set to true when suspending the main loop
#endif

uint8_t batt_pct;                 // Battery percentage charge

volatile float inst_cadence = 0;  // Cadence calculated in the crank ISR
volatile uint16_t crank_count = 0;      // Cumulative crank rotations - set by the crank sensor ISR, reported by CPS and used to determine cadence
volatile uint32_t crank_event_time = 0; // Set to the most recent crank event time by the crank sensor ISR [ms]
//volatile uint32_t last_change_time;     // Used in the crank sensor ISR for sensor debounce [ms]. Initialized to millis() in Setup().
volatile bool new_crank_event = false;  // Set by the crank sensor ISR; cleared by the main loop
volatile uint16_t crank_ticks;          // 1/1024 sec per tick crank clock, for Cycling Power Measurement [ticks]

//uint32_t prior_event_time = 0;          // Used in the main loop to hold the time of the last reported crank event [ms]

bool ftm_active = true;          // Once a client connects with either service, we stop updating the other.
bool cp_active = true;

uint8_t display_state = 2;          // Display state: 0 = off, 1 = dim, 2 = full on

// U8G2 display instance. What goes here depends upon the specific display and interface. See U8G2 examples.
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R1, /* reset=*/ U8X8_PIN_NONE); // 128 x 64 SH1106 display on hardware I2C
//U8G2_SH1106_128X64_WINSTAR_F_HW_I2C display(U8G2_R1, /* reset=*/ U8X8_PIN_NONE);
//U8G2_SH1106_128X64_VCOMH0_F_HW_I2C display(U8G2_R1, /* reset=*/ U8X8_PIN_NONE); // Provides a wider setContrast() range but at the expense of uniformity when dimmed

// FTMS Service Definitions
BLEService        svc_ftms = BLEService(0x1826);                 // FiTness machine service
BLECharacteristic char_ftm_feature = BLECharacteristic(0x2ACC);  // FiTness machine Feature characteristic
BLECharacteristic char_bike_data = BLECharacteristic(0x2AD2);    // Indoor Bike Data characteristic

// CPS Service Definitions
BLEService        svc_cps = BLEService(0x1818);                    // Cycling Power Service
BLECharacteristic char_cp_feature = BLECharacteristic(0x2A65);     // Cycling Power Feature
BLECharacteristic char_cp_measurement = BLECharacteristic(0x2A63); // Cycling Power Measurement
BLECharacteristic char_sensor_loc = BLECharacteristic(0x2A5D);     // Sensor Location

BLEDis bledis;    // DIS (Device Information Service) helper class instance
//BLEBas blebas;    // BAS (Battery Service) helper class instance
#ifdef BLEUART
BLEUart bleuart;  // UART over BLE
#endif

/*********************************************************************************
  Display code
* ********************************************************************************/

void display_setup(void)
{
  display.begin();
  //display.enableUTF8Print();  // Can leave this out if using no symbols
  display.setFontMode(1);  // "Transparent": Character background not drawn (since we clear the display anyway)
}

void right_just(uint16_t number, int x, int y, int width)
// This works for the current font, but it ought to count pixels rather than characters.
{
  if (number < 10) x = x + width;
  if (number < 100) x = x + width;
  display.setCursor(x, y);
  display.print(number);
}

#define BWIDTH 16
#define BHEIGHT 8
#define BUTTON 2
void draw_batt(uint8_t pos_x, uint8_t pos_y, uint8_t pct)
{
  display.drawFrame(pos_x, pos_y, BWIDTH, BHEIGHT);
  display.drawBox(pos_x + BWIDTH, pos_y + ((BHEIGHT - BUTTON) / 2), BUTTON, BUTTON);
  display.drawBox(pos_x + 2, pos_y + 2, pct * (BWIDTH - 4) / 100, (BHEIGHT - 4));
}

void display_numbers()
{
  display.clearBuffer();
  display.setFont(u8g2_font_helvR10_tr);
  display.setCursor(0, 16);
  display.print("RPM");
  display.setCursor(0, 58);
  if (gear_display) {
    display.print("GEAR");
  } else {
    display.print("RES %");
  }
  display.setCursor(0, 100);
  display.print("WATTS");

  draw_batt(46, 0, round(batt_pct));

  display.setFont(u8g2_font_helvB24_tn);
  right_just((int) round(cadence), 10, 43, 18);
  if (gear_display) {
    right_just(inst_gear, 10, 85, 18);
  } else {
    right_just((int) round(resistance), 10, 85, 18);
  }
  right_just((int) round(power), 10, 127, 18);

  display.sendBuffer();
}

/*********************************************************************************
  Analog input processing - resistance magnet position and battery

* ********************************************************************************/

void ADC_setup(void)               // Set up the ADC for ongoing resistance measurement
{
  analogReference(AR_INTERNAL);   // 3.6V
  analogOversampling(ANALOG_OVERSAMPLE);
  analogReadResolution(10);       // 10 bits for better gear delineation and for battery measurement
  delay(1);                       // Let the ADC settle before any measurements
}

float readVBAT(void)   // Compacted from the Adafruit example
{
  float mvolts;

  //analogReference(AR_INTERNAL_3_0);                    // Set the analog reference to 3.0V (default = 3.6V)
  //analogReadResolution(12); // Can be 8, 10, 12 or 14  // Set the resolution to 12-bit (0..4095)
  //delay(1);                                            // Let the ADC settle

  mvolts = analogRead(BATTERY_PIN) * VBAT_MV_PER_LSB;

  //ADC_setup();                                         // Set the ADC back to our default settings

  if (mvolts < 3300)                                  // LiPo model...
    return 0;                                         //   Dead at 3.3V
  if (mvolts < 3600)                                  //   Last 10% - linear from 3.3 - 3.6 V
    return (mvolts - 3300) * 0.03333;
  return min(10 + ((mvolts - 3600) * 0.15F ), 100);   //   10-100% - linear from 3.6 - 4.2 V
}

/*****************************************************************************************************
 * Gear and power determination
 *****************************************************************************************************/

int gear_lookup(float resistance) 
{
    int ix = inst_gear;                                               // The gear points to the top bound
    
    if (resistance >= gears[ix]) {                                    // If above the top bound, index up
        if (ix >= tablen) return tablen;                              // But not if already at the top
            for ( ; (resistance > gears[++ix]) && (ix < tablen); );   // Index up until resistance <= top bound or at end of the table 
        return ix;
    } else {
        for ( ; (resistance < gears[--ix]) && (ix > 0); );            // Index down until resistance >= bottom bound or at end of table
        return ++ix;                                                  // Decremented index is pointing to the bottom bound, so add 1
    } 
}

float sinterp(const float x_ref[], const float y_ref[], const float slopes[], int index, float x)
{
    float x1 = x_ref[index - 1];           // Index points to the top bound
    float y1 = y_ref[index - 1]; 
    return y1 + (x - x1) * slopes[index];  // Slope[index] is the slopes of the trailing interval
}

/*********************************************************************************
  Bluetooth
     Set up and add services.
     Bluetooth is started at reset. In the absence of pedaling for BLE_OFF_TIME sec,
     disconnect if connected and stop advertising. Pedal events (wakeup) restart
     advertising.
* ********************************************************************************/

void startAdv(void)
{
  // FiTness Machine Service requires a Service Data field specifying bike support and availability
  // Per https://www.bluetooth.com/specifications/assigned-numbers/generic-access-profile/) Service Data is Type 0x16
  // Contents are
  //      B0-1 - Fitness Machine Service UUID - UINT16 - 2 bytes  = 0x1826
  //      B2   - Flags                        - UINT8  - 1 byte   = 0x01   (Machine available)
  //      B3-4 - Fitness Machine Type         - UINT16 - 2 bytes  = 0x0020 (Indoor bike supported (bit 5))

  uint8_t FTMS_Adv_Data[5] = { 0x26, 0x18, 0x01, 0x20, 0x00 };

  Bluefruit.Advertising.addService(svc_ftms, svc_cps);      // Advertise the services
  Bluefruit.Advertising.addData(0x16, FTMS_Adv_Data, 5);    // Required data field for FTMS

  #ifdef BLEUART
  Bluefruit.Advertising.addService(bleuart);
  #endif

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  Bluefruit.Advertising.addName();                 // Include name

  Bluefruit.Advertising.restartOnDisconnect(true); // Enable auto advertising if disconnected
  Bluefruit.Advertising.setInterval(32, 244);      // Fast mode 20 ms, slow mode 152.5 ms, in unit of 0.625 ms
                                                   // We can be pretty aggressive since BLE will turn off when not in use
  // For recommended advertising interval
  // https://developer.apple.com/library/content/qa/qa1931/_index.html
  Bluefruit.Advertising.setFastTimeout(30);        // number of seconds in fast mode

  Bluefruit.Advertising.start(0);                  // 0 = Don't stop advertising after n seconds
}

void stopBLE(void)
{
  DEBUG("Stopping BLE", "\n")
  // If we're connected, disconnect. Since we only allow one connection, we know that the handle is 0
  // if (Bluefruit.connected(0)) Bluefruit.disconnect(0);
  Bluefruit.disconnect(0); // disconnect() includes check for whether connected
  delay(100);              // If restartOnDisconnect(true), this is needed for advertising to be stopped

  // If we're advertising, stop advertising.
  // if (Bluefruit.Advertising.isRunning()) Bluefruit.Advertising.stop(); // Or should this be time-limited?
  Bluefruit.Advertising.stop(); // this looks like it will work even if advertising isn't running
}

void restartBLE(void)
{
  if ( !Bluefruit.connected(0) && !Bluefruit.Advertising.isRunning()) {
    DEBUG("Restarting BLE", "\n")
    Bluefruit.Advertising.start(0);
  }
}

void setupFTMS(void)
{
  // Configure and start the FTM service
  // See:
  // Supported Characteristics:
  // Name                         UUID    Requirement Properties
  // ---------------------------- ------  ----------- ----------
  // Fitness Machine Feature      0x2ACC  Mandatory   Read
  // Indoor Bike Data             0x2A38  Mandatory   Notify

  // First .begin the service, prior to .begin on characteristics
  svc_ftms.begin();

  // Note: You must call .begin() on the BLEService before calling .begin() on
  // any characteristic(s) within that service definition.. Calling .begin() on
  // a BLECharacteristic will cause it to be added to the last BLEService that
  // was 'begin()'ed!

  // Configure the Fitness Machine Feature characteristic
  // See:
  // Properties = Read
  // Fixed Len  = 4
  //    B0      = UINT8  - Flag (MANDATORY)
  //      7     = 0 - Resistance level supported
  //      6     = 0 - Step count supported
  //      5     = 0 - Pace supported
  //      4     = 0 - Elevation gain supported
  //      3     = 0 - Inclination supported
  //      2     = 0 - Total distance supported
  //      1     = 1 - Cadence supported
  //      0     = 0 - Average speed supported
  //    B1      = UINT8
  //      7     = 0 - Force on belt and power output supported
  //      6     = 1 - Power measurement supported
  //      5     = 0 - Remaining time supported
  //      4     = 0 - Elapsed time supported
  //      3     = 0 - Metabolic equivalent supported
  //      2     = 0 - Heart rate measurement supported
  //      1     = 0 - Expended energy supported
  //      0     = 0 - Stride count supported
  //    B2      = UINT8
  //    2-7     = 0 - RESERVED
  //      0     = 0 - User data retention supported
  //    B3      = UINT8 - RESERVED

  char_ftm_feature.setProperties(CHR_PROPS_READ);
  char_ftm_feature.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  char_ftm_feature.setFixedLen(4);
  char_ftm_feature.setCccdWriteCallback(ftm_cccd_callback);  // Optionally capture CCCD updates

  char_ftm_feature.begin();

  uint8_t ftmf_data[4] = { 0b00000010, 0b01000000, 0b00000000, 0b00000000 };
  char_ftm_feature.write(ftmf_data, 4);

  // Configure the Indoor Bike Data characteristic - See 4.9.1 IN  FTMS_V1.0.pdf
  // See:
  // Properties = Notify
  // Fixed Len  = 8
  //    B0      = UINT8 - Flag (MANDATORY)
  //      7     = 0 - Average power present
  //      6     = 1 - Instantaneous power present
  //      5     = 1 - Resistance level present
  //      4     = 0 - Total distance present
  //      3     = 0 - Average cadence present
  //      2     = 1 - Instantaneous cadence present
  //      1     = 0 - Average speed present
  //      0     = 0 - More Data (Instantaneous speed field not present)
  //    B1      = UINT 8
  //      4     = 0 - Remaining time present
  //      3     = 0 - Elapsed time present
  //      2     = 0 - Metabolic equivalent present
  //      1     = 0 - Heart rate present
  //      0     = 0 - Expended energy present
  //    B2-3    = Instantaneous speed   - UINT16 - Km/Hr with 0.01 resolution
  //    B4-5    = Instantaneous cadence - UINT16 - 1/min with 0.5 resolution
  //    B6-7    = Instantaneous power   - UINT16 - W with 1.0 resolution

  char_bike_data.setProperties(CHR_PROPS_NOTIFY);
  char_bike_data.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  char_bike_data.setFixedLen(10);

  char_bike_data.begin();

  // Load flags into the data array. They won't change.
  bike_data.bytes[0] = 0b01100100;
  bike_data.bytes[1] = 0b00000000;
}

void setupCPS(void)
{
  // Configure and start the CP service
  // See:
  // Supported Characteristics:
  // Name                         UUID    Requirement Properties
  // ---------------------------- ------  ----------- ----------
  // Cycling Power Feature        0x2A65  Mandatory   Read
  // Cycling Power Measurement    0x2A63  Mandatory   Notify
  // Sensor Location              0x2A5D  Mandatory   Read

  // First .begin the service, prior to .begin on characteristics
  svc_cps.begin();

  // Configure the Cycling Power Feature characteristic
  // See https://github.com/sputnikdev/bluetooth-gatt-parser/blob/master/src/main/resources/gatt/characteristic/org.bluetooth.characteristic.cycling_power_feature.xml
  // Properties = Read
  // Fixed len = 4
  //
  // B0
  //   .7  - Accumulated energy supported              - 0
  //   .6  - Top and bottom dead spot angles supported - 0
  //   .5  - Extreme angles supported                  - 0
  //   .4  - Extreme magnitudes supported              - 0
  //   .3  - Crank revolution data supported           - 0
  //   .2  - Wheel revolution data supported           - 0
  //   .1  - Accumulated torque supported              - 0
  //   .0  - Pedal power balance supported             - 0
  // B1
  //   .7  - Span Length Adjustment Supported          - 0
  //   .6  - Chain Weight Adjustment Supported         - 0
  //   .5  - Chain Length Adjustment Supported         - 0
  //   .4  - Crank Length Adjustment Supported         - 0
  //   .3  - Multiple Sensor Locations Supported       - 0
  //   .2  - Cyc Pow Meas Char Cont Masking Supported  - 0
  //   .1  - Offset Compensation Supported             - 0
  //   .0  - Offset Compensation Indicator Supported   - 0
  // B2
  //   .7  - Reserved
  //   .6  - Reserved
  //  .4-5 - Distribute System Support                 - 0
  //  .4-5 - [00 Legacy; 01 No; 02 Yes; 03 RFU         - 0
  //   .3  - Enhanced Offset Compensation Supported    - 0
  //   .2  - Factory Calibration Date Supported        - 0
  //   .1  - Instantaneous Measu Direction Supported   - 0
  //   .0  - Sensor Meas Context (0 Force, 1 Torque)   - 0
  // B3
  //  .0-7 - Reserved

  uint8_t cpf_data[4] = { 0x0, 0x0, 0x0, 0x0 };
  char_cp_feature.setProperties(CHR_PROPS_READ);
  char_cp_feature.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  char_cp_feature.setFixedLen(4);
  char_cp_feature.begin();
  char_cp_feature.write(cpf_data, 4);

  // Configure the Sensor Location characteristic
  // See https://github.com/sputnikdev/bluetooth-gatt-parser/blob/master/src/main/resources/gatt/characteristic/org.bluetooth.characteristic.sensor_location.xml
  // Properties = Read
  // Fixed len = 4
  //
  // 0x01, 0x00, 0x00, 0x00 = "Other"
  uint8_t cl_data[4] = { 0x01, 0x00, 0x00, 0x00 };
  char_sensor_loc.setProperties(CHR_PROPS_READ);
  char_sensor_loc.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  char_sensor_loc.setFixedLen(4);
  char_sensor_loc.begin();
  char_sensor_loc.write(cl_data, 4);

  // Configure the Cycle Power Measurement characteristic
  // See https://github.com/sputnikdev/bluetooth-gatt-parser/blob/master/src/main/resources/gatt/characteristic/org.bluetooth.characteristic.cycling_power_measurement.xml
  // Properties = Notify
  // Fixed len = 8 // Flags[2], InstPower[2], CrankRev[2], Timestamp[2]
  //
  // Flags = { 0x20, 0x00 } = Crank Revolution Data present

  char_cp_measurement.setProperties(CHR_PROPS_NOTIFY);
  char_cp_measurement.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  char_cp_measurement.setFixedLen(8);
  char_cp_measurement.setCccdWriteCallback(cps_cccd_callback);  // Optionally capture CCCD updates
  char_cp_measurement.begin();

  power_data.bytes[0] = 0x20;  // Won't change
  power_data.bytes[1] = 0.00;

}

void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  //Serial.print("Connected to ");
  //Serial.println(central_name);
  //Serial.print("Connection handle ");
  //Serial.println(conn_handle);  // Find out if this is simply 0!

  // Once connected to central, apply voltage to the resistance sensing pot
  // Now that we have a display, we no longer do this here
  //pinMode(RESISTANCE_TOP, OUTPUT);
  //  pinMode(RESISTANCE_BOTTOM, OUTPUT);
  //digitalWrite(RESISTANCE_TOP, HIGH);
  //  digitalWrite(RESISTANCE_BOTTOM, LOW);
}

/**
   Callback invoked when a connection is dropped
   @param conn_handle connection where this event happens
   @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
*/
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  //Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
  //Serial.println("Advertising!");

  // Start over with both characteristics active
  ftm_active = true;
  cp_active = true;
}

void ftm_cccd_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t cccd_value) // Notify callback for FTM characteristic
{
  // Display the raw request packet
  //Serial.print("FTM CCCD Updated: ");
  //Serial.printBuffer(request->data, request->len);
  //Serial.print(cccd_value);
  //Serial.println("");

  // Check the characteristic this CCCD update is associated with in case
  // this handler is used for multiple CCCD records.
  // Because we use separate callbacks, we probably do not need these conditionals.
  if (chr->uuid == char_bike_data.uuid) {
    if (chr->notifyEnabled(conn_hdl)) {
      //Serial.println("Indoor Bike Data 'Notify' enabled");
      cp_active = false;  // Turn off notify() updates to the other characteristic
    } else {
      //Serial.println("Indoor Bike Data 'Notify' disabled");
    }
  }
}

void cps_cccd_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t cccd_value) // Notify callback for CPS characteristic
{
  // Display the raw request packet
  //Serial.print("CPS CCCD Updated: ");
  //Serial.printBuffer(request->data, request->len);
  //Serial.print(cccd_value);
  //Serial.println("");

  // Check the characteristic this CCCD update is associated with in case
  // this handler is used for multiple CCCD records.
  // Because we use separate callbacks, we probably do not need these conditionals.
  if (chr->uuid == char_cp_measurement.uuid) {
    if (chr->notifyEnabled(conn_hdl)) {
      //Serial.println("Cycling Power Measurement 'Notify' enabled");
      ftm_active = false;  // Turn off notify() updates to the other characteristic
    } else {
      //Serial.println("Cycling Power Measurement 'Notify' disabled");
    }
  }
}

void format_bike_data(float bspeed, float cadence, float resistance, float power)
{
  //    B2-3    = Instantaneous speed   - UINT16 - Km/Hr with 0.01 resolution
  //    B4-5    = Instantaneous cadence - UINT16 - 1/min with 0.5 resolution
  //    B6-7    = Instantaneous power   - UINT16 - W with 1.0 resolution

  uint16_t speed_int = round(bspeed / 0.01);
  uint16_t cadence_int = round(cadence / 0.5);
  uint16_t power_int = round(power);
  uint16_t resistance_int = round(resistance);

  bike_data.words[1] = speed_int;
  bike_data.words[2] = cadence_int;
  bike_data.words[3] = resistance_int;
  bike_data.words[4] = power_int;
}

void format_power_data(void)
{
  // Fields are
  //   B2-3     = Instantaneous power - UINT16 - W with 1.0 resolution
  //   B4-5     = Crank revolutions   - UINT16 - [unitless]
  //   B6-7     = Last crank event time - UINT16 - s with 1/1024 resolution (1024 counts/sec)

  //uint16_t power_int = round(power);
  //uint16_t revs = crank_count;
  //uint16_t et = ((crank_event_time & 0xFFFF) * 1024 / 1000) & 0xFFFF ; // uint32 millisec to uint16 1024ths

  power_data.words[1] = round(power);
  noInterrupts();  // crank_count and crank_ticks are set in the crank ISR
  power_data.words[2] = crank_count;
  power_data.words[3] = crank_ticks;
  interrupts();

}

/*********************************************************************************
  --- OLD ---
  ISR for crank sensor events. Triggerd on any change.
  To keep this short, it simply identifies a crank event as sensor active (0) when it's been inactive (1)
  for at least MIN_INACTIVE ms. Sets the new_crank_event flag, increments the crank event count, and records the crank_event_time.
  The main loop handles calculating the cadence, formatting of crank/cadence data and doing BLE notify.
**********************************************************************************

uint8_t prev_crank_state = 0b10;
uint32_t last_event_time = 0;
const uint32_t MIN_INACTIVE = 100; // milliseconds

void crank_callback()
{
  uint32_t now = millis();
  uint32_t dt;
  uint8_t state = prev_crank_state | digitalRead(CRANK_PIN);  // Yields a combined state between 0b00 and 0b11
  
  #ifdef POWERSAVE
  if (suspended) resume();   
  #endif

  switch (state) {
    case 0b00 :   // Was low (active) and still low - spurious/noise.
      //prev_crank_state = 0b00;
      break;
    case 0b01 :   // Was low (active) and now high - nothing to do except note the event.
      prev_crank_state = 0b10;  // previous is current shifted left
      break;
    case 0b10 :   // Was high (inactive) and now low (active) - depends upon whether inactive long enough.
      dt = now - crank_event_time; // ms since last true leading edge
      if (dt > MIN_INACTIVE) {     // True crank sensor leading edge.
        crank_count++;           // Accumulated crank rotations - used by Cycling Power Measurement and by the main loop to get cadence
        crank_ticks += min(dt * 1024 / 1000, 0xFFFF); // Crank event clock in 1/1024 sec ticks - used by Cycling Power Measurement
        new_crank_event = true;    // This tells the main loop that there is new crank data
        crank_event_time = now;
        inst_cadence = 60000 / (crank_event_time - prior_event_time);
      }
      prev_crank_state = 0b00;
      break;
    case 0b11 :   // Was high (inactive) and still high - spurious/noise.
      break;
  }
  last_change_time = now;
}
*/

/*********************************************************************************************
  Simplified ISR for crank sensor events. Triggerd on the pin tranisitioning low (switch closure).
  As long as there's been sufficient quiet time, trust the hardware and call it a crank event.
  Interrupts occurring closer together than MIN_ACTIVE are quickly dismissed.
**********************************************************************************************/

const uint32_t MIN_INACTIVE = 300; // milliseconds (corresponds to 200 rpm)

void crank_callback()
{
  uint32_t now = millis();
  //uint32_t now = xTaskGetTickCountFromISR();    // FreeRTOS gives time in 1/1024ths sec, whcih is what's needed for CPM
  uint32_t dt = now - crank_event_time;
  
  if (dt < MIN_INACTIVE) return;  

  #ifdef POWERSAVE
  if (suspended) resume();   
  #endif

  // The following are needed for the Cycling Power Measurement characteristic
  crank_count++;                                 // Accumulated crank rotationst
  crank_ticks += min(dt * 1024 / 1000, 0xFFFF);  // Crank event clock in 1/1024 sec ticks
  //crank_tickes += ( dt & 0xFFFF );               // dt is in FreeRTOS ticks, which are 1/1024 sec as needed for CPM

  crank_event_time = now;
  new_crank_event = true;     // Tell the main loop that there is new crank data
  inst_cadence = 60000 / dt;  // This used to be done in the main loop
  //inst_cadence = 61440 / dt;  // RPM = 60 / ( dt * 1000/1024 )
}

/********************************************************************************
 * Calibration: ADC reading --> normalized resistance
 ********************************************************************************/
void init_cal()
{
  res_offset = RESISTANCE_OFFSET;  // Later, these will be read from a file if present
  res_factor = RESISTANCE_FACTOR;
}

void setup()
{
#ifdef USE_SERIAL
  Serial.begin(115200);
  uint16_t ticks;
  for (ticks = 1000; !Serial && (ticks > 0); ticks--) delay(10); // Serial, if present, takes some time to connect
  if (Serial)
  {
    serial_present = true;
    Serial.setTimeout(5000);
  }
#endif

  display_setup();

  init_cal();

  Bluefruit.begin();

  // Set power - needs only to work in pretty close range
  Bluefruit.setTxPower(BLE_TX_POWER);
  Bluefruit.setConnLedInterval(BLE_LED_INTERVAL);

  // Set the advertised device name (keep it short!)
  Bluefruit.setName("KBikeBLE");

  // Set the connect/disconnect callback handlers
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Configure and Start the Device Information Service
  //Serial.println("Configuring the Device Information Service");
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  // Start the BLE Battery Service and set it to 100%
  //  Serial.println("Configuring the Battery Service");
  //  blebas.begin();
  //  blebas.write(100);

  // Setup the FiTness Machine and Cycling Power services
  setupFTMS();
  setupCPS();

  // Start the BLEUart service
  #ifdef BLEUART
  bleuart.begin();
  #endif

  // Setup the advertising packet(s)
  startAdv();

  //Serial.println("Advertising");

  // Crank sensing. A falling edge (switch closure) triggers the interrupt. This counts as a crank event (rotation) 
  // if it's been long enough since the last event. The rider could conceivably initiate faux rotations by holding
  // the crank right at a certain spot, but there are similar risks with any scheme.
  crank_event_time = millis();
  pinMode(CRANK_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CRANK_PIN), crank_callback, FALLING);

  // Apply voltage to the resistance pot
  pinMode(RESISTANCE_TOP, OUTPUT);
  digitalWrite(RESISTANCE_TOP, HIGH);

  // Set both notify characteristics to be active. Whichever the client responds to becomes the sole active characteristic
  ftm_active = true;
  cp_active = true;

  // Set up the analog input for resistance measurement
  ADC_setup();

  //sd_power_mode_set(NRF_POWER_MODE_LOWPWR);  see https://forums.adafruit.com/viewtopic.php?f=24&t=128823&sid=4f70bc48daaf47bd752bf8d108291049&start=75
}

/*********************************************************************************************
 * Suspend (power save) 
 *   - de-energizes the resistance sense pot
 *   - suspends the main loop
 *********************************************************************************************/
#ifdef POWERSAVE
void suspend()
{
  digitalWrite(RESISTANCE_TOP, LOW);   // De-energize the resistance pot
  //suspendLoop();                     // Suspend the main loop(). resumeLoop() doesn't seem to work.        
  suspended = true;                    // Setting this causes waitForEvent() in the main loop.
}

void resume()
{
  digitalWrite(RESISTANCE_TOP, HIGH);
  //resumeLoop();
  suspended = false;
}
#endif
/**********************************************************************************************
   Main loop()

   We define these periodic tasks:
     * Checking the resistance measurement. It helps for this to be updated 2/sec.
     * Checking for crank event(s) and recalculate cadence and power. This could be totally
       event-based, but here we're doing it once per second.
     * Updating BLE service data, if connected. Service specs call for this to be about 1/sec
     * Checking the battery. 1/min seems like enough.
     * Updating the display: 1/sec unless the resistance has changed.

   We use a little scheduler that calls each function at the appropriate times.

   The Adafruit board uses Open RTOS, and for minimal power we could use the RTOS scheduler.
   However, Open RTOS already provides a delay() function that's basically a sleep, so there's
   little (I think) benefit and it would make the code even more board-dependent.
 **********************************************************************************************/

#define TICK_INTERVAL 500 // ms
#define BATT_TICKS 64     // Battery check every __ ticks
#define DEFAULT_TICKS 2   // Default

uint8_t ticker = 0;       // Ticker inits to zero, also is reset under certain circumstances
uint16_t last_crank_count = 0;
uint8_t crank_still_timer = 3; // Number of updates with no crank event to call the cadence zero
uint16_t stop_time;

uint8_t prev_resistance = 0;
bool need_display_update;

void lever_check()  // Moving the gear lever to the top switches the resistance/gear display mode
{
  lever_state = (lever_state << 1) & 0b00000011 | (resistance > BRAKE) ;
  if (lever_state == 0b00000001) {
    gear_display = !gear_display;
#ifdef LEVER_EXTRAS
    LEVER_EXTRAS
#endif
  }
}

void update_resistance()
{
  /* In-code oversampling - superceded by oversampling in the nRF52 library
  float rx = analogRead(RESISTANCE_PIN);
  for (int ir = 0; ir < 2; ir++) 
  {
    delay(10);
    rx += analogRead(RESISTANCE_PIN);
  }
  raw_resistance = rx/3;
  */
  raw_resistance = analogRead(RESISTANCE_PIN);  // ADC set to oversample
  resistance = max((raw_resistance - res_offset) * res_factor, 0);
  inst_gear = gear_lookup(resistance);
  //resistance_sq = resistance * resistance;
  uint8_t int_resistance = round(resistance);
  if (prev_resistance != int_resistance) {
    lever_check();
    //inst_gear = max(floor(GC + GB * resistance + GA * resistance_sq), 1) ;
    need_display_update = true;
    prev_resistance = int_resistance;
    DEBUG("Raw resistance ", "")
    DEBUG(raw_resistance, "\n")
  }
}

void update_battery()
{
  batt_pct = readVBAT();
}

void process_crank_event()
{
  // On a new crank event (flag set by the crank event ISR), update 
  if (new_crank_event)
  {
    stop_time = 0;                  // Hold off power saving timeout

    if (display_state < 2)          // Be sure the display is on
    {
      display.setPowerSave(0);
      display.setContrast(CONTRAST_FULL);
      display_state = 2;
      ticker = BATT_TICKS;          // Force battery check after the display has been off
    }

    crank_still_timer = 0b100;     // Reset the shift register used to detect no pedaling
                                   // 3 shifts = 3 seconds without an event indicates no pedaling
                                   
    restartBLE();                  // Be sure BLE is running

    new_crank_event = false;     // Reset the flag

    // Calculate cadence and clear the event flag
//    noInterrupts(); // crank_count and crank_event_time mustn't change
//    inst_cadence = (crank_count - last_crank_count) * 60000 / (crank_event_time - prior_event_time);
//    last_crank_count = crank_count;
//    prior_event_time = crank_event_time;
//    interrupts();
  }
  else
  {
    crank_still_timer = crank_still_timer >> 1;
    if (crank_still_timer == 0)
    {
      inst_cadence = 0;
      if (++stop_time > BLANK_TIME)
      {
        display.setPowerSave(1);
        display_state = 0;
      }
      if (stop_time > DIM_TIME) 
      {
        display.setContrast(CONTRAST_DIM);
        display_state = 1;
      }
      if (stop_time == BLE_OFF_TIME)
        stopBLE(); // Note == rather than >   - Don't waste time "re-stopping"
#ifdef POWERSAVE
      if (stop_time == POWERDOWN_TIME)
        suspend();
#endif
    }
  }
}

void updateBLE()
{
  if ( Bluefruit.connected() ) {

    // Update data (with notify()) for the active characteristics. Both are active until the client responds to one of them.
    if (cp_active) {
      format_power_data();
      char_cp_measurement.notify((uint8_t *) &power_data, 8);
    }

    if (ftm_active) {
      bspeed = 20 * cadence / 60;   // NOT A REAL CAL OF ANY KIND!
      format_bike_data(bspeed, cadence, resistance, power);
      char_bike_data.notify((uint8_t *) &bike_data, 10);
    }
  }
}

bool new_cal = false;
float new_offset;
float new_factor;
char input;
int n;

char serial_cmd()
{
  int cmd = Serial.read();                   // Grab the first character
  while(Serial.available()) Serial.read();   // Flush the rest - eliminates type-ahead and CR/LF
  return cmd;
}

bool serial_confirm(String prompt, char expected)
{
  while(Serial.available()) Serial.read();   // Flush
  Serial.print(prompt);
  n = Serial.readBytes(&input, 1);
  if (n > 0) Serial.println(input);
  while(Serial.available()) Serial.read();
  return (input == expected);
}

void serial_check(void)
{
  // Simple serial monitor
  // Commands are
  //     R - report raw resistance value
  //     O - report the current offset
  //     F - report the current scale factor
  //     E - report current cal values
  //     C - enter cal values
  //     A - make new cal values active
  //     W - write cal values to file


  char cmd = serial_cmd();
  if (cmd < 0) return;

  switch(cmd) {
    case 'R' :
       Serial.print("\nRaw R ");
       Serial.println(raw_resistance);
       break;
    case 'O' :
       Serial.print("\nOffset ");
       Serial.println(res_offset, 2);
       break;
    case 'F' :
       Serial.print("\nFactor ");
       Serial.println(res_factor, 4);
       break;
    case 'C' :
       Serial.print("\nEnter cal...\n   --> New slope --> ");
       new_factor = Serial.parseFloat(SKIP_WHITESPACE);
       if (new_factor == 0) {
        Serial.println(" ... timeout\n");
        break;
       }
       Serial.println(new_factor);
       Serial.print("   --> New intercept --> ");
       new_offset = Serial.parseFloat(SKIP_WHITESPACE);
       if (new_offset == 0) {
        Serial.println(" ... timeout\n");
        break;
       }
       Serial.println(new_offset);
       Serial.println("Use A to activate.\n");
       new_cal = true;
       break;
    case 'A' :
       if (!new_cal) {
        Serial.println("\nEnter cal before trying to activate.");
        break;
       }
       Serial.println("\nReady to activate...");
       Serial.print("   Factor ");
       Serial.println(new_factor, 4);
       Serial.print("   Offset ");
       Serial.println(new_offset, 2);
       if (!serial_confirm(" A to activate; any other key or 5 seconds to skip --> ", 'A')) {
        Serial.println(" ... skipped\n");
        break;
       }
       res_factor = new_factor;
       res_offset = new_offset;
       new_cal = false;
       Serial.println("\n New cal factors active.\n");
       break;
    case 'W' :
       Serial.println(" Ready to save cal...");
       Serial.print("   Factor ");
       Serial.println(res_factor, 4);
       Serial.print("    OFFSET ");
       Serial.println(res_offset, 2);
       Serial.print(" W to write, or any other key or 5 seconds to skip -->");
       if (!serial_confirm(" W to write; any other key or 5 seconds to skip --> ", 'W')) {
        Serial.println(" ... skipped\n");
        break;
       }
       //write_cal_file();
       Serial.println("\n** If we had the filesystem set up, cal would have been written. **\n");
       break;
  }
}

#ifdef BLEUART
void bleuart_check(void)
{
  while (bleuart.available())
  {
    uint8_t ch = (uint8_t) bleuart.read();
    bleuart.write(ch);
  }
}
#endif

void LED_flash(int times, int ms)
{
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_RED, 1);
    delay(ms);
    digitalWrite(LED_RED, 0);
    if (i < times-1) delay(ms);
  }
}

void loop()
{
  // If suspended for power saving, wait here. The CPU should stop in a low power state, then continue after a crank interrupt.
  #ifdef POWERSAVE
  if (suspended) 
  {
    //LED_flash(2, 1000); // Show that we're here with 2 one-second flashes
    waitForEvent();  // This returns immediately since there's been an interrupt since the last call
    waitForEvent();  // Appears also to return due to something else such as FreeRTOS tick interrupt
    delay(5000);     // At least slow the loop down!
    //LED_flash(3, 1000); // Show that we're back with 3 one-second flashes
    return;
  }
  #endif

  // Otherwise, do what's needed based on the ticker value

  need_display_update = false;      // Various tasks can indicate that the display needs to be redrawn

  // Things that happens on every tick
  update_resistance();

  #ifdef SERIAL
  serial_check();
  #endif

  #ifdef BLEUART
  bleuart_check();
  #endif

  // Things happen at the default tick interval
  if ((ticker % DEFAULT_TICKS) == 0) {
    
    process_crank_event();
    
    cadence = inst_cadence;
    //cadence = (cadence + inst_cadence) / 2; //TEMPORARY UNTIL WE FIND MISSED EVENT BUG!
    
    float inst_power = max(sinterp(gears, power90, slopes, inst_gear, resistance) * ( PC2 + PB2 * cadence + PA2 * cadence * cadence), 0);
    //float inst_power = max((PC1 + PB1 * resistance + PA1 * resistance_sq) * ( PC2 + PB2 * cadence + PA2 * cadence * cadence), 0);
    power = inst_power;
    //power = (power + inst_power) / 2;
    
    need_display_update = true;
    updateBLE();
  }

  // Things that happen on BATT_TICKS
  if ((ticker % BATT_TICKS) == 0) {
    update_battery();
  }

  // Final things, as needed, on every tick
  if (need_display_update && (display_state > 0)) display_numbers();

  ticker++;
  delay(TICK_INTERVAL);
}

/* The idle task is another documented approach to power saving. It is NOT clear that this should be necessary when using tickless idle.
void vApplicationIdleHook( void )
{
  waitForEvent();
}
*/