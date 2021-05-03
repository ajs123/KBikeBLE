/* Bluetooth console for Keiser M3 **********************************************************************************************
*    Baseline (unversioned) code - start services, simulate pedaling and power
*    Rev 0.1 - defined D7 (button) as pedal sensor input and added ISRs for sensor events
*    Rev 0.2 - transitioned to millis() for timing
*            - completed initial format/notify of Cycling Power Measurement
*            - update CPM once per second with or without a crank event (good idea, or not?)
*            - start with notify() both FTM (bike data) and CPM, then when client responds to one disable the other
*    Rev 0.3 - set up for resistance pot on Gnd, analog in, digital out (in order to turn off by setting high)
*            - added resistance to bike data (FTM Char) - not sure any clients do anything with it
*    Rev 0.4 - Added OLED display. Display blanks after extended time without a crank event.
*            - To keep the display active when BT disconnected, moved the crank event processing outside the bt_connected loop.
*    Rev 0.5 - Interface with the actual bike instead of user button and test pot.
*            - Calibration: 
*               - Resistance reading are normalized from 0 to 100
*               - Curve fit (polynomial) to map normalized resistance to Keiser gear
*               - Curve fit (polynomial) for power vs. resistance and power vs. speed
*    Rev 0.6 - Moving the gear lever to the top switches the display mode between Keiser gear and % resistance
*    Rev 0.7 - Fixed cadence clock wraparound bug
*            - Some code cleanup and comments
*            
*    TO DO List    .
*      * Because we stop BLE advertising when any client connects, apps that don't disconnect can hog the connection.
*        Consider continuing to advertise, or some more elegant way to handle this.
*      * Consider a #define DEBUG that turns on Serial and the LED heartbeat
*      * If continuing to implement FTMS, implement a real calc for speed (mph/kph) as a function of power and cadence
*      * Monitor battery level and provide an indicator
*      * Clean up code in connect callbacks
*      * Look at ways to further save power, especially when not connected 
*          * CPU sleep, and pedal to wake
*          * Stop bluetooth advertising under appropriate circumstances
*          * Only energize the pot when needing a resistance reading (significant savings?)
*          * Bluetooth parameters
*      * Consider more careful calibration to the bike, as well as the eddy current brake model used
*      * Use a proper (pixel-based) right-justification on the display      
*      * Something more flexible and elegant than the current hard-coded display routine
*          * In any event, choose between globals and arguments on the display routine(s)
*      * Improve the display: larger size or double area
*          * Things maybe to add
*              * Resistance % (with Keiser gear, or option based on #define or other)
*              * Accumulated  data
*                  * Miles - how to determine??
*                  * Calories
*                  * Elapsed time
*      * Other uses for swings of the resistance lever as a signal
*          * e.g., reset ride between people
*          
*    ONE DAY?
*      * Servo on the resistance for full FTMS function!
********************************************************************************************************************************/

/*********************************************************************
* Development started from the Adafruit HRM BLE example
/*********************************************************************
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

/*********************************************************************
 * Keiser M3 interface through the RJ9 jack (standard colors)
 *    Green  - CRANK_PIN - crank switch to ground (dry switch, not Hall effect)
 *             To a digital input. Transient protection is recommended.
 *    Black  - RESISTANCE_TOP - resistance sense pot high
 *             Use a digital output so it can be turned off to save power
 *    Red    - RESISTANCE_PIN - resistance sense pot wiper
 *             To an analog input channel
 *    Yellow - GROUND - resistance sense pot low / crank switch low side
 *    
 *    Pot is 10K
 *       Red to Yellow at min resistance: 0.33 Ohms
 *       Red to Yellow at max resistance: 5.8  Ohms
 *       With high side through nrf52040 digital output...
 *          Min 8-bit ADC reading 4
 *          Max ADC reading (at orig Keiser transition from 24 to "88") 121
 *********************************************************************/
//#define CRANK_PIN  7  // Pushbutton on the Adafruit nrf52840 Express, for debugging
#define CRANK_PIN 9
#define RESISTANCE_PIN A1
#define RESISTANCE_TOP 10

/********************************************************************
  * Calibration data
  *    Torque should be modelable as the product of a flux factor and a speed factor
  *    Power is then torque * speed^2
  *    Here, the cal is a direct fit of power vs. resistance setting @ constant speed
  *                 times a fit of power vs. speed at constant resistance setting
  *    Power = (7.68 - 0.459*R + 0.0685 * R^2) * (-2.93 + 0.0884*C - 3.73E-4*C^2)
  *                       ( power vs. R @ 60 ) * ( power vs. C @ Keiser gears 10 and 14 )
  *         where R = normalized resistance ( (Raw - min) * Factor )
  *               C = cadence
  *               
  *    Keiser gears - data taken at the bottom of the range for each gear, 
  *                   so floor() should be the gear as displayed
  *       Gear = -1.47 + 0.444*R - 1.67E-3*R^2
  *           (This is gear bottom, so the floor() should be the gear as displayed)
  ********************************************************************/

// Normalized resistance is ( reading - MIN ) * FACTOR
#define MIN_RESISTANCE_READING 4
#define MAX_RESISTANCE_READING 121
#define RESISTANCE_FACTOR 0.8547 // (ADC - MIN_RESISTANCE_READING) * RESISTANCE_FACTOR --> 0 .. 100 scale

/// These all use y = A*x^2 + B*x + C
#define PC1 7.68      // Power vs. R @ reference RPM
#define PB1 -0.459
#define PA1 0.0685
#define PC2 0         // Power/Power_ref vs. RPM
#define PB2 0.0137
#define PA2 8.97E-05

#define GC -1.47      // Keiser gear vs. normalized resistance
#define GB 0.444
#define GA -1.67E-03

// Labels for the display. Right now, they're hard-coded in the display routine.
//const char GEAR_L[] = "GEAR";
//const char POWER_L[] = "WATTS";
//const char CADENCE_L[] = "RPM";
//const char RESISTANCE_L[] = "RES %";

// Pushing the gearshift lever to the top switches between display of Keiser gear number and display of % resistance
uint8_t lever_state = 0b00000000;
bool gear_display = true;
#define BRAKE 100 

// BLE data blocks addressable as bytes for flags and words for data
union ble_data_block {  // Used to set flag bytes (0, 1) and data uint16's (words 1, ...)
  uint8_t bytes[2];
  uint16_t words[16];
};
union ble_data_block bike_data;  // For the Bike Data Characteristic (FiTness Machine Service)
union ble_data_block power_data; // For the Cycling Power Measurement Characteristic (Cycling Power Service)

// Globals. Variable accessed in multiple places are declared here. 
// Those used only in specific functions are declared within or nearby.
float cadence;                   // Pedal cadence
float power;                     // Power, calculated from resistance and cadence to match Keiser's estimates
float bspeed;                    // Bike speed, required by FTMS, to be estimated from power and cadence.
float resistance;                // Normalized resistance, determined from the eddy current brake magnet position
float inst_gear;                 // Keiser gear number, determined by cal to the Keiser computer
uint16_t crank_count = 0;        // Count of crank rotations, for CPS

volatile uint32_t crank_event_time = 0; // Set to the most recent crank event time by the crank sensor ISR [ms]
volatile uint32_t last_change_time;     // Used in the crank sensor ISR for sensor debounce [ms]. Initialized to millis() in Setup().
volatile bool new_crank_event = false;  // Set by the crank sensor ISR; cleared by the main loop
volatile uint16_t crank_ticks;          // 1/1024 sec per tick crank clock, for Cycling Power Measurement [ticks]

uint32_t prior_event_time = 0;          // Used in the main loop to hold the time of the last reported crank event [ms]

bool ftm_active = true;          // Once a client connects with either service, we stop updating the other.
bool cp_active = true;

// U8G2 display instance. What goes here depends upon the specific display and interface. See U8G2 examples.
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R1, /* reset=*/ U8X8_PIN_NONE); // 128 x 64 SH1106 display on hardware I2C

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

void display_setup(void)
{
  display.begin();
  display.enableUTF8Print();  // Can leave this out if using no symbols
}

void right_just(uint16_t number, int x, int y, int width)
// This works for the current font, but it ought to count pixels rather than characters.
{
  if (number < 10) x = x + width;
  if (number < 100) x = x + width;
  display.setCursor(x, y);
  display.print(number);
}

void display_numbers()
{
  display.clearBuffer();
  display.setFont(u8g2_font_helvR10_tf);
  display.setCursor(0, 11);
  display.print("RPM");
  display.setCursor(0, 53);
  if (gear_display) {
    display.print("GEAR");
  } else {
    display.print("RES %");
  }
  display.setCursor(0, 95);
  display.print("WATTS");

  display.setFont(u8g2_font_helvB24_tf);
  right_just((int) round(cadence), 10, 39, 18);
  if (gear_display) {
    right_just((int) round(inst_gear), 10, 81, 18);
  } else {
    right_just((int) round(resistance), 10, 81, 18);
  }
  right_just((int) round(power), 10, 123, 18);
  //Serial.println((int) round(resistance));

  display.sendBuffer();
}

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

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  Bluefruit.Advertising.addName();                 // Include name

  Bluefruit.Advertising.restartOnDisconnect(true); // Enable auto advertising if disconnected
  Bluefruit.Advertising.setInterval(32, 244);      // Fast mode 20 ms, slow mode 152.5 ms, in unit of 0.625 ms
  // For recommended advertising interval
  // https://developer.apple.com/library/content/qa/qa1931/_index.html
  Bluefruit.Advertising.setFastTimeout(30);        // number of seconds in fast mode

  Bluefruit.Advertising.start(0);                  // 0 = Don't stop advertising after n seconds
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

  // De-energize the resistance pot to save energy
  //digitalWrite(RESISTANCE_TOP, LOW);
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

  uint16_t power_int = round(power);
  uint16_t revs = crank_count;
  //uint16_t et = ((crank_event_time & 0xFFFF) * 1024 / 1000) & 0xFFFF ; // uint32 millisec to uint16 1024ths

  power_data.words[1] = power_int;
  power_data.words[2] = revs;
  power_data.words[3] = crank_ticks;

}

// ISR for crank sensor events. Triggerd on any change.
// To keep this short, it simply identifies a crank event as sensor active (0) when it's been inactive (1)
// for at least MIN_INACTIVE and sets the new_crank_event flag, increments the crank event count, and records the crank_event_time.
// The main loop handles formatting of crank/cadence data and doing BLE notify.

uint8_t prev_crank_state = 0b10;
const uint32_t MIN_INACTIVE = 100; // milliseconds

void crank_callback()
{
  uint32_t now = millis();
  uint8_t state = prev_crank_state | digitalRead(CRANK_PIN);  // Yields a combined state between 0b00 and 0b11

  switch (state) {
    case 0b00 :   // Was low (active) and still low - spurious/noise.
      prev_crank_state = 0b00;
      break;
    case 0b01 :   // Was low (active) and now high - nothing to do except note the event.
      prev_crank_state = 0b10;  // previous is current shifted left
      break;
    case 0b10 :   // Was high (inactive) and now low (active) - depends upon whether inactive long enough.
      if ((now - last_change_time) > MIN_INACTIVE) {  // True crank sensor leading edge.
        new_crank_event = true;  // This tells the main loop that there is new crank data
        crank_count++;           // Accumulated crank rotations - used by Cycling Power Measurement and by the main loop to get cadence
        crank_ticks += min((now - crank_event_time) * 1024/1000, 0xFFFF);  // Crank event clock in 1/1024 sec ticks - used by Cycling Power Measurement
                                                                           // If the crank has been still for > 2^16 ms, just add 2^16 ms
        crank_event_time = now;
      }
      prev_crank_state = 0b00;
      break;
    case 0b11 :   // Was high (inactive) and still high - spurious/noise.
      break;
  }
  last_change_time = now;
}

void lever_check()  // Moving the gear lever to the top switches the resistance/gear display mode
{
  lever_state = (lever_state << 1) & 0b00000011 | (resistance > BRAKE) ;
  if (lever_state == 0b00000001) gear_display = !gear_display;
}

void setup()
{
//  Serial.begin(115200);
//  uint16_t ticks;
//  for (ticks = 1000; !Serial && (ticks > 0); ticks--) delay(10); // Serial, if present, takes some time to connect

  display_setup();

  //Serial.println(ticks);
  //Serial.println("Bluefruit52 FTMS Example");
  //Serial.println("-----------------------\n");

  // Initialise the Bluefruit module
  //Serial.println("Initialise the Bluefruit nRF52 module");
  Bluefruit.begin();

  // Set the advertised device name (keep it short!)
  //Serial.println("Setting Device Name to 'Bikefruit'");
  Bluefruit.setName("Bikefruit");

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
  //Serial.println("Configuring and starting services");
  setupFTMS();
  setupCPS();

  // Setup the advertising packet(s)
  //Serial.println("Setting up the advertising payload(s)");
  startAdv();

  //Serial.println("Advertising");

  last_change_time = millis();
  pinMode(CRANK_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CRANK_PIN), crank_callback, CHANGE);

  // Apply voltage to the resistance pot
  pinMode(RESISTANCE_TOP, OUTPUT);
  digitalWrite(RESISTANCE_TOP, HIGH);

  // Set both notify characteristics to be active. Whichever the client responds to becomes the sole active characteristic
  ftm_active = true;
  cp_active = true;

  // Set up the analog input for resistance measurement
  analogReference(AR_INTERNAL);  // 3.6V
  analogReadResolution(8);       // Little use for more than 8 bits
}


float inst_cadence = 0;
uint16_t last_crank_count = 0;
uint8_t crank_timer = 0b00000100; // 3 update cycles until setting cadence to zero
uint16_t stop_time;
#define DIM_TIME 60
#define BLANK_TIME 300

void loop()
{
  //digitalToggle(LED_RED);

  // On a new crank event, calculate cadence (needed for both cps and ftm) and set flags
  if ( new_crank_event ) {
    //Serial.print("New crank event ");
    //Serial.print(crank_count);
    //Serial.print(" at ");
    //Serial.println(crank_event_time);
    stop_time = 0;
    display.setPowerSave(0);
    display.setContrast(255);
    new_crank_event = false;
    inst_cadence = (crank_count - last_crank_count) * 60000 / (crank_event_time - prior_event_time);
    last_crank_count = crank_count;
    prior_event_time = crank_event_time;
    crank_timer = 0b00000100;
  }
  // No crank event for N seconds, where N = bit position of initial crank_timer, interpreted as stopping
  else {
    crank_timer = ( crank_timer >> 1 );
    //Serial.print("No event. Crank timer is ");
    //Serial.println(crank_timer, BIN);
    if (crank_timer == 0) {
      inst_cadence = 0;
      if (++stop_time > BLANK_TIME) display.setPowerSave(1);
      else if (stop_time > DIM_TIME) display.setContrast(100);
    }
  }

  float raw_resistance = analogRead(RESISTANCE_PIN);  
  resistance = max((raw_resistance - MIN_RESISTANCE_READING) * RESISTANCE_FACTOR, 0); // Can be > 100 but not < 0
  lever_check();
  float resistance_sq = resistance * resistance;
  inst_gear = max(floor(GC + GB * resistance + GA * resistance_sq), 1) ;
  float inst_power = (PC1 + PB1 * resistance + PA1 * resistance_sq) * ( PC2 + PB2 * cadence + PA2 * cadence * cadence);
  
  //Serial.print("Raw, calc resistance = ");
  //Serial.print(raw_resistance);
  //Serial.print(",  ");
  //Serial.println(resistance);

  cadence = (cadence + inst_cadence) / 2;
  power = (power + inst_power) / 2;

  // Update the display
  display_numbers();
  //display_numbers(cadence, resistance, inst_gear);
  //display_numbers(cadence, resistance, 350);

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
  // Only send update once per second
  delay(1000);
}
