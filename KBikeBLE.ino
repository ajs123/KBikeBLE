/* Bluetooth console for Keiser M3 **********************************
    V0.16
*********************************************************************/

#include <Arduino.h>
#include <bluefruit.h> // nrf52 built-in bluetooth
#include <U8g2lib.h>   // OLED library
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C // For this hardware (nrf52840) should just be able to include Wire.h
#include <Wire.h>
#endif

#include "options.h"            // Options of most interest to the end user
#include "bike_interface.h"     // Defines the hardware connections to the bike
#include "power_gear_tables.h"
#include "calibration.h"
#include "serial_commands.h"

/**********************************************************************
 * Optional functions and debugging
 **********************************************************************/

//#define USE_SERIAL     // Incorporate USB serial functions. Will attempt serial connection at startup.
                         // CAREFUL! With 1/sec updates through a timer task, it must always finish in < 1 sec.
#define BLEUART // Activates serial over BLE
#define BLEBAS  // Activate BLE battery service

//#define QUICKTIMEOUT     // Quick timeout options for testing power saving functions
//#define DEBUGGING        // Activates debug code. Requires USE_SERIAL for any serial console bits.

// Anything here is inserted into lever_check(), usually to avoid having to wait for some condition 
//#define LEVER_EXTRAS batt_pct = 19; batt_low = true;
                                 
#ifdef QUICKTIMEOUT
#define DIM_TIME 20       //60         // Duration (sec) of no pedaling to dim display (if supported)
#define BLANK_TIME 30     //180      // Duration (sec) to blank display
#define BLE_PS_TIME 30   //900    // Duration (sec) to turn off Bluetooth to save power. Will disconnect from Central after this time.
#define POWERDOWN_TIME 40 //1200 // Duration (sec) to suspend the main loop pending a crank interrupt
#endif

#if defined(USE_SERIAL) && defined(DEBUGGING)
#define DEBUG(x, l) \
  Serial.print(x);  \
  Serial.print(l);
#else
#define DEBUG(x, l)
#endif

#ifdef DEBUGGING // Stepwise startup to watch for power consumption
uint8_t stepnum = 0;
#define STEP(N) \
  cadence = N; \
  display_numbers(); \
  delay(5000);
#define DWAIT() \
  delay(5000);
#else
#define STEP(N)
#define DWAIT()
#endif

/************************************************************************
 * Miscellany
 ************************************************************************/

// The battery is measured through a divider providing half the voltage
#define VBAT_MV_PER_LSB 7.03125 // 3600 mV ref / 1024 steps * 2.0 divider ratio

// Round for positive numbers
#define roundpos(x) ((x) + 0.5)

#define ANALOG_OVERSAMPLE 4 // Resistance measurements can be noisy!

// Pushing the gearshift lever to the top switches between display of Keiser gear number and display of % resistance
uint8_t lever_state = 0b00000000;
bool gear_display = GEAR_DISPLAY;
#define BRAKE 100 // Edge of the valid range, less than the max reading

/********************************************************************************
   Globals
 ********************************************************************************/
// BLE data blocks addressable as bytes for flags and words for data
union ble_data_block
{ // Used to set flag bytes (0, 1) and data uint16's (words 1, ...)
  uint8_t bytes[2];
  uint16_t words[16];
};
union ble_data_block bike_data;  // For the Bike Data Characteristic (FiTness Machine Service)
union ble_data_block power_data; // For the Cycling Power Measurement Characteristic (Cycling Power Service)

// Globals. Variable accessed in multiple places are declared here.
// Those used only in specific functions are declared within or nearby.

float cadence;         // Pedal cadence, determined from crank event timing
float power;           // Power, calculated from resistance and cadence to match Keiser's estimates
float bspeed;          // Bike speed, required by FTMS, to be estimated from power and cadence. A real estimate is TO DO
float inst_resistance; // Normalized resistance reading, determined from the eddy current brake magnet position
float resistance;      // Normalized resistance used in power calculations (can be filtered)
float disp_resistance; // Resistance valued that's displayed (can be filtered)
uint32_t raw_resistance;  // Raw resistance measurement from the ADC. Global because it's reported in the serial monitor
float res_offset;      // Cal fators - raw_resistance to normalized resistance
float res_factor;
bool serial_present = false;
//float resistance_sq;             // Used in both gear and power calcs if not using the lookup table
uint8_t gear;          // Gear number: Index into power tables and, optionally, displayed
#if (POWERSAVE > 0) && (POWERSAVE != 1)
bool suspended;        // Set to true when suspending the main loop
#endif

float batt_mvolts;  // Battery voltage
uint8_t batt_pct;   // Battery percentage charge
bool batt_low;      // Battery low

#define TICK_INTERVAL 500 // ms
uint8_t ticker = 0; // Ticker for the main loop scheduler - inits to zero, also is reset under certain circumstances


volatile float inst_cadence = 0;        // Cadence calculated in the crank ISR
volatile uint16_t crank_count = 0;      // Cumulative crank rotations - set by the crank sensor ISR, reported by CPS and used to determine cadence
volatile uint32_t crank_event_time = 0; // Set to the most recent crank event time by the crank sensor ISR [ms]
//volatile uint32_t last_change_time;   // Used in the crank sensor ISR for sensor debounce [ms]. Initialized to millis() in Setup().
volatile bool new_crank_event = false; // Set by the crank sensor ISR; cleared by the main loop
volatile uint16_t crank_ticks;         // 1/1024 sec per tick crank clock, for Cycling Power Measurement [ticks]

//uint32_t prior_event_time = 0;          // Used in the main loop to hold the time of the last reported crank event [ms]

bool ftm_active = true; // Once a client connects with either service, we stop updating the other.
bool cp_active = true;

uint8_t display_state = 2; // Display state: 0 = off, 1 = dim, 2 = full on

// U8G2 display instance. What goes here depends upon the specific display and interface. See U8G2 examples.
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R1, /* reset=*/U8X8_PIN_NONE); // 128 x 64 SH1106 display on hardware I2C
//U8G2_SH1106_128X64_WINSTAR_F_HW_I2C display(U8G2_R1, /* reset=*/ U8X8_PIN_NONE);
//U8G2_SH1106_128X64_VCOMH0_F_HW_I2C display(U8G2_R1, /* reset=*/ U8X8_PIN_NONE); // Provides a wider setContrast() range but at the expense of uniformity when dimmed

// FTMS Service Definitions
BLEService svc_ftms = BLEService(0x1826);                       // FiTness machine service
BLECharacteristic char_ftm_feature = BLECharacteristic(0x2ACC); // FiTness machine Feature characteristic
BLECharacteristic char_bike_data = BLECharacteristic(0x2AD2);   // Indoor Bike Data characteristic

// CPS Service Definitions
BLEService svc_cps = BLEService(0x1818);                           // Cycling Power Service
BLECharacteristic char_cp_feature = BLECharacteristic(0x2A65);     // Cycling Power Feature
BLECharacteristic char_cp_measurement = BLECharacteristic(0x2A63); // Cycling Power Measurement
BLECharacteristic char_sensor_loc = BLECharacteristic(0x2A5D);     // Sensor Location

BLEDis bledis; // DIS (Device Information Service) helper class instance
#ifdef BLEBAS
BLEBas blebas; // BAS (Battery Service) helper class instance
#endif
#ifdef BLEUART
BLEUart bleuart; // UART over BLE
#endif

/*********************************************************************************
  Display code
**********************************************************************************/

void display_setup(void)
{
  display.begin();
  display.setContrast(CONTRAST_FULL);
  //display.enableUTF8Print();  // Can leave this out if using no symbols
  display.setFontMode(1); // "Transparent": Character background not drawn (since we clear the display anyway)
}

void right_just(uint16_t number, int x, int y, int width)
// This works for the current font, but it ought to count pixels rather than characters.
{
  if (number < 10)
    x = x + width;
  if (number < 100)
    x = x + width;
  display.setCursor(x, y);
  display.print(number);
}

#define BWIDTH 16
#define BHEIGHT 8
#define BUTTON 2
#define BATT_POS_X 46
#define BATT_POS_Y 0
#define roundpct(pct, max) (((pct * max) + 50) / 100) // Integer arithmetic rounded % of max
void draw_batt(uint8_t pct)
{
  display.drawFrame(BATT_POS_X, BATT_POS_Y, BWIDTH, BHEIGHT);                                  // Battery body
  display.drawBox(BATT_POS_X + BWIDTH, BATT_POS_Y + ((BHEIGHT - BUTTON) / 2), BUTTON, BUTTON); // Battery "button"
  //display.setDrawColor(0);
  //display.drawBox(BATT_POS_X + 2, BATT_POS_Y + 2, BWIDTH - 4, BHEIGHT - 4);
  display.setDrawColor(1);
  display.drawBox(BATT_POS_X + 2, BATT_POS_Y + 2, roundpct(pct, (BWIDTH - 4)), (BHEIGHT - 4)); // Filled area proportional to charge estimate
}

void blank_batt()
{
  display.setDrawColor(0);
  display.drawBox(BATT_POS_X, BATT_POS_Y, BWIDTH + BUTTON, BHEIGHT);  // Erase the battery
}

#if (0) // not used
void update_batt(uint8_t pct)
{
  display.setDrawColor(0); // Empty interior
  display.drawBox(BATT_POS_X + 2, BATT_POS_Y + 2, BWIDTH - 4, BHEIGHT - 4);
  display.setDrawColor(1);
  display.drawBox(BATT_POS_X + 2, BATT_POS_Y + 2, pct * (BWIDTH - 4) / 100, (BHEIGHT - 4)); // Filled area proportional to charge estimate
}
#endif

uint16_t prev_cad = 0;
uint16_t prev_rg = 0;
uint16_t prev_pwr = 0;
uint8_t prev_batt = 0;
const char *rg_labels[2] = {"GEAR", "RES %"};
const char *rg_label = rg_labels[0];

void display_numbers()
{
  uint16_t rg = gear_display ? gear : roundpos(disp_resistance);
  uint16_t pwr = roundpos(power);
  uint16_t cad = roundpos(cadence);
  if ((cad != prev_cad) || (rg != prev_rg) || (pwr != prev_pwr) || (batt_pct != prev_batt) || batt_low)
  {
    display.clearBuffer();
    display.setFont(u8g2_font_helvB10_tr);
    display.setCursor(0, 16);
    display.print("RPM");
    display.setCursor(0, 58);
    display.print(rg_label);
    display.setCursor(0, 100);
    display.print("WATTS");

    if (!batt_low || (ticker % 2)) draw_batt(batt_pct);
    //else blank_batt();

    display.setFont(u8g2_font_helvB24_tn);
    right_just(cad, 10, 43, 18);
    right_just(rg, 10, 85, 18);
    right_just(pwr, 10, 127, 18);

    display.sendBuffer();

    prev_cad = cad;
    prev_rg = rg;
    prev_pwr = pwr;
    prev_batt = batt_pct;
  }
}

/*********************************************************************************
  Analog input processing - resistance magnet position and battery
* ********************************************************************************/

void ADC_setup(void) // Set up the ADC for ongoing resistance measurement
{
  analogReference(AR_INTERNAL); // 3.6V
  analogOversampling(ANALOG_OVERSAMPLE);
  analogReadResolution(10); // 10 bits for better gear delineation and for battery measurement
  delay(1);                 // Let the ADC settle before any measurements. Only important if changing the reference or possibly resolution
}

/*****************************************************************************************************
 * Gear and power determination
 *****************************************************************************************************/

int gear_lookup(float resistance)
{
  int ix = gear; // The gear points to the top bound

  if (resistance >= gears[ix])
  { // If above the top bound, index up
    if (ix >= tablen)
      return tablen; // But not if already at the top
    for (; (resistance > gears[++ix]) && (ix < tablen);)
      ; // Index up until resistance <= top bound or at end of the table
    return ix;
  }
  else
  {
    for (; (resistance < gears[--ix]) && (ix > 0);)
      ;          // Index down until resistance >= bottom bound or at end of table
    return ++ix; // Decremented index is pointing to the bottom bound, so add 1
  }
}

float sinterp(const float x_ref[], const float y_ref[], const float slopes[], int index, float x)
{
  float x1 = x_ref[index - 1]; // Index points to the top bound
  float y1 = y_ref[index - 1];
  return y1 + (x - x1) * slopes[index]; // Slope[index] is the slopes of the trailing interval
}

/*********************************************************************************
  Bluetooth
     Set up and add services.
     Bluetooth is started at reset. In the absence of pedaling for BLE_PS_TIME sec,
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

  uint8_t FTMS_Adv_Data[5] = {0x26, 0x18, 0x01, 0x20, 0x00};

  Bluefruit.Advertising.addService(svc_ftms, svc_cps);   // Advertise the services
  Bluefruit.Advertising.addData(0x16, FTMS_Adv_Data, 5); // Required data field for FTMS

#ifdef BLEUART
  Bluefruit.Advertising.addService(bleuart);
#endif

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  Bluefruit.Advertising.addName(); // Include name

  Bluefruit.Advertising.restartOnDisconnect(true); // Enable auto advertising if disconnected
  Bluefruit.Advertising.setInterval(32, 244);      // Fast mode 20 ms, slow mode 152.5 ms, in unit of 0.625 ms
                                                   // We can be pretty aggressive since BLE will turn off when not in use
  // For recommended advertising interval
  // https://developer.apple.com/library/content/qa/qa1931/_index.html
  Bluefruit.Advertising.setFastTimeout(30); // number of seconds in fast mode

  Bluefruit.Advertising.start(0); // 0 = Don't stop advertising after n seconds
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
  if (!Bluefruit.connected(0) && !Bluefruit.Advertising.isRunning())
  {
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
  char_ftm_feature.setCccdWriteCallback(ftm_cccd_callback); // Optionally capture CCCD updates

  char_ftm_feature.begin();

  uint8_t ftmf_data[4] = {0b00000010, 0b01000000, 0b00000000, 0b00000000};
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

  uint8_t cpf_data[4] = {0x0, 0x0, 0x0, 0x0};
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
  uint8_t cl_data[4] = {0x01, 0x00, 0x00, 0x00};
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
  char_cp_measurement.setCccdWriteCallback(cps_cccd_callback); // Optionally capture CCCD updates
  char_cp_measurement.begin();

  power_data.bytes[0] = 0x20; // Won't change
  power_data.bytes[1] = 0.00;
}

void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection *connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = {0};
  connection->getPeerName(central_name, sizeof(central_name));
}

/**
   Callback invoked when a connection is dropped
   @param conn_handle connection where this event happens
   @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
*/
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void)conn_handle;
  (void)reason;

  // Start over with both characteristics active
  ftm_active = true;
  cp_active = true;
}

void ftm_cccd_callback(uint16_t conn_hdl, BLECharacteristic *chr, uint16_t cccd_value) // Notify callback for FTM characteristic
{
  // Check the characteristic this CCCD update is associated with in case
  // this handler is used for multiple CCCD records.
  // Because we use separate callbacks, we probably do not need these conditionals.
  if (chr->uuid == char_bike_data.uuid)
  {
    if (chr->notifyEnabled(conn_hdl))
    {
      cp_active = false; // Turn off notify() updates to the other characteristic
    }
    else
    {
    }
  }
}

void cps_cccd_callback(uint16_t conn_hdl, BLECharacteristic *chr, uint16_t cccd_value) // Notify callback for CPS characteristic
{
  // Check the characteristic this CCCD update is associated with in case
  // this handler is used for multiple CCCD records.
  // Because we use separate callbacks, we probably do not need these conditionals.
  if (chr->uuid == char_cp_measurement.uuid)
  {
    if (chr->notifyEnabled(conn_hdl))
    {
      ftm_active = false; // Turn off notify() updates to the other characteristic
    }
    else
    {
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

  power_data.words[1] = roundpos(power);
  noInterrupts(); // crank_count and crank_ticks are set in the crank ISR
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
  
  if (suspended) resume();   

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
  As long as there's been more than MIN_INACTIVE time, trust the hardware and call it a crank event.
  Interrupts occurring closer together than MIN_ACTIVE are quickly dismissed.
**********************************************************************************************/

const uint32_t MIN_INACTIVE = 300; // milliseconds (corresponds to 200 rpm)

void crank_callback()
{
  //uint32_t now = millis(); // millis() is calculated from the FreeRTOS tick count and rate, so just use the tick count
  uint32_t now = xTaskGetTickCountFromISR(); // This depends upon the default tick interval of 1/1024 sec
  uint32_t dt = now - crank_event_time;

  if (dt < MIN_INACTIVE)     // Debounce: ignore crank events not spaced at least this far apart
    return;

#if (POWERSAVE > 0) && (POWERSAVE != 1)
  if (suspended)
    resume();
#endif

  // The following are needed for the Cycling Power Measurement characteristic
  crank_count++; // Accumulated crank rotationst
  //crank_ticks += min(dt * 1024 / 1000, 0xFFFF);  // Crank event clock in 1/1024 sec ticks
  crank_ticks = now & 0xFFFF;

  crank_event_time = now;
  new_crank_event = true; // Tell the main loop that there is new crank data
  //inst_cadence = 60000 / dt;  // This used to be done in the main loop
  inst_cadence = 61440 / dt; // RPM = 60 / ( dt * 1000/1024 );
}

/********************************************************************************
 * Calibration: ADC reading --> normalized resistance
 ********************************************************************************/
void init_cal()
{
  res_offset = RESISTANCE_OFFSET; // Later, these will be read from a file if present
  res_factor = RESISTANCE_FACTOR;
}

SoftwareTimer update_timer;

void setup()
{
#ifdef USE_SERIAL
  Serial.begin(115200);
  uint16_t ticks;
  for (ticks = 1000; !Serial && (ticks > 0); ticks--)
    delay(10); // Serial, if present, takes some time to connect
  if (Serial)
  {
    serial_present = true;
    Serial.setTimeout(5000);
  }
#endif

  DWAIT()

  display_setup();

  init_cal();

  STEP(10)

  Bluefruit.begin();

  STEP(20)

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
#ifdef BLEBAS
  blebas.begin();
#endif
  //  blebas.write(100);

  // Setup the FiTness Machine and Cycling Power services
  setupFTMS();
  setupCPS();

// Start the BLEUart service
#ifdef BLEUART
  bleuart.begin();
  //bleuart.bufferTXD(true);
  console_init();
  console_clear();
  console_reset();
#endif

STEP(30)

  // Setup the advertising packet(s)
  startAdv();

  // Crank sensing. A falling edge (switch closure) triggers the interrupt. This counts as a crank event (rotation)
  // if it's been long enough since the last event. The rider could conceivably initiate faux rotations by holding
  // the crank right at a certain spot, but there are similar risks with any scheme.
  crank_event_time = xTaskGetTickCountFromISR();
  pinMode(CRANK_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CRANK_PIN), crank_callback, FALLING);

STEP(40)

  // Apply voltage to the resistance pot
  pinMode(RESISTANCE_TOP, OUTPUT);
  digitalWrite(RESISTANCE_TOP, HIGH);

  // Set both notify characteristics to be active. Whichever the client responds to becomes the sole active characteristic
  ftm_active = true;
  cp_active = true;

STEP(50)

  // Set up the analog input for resistance measurement
  ADC_setup();

  update_timer.begin(TICK_INTERVAL, update);
  update_timer.start();
  suspendLoop();
}

/*********************************************************************************************
 * Suspend (power save) 
 *   - de-energizes the resistance sense pot
 *   - If POWERSAVE == 1, powers the system down, with the crank switch configured as reset
 *   - Otherwise, stops the scheduled update() task
 * See the notes in each section
 *********************************************************************************************/
#if (POWERSAVE > 0)
#if (POWERSAVE == 1)
/*
Power down until reset. Reset is by a selected level (not edge) on the crank switch GPIO pin.
We configure reset to occur when the crank switch changes to the opposite of its current state.

Though the crank switch is usually open because the range of movement during which the magnet is
nearby is small, it's possible for the pedals to be stopped with the switch closed. The stock
systemOff() sets up a pullup if reset is to be active-low, or a pulldown if it's to be active-high.
Here, with the non-GPIO side of the crank switch hard wired to ground, we're stuck with a pullup.
Our copy of the stock systemOff() substitutes PULLUP for PULLDOWN when SENSE_HIGH is needed. 

The compromise made is in battery usage: the closed switch with the 22K pullup consumes 0.15 mA
continuously. The prototype system uses about 1.0 mA when shut down due to power still being applied
to the nRF52840 and OLED display boards, so the additional 0.15 mA isn't terrible. 
Alternatives include (1) active control over the non-GPIO side of the switch, which
would affect the resistance measurements; (2) AC coupling of the switch (parallel RC - would consume)
less, but not zero, current; (3) a very weak external pullup.  
*/
void turnOff(uint32_t pin, uint8_t wake_logic)
{
  //  for(int i=0; i<8; i++)
  //  {
  //    NRF_POWER->RAM[i].POWERCLR = 0x03UL;
  //  }

  pin = g_ADigitalPinMap[pin];

  if (wake_logic)
  {
    //Original systemOff() uses NRF_GPIO_PIN_PULLDOWN here
    nrf_gpio_cfg_sense_input(pin, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_HIGH);
  }
  else
  {
    nrf_gpio_cfg_sense_input(pin, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
  }

  uint8_t sd_en;
  (void)sd_softdevice_is_enabled(&sd_en);

  // Enter System OFF state
  if (sd_en)
  {
    sd_power_system_off();
  }
  else
  {
    NRF_POWER->SYSTEMOFF = 1;
  }
}

void suspend()
{
  digitalWrite(RESISTANCE_TOP, LOW);            // De-energize the resistance pot
  turnOff(CRANK_PIN, !digitalRead(CRANK_PIN));  // Shut down, with a change in the crank switch causing reset
}

#else
/* 
  Enter the power save state by stopping the update() task. 
  Resume by re-starting it just as in Setup().
*/
void suspend()
{
  digitalWrite(RESISTANCE_TOP, LOW); // De-energize the resistance pot
  update_timer.stop();
  suspended = true;
}

void resume()
{
  digitalWrite(RESISTANCE_TOP, HIGH);
  update_timer.start();
  suspended = false;
}
#endif
#endif

/**********************************************************************************************
   Main "loop()"

   We define these periodic tasks:
     * Checking the resistance measurement. It helps the user for this to be updated 2/sec.
     * Checking for crank event(s) that were handled by the crank ISR, recalculating power, 
       and determine any need for power savings steps.
     * Updating BLE service data, if connected. Service specs call for this to be about 1/sec
     * Checking the battery. Every 30 sec seems like plenty.
     * Updating the display as needed.

   These are all handled by a single function, update(), which calls each individual function 
   at the appropriate times. update() is scheduled as a recurring task in FreeRTOS which is part
   of the Adafruit Arduino core for the nRF52840. This gives better performance (lower energy) than
   using the traditional loop() with delay(), despite the FreeRTOS tickless idle mode by which
   delay() ought to basically be a sleep().

   To provide good responsiveness for the user, twice per second the bike resistance is measured 
   and the display is updated if anything has changed. The battery level is measured every 30 seconds. 
   Everything else - power calculation and updating Bluetooth are done once per second. These could be 
   scheduled as three separate recurring tasks in FreeRTOS but that doesn't seem worth the trouble.
 **********************************************************************************************/

#define BATT_TICKS 60     // Battery check every __ ticks
#define DEFAULT_TICKS 2   // Default stuff every __ ticks

uint16_t last_crank_count = 0;
uint8_t crank_still_timer = 3; // No pedaling is defined as this many crank checks with no event
uint16_t stop_time;

void lever_check() // Moving the gear lever to the top switches the resistance/gear display mode
{
  lever_state = (lever_state << 1) & 0b00000011 | (inst_resistance > BRAKE);
  if (lever_state == 0b00000001)
  {
    gear_display = !gear_display;
    rg_label = rg_labels[(int)!gear_display];
#ifdef LEVER_EXTRAS
    LEVER_EXTRAS
#endif
  }
}

void update_resistance()
{
  raw_resistance = analogRead(RESISTANCE_PIN); // ADC set to oversample
  inst_resistance = max((raw_resistance - res_offset) * res_factor, 0);
  resistance = (RESISTANCE_FILTER * resistance + inst_resistance) / (RESISTANCE_FILTER + 1);
  gear = gear_lookup(resistance);
  disp_resistance = (RESISTANCE_DISPLAY_FILTER * resistance + inst_resistance) / (RESISTANCE_DISPLAY_FILTER + 1);
  //resistance_sq = resistance * resistance;
  //gear = max(floor(GC + GB * resistance + GA * resistance_sq), 1) ;
  lever_check();
}

void update_battery()
{
  batt_mvolts = analogRead(BATTERY_PIN) * VBAT_MV_PER_LSB;

  if (batt_mvolts < 3300)                                         // LiPo model...
    batt_pct = 0.0;                                               //   Dead at 3.3V
  else if (batt_mvolts < 3600)                                    
    batt_pct = (batt_mvolts - 3300) * 0.03333;                    //   Last 10% - linear from 3.3 - 3.6 V
  else 
    batt_pct = min(10.0 + ((batt_mvolts - 3600) * 0.15F), 100.0); //   10-100% - linear from 3.6 - 4.2 V

  batt_low = batt_pct < BATT_LOW;
#ifdef BLEBAS
  blebas.write(batt_pct);
#endif
}

void process_crank_event()
{
  if (new_crank_event) // On a new crank event (flag set by the crank event ISR), update
  {
    stop_time = 0; // Hold off power saving timeout

    if (display_state < 2) // Be sure the display is on
    {
      display.setPowerSave(0);
      display.setContrast(CONTRAST_FULL);
      display_state = 2;
      ticker = BATT_TICKS; // Force battery check after the display has been off
    }

    crank_still_timer = 0b100; // Reset the shift register used to detect no pedaling
                               // 3 shifts = 3 seconds without an event indicates no pedaling

    restartBLE(); // Be sure BLE is running

    new_crank_event = false; // Reset the flag

    // Calculate cadence and clear the event flag
    //    noInterrupts(); // crank_count and crank_event_time mustn't change
    //    inst_cadence = (crank_count - last_crank_count) * 60000 / (crank_event_time - prior_event_time);
    //    last_crank_count = crank_count;
    //    prior_event_time = crank_event_time;
    //    interrupts();
  }
  else // If no crank event, check timeouts
  {
    crank_still_timer = crank_still_timer >> 1;
    if (crank_still_timer == 0)
    {
      inst_cadence = 0;
      stop_time++;
      switch (display_state)
      {
      case 2: // Full brightness
        if (stop_time > DIM_TIME)
        {
          display.setContrast(CONTRAST_DIM);
          display_state = 1;
        }
        break;
      case 1: // Dimmed
        if (stop_time > BLANK_TIME)
        {
          display.setPowerSave(1);
          display_state = 0;
        }
        break;
      }
      if (Bluefruit.connected())
      {
        if (stop_time >= BLE_PS_TIME) // Active BLE connection:
        {                              // At BLE_PS_TIME, stop BLE and optionally suspend
          stopBLE();
#if (POWERSAVE > 0)
          suspend();
#endif
        }
      }
      else                             // No active BLE connection:
      {                                // At NO_BLE_PS_TIME, optionally suspend
#if (POWERSAVE > 0)
        if (stop_time >= NO_BLE_PS_TIME)
        {
          Bluefruit.Advertising.stop(); // Ensure that we don't shut down with the BLE LED lit
          suspend();
        }
#endif
      }
    }
  }
}

void updateBLE()
{
  if (Bluefruit.connected())
  {

    // Update data (with notify()) for the active characteristics. Both are active until the client responds to one of them.
    if (cp_active)
    {
      format_power_data();
      char_cp_measurement.notify((uint8_t *)&power_data, 8);
    }

    if (ftm_active)
    {
      bspeed = 20 * cadence / 60; // NOT A REAL CAL OF ANY KIND!
      format_bike_data(bspeed, cadence, resistance, power);
      char_bike_data.notify((uint8_t *)&bike_data, 10);
    }
  }
}

#ifdef USE_SERIAL

bool new_cal = false;
float new_offset;
float new_factor;
char input;
int n;

char serial_cmd()
{
  int cmd = Serial.read(); // Grab the first character
  while (Serial.available())
    Serial.read(); // Flush the rest - eliminates type-ahead and CR/LF
  return cmd;
}

bool serial_confirm(String prompt, char expected)
{
  while (Serial.available())
    Serial.read(); // Flush
  Serial.print(prompt);
  n = Serial.readBytes(&input, 1);
  if (n > 0)
    Serial.println(input);
  while (Serial.available())
    Serial.read();
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
  if (cmd < 0)
    return;

  switch (cmd)
  {
  case 'R':
    Serial.print("\nRaw R ");
    Serial.println(raw_resistance);
    break;
  case 'O':
    Serial.print("\nOffset ");
    Serial.println(res_offset, 2);
    break;
  case 'F':
    Serial.print("\nFactor ");
    Serial.println(res_factor, 4);
    break;
  case 'C':
    Serial.print("\nEnter cal...\n   --> New slope --> ");
    new_factor = Serial.parseFloat(SKIP_WHITESPACE);
    if (new_factor == 0)
    {
      Serial.println(" ... timeout\n");
      break;
    }
    Serial.println(new_factor);
    Serial.print("   --> New intercept --> ");
    new_offset = Serial.parseFloat(SKIP_WHITESPACE);
    if (new_offset == 0)
    {
      Serial.println(" ... timeout\n");
      break;
    }
    Serial.println(new_offset);
    Serial.println("Use A to activate.\n");
    new_cal = true;
    break;
  case 'A':
    if (!new_cal)
    {
      Serial.println("\nEnter cal before trying to activate.");
      break;
    }
    Serial.println("\nReady to activate...");
    Serial.print("   Factor ");
    Serial.println(new_factor, 4);
    Serial.print("   Offset ");
    Serial.println(new_offset, 2);
    if (!serial_confirm(" A to activate; any other key or 5 seconds to skip --> ", 'A'))
    {
      Serial.println(" ... skipped\n");
      break;
    }
    res_factor = new_factor;
    res_offset = new_offset;
    new_cal = false;
    Serial.println("\n New cal factors active.\n");
    break;
  case 'W':
    Serial.println(" Ready to save cal...");
    Serial.print("   Factor ");
    Serial.println(res_factor, 4);
    Serial.print("    OFFSET ");
    Serial.println(res_offset, 2);
    Serial.print(" W to write, or any other key or 5 seconds to skip -->");
    if (!serial_confirm(" W to write; any other key or 5 seconds to skip --> ", 'W'))
    {
      Serial.println(" ... skipped\n");
      break;
    }
    //write_cal_file();
    Serial.println("\n** If we had the filesystem set up, cal would have been written. **\n");
    break;
  }
}
#endif

#ifdef BLEUART
#define SBUF_LEN 20
char sbuffer[SBUF_LEN];  // Serial console input buffer
char cmd[SBUF_LEN];     // Tokenized command
char arg[SBUF_LEN];     // Tokenized argument
uint8_t sbix;             // Current index into input buffer
bool cmd_rcvd;          // Console state: command received
bool new_input;         // Console state: new input ready to process
uint8_t cmd_number;             // Current command (= position in the table)
uint8_t awaiting_conf = AWAITING_NONE;      // Console state: awaiting confirmation for this numbered command
uint8_t awaiting_timer;  // Timeout counter for confirmation
float new_factor;
float new_offset;
  
TaskHandle_t ble_task_handle;

inline void console_clear()  // Clears the console for fresh input
{
  sbix = 0; 
  cmd_rcvd = false; 
  cmd[0] = 0; 
  arg[0] = 0; 
  new_input = false; 
  //bleuart.flush();  // clear the input to avoid typeahead
}

#define AWAITING_CONF() awaiting_conf != AWAITING_NONE

inline void console_reset()
{
  new_factor = res_factor;
  new_offset = res_offset;
}

inline void console_init()
{
  bool result = xTaskCreate(bleuart_check, "BLEuartCheck", SCHEDULER_STACK_SIZE_DFLT, ( void * ) 1, TASK_PRIO_LOW, &ble_task_handle);
}

void bleuart_take_input()
{
  while (bleuart.available())
    {
      uint8_t c = (uint8_t) bleuart.read();
      switch (c)
      {
      case 32: // delimeter
        if (sbix == 0)
          break; // Ignore leading spaces in both the command and the argument
        if (!cmd_rcvd)
        {
          memcpy(&cmd, &sbuffer, sbix); // Save the command and ready the buffer for an argument
          cmd[sbix] = 0;
          cmd_rcvd = true;
          sbix = 0;
        }
        else
        {
          if (sbix < SBUF_LEN - 1)
            sbuffer[sbix++] = c; // Keep embedded spaces in the argument
        }
        break;

      case 10:
      case 13:
        //if (new_input) break;
        if (sbix > 0) // Anything in the buffer?
        {
          if (!cmd_rcvd)
          {
            memcpy(&cmd, &sbuffer, sbix); // Save the command
            cmd[sbix] = 0;
            cmd_rcvd = true;
          }
          else
          {
            memcpy(&arg, &sbuffer, sbix); // ...or the argument
            arg[sbix] = 0;
          }
        }
        if (cmd_rcvd)
          new_input = true;
        break;

      default:
        if (sbix < SBUF_LEN - 1)
          sbuffer[sbix++] = tolower(c); // Add the character to the pending command or argument
        //std::cout << sbuffer << "\n";
        break;
      }
    }
}

// BLEUart messages are limited to the BLE MTU - 3. The bleuart library print/write functions will truncate
// anything longer. So we need our own function to send longer strings in pieces.


#define BLEUART_MAX_MSG 20
/* using absolute memory addresses
void bleuart_send(const char * message)
{
  const char * addr = message;
  const char * last = message + strlen(message);
  while (last - addr >= BLEUART_MAX_MSG)
  {
    bleuart.write(addr, BLEUART_MAX_MSG);
    addr += BLEUART_MAX_MSG;
  }
  if (addr < last)
  {
    bleuart.write(addr, last - addr);
  }
}
*/

/* Using relative memory addresses */
void bleuart_send(const char * message)
{
  size_t index = 0;
  size_t last = strlen(message);  
  while (last - index >= BLEUART_MAX_MSG)
  {
    bleuart.write(message + index, BLEUART_MAX_MSG);
    index += BLEUART_MAX_MSG;
  }
  if (index < last)
  {
    bleuart.write(message + index, last - index);
  }
}

#define PBLEN 128
char pbuffer[PBLEN];
#define RESPOND(string) bleuart_send(string)
#define BLE_PRINT(format, args...) snprintf(pbuffer, PBLEN, format, args); \
                                   bleuart_send(pbuffer)

void cmd_batt()
{
  BLE_PRINT("Battery voltage %.2f \n", batt_mvolts/1000);//batt_mvolts*1000);
  BLE_PRINT(" %d%% charge\n\n", batt_pct);
}

void cmd_res()
{
  uint32_t last_resistance = raw_resistance;
  for (uint8_t i = 100; i > 0; i--)                     // Runs for 100 samples or until interrupted by input
  {
    if (raw_resistance != last_resistance)
    {
      BLE_PRINT("Raw ADC value %d\n", raw_resistance);
      BLE_PRINT("Resistance %.1f%%\n", resistance);
      BLE_PRINT("Keiser gear number %d\n\n", gear);
      last_resistance = raw_resistance;
    }
    if (bleuart.available())
    {
      bleuart.flush();
      break;
    }
    delay(TICK_INTERVAL);
  }
}

  void cmd_showcal()
  {
    BLE_PRINT("Offset %.1f; factor %.4f\n\n", res_offset, res_factor);
  }

  void cmd_factor()
  {
    if (arg[0] == 0) //Should check for numeric as well
    {
      RESPOND("Factor command requires a numeric value.\n\n");
      return;
    }
    new_factor = atof(arg);
    BLE_PRINT("New factor will be %.4f  .\n", new_factor);
    RESPOND("Use activate to have the bike use the new value.\n\n");
  }

  void cmd_offset()
  {
    if (arg[0] == 0) //Should check for numeric as well
    {
      RESPOND("Offset command requires a numeric value.\n\n");
      return;
    }
    new_offset = atof(arg);
    BLE_PRINT("New offset will be %.1f  .\n", new_offset);
    RESPOND("Use activate to have the bike use the new value.\n\n");
  }

  void cmd_defaults()
  {
    new_offset = RESISTANCE_OFFSET;
    new_factor = RESISTANCE_FACTOR;
    RESPOND("\nDefaults...\n");
    BLE_PRINT("New offset will be %.1f  .\n", new_offset);
    BLE_PRINT("New factor will be %.4f  .\n", new_factor);
    RESPOND("Use activate to have the bike use these values.\n\n");
  }

  uint8_t i;
  uint32_t base;
  uint32_t reading;

  void delay_message(uint8_t steps, uint16_t time)
  {
    for (i = steps; i > 0; i--)
    {
      BLE_PRINT(" %d..", i);
      delay(time);
    }
  }
  void cmd_cal()
  {
    if (AWAITING_CONF())
    {
      RESPOND("\n\nBe sure that the resistance lever is at the bottom (lowest gear).\n");
      RESPOND("Rotate the flywheel so that the tool contacts the magnet assembly.\n");
      delay_message(5, 1000);
      //base = analogRead(RESISTANCE_PIN);  // Baseline reading - should increase from here
      RESPOND("\nRotate the magnet assembly by hand so that the magnet settles into the pocket in the cal tool.\n");
      RESPOND("Do not use the lever!\n");
      delay_message(5, 1000);
      RESPOND("\nHold while readings are taken...\n");
      uint32_t cal_reading = 0;
      for (uint8_t i = 10; i > 0; i--)
      {
        reading = analogRead(RESISTANCE_PIN);
        BLE_PRINT("   %d \n", reading);
        cal_reading += reading;
        delay(200);
      }
      cal_reading /= 10;
      BLE_PRINT("Done. Average was %d.\n", cal_reading);
      new_offset = cal_reading - CALTOOL_RES/res_factor;
      BLE_PRINT("The new offset is %.1f.\n", new_offset);
      RESPOND("Use activate to have the bike use these values.\n\n");
      awaiting_conf = AWAITING_NONE;
  }
  else 
  {
      RESPOND("Enter the calibration procedure.\n");
      RESPOND("If for any reason you need to change the cal factor, enter and activate it *before* proceeding.\n");
      RESPOND("Have the calibration tool on the flywheel.\n");
      RESPOND("Y to continue...");
      awaiting_conf = cmd_number;
      awaiting_timer = CONFIRMATION_TIMEOUT ;

  }
}

void cmd_activate()
{
  if (AWAITING_CONF()) 
  {
      res_offset = new_offset;
      res_factor = new_factor;
      RESPOND("\nActivate factor and offset confirmed.\n");
      BLE_PRINT("Offset %.1f; factor %.4f\n\n", res_offset, res_factor);
      awaiting_conf = AWAITING_NONE;
  }
  else 
  {
      BLE_PRINT("Factor will be %.4f and offset will be %.1f .\n", new_factor, new_offset);
      RESPOND("Y to make these the active values...");
      awaiting_conf = cmd_number;
      awaiting_timer = CONFIRMATION_TIMEOUT ;
  }
}

void cmd_write()
{
  if (AWAITING_CONF()) 
  {
      RESPOND("\nWrite confirmed.\n");
      awaiting_conf = AWAITING_NONE;
  }
  else 
  {
      RESPOND("Currently active factor and offset are...\n");
      BLE_PRINT("Offset %.1f; factor %.4f\n", res_offset, res_factor);
      RESPOND("Y to write these to the file...");
      awaiting_conf = cmd_number;
      awaiting_timer = CONFIRMATION_TIMEOUT ;
  }
}

void cmd_read()
{
  RESPOND("Here, we'd read from the filesystem.\n");
}

void cmd_help()
{
    RESPOND("\n");
    for (int i = 0; i < n_cmds; i++)
    {
        BLE_PRINT("%s - %s \n", cmd_table[i].cmd, cmd_table[i].help);
        // RESPOND(cmd_table[i].cmd);
        // RESPOND(" - ");
        // RESPOND(cmd_table[i].help);
        // RESPOND("\n");
    }
}

void process_cmd()
{
  if (AWAITING_CONF())     // Awaiting confirmation - look for a "y" and call the same handler, or cancel
  {
      if (!strcmp(cmd, "y"))
      {
          cmd_table[awaiting_conf].cmdHandler();
      }
      else {
          RESPOND("Canceled.\n");
          awaiting_conf = AWAITING_NONE;
      }
      //awaiting_conf = AWAITING_NONE;   // Each handler needs to reset awaiting_conf
      return;
  }
  
  int i = n_cmds;
  while (i--)
  {
    if (!strcmp(cmd, cmd_table[i].cmd))
    {
      cmd_number = i;
      cmd_table[i].cmdHandler();
      return;
    }
  }
  RESPOND("Not a valid command.");
  cmd_help();
  return;
}

void bleuart_check(void *notUsed)
{
  uint32_t puNotificationValue;
  while (1)
  {
    xTaskNotifyWait(0x00, portMAX_DELAY, &puNotificationValue, DELAY_FOREVER); // Just sit here, blocked, until update() notifies us
    bleuart_take_input();  // Take in whatever's there
    if (new_input)
    {
      process_cmd();       
      console_clear();
      //RESPOND(" ");
    }
    else if (AWAITING_CONF())
    {
      RESPOND(".");
      if (--awaiting_timer == 0)
      {
        awaiting_conf = AWAITING_NONE;
        RESPOND("Cancelled.\n\n");
        console_clear();
      }
    }
  }
}

#endif

void LED_flash(int times, int ms)
{
  for (int i = 0; i < times; i++)
  {
    digitalWrite(LED_RED, 1);
    delay(ms);
    digitalWrite(LED_RED, 0);
    if (i < times - 1)
      delay(ms);
  }
}

void loop() // We're using a FreeRTOS scheduled task, so this is empty.
{
};

void update(TimerHandle_t xTimerID)
{
// Things that happens on every tick -------------------------------------------------------------------------------------
  update_resistance();

#ifdef USE_SERIAL
  //serial_check();  // Can't do this here in its current form because it can take longer than the timer interval
#endif

// Things happen at the default tick interval ----------------------------------------------------------------------------
  if ((ticker % DEFAULT_TICKS) == 0)
  {
    process_crank_event();

    cadence = inst_cadence;
    float inst_power = max(sinterp(gears, power90, slopes, gear, resistance) * (PC2 + PB2 * cadence + PA2 * cadence * cadence), 0);
    //float inst_power = max((PC1 + PB1 * resistance + PA1 * resistance_sq) * ( PC2 + PB2 * cadence + PA2 * cadence * cadence), 0);
    power = inst_power;

    updateBLE();
  }

#ifdef BLEUART
  // NOTE: It may be OK to just unblock. The if() is intended to avoid the overhead of the task notify
  if (bleuart.available() || AWAITING_CONF()) xTaskNotifyGive(ble_task_handle); // Unblock bleuart_check()
#endif

  // Things that happen on BATT_TICKS ------------------------------------------------------------------------------------
  if ((ticker % BATT_TICKS) == 0)
  {
    update_battery();
  }

  // Final things, as needed, on every tick ------------------------------------------------------------------------------
  if (display_state > 0)
    display_numbers();

  ticker++;
  //delay(TICK_INTERVAL);  // Not used when this routine is triggered by a timer.
}
