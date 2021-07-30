/* Bluetooth console for Keiser M3 **********************************
    V0.17
*********************************************************************/
#define VERSION "0.17"

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
#include "globals.h"
#include "bike_interface.h"     // Defines the hardware connections to the bike
#include "BLE_services.h"       // Flags/fields for the BLE services
#include "power_gear_tables.h"
#include "calibration.h"
#include "serial_commands.h"
#include "param_store.h"
//#include "KB_analog.h"

/**********************************************************************
 * Optional functions and debugging
 **********************************************************************/

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
  Serial.print(F(x));  \
  Serial.print(F(l));
#else
#define DEBUG(x, l)
#endif

#ifdef DEBUGGING // Pauses at points in startup can be helpful in checking power consumption
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

/*********************************************************************************
  Display code
**********************************************************************************/

#define DISPLAY_LABEL_FONT u8g2_font_helvB10_tr
#define DISPLAY_NUMBER_FONT u8g2_font_helvB24_tn
// Log used at startup
#define LOG_FONT u8g2_font_7x14_tf
#define LOG_FONT_BIG u8g2_font_7x14_tf
#define LOG_WIDTH 9 
#define LOG_HEIGHT 10 
uint8_t u8log_buffer[LOG_WIDTH * LOG_HEIGHT];
U8G2LOG displog;

void display_setup()
{
  display.begin();
  display.setContrast(CONTRAST_FULL);
  //display.enableUTF8Print();  // Can leave this out if using no symbols
  display.setFontMode(1); // "Transparent": Character background not drawn (since we clear the display anyway)
  
  displog.begin(display, LOG_WIDTH, LOG_HEIGHT, u8log_buffer);
  displog.setLineHeightOffset(0);
  displog.setRedrawMode(0);
}

void right_just(uint16_t number, int x, int y, int width)
// This works for the current font, but it ought to count pixels rather than characters.
{
  if (number < 10)
    x += width;
  if (number < 100)
    x += width;
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
    display.setFont(DISPLAY_LABEL_FONT);
    display.setCursor(0, 16);
    display.print("RPM");
    display.setCursor(0, 58);
    display.print(rg_label);
    display.setCursor(0, 100);
    display.print("WATTS");

    if (!batt_low || (ticker % 2)) draw_batt(batt_pct);
    //else blank_batt();

    display.setFont(DISPLAY_NUMBER_FONT);
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
//#define analogReference KB_analog::analogReference
//#define analogOversampling KB_analog::analogOversampling
//#define analogSampleTime KB_analog::analogSampleTime
//#define analogReadResolution KB_analog::analogReadResolution
//#define analogRead KB_analog::analogRead

eAnalogReference analog_reference = AR_INTERNAL;

void ADC_calibrate_offset() // Periodic calibration of the ADC
{
  digitalWrite(PIN_LED1, HIGH);
  NRF_SAADC->EVENTS_CALIBRATEDONE = 0;
  NRF_SAADC->TASKS_CALIBRATEOFFSET = SAADC_TASKS_CALIBRATEOFFSET_TASKS_CALIBRATEOFFSET_Trigger;
  delay(1);
  //while (!NRF_SAADC->EVENTS_CALIBRATEDONE) delay(1); // This should work
  if (!NRF_SAADC->EVENTS_CALIBRATEDONE) delay(1000);
  NRF_SAADC->EVENTS_CALIBRATEDONE = 0;
  digitalWrite(PIN_LED1, LOW);
}

void ADC_setup() // Set up the ADC for ongoing resistance measurement
{
  analogReference(analog_reference); // 3.6V
  ADC_calibrate_offset();
  //analogReference(AR_VDD4); // VDD = nominal 3.3V - Resistance readings will be proportional to high side voltage
  analogOversampling(ANALOG_OVERSAMPLE);
  #ifdef SAADC_TACQ
    analogSampleTime(20);
  #endif
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

void startAdv()
{
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

void stopBLE()
{
  //DEBUG("Stopping BLE", "\n")
  // If we're connected, disconnect. Since we only allow one connection, we know that the handle is 0
  // if (Bluefruit.connected(0)) Bluefruit.disconnect(0);
  Bluefruit.disconnect(0); // disconnect() includes check for whether connected
  delay(100);              // If restartOnDisconnect(true), this is needed for advertising to be stopped

  // If we're advertising, stop advertising.
  // if (Bluefruit.Advertising.isRunning()) Bluefruit.Advertising.stop(); // Or should this be time-limited?
  Bluefruit.Advertising.stop(); // this looks like it will work even if advertising isn't running
}

void restartBLE()
{
  if (!Bluefruit.connected(0) && !Bluefruit.Advertising.isRunning())
  {
    //DEBUG("Restarting BLE", "\n")
    Bluefruit.Advertising.start(0);
  }
}

void setupFTMS()
{
  // Configure and start the FTM service
  // First .begin the service, prior to .begin on characteristics
  svc_ftms.begin();

  // Note: You must call .begin() on the BLEService before calling .begin() on
  // any characteristic(s) within that service definition.. Calling .begin() on
  // a BLECharacteristic will cause it to be added to the last BLEService that
  // was 'begin()'ed!

  char_ftm_feature.setProperties(CHR_PROPS_READ);
  char_ftm_feature.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  char_ftm_feature.setFixedLen(sizeof(ftmf_data));
  // Should the following callback be set for ftm_feature or for bike_data??
  char_ftm_feature.setCccdWriteCallback(ftm_cccd_callback); // Optionally capture CCCD updates

  char_ftm_feature.begin();

  char_ftm_feature.write(ftmf_data, sizeof(ftmf_data));

  char_bike_data.setProperties(CHR_PROPS_NOTIFY);
  char_bike_data.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  char_bike_data.setFixedLen(sizeof(bike_data));

  char_bike_data.begin();

}

void setupCPS()
{
  // Configure and start the CP service
  // First .begin the service, prior to .begin on characteristics
  svc_cps.begin();

  char_cp_feature.setProperties(CHR_PROPS_READ);
  char_cp_feature.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  char_cp_feature.setFixedLen(sizeof(cpf_data));
  char_cp_feature.begin();
  char_cp_feature.write(cpf_data, sizeof(cpf_data));

  char_sensor_loc.setProperties(CHR_PROPS_READ);
  char_sensor_loc.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  char_sensor_loc.setFixedLen(sizeof(cl_data));
  char_sensor_loc.begin();
  char_sensor_loc.write(cl_data, sizeof(cl_data));

  char_cp_measurement.setProperties(CHR_PROPS_NOTIFY);
  char_cp_measurement.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  char_cp_measurement.setFixedLen(sizeof(power_data));
  char_cp_measurement.setCccdWriteCallback(cps_cccd_callback); // Optionally capture CCCD updates
  char_cp_measurement.begin();

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

void format_bike_data()
{
  //    B2-3    = Instantaneous speed   - UINT16 - Km/Hr with 0.01 resolution
  //    B4-5    = Instantaneous cadence - UINT16 - 1/min with 0.5 resolution
  //    B6-7    = Instantaneous power   - UINT16 - W with 1.0 resolution

  uint16_t speed_int = roundpos(bspeed / 0.01);
  uint16_t cadence_int = roundpos(cadence / 0.5);
  uint16_t power_int = roundpos(power);
  uint16_t resistance_int = roundpos(resistance);

  bike_data.data[0] = speed_int;
  bike_data.data[1] = cadence_int;
  bike_data.data[2] = resistance_int;
  bike_data.data[3] = power_int;
}

void format_power_data()
{
  // Fields are
  //   B2-3     = Instantaneous power - UINT16 - W with 1.0 resolution
  //   B4-5     = Crank revolutions   - UINT16 - [unitless]
  //   B6-7     = Last crank event time - UINT16 - s with 1/1024 resolution (1024 counts/sec)

  //uint16_t power_int = round(power);
  //uint16_t revs = crank_count;
  //uint16_t et = ((crank_event_time & 0xFFFF) * 1024 / 1000) & 0xFFFF ; // uint32 millisec to uint16 1024ths

  power_data.data[0] = roundpos(power);
  noInterrupts(); // crank_count and crank_ticks are set in the crank ISR
  power_data.data[1] = crank_count;
  power_data.data[2] = crank_ticks;
  interrupts();
}

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
  display.setFont(LOG_FONT_BIG);
  displog.print(F("KBikeBLE\n"));
  displog.println(F(VERSION));
  displog.println();
  display.setFont(LOG_FONT);

  if (!read_param_file("offset", &res_offset, sizeof(res_offset)))
  {
    res_offset = RESISTANCE_OFFSET;
    write_param_file("offset", &res_offset, sizeof(res_offset));
    displog.println(F("OFFSET"));
    displog.println(F("DEFAULT"));
    displog.println(F("WRITE:"));
    displog.println(res_offset);
  }
  else
  {
    displog.println(F("OFFSET")); 
    displog.println(F(" READ:"));
    displog.println(res_offset);
  }
  if (!read_param_file("factor", &res_factor, sizeof(res_factor)))
  {
    res_factor = RESISTANCE_FACTOR;
    write_param_file("factor", &res_factor, sizeof(res_factor));
    displog.println(F("\nFACTOR"));
    displog.println(F("DEFAULT"));
    displog.println(F("WRITE:"));
    displog.printf("%.1f\n", res_factor);
  }
  else
  {
    displog.println(F("\nFACTOR"));
    displog.println(F(" READ:"));
    displog.printf("%.4f\n", res_factor);
  }

  delay(5000);
}

/********************************************************************************
 * Startup
 ********************************************************************************/

SoftwareTimer update_timer;

void setup()
{
#ifdef USE_SERIAL
  Serial.begin(115200);
#endif

  // Start the OLED display
  display_setup(); 

  // Set up the internal filesystem (LittleFS), read files, and where any files are missing write defaults
  setup_InternalFS();
  init_cal();

  // Set up Bluetooth
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
  #ifdef BLEBAS
    blebas.begin();
  #endif

  // Setup the FiTness Machine and Cycling Power services
  setupFTMS();
  setupCPS();

  // Start the BLEUart service
  #ifdef BLEUART
    bleuart.begin();
  #endif

  // Start the console
  #if defined(BLEUART) || defined(USE_SERIAL)
    console_init();
  #endif

  // Setup the advertising packet(s)
  startAdv();

  // Crank sensing. A falling edge (switch closure) triggers the interrupt. This counts as a crank event (rotation)
  // if it's been long enough since the last event. The rider could conceivably initiate faux rotations by holding
  // the crank right at a certain spot, but there are similar risks with any scheme.
  crank_event_time = xTaskGetTickCount(); // xTaskGetTickCountFromISR();
  pinMode(CRANK_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CRANK_PIN), crank_callback, FALLING);

  // Apply voltage to the resistance pot
  pinMode(RESISTANCE_TOP, OUTPUT);
  digitalWrite(RESISTANCE_TOP, HIGH);

  // Set up the analog input for resistance measurement
  ADC_setup();

  // Set both notify characteristics to be active. Whichever the client responds to becomes the sole active characteristic
  ftm_active = true;
  cp_active = true;

  // With crank sensing handled by the interrupt, everything else - 
  //   - Bluetooth updates
  //   - Display updates
  //   - Battery check
  //   - Power savings when pedaling stops
  //   - Notifying the BLEUart console to check for input
  // is handled by the recurring update() task.
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
Power down until reset. Reset will be caused by a selected level (not edge) on the crank switch GPIO pin.
The crank switch closes at one point in the rotation and is likely to be open. We can't be certain,
so we configure reset to occur when the crank switch changes to the opposite of its current state.

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
  If full = true, also de-energize the resistance sense pot
  Resume by re-starting it just as in Setup().
*/
void suspend(bool full = true)
{
  if (full) digitalWrite(RESISTANCE_TOP, LOW); // De-energize the resistance pot
  update_timer.stop();
  suspended = true;
}

void resume()
{
  digitalWrite(RESISTANCE_TOP, HIGH);
  ADC_calibrate_offset();
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
   at the appropriate times. update() is scheduled as a recurring task, twice per second,
   in FreeRTOS (part of the Adafruit Arduino core for the nRF52840. In the extended absence of 
   pedaling, one option for power savings is to simply un-schedule this task, leaving nothing 
   to do until pedaling resumes, and allowing the system to default into a low-power state.

   The BLE console interface is handled by a low-priority process that sits idle until
   triggered (the FreeRTOS term is Notify). update() does this once per second.
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
  //raw_pot_top = analogRead(RESISTANCE_TOP);
  //raw_pot_wiper = analogRead(RESISTANCE_PIN);
  //raw_resistance = raw_pot_wiper * ( ( (3300 * 1024) + 1800) / 3600 )  / raw_pot_top;  // (3.3 V Vdd)/(3.6 V Vref)*(ADC counts), rounded
  inst_resistance = max((raw_resistance - res_offset) * res_factor, 0);
  resistance = (RESISTANCE_FILTER * resistance + 2 * inst_resistance) / (RESISTANCE_FILTER + 2);
  gear = gear_lookup(resistance);
  disp_resistance = (RESISTANCE_DISPLAY_FILTER * resistance + 2 * inst_resistance) / (RESISTANCE_DISPLAY_FILTER + 2);
  //resistance_sq = resistance * resistance;
  //gear = max(floor(GC + GB * resistance + GA * resistance_sq), 1) ;
  lever_check();
}

void update_battery()
{
  
  if (analog_reference != AR_INTERNAL)
  {
    analogReference(AR_INTERNAL);
    delay(1);
  }
  batt_mvolts = analogRead(BATTERY_PIN) * VBAT_MV_PER_LSB;
  if (analog_reference != AR_INTERNAL)
  {
    analogReference(analog_reference);
    delay(1);
  }

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

    restartBLE(); // Be sure BLE is running (RestartBLE checks whether it already is.
                  // there should possibly be a state flag.

    new_crank_event = false; // Reset the flag
  }
  else // If no crank event, check timeouts
  {
    crank_still_timer = crank_still_timer >> 1;
    if (crank_still_timer == 0)
    {
      inst_cadence = 0;
      stop_time++;

      // Timeouts related to the display
      switch (display_state)
      {
      case 2:                     // Full brightness - see whether it's time to dim
        if (stop_time > DIM_TIME)
        {
          display.setContrast(CONTRAST_DIM);
          display_state = 1;
        }
        break;
      case 1:                     // Dimmed - see whether it's time to turn off
        if (stop_time > BLANK_TIME)
        {
          display.setPowerSave(1);
          display_state = 0;
        }
        break;
      }

      // Timeouts related to further power savings - different according to Bluetooth state
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
      char_cp_measurement.notify((uint8_t *)&power_data, 8);     // The cast should not be needed
    }

    if (ftm_active)
    {
      bspeed = 20 * cadence / 60; // NOT A REAL CAL OF ANY KIND!
      format_bike_data();
      char_bike_data.notify((uint8_t *)&bike_data, 10);
    }
  }
}

#if defined(BLEUART) || defined(USE_SERIAL)
#define SBUF_LEN 40  
char sbuffer[SBUF_LEN];                 // Serial console input buffer
char cmd[SBUF_LEN];                     // Tokenized command
char arg[SBUF_LEN];                     // Tokenized argument
uint8_t sbix;           // Current index into input buffer
bool cmd_rcvd;          // Console state: command received
bool new_input;         // Console state: new input ready to process
uint8_t cmd_number;             // Current command (= position in the table)
uint8_t awaiting_conf = AWAITING_NONE;      // Console state: awaiting confirmation for this numbered command
uint8_t awaiting_timer;  // Timeout counter for confirmation
float new_factor;
float new_offset;
  
TaskHandle_t console_task_handle;

Stream * console = nullptr;

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
  #ifdef USE_SERIAL    // These shouldn't be needed
    console = &Serial;
  #endif
  #ifdef BLEUART
    console = &bleuart;
  #endif
  bool result = xTaskCreate(console_check, "ConsoleCheck", SCHEDULER_STACK_SIZE_DFLT, 
                            ( void * ) 1, TASK_PRIO_LOW, &console_task_handle);
  console_clear();
  console_reset();
}

bool console_source_check()
{
  bool avail = false;

  #ifdef USE_SERIAL                 // If enabled, check Serial
    bool s = Serial.available();
    if (s) 
    {
      console = &Serial;
      avail = true;
    }
  #endif

  #ifdef BLEUART                    // If enabled, check BLEUart
  bool b = bleuart.available();
  if (b)
  {
    console = &bleuart;
    avail = true;
    #ifdef USE_SERIAL
      if (s)                         // BLE has precedence, so dump any serial input
        while (Serial.available()) Serial.read();   
    #endif
  }
  #endif
  
  return avail;
}

void console_take_input()
{
  while (console->available())  // Might need && !new_input and flush at the end to be sure to get only one line
    {
      uint8_t c = (uint8_t) console->read();
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
        break;
      }
    }
}

// void console_send(const char * message)
// {
//   console->write((uint8_t*) message, strlen(message));
// } 

#define PBLEN 128
char pbuffer[PBLEN];
#define CONSOLE_RESP(string) console->write((uint8_t *) string, strlen(string)) //console_send(string)
#define CONSOLE_PRINT(format, args...) snprintf(pbuffer, PBLEN, format, args); \
                                       console->write((uint8_t *) pbuffer, strlen(pbuffer)) //console_send(pbuffer)

void cmd_batt()
{
  CONSOLE_PRINT("Battery voltage %.2f \n", batt_mvolts/1000);//batt_mvolts*1000);
  CONSOLE_PRINT(" %d%% charge\n\n", batt_pct);
}

void cmd_res()
{
  uint32_t last_resistance = raw_resistance;
  uint32_t sum_resistance = 0;
  uint8_t n_resistance = 0;
  uint8_t max_n = 100; // Default

  double mean = 0;
  double M2 = 0;
  double variance = 0;

  if (arg[0] != 0)
  {
    max_n = min(0xFF, max(atoi(arg), 1));
  }
  for (uint8_t i = max_n; i > 0; i--)                     // Runs for 100 samples or until interrupted by input
  {
    sum_resistance += raw_resistance;
    n_resistance++;
    if (true) //raw_resistance != last_resistance)
    {
      CONSOLE_PRINT("Raw ADC value %d\n", raw_resistance);
      CONSOLE_PRINT("Resistance %.1f%%\n", resistance);
      CONSOLE_PRINT("Keiser gear number %d\n\n", gear);
      last_resistance = raw_resistance;

      double delta = raw_resistance - mean;
      mean += delta / n_resistance;
      M2 += delta * (raw_resistance - mean);
      variance = M2 / n_resistance;
    }
    if (console->available())
    {
      while (console->available()) console->read();  //NOTE: bleuart.flush() flushes input; Serial.flush() flushes output
      break;
    }
    delay(TICK_INTERVAL);
  }
  if (n_resistance > 0) 
  {
    CONSOLE_PRINT("%d measurements.\n", n_resistance);
    CONSOLE_PRINT("Average ADC value %.1f\n", (float)sum_resistance / n_resistance);
    CONSOLE_PRINT("Estimated mean %.1f; SD %.1f . \n\n", mean, sqrt(variance));
  }
}

void cmd_adcref()
{
  if (analog_reference == AR_INTERNAL)
  {
    analog_reference = AR_VDD4;
    res_offset *= (3.6 / 3.3);
    res_factor /= (3.6 / 3.3);
    CONSOLE_RESP("ADC reference is now Vdd.\n\n");
  }
  else
  {
    analog_reference = AR_INTERNAL;
    res_offset *= (3.3 / 3.6);
    res_factor /= (3.3 / 3.6);
    CONSOLE_RESP("ADC reference is now internal 3.6V.\n\n");
  }
  analogReference(analog_reference);
}

void cmd_adccal()
{
  CONSOLE_RESP("Doing ADC calibration...");
  ADC_calibrate_offset();
  CONSOLE_RESP("Done.\n\n");
}

  void cmd_showcal()
  {
    CONSOLE_PRINT("Offset %.1f; factor %.4f\n\n", res_offset, res_factor);
  }

  void cmd_factor()
  {
    if (arg[0] == 0) //Should check for numeric as well
    {
      CONSOLE_RESP("Factor command requires a numeric value.\n\n");
      return;
    }
    new_factor = atof(arg);
    CONSOLE_PRINT("New factor will be %.4f  .\n", new_factor);
    CONSOLE_RESP("Use activate to have the bike use the new value.\n\n");
  }

  void cmd_offset()
  {
    if (arg[0] == 0) //Should check for numeric as well
    {
      CONSOLE_RESP("Offset command requires a numeric value.\n\n");
      return;
    }
    new_offset = atof(arg);
    CONSOLE_PRINT("New offset will be %.1f  .\n", new_offset);
    CONSOLE_RESP("Use activate to have the bike use the new value.\n\n");
  }

  void cmd_defaults()
  {
    new_offset = RESISTANCE_OFFSET;
    new_factor = RESISTANCE_FACTOR;
    CONSOLE_RESP("Defaults...\n");
    CONSOLE_PRINT("New offset will be %.1f  .\n", new_offset);
    CONSOLE_PRINT("New factor will be %.4f  .\n", new_factor);
    CONSOLE_RESP("Use activate to have the bike use these values.\n\n");
  }

  uint8_t i;
  uint32_t base;
  uint32_t reading;

  void delay_message(uint8_t steps, uint16_t time)
  {
    for (i = steps; i > 0; i--)
    {
      CONSOLE_PRINT(" %d..", i);
      delay(time);
    }
  }

  void cmd_cal()
  {
    if (AWAITING_CONF())
    {
      CONSOLE_RESP("\n\nPlace the calibration tool on the flywheel and rotate the flywheel so that the calibration tool contacts the magnet assembly.\n");
      delay_message(5, 1000);
      //base = analogRead(RESISTANCE_PIN);  // Baseline reading - should increase from here
      CONSOLE_RESP("\nRotate the magnet assembly by hand so that the magnet settles into the pocket in the calibration tool.");
      CONSOLE_RESP(" Do not use the lever!\n");
      delay_message(5, 1000);
      CONSOLE_RESP("\n\nHold while readings are taken...\n");

      suspend(false);  // Suspend updates so that resistance updates don't interfere, but leave the sense pot energized
      uint32_t cal_reading = 0;
      for (uint8_t i = 10; i > 0; i--)
      {
        reading = analogRead(RESISTANCE_PIN);
        CONSOLE_PRINT("   %d \n", reading);
        cal_reading += reading;
        delay(200);
      }
      resume();       // Resume updates

      cal_reading /= 10;
      CONSOLE_PRINT("Done. Average was %d.\n", cal_reading);
      new_offset = cal_reading - CALTOOL_RES/res_factor;
      CONSOLE_PRINT("\nThe new offset is %.1f.\n", new_offset);
      CONSOLE_RESP("\nUse activate to have the bike use these values.");
      CONSOLE_RESP("If the readings weren't consistent, you can try again.\n\n");
      awaiting_conf = AWAITING_NONE;
  }
  else 
  {
      CONSOLE_RESP("Enter the calibration procedure.\n");
      CONSOLE_RESP("This will set the offset to match the bike.\n");
      CONSOLE_RESP("If for any reason you need to change the factor, enter and activate it *before* proceeding.\n");
      CONSOLE_RESP("\nBe sure that the resistance lever is at the bottom (lowest gear)");
      CONSOLE_RESP(" and have the calibration tool ready to place on the flywheel.\n");
      CONSOLE_RESP("Y to continue...");
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
      CONSOLE_RESP("\nActivate factor and offset confirmed.\n\n");
      CONSOLE_PRINT("Offset %.1f; factor %.4f\n\n", res_offset, res_factor);
      awaiting_conf = AWAITING_NONE;
  }
  else 
  {
      CONSOLE_PRINT("Factor will be %.4f and offset will be %.1f .\n", new_factor, new_offset);
      CONSOLE_RESP("Y to make these the active values...");
      awaiting_conf = cmd_number;
      awaiting_timer = CONFIRMATION_TIMEOUT ;
  }
}

void cmd_write()
{
  if (AWAITING_CONF()) 
  {
      if (write_param_file("offset", &res_offset, sizeof(res_offset)) &
          write_param_file("factor", &res_factor, sizeof(res_factor)) )
          {
            CONSOLE_RESP("\nWrite confirmed.\n\n");
          }
          else
          {
            CONSOLE_RESP("\nFailed to write the files.\n\n");
          }
      awaiting_conf = AWAITING_NONE;
  }
  else 
  {
      CONSOLE_RESP("Currently active factor and offset are...\n");
      CONSOLE_PRINT("Offset %.1f; factor %.4f\n", res_offset, res_factor);
      CONSOLE_RESP("Y to write these to the file...");
      awaiting_conf = cmd_number;
      awaiting_timer = CONFIRMATION_TIMEOUT ;
  }
}

void cmd_read()
{
  if (read_param_file("offset", &new_offset, sizeof(new_offset)) &
      read_param_file("factor", &new_factor, sizeof(new_factor)))
    {
      CONSOLE_RESP("Read from the parameter files:\n");
      CONSOLE_PRINT("Offset %.1f; factor %.4f\n\n", new_offset, new_factor);
      CONSOLE_RESP("Use activate to have the bike use these values.\n\n");
    }
}

void cmd_comment()
{
  CONSOLE_PRINT("%s\n\n", arg);
}

void cmd_help()
{
    CONSOLE_RESP("\n");
    for (int i = 0; i < n_cmds; i++)
    {
        CONSOLE_PRINT("%s - %s \n", cmd_table[i].cmd, cmd_table[i].help);
    }
    CONSOLE_RESP("\n");
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
          CONSOLE_RESP("Canceled.\n");
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
  CONSOLE_RESP("Not a valid command.");
  cmd_help();
  return;
}

void console_check(void *notUsed)
{
  uint32_t puNotificationValue;
  while (1)
  {
    xTaskNotifyWait(0x00, portMAX_DELAY, &puNotificationValue, DELAY_FOREVER); // Just sit here, blocked, until update() notifies us
    console_take_input();  // Take in whatever's there
    if (new_input)
    {
      process_cmd();       
      console_clear();
      //RESPOND(" ");
    }
    else if (AWAITING_CONF())
    {
      CONSOLE_RESP(".");
      if (--awaiting_timer == 0)
      {
        awaiting_conf = AWAITING_NONE;
        CONSOLE_RESP("Cancelled.\n\n");
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

#if defined(BLEUART) || defined(USE_SERIAL)
  // If there's console input, select the corresponding source and unblock the handler
  // Note the order of operations: Don't switch sources while awaiting confirmation.
  if (AWAITING_CONF() || console_source_check()) xTaskNotifyGive(console_task_handle); 
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
