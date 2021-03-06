/********************************************************************************
   Globals
 ********************************************************************************/
#ifndef GLOBALS_
#define GLOBALS_

#include <Arduino.h>
#include <bluefruit.h> // nrf52 built-in bluetooth
#include "options.h"

// U8G2 display instance. What goes here depends upon the specific display and interface. See U8G2 examples.
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R1, /* reset=*/U8X8_PIN_NONE); // 128 x 64 SH1106 display on hardware I2C

// Globals. Variable accessed in multiple places are declared here.
// Those used only in specific functions are declared within or nearby.

float cadence;         // Pedal cadence, determined from crank event timing
float power;           // Power, calculated from resistance and cadence to match Keiser's estimates
float bspeed;          // Bike speed, required by FTMS, to be estimated from power and cadence. A real estimate is TO DO
float inst_resistance; // Normalized resistance reading, determined from the eddy current brake magnet position
float resistance;      // Normalized resistance used in power calculations (can be filtered)
float disp_resistance; // Resistance valued that's displayed (can be filtered)
uint32_t raw_resistance;  // Raw resistance measurement from the ADC. Global because it's reported in the serial monitor
uint8_t lever_state = 0b00000000; // Indicates gearshift near top (bit 0) or was on last check (bit 1)
float res_offset;      // Cal fators - raw_resistance to normalized resistance
float res_factor;
bool serial_present = false;
//float resistance_sq;             // Used in both gear and power calcs if not using the lookup table
uint8_t gear;          // Gear number: Index into power tables and, optionally, displayed
#if (POWERSAVE > 0) && (POWERSAVE != 1)
bool suspended;        // Set to true when suspending the main loop
#endif

#if HAVE_BATTERY
  float batt_mvolts;  // Battery voltage
  uint8_t batt_pct;   // Battery percentage charge
  bool batt_low;      // Battery low
  #if WATCH_VDD
    float Vdd_mvolts; // Vdd reading (from RESISTANCE_TOP pin)
  #endif
#endif

#define TICK_INTERVAL 500 
uint8_t ticker = 0; // Ticker for the main loop scheduler - inits to zero, also is reset under certain circumstances

volatile float inst_cadence = 0;        // Cadence calculated in the crank ISR
volatile uint16_t crank_count = 0;      // Cumulative crank rotations - set by the crank sensor ISR, reported by CPS and used to determine cadence
volatile uint32_t crank_event_time = 0; // Set to the most recent crank event time by the crank sensor ISR [ms]
volatile bool new_crank_event = false; // Set by the crank sensor ISR; cleared by the main loop
volatile uint16_t crank_ticks;         // 1/1024 sec per tick crank clock, for Cycling Power Measurement [ticks]

bool ftm_active = true; // Once a client connects with either service, we stop updating the other.
bool cp_active = true;

uint8_t display_state = 2; // Display state: 0 = off, 1 = dim, 2 = full on

bool gear_display = GEAR_DISPLAY;     // Determines whether the display shows the gear or the % resistance

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
#define BLEUART_MAX_MSG 20

// Child class of BLEUart to provide BLEUart with write(uint8_t *, size_t) 
// that behaves the same as the equivalent in Serial. Allows us to define
// console as a pointer to Stream that works with both BLEUart and Serial.
class cBLEUart : public BLEUart
{
  //using BLEUart::BLEUart;
public:
  size_t write(const uint8_t *content, size_t len);
};
  
// Piecewise unbuffered write of longer messages
// The parent unbuffered BLEUart::write() truncates messages longer than can 
// fit in one packet, while buffered writes work differently from Serial.
size_t cBLEUart::write(const uint8_t *content, size_t len) 
{
  const uint8_t * index = content;
  const uint8_t * last = content + len;  
  while (last - index >= BLEUART_MAX_MSG)
  {
    BLEUart::write(index, BLEUART_MAX_MSG); 
    index += BLEUART_MAX_MSG;
  }
  if (index < last)
  {
    BLEUart::write(index, last - index);
  }
  return len;
}
  
cBLEUart bleuart; //(int)BLEUART_MAX_MSG); 

#endif

#endif
