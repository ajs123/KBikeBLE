// BLE services

#ifndef BLE_SERVICES_
#define BLE_SERVICES_

/* Cycling Power Service (CPS) ---------------------------------------------------------------------------------
    Supported Characteristics:
    Name                         UUID    Requirement Properties
    ---------------------------- ------  ----------- ----------
    Cycling Power Feature        0x2A65  Mandatory   Read
    Cycling Power Measurement    0x2A63  Mandatory   Notify
    Sensor Location              0x2A5D  Mandatory   Read 
*/

  /* Cycling Power Feature characteristic
     See https://github.com/sputnikdev/bluetooth-gatt-parser/blob/master/src/main/resources/gatt/characteristic/org.bluetooth.characteristic.cycling_power_feature.xml
     Properties = Read
     Fixed len = 4

    B0
    .7  - Accumulated energy supported              - 0
    .6  - Top and bottom dead spot angles supported - 0
    .5  - Extreme angles supported                  - 0
    .4  - Extreme magnitudes supported              - 0
    .3  - Crank revolution data supported           - 0
    .2  - Wheel revolution data supported           - 0
    .1  - Accumulated torque supported              - 0
    .0  - Pedal power balance supported             - 0
    B1
    .7  - Span Length Adjustment Supported          - 0
    .6  - Chain Weight Adjustment Supported         - 0
    .5  - Chain Length Adjustment Supported         - 0
    .4  - Crank Length Adjustment Supported         - 0
    .3  - Multiple Sensor Locations Supported       - 0
    .2  - Cyc Pow Meas Char Cont Masking Supported  - 0
    .1  - Offset Compensation Supported             - 0
    .0  - Offset Compensation Indicator Supported   - 0
    B2
    .7  - Reserved
    .6  - Reserved
    .4-5 - Distribute System Support                 - 0
    .4-5 - [00 Legacy; 01 No; 02 Yes; 03 RFU         - 0
    .3  - Enhanced Offset Compensation Supported    - 0
    .2  - Factory Calibration Date Supported        - 0
    .1  - Instantaneous Measu Direction Supported   - 0
    .0  - Sensor Meas Context (0 Force, 1 Torque)   - 0
    B3
    .0-7 - Reserved 
    */

  const uint8_t cpf_data[4] = {0x0, 0x0, 0x0, 0x0};

    /* Sensor Location characteristic
       See https://github.com/sputnikdev/bluetooth-gatt-parser/blob/master/src/main/resources/gatt/characteristic/org.bluetooth.characteristic.sensor_location.xml
       Properties = Read
       Fixed len = 4
       0x01, 0x00, 0x00, 0x00 = "Other"
    */

  const uint8_t cl_data[4] = {0x01, 0x00, 0x00, 0x00};

/* Cycle Power Measurement characteristic
   See https://github.com/sputnikdev/bluetooth-gatt-parser/blob/master/src/main/resources/gatt/characteristic/org.bluetooth.characteristic.cycling_power_measurement.xml
   Properties = Notify
   Fixed len = 8 // Flags[2], InstPower[2], CrankRev[2], Timestamp[2]
   Flags = { 0x20, 0x00 } = Crank Revolution Data present
 */

//  #define CPM_DATA_0 0x20
//  #define CPM_DATA_1 0x00
/* The union (or struct) holding the transmitted data should probably be initialized here...*/
struct power_data_t
{
  uint8_t flags[2] {0x20, 0x00};  // These don't change
  uint16_t data[3];               // These are loaded with the data
} power_data;


/* FiTness Machine Service (FTMS) -----------------------------------------------------------------------------
   Supported Characteristics:
   Name                         UUID    Requirement Properties
   ---------------------------- ------  ----------- ----------
   Fitness Machine Feature      0x2ACC  Mandatory   Read
   Indoor Bike Data             0x2A38  Mandatory   Notify
*/

  /* FiTness Machine Service requires a Service Data field specifying bike support and availability
     Per https://www.bluetooth.com/specifications/assigned-numbers/generic-access-profile/) Service Data is Type 0x16
     Contents are
       B0-1 - Fitness Machine Service UUID - UINT16 - 2 bytes  = 0x1826
       B2   - Flags                        - UINT8  - 1 byte   = 0x01   (Machine available)
       B3-4 - Fitness Machine Type         - UINT16 - 2 bytes  = 0x0020 (Indoor bike supported (bit 5))
  */

  const uint8_t FTMS_Adv_Data[5] = {0x26, 0x18, 0x01, 0x20, 0x00};

  /* Fitness Machine Feature characteristic
    See:
    Properties = Read
    Fixed Len  = 4
        B0      = UINT8  - Flag (MANDATORY)
          7     = 0 - Resistance level supported
          6     = 0 - Step count supported
          5     = 0 - Pace supported
          4     = 0 - Elevation gain supported
          3     = 0 - Inclination supported
          2     = 0 - Total distance supported
          1     = 1 - Cadence supported
          0     = 0 - Average speed supported
        B1      = UINT8
          7     = 0 - Force on belt and power output supported
          6     = 1 - Power measurement supported
          5     = 0 - Remaining time supported
          4     = 0 - Elapsed time supported
          3     = 0 - Metabolic equivalent supported
          2     = 0 - Heart rate measurement supported
          1     = 0 - Expended energy supported
          0     = 0 - Stride count supported
        B2      = UINT8
        2-7     = 0 - RESERVED
          0     = 0 - User data retention supported
        B3      = UINT8 - RESERVED 
  */

  const uint8_t ftmf_data[4] = {0b00000010, 0b01000000, 0b00000000, 0b00000000};

  /* Indoor Bike Data characteristic - See 4.9.1 IN  FTMS_V1.0.pdf
    See:
    Properties = Notify
    Fixed Len  = 8
        B0    = UINT8 - Flag (MANDATORY)
        7     = 0 - Average power present
        6     = 1 - Instantaneous power present
        5     = 1 - Resistance level present
        4     = 0 - Total distance present
        3     = 0 - Average cadence present
        2     = 1 - Instantaneous cadence present
        1     = 0 - Average speed present
        0     = 0 - More Data (Instantaneous speed field not present)
        B1    = UINT 8
        4     = 0 - Remaining time present
        3     = 0 - Elapsed time present
        2     = 0 - Metabolic equivalent present
        1     = 0 - Heart rate present
        0     = 0 - Expended energy present
        B2-3  = Instantaneous speed   - UINT16 - Km/Hr with 0.01 resolution
        B4-5  = Instantaneous cadence - UINT16 - 1/min with 0.5 resolution
        B6-7  = Instantaneous power   - UINT16 - W with 1.0 resolution */

struct bike_data_t
{
  uint8_t flags[2] {0b01100100, 0b00000000};  // These don't change
  uint16_t data[4];                           // These are loaded with the data
} bike_data;

#endif