/********************************************************************
    Calibration
 ********************************************************************/

// Normalized resistance (0-100 scale) is ( ADC reading - OFFSET ) * FACTOR
// Normalized resistance will exceed 100% because there is magnet assembly excursion beyond the top of the range,
//    prior to the mechanical brake contacting the flywheel, and at the bottom of the range as well.

// These are default cals, used if the filesystem has nothing.
// These are from a regression of raw ADC values vs. % resistance on the Keiser power calibration graph.
// The factor shouldn't change as long as we're using Vdd as the reference.
// The offset changes from bike to bike due to how the magnet assembly engages with the sense pot.
#define RESISTANCE_OFFSET 176.8 
#define RESISTANCE_FACTOR 0.1993

// Calibration with the Keiser tool assumes that the normalized resistance is this value when the
// magnet is snug against the semicircular pocket in the tool. This corresponds to the bottom
// edge of gear 14 in Keiser's graph.
#define CALTOOL_RES 40

// Ratio of power to (power @ reference cadence) vs. RPM, as Ax^2 + Bx + C
#define PC2 -2.70E-03 // Power/Power_ref vs. RPM
#define PB2 6.79E-03
#define PA2 4.75E-05

// Speed vs. sqrt(power) to mimic Keiser's display
// See https://ihaque.org/posts/2020/12/25/pelomon-part-ib-computing-speed/ and https://github.com/ptx2/gymnasticon/pull/89
// Originals in those sources provide miles/h instead of the required Km/h
// Coefficient order: speed = sum( coeff[i] * power^i )
//const float SC_LT26[] = {  0.057, -0.172, 0.759, -0.079}; // for mph
//const float SC_GE26[] = { -1.635,  2.325, 0.064,  0.001};
const float SC_LT26[] = {  0.095, -0.287, 1.265, -0.132}; // for Km/h
const float SC_GE26[] = { -2.725,  3.875, 0.107,  0.0017};
#define SPEED_FIT_ORDER 3
#define SPEED_FIT_POWER_THRESHOLD 26.0

// Power vs. resistance at reference cadence (90 RPM) as Ax^2 + Bx + C
// This has been superceded by use of Keiser's published calibration curve.
// The following, which is pretty good from Keiser gears 2-20, was abandonded in favor of a table lookup//
//#define PC1 29.3      // Power_ref vs. R @ reference cadence (Ref cadence is 90 RPM)
//#define PB1 -1.97     // This fit is done for gears 2 to 22.
//#define PA1 0.135     // Covering the full range requires a 4th order polynomial or
                        // another, e.g., table lookup, approach.

// Keiser gear vs. normalized resistance, as Ax^2 + Bx + C
// This was abandoned in favor of the table lookup
//#define GC -2.97      // Keiser gear vs. normalized resistance
//#define GB 0.516
//#define GA -2.35E-03
