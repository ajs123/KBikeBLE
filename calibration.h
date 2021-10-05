/********************************************************************
    Calibration data
       Here, the cal is a direct fit of power vs. resistance setting @ constant speed
                    times a fit of power vs. speed at constant resistance setting
       Power = ( power@90rpm vs. R ) * ( power/power@90 vs. C )
            where R = normalized resistance (0-100 over full range of Keier's graph)
                  C = cadence (RPM)

       Gear is related to resistance by the R at the bottom of the range for each gear
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

// Power vs. resistance at reference cadence (90 RPM) as Ax^2 + Bx + C
// The following, which is pretty good from Keiser gears 2-20, was abandonded in favor of a table lookup//
//#define PC1 29.3      // Power_ref vs. R @ reference cadence (Ref cadence is 90 RPM)
//#define PB1 -1.97     // This fit is done for gears 2 to 22.
//#define PA1 0.135     // Covering the full range requires a 4th order polynomial or
                        // another, e.g., table lookup, approach.

// Ratio of power to (power @ reference cadence) vs. RPM, as Ax^2 + Bx + C
#define PC2 -2.70E-03 // Power/Power_ref vs. RPM
#define PB2 6.79E-03
#define PA2 4.75E-05

// Keiser gear vs. normalized resistance, as Ax^2 + Bx + C
// This was abandoned in favor of the table lookup
//#define GC -2.97      // Keiser gear vs. normalized resistance
//#define GB 0.516
//#define GA -2.35E-03