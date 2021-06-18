// Gear and power tables.

// Keiser gears are unevenly spaced. Some users may like them. Others may appreciate gears
// that are more uniformly spaced. A 2nd order polynomial fit to Keiser's gears provides this.
// Others will prefer resistance from 1-100 for the added precision.

// The power curves need 4th order polynomials to capture them through the full range.
// This would require 7 floating point multiplies (R^2, R^3, R^4 and four coefficients)

// Three 2nd order polynomials, covering low (1-3), mid (2-20), and high (18-24) would work as well.
// This requires 3 floating point multiplies (R^2 and two coefficients).

// A lookup table with interpolation captures the gear spacing (to the degree we care)
// and captures the power curve more efficiently.
// This lookup table requires a series of comparisons to determine where we are in the table, then
//    - one multiply and one divide if storing only the resistance and power tables
//    - just one multiply if storing resistance, power, and pre-computed slopes

// Here, the resistance corresponding to the bottom of each Keiser gear range is used for the 
// resistance table, so the index into that table corresponds to the current Keiser gear.
// If not wanting the option of displaying the "original" Keiser gear, Keiser's power vs. resistance
// table could be digitized as more uniform intervals. This would eliminate the 3 floating point 
// multiplies associated with display of a gear number.

// Note that the lookup table is fully bounded by zero and max values. 

/* This table for Keiser's original (unevenly spaced) gears
const int tablen = 24;
const float gears[tablen+1] = {  0.00,  7.51, 11.40, 14.59, 18.57, 20.12, 23.32, 24.87, 
                                27.03, 29.53, 31.87, 34.20, 36.70, 39.72, 42.83, 46.03, 
                                49.14, 53.11, 56.99, 60.97, 66.49, 72.02, 78.41, 89.38, 100.00 };
                                
const float power90[tablen+1] = { 20.9,  24.3,  27.7,  33.5,  40.4,  45.0,  54.3,  61.3,  
                                  71.7,  84.4,  98.3, 118.0, 136.6, 163.3, 193.4, 225.9, 
                                 261.9, 308.3, 360.5, 415.0, 497.4, 577.5, 671.5, 796.8, 878.0 };

const float pwr_slope[tablen+1] =   {    0,  0.446,  0.884,  1.805,  1.742,  2.976,  2.896,  4.471,
                                     4.830,  5.089,  5.965,  8.456,  7.408,  8.826,  9.701, 10.166,
                                    11.569, 11.683, 13.437, 13.730, 14.908, 14.488, 14.709, 11.425,  7.642 };
*/

/* This table for equivalent evenly-spaced gears */
const int tablen = 24;
const float gears[tablen+1] = { 0.00, 10.10, 12.25, 14.46, 16.72, 19.03, 21.41, 23.85,
                               26.36, 28.95, 31.63, 34.40, 37.28, 40.27, 43.41, 46.69,
                               50.16, 53.84, 57.78, 62.05, 66.74, 72.00, 78.13, 85.77, 100};

const float power90[tablen+1] = {  22.40,  25.23,  27.55,  30.84,  35.31,  41.16,  48.59,  57.86,
                                   69.20,  82.90,  99.25, 118.56, 141.20, 167.55, 198.05, 233.17,
                                  273.45, 319.52, 372.07, 431.91, 499.98, 577.34, 665.11, 763.85, 877.84 };

const float slopes[tablen+1] = {  0.00,  0.28,  1.07,  1.49,  1.98,  2.52,  3.13,  3.80,
                                  4.52,  5.29,  6.11,  6.97,  7.87,  8.79,  9.74, 10.69,
                                 11.62, 12.51, 13.33, 14.03, 14.52, 14.70, 14.32, 12.92, 8.01 };
