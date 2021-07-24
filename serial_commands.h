// The parser - calls the appropriate function; leaves argument parsing to that function

#ifndef SERIAL_COMMANDS_
#define SERIAL_COMMANDS_

typedef void cmdHandler_t();  // All handlers take nothing and return nothing

struct cmd_table_t {
  const char* cmd;              // command
  cmdHandler_t* cmdHandler;     // handler function
  const char* help;             // help text
} ;

// Define the command set
extern cmdHandler_t cmd_batt, cmd_res, cmd_showcal, cmd_factor, cmd_offset, cmd_cal, cmd_activate, cmd_write, cmd_read, cmd_defaults, cmd_help;

const cmd_table_t PROGMEM cmd_table[] = {
  {"batt", cmd_batt, "Show battery status."},
  {"res", cmd_res, "Monitor ADC readings. Any input to stop."},
  {"showcal", cmd_showcal, "Show the calibration currently in use."},
  {"factor", cmd_factor, "<float> Enter a new cal factor (should not be necessary)."},
  {"offset", cmd_offset, "<float> Enter a new calibration offset (follow with activate)."},
  {"calibrate", cmd_cal, "Start tool-based calibration. Requires the Keiser cal tool."},
  {"activate", cmd_activate, "Make the new calibration values active. Requires confirmation."},
  {"write", cmd_write, "Write currently active calibration to the calibration file. Requires confirmation."},
  {"read", cmd_read, "Read calibration from the calibration file."},
  {"defaults", cmd_defaults, "Set calibration to hard-coded defaults. Requires confirmation."},
  {"help", cmd_help, "This list."}
} ;

#define AWAITING_NONE 0xFF  // Code for not waiting for any confirmation. Must not point to a command table entry that requires conf.

const int n_cmds = sizeof(cmd_table) / sizeof (cmd_table_t);


// The above supports a single fixed command set.
// The following allows multiple command sets to be defined.

// A command set is a command count and a pointer to a dispatch table
struct cmd_set_t {
    const int n_cmds;
    const cmd_table_t* cmd_table;
};

// The command set for the top-level commands
cmd_set_t cmd_set = {
    sizeof(cmd_table) / sizeof(cmd_table_t),
    cmd_table
};

// Pointer to the command set in use
cmd_set_t* curr_cmd_set;

/* How to
void test()
{
    curr_cmd_set = &cmd_set;                  // Activate a command set
    curr_cmd_set->cmd_table[1].cmdHandler();  // Call a member of the set in use
}
*/

#endif