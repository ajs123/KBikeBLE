// Parameter storage using the LittleFS flash memory filesystem

#ifndef PARAM_STORE_
#define PARAM_STORE_
#include <Adafruit_LittleFS.h>
#include <Adafruit_LittleFS_File.h>
#include <InternalFileSystem.h>

#define PARAM_FOLDER "/KBikeBLE"

// Start the InternalFS and ensure that the parameter folder is there.
void setup_InternalFS(void);

// Read a parameter from InternalFS.
//   name = filename (typically, the name of the variable)
//   value = receives the value
//   length = number of bytes (should be sizeof(variable) where variable is what's pointed to by value)
bool read_param_file(const char * name, void * value, const uint8_t length);

// Write a parameter to InternalFS
//   name = filename (typically the name of the variable)
//   value = the value to write
//   length = number of bytes (should be sizeof(variable) where variable is what's pointed to by value)
bool write_param_file(const char * name, const void * value, uint8_t length);

#endif