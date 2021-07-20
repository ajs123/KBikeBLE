#ifndef PARAM_STORE_
#define PARAM_STORE_
#include <Adafruit_LittleFS.h>
#include <Adafruit_LittleFS_File.h>
#include <InternalFileSystem.h>

#define PARAM_FOLDER "/KBikeBLE"

// Construct a full file path from the directory path and filename
//   dest = the full file path
//   path = directory path
//   file = file name
inline void make_path(char * dest, const char * path, const char * file);

//using namespace Adafruit_LittleFS_Namespace;
//File file(InternalFS);

// Start the InternalFS and ensure that the parameter folder is there.
void setup_InternalFS(void);

//byte stored_value[8];
//char filepath[LFS_NAME_MAX + 1];

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