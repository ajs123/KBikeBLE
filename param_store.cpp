#include <Arduino.h>
#include <Adafruit_LittleFS.h>
#include <Adafruit_LittleFS_File.h>
#include <InternalFileSystem.h>

#define PARAM_FOLDER "/KBikeBLE"
inline void make_path(char * dest, const char * path, const char * file)
{
  strncpy(dest, path, LFS_NAME_MAX);
  strncat(dest, "/", LFS_NAME_MAX);
  strncat(dest, file, LFS_NAME_MAX);
}

using namespace Adafruit_LittleFS_Namespace;
File file(InternalFS);

// Start the InternalFS and ensure that the parameter folder is there.
void setup_InternalFS(void)
{
  InternalFS.begin();
  if (InternalFS.exists(PARAM_FOLDER)) return;
  InternalFS.mkdir(PARAM_FOLDER);
}

byte stored_value[8];
char filepath[LFS_NAME_MAX + 1];

bool read_param_file(const char * name, void * value, const uint8_t length)
{
  make_path(filepath, PARAM_FOLDER, name);
  if (!InternalFS.exists(filepath)) return false;

  uint8_t len = min(length, 8);
  //File file(filepath, FILE_O_READ, InternalFS);
  if (!file.open(filepath, FILE_O_READ)) return false;
  uint8_t written = file.read(stored_value, len);
  file.close();
  if (written != len) return false;

  memcpy(value, stored_value, len);          // Alt is to use text (atoi, etc) 
  return true;
}

bool write_param_file(const char * name, const void * value, uint8_t length)
{
  make_path(filepath, PARAM_FOLDER, name);
  if (InternalFS.exists(filepath)) InternalFS.remove(filepath);  // Example in bonding.cpp shows remove and re-write instead of open/seek/write.
  
  uint8_t len = min(length, 8);
  if (!file.open(filepath, FILE_O_WRITE)) return false;
  uint8_t written = file.write( (const char *) value, len);
  file.close();
  if (written != len) return false;
  return true;
  byte var;
}