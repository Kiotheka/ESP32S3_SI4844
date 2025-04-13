/*
  eeprom.h -ported by celio to Esp32 from esp8266 eeprom espidf
          
  Uses a nvs byte array to emulate eeprom
  
*/

#ifndef eeprom_h
#define eeprom_h
#ifndef eeprom_FLASH_PARTITION_NAME
#define eeprom_FLASH_PARTITION_NAME "eeprom"
#endif
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <esp_log.h>
#include <esp_log.h>

typedef uint32_t nvs_handle;

class eepromClass {
  public:
    eepromClass(uint32_t sector);
    eepromClass(const char* name);
    eepromClass(void);
    ~eepromClass(void);

    bool begin(size_t size);
    uint8_t read(int address);
    void write(int address, uint8_t val);
    uint16_t length();
    bool commit();
    void end();

    uint8_t * getDataPtr();
    uint16_t convert(bool clear, const char* eepromname = "eeprom", const char* nvsname = "eeprom");

    template<typename T>
    T &get(int address, T &t) {
      if (address < 0 || address + sizeof(T) > _size)
        return t;

      memcpy((uint8_t*) &t, _data + address, sizeof(T));
      return t;
    }

    template<typename T>
    const T &put(int address, const T &t) {
      if (address < 0 || address + sizeof(T) > _size)
        return t;

      memcpy(_data + address, (const uint8_t*) &t, sizeof(T));
      _dirty = true;
      return t;
    }

    uint8_t readByte(int address);
    int8_t readChar(int address);
    uint8_t readUChar(int address);
    int16_t readShort(int address);
    uint16_t readUShort(int address);
    int32_t readInt(int address);
    uint32_t readUInt(int address);
    int32_t readLong(int address);
    uint32_t readULong(int address);
    int64_t readLong64(int address);
    uint64_t readULong64(int address);
    float_t readFloat(int address);
    double_t readDouble(int address);
    bool readBool(int address);
    size_t readString(int address, char* value, size_t maxLen);
   // String readString(int address);
    size_t readBytes(int address, void * value, size_t maxLen);
    template <class T> T readAll (int address, T &);

    size_t writeByte(int address, uint8_t value);
    size_t writeChar(int address, int8_t value);
    size_t writeUChar(int address, uint8_t value);
    size_t writeShort(int address, int16_t value);
    size_t writeUShort(int address, uint16_t value);
    size_t writeInt(int address, int32_t value);
    size_t writeUInt(int address, uint32_t value);
    size_t writeLong(int address, int32_t value);
    size_t writeULong(int address, uint32_t value);
    size_t writeLong64(int address, int64_t value);
    size_t writeULong64(int address, uint64_t value);
    size_t writeFloat(int address, float_t value);
    size_t writeDouble(int address, double_t value);
    size_t writeBool(int address, bool value);
    size_t writeString(int address, const char* value);
    //size_t writeString(int address, String value);
    size_t writeBytes(int address, const void* value, size_t len);
    template <class T> T writeAll (int address, const T &);

  protected:
    nvs_handle _handle;
    uint8_t* _data;
    size_t _size;
    bool _dirty;
    const char* _name;
};

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_eeprom)
extern eepromClass eeprom;
#endif

#endif
