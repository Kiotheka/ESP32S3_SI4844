/*
  eeprom.h -ported by Paolo Becchi to Esp32 from esp8266 eeprom
           -Modified by Elochukwu Ifediora <ifedioraelochukwuc@gmail.com>
           -Converted to nvs lbernstone@gmail.com

  Uses a nvs byte array to emulate eeprom

  Copyright (c) 2014 Ivan Grokhotkov. All rights reserved.
  This file is part of the esp8266 core for Arduino environment.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "eeprom.h"
#include <nvs.h>
#include <nvs_flash.h>
#include <esp_partition.h>
#define TAG "eeprom"
eepromClass::eepromClass(void)
  : _handle(0)
  , _data(0)
  , _size(0)
  , _dirty(false)
  , _name("eeprom")
{
}

eepromClass::eepromClass(uint32_t sector)
// Only for compatiility, no sectors in nvs!
  : _handle(0)
  , _data(0)
  , _size(0)
  , _dirty(false)
  , _name("eeprom")
{
}

eepromClass::eepromClass(const char* name)
  : _handle(0)
  , _data(0)
  , _size(0)
  , _dirty(false)
  , _name(name)
{
}

eepromClass::~eepromClass() {
  end();
}

bool eepromClass::begin(size_t size) {
  if (!size) {
      return false;
  }

  esp_err_t err = nvs_flash_init();

  if (err != ESP_OK) {  
    ESP_LOGE(TAG,"nvs_flash no inicialized!: %d", err);
      return false;

  }

  esp_err_t res = nvs_open(_name, NVS_READWRITE, &_handle);
  if (res != ESP_OK) {    
      ESP_LOGE(TAG,"Unable to open NVS namespace: %d", res);
      return false;
  }

  size_t key_size = 0;
  res = nvs_get_blob(_handle, _name, NULL, &key_size);
  if(res != ESP_OK && res != ESP_ERR_NVS_NOT_FOUND) {
      ESP_LOGE(TAG,"Unable to read NVS key: %d", res);
      return false;
  }
  if (size < key_size) { // truncate
      ESP_LOGW(TAG,"truncating eeprom from %d to %d", key_size, size);
      uint8_t* key_data = (uint8_t*) malloc(key_size);
      if(!key_data) {
         ESP_LOGE(TAG,"Not enough memory to truncate eeprom!");
         return false;
      }
      nvs_get_blob(_handle, _name, key_data, &key_size);
      nvs_set_blob(_handle, _name, key_data, size);
      nvs_commit(_handle);
      free(key_data);
  }
  else if (size > key_size) { // expand or new
      size_t expand_size = size - key_size;
      uint8_t* expand_key = (uint8_t*) malloc(expand_size);
      if(!expand_key) {
         ESP_LOGE(TAG,"Not enough memory to expand eeprom!");
         return false;
      }
      // check for adequate free space
      if(nvs_set_blob(_handle, "expand", expand_key, expand_size)) {
        ESP_LOGE(TAG,"Not enough space to expand eeprom from %d to %d", key_size, size);
        free(expand_key);
        return false;
      }
      free(expand_key);
      nvs_erase_key(_handle, "expand");
      uint8_t* key_data = (uint8_t*) malloc(size);
      if(!key_data) {
         ESP_LOGE(TAG,"Not enough memory to expand eeprom!");
         return false;
      }
      memset(key_data, 0xFF, size);
      if(key_size) {
        ESP_LOGI(TAG,"Expanding eeprom from %d to %d", key_size, size);
	// hold data while key is deleted
        nvs_get_blob(_handle, _name, key_data, &key_size);
        nvs_erase_key(_handle, _name);
      } else {
        ESP_LOGI(TAG,"New eeprom of %d bytes", size);
      }
      nvs_commit(_handle);
      nvs_set_blob(_handle, _name, key_data, size);
      free(key_data);
      nvs_commit(_handle);
  }

  if (_data) {
    delete[] _data;
  }

  _data = (uint8_t*) malloc(size);
  if(!_data) {
    ESP_LOGE(TAG,"Not enough memory for %d bytes in eeprom", size);
    return false;
  }
  _size = size;
  nvs_get_blob(_handle, _name, _data, &_size);
  return true;
}

void eepromClass::end() {
  if (!_size) {
    return;
  }

  commit();
  if (_data) {
    delete[] _data;
  }
  _data = 0;
  _size = 0;

  nvs_close(_handle);
  _handle = 0;
}

uint8_t eepromClass::read(int address) {
  if (address < 0 || (size_t)address >= _size) {
    return 0;
  }
  if (!_data) {
    return 0;
  }

  return _data[address];
}

void eepromClass::write(int address, uint8_t value) {
  if (address < 0 || (size_t)address >= _size)
    return;
  if (!_data)
    return;

  // Optimise _dirty. Only flagged if data written is different.
  uint8_t* pData = &_data[address];
  if (*pData != value)
  {
    *pData = value;
    _dirty = true;
  }
}

bool eepromClass::commit() {
  bool ret = false;
  if (!_size) {
    return false;
  }
  if (!_data) {
    return false;
  }
  if (!_dirty) {
    return true;
  }

  esp_err_t err = nvs_set_blob(_handle, _name, _data, _size);
  if (err != ESP_OK) {
    ESP_LOGE(TAG,"error in write: %s", esp_err_to_name(err));
  } else {
    _dirty = false;
    ret = true;
  }

  return ret;
}

uint8_t * eepromClass::getDataPtr() {
  _dirty = true;
  return &_data[0];
}

/*
   Get eeprom total size in byte defined by the user
*/
uint16_t eepromClass::length ()
{
  return _size;
}

/* 
   Convert eeprom partition into nvs blob
   Call convert before you call begin
*/
uint16_t eepromClass::convert (bool clear, const char* eepromname, const char* nvsname)
{
  uint16_t result = 0;
  const esp_partition_t* mypart = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, eepromname);
  if (mypart == NULL) {
    ESP_LOGI(TAG,"eeprom partition not found for conversion");
    return result;
  }

  size_t size = mypart->size;
  uint8_t* data = (uint8_t*) malloc(size);
  if (!data) {
    ESP_LOGE(TAG,"Not enough memory to convert eeprom!");
    goto exit;
  }

  if (esp_partition_read (mypart, 0, (void *) data, size) != ESP_OK) {
    ESP_LOGE(TAG,"Unable to read eeprom partition");
    goto exit;
  }

  bool empty;
  empty = true;
  for (int x=0; x<size; x++) {
    if (data[x] != 0xFF) {
      empty = false;
      break;
    }
  }
  if (empty) {
    ESP_LOGI(TAG,"eeprom partition is empty, will not convert");
    goto exit;
  }

  nvs_handle handle;
  if (nvs_open(nvsname, NVS_READWRITE, &handle) != ESP_OK) {
    ESP_LOGE(TAG,"Unable to open NVS");
    goto exit;
  }
  esp_err_t err;
  err = nvs_set_blob(handle, nvsname, data, size);
  if (err != ESP_OK) {
    ESP_LOGE(TAG,"Unable to add eeprom data to NVS: %s", esp_err_to_name(err));
    goto exit;
  }
  result = size;
 
  if (clear) {
    if (esp_partition_erase_range (mypart, 0, size) != ESP_OK) {
      ESP_LOGW(TAG,"Unable to clear eeprom partition");
    }
  } 
exit:
  free(data);
  return result;
}

/*
   Read 'value' from 'address'
*/
uint8_t eepromClass::readByte (int address)
{
  uint8_t value = 0;
  return eepromClass::readAll (address, value);
}

int8_t eepromClass::readChar (int address)
{
  int8_t value = 0;
  return eepromClass::readAll (address, value);
}

uint8_t eepromClass::readUChar (int address)
{
  uint8_t value = 0;
  return eepromClass::readAll (address, value);
}

int16_t eepromClass::readShort (int address)
{
  int16_t value = 0;
  return eepromClass::readAll (address, value);
}

uint16_t eepromClass::readUShort (int address)
{
  uint16_t value = 0;
  return eepromClass::readAll (address, value);
}

int32_t eepromClass::readInt (int address)
{
  int32_t value = 0;
  return eepromClass::readAll (address, value);
}

uint32_t eepromClass::readUInt (int address)
{
  uint32_t value = 0;
  return eepromClass::readAll (address, value);
}

int32_t eepromClass::readLong (int address)
{
  int32_t value = 0;
  return eepromClass::readAll (address, value);
}

uint32_t eepromClass::readULong (int address)
{
  uint32_t value = 0;
  return eepromClass::readAll (address, value);
}

int64_t eepromClass::readLong64 (int address)
{
  int64_t value = 0;
  return eepromClass::readAll (address, value);
}

uint64_t eepromClass::readULong64 (int address)
{
  uint64_t value = 0;
  return eepromClass::readAll (address, value);
}

float_t eepromClass::readFloat (int address)
{
  float_t value = 0;
  return eepromClass::readAll (address, value);
}

double_t eepromClass::readDouble (int address)
{
  double_t value = 0;
  return eepromClass::readAll (address, value);
}

bool eepromClass::readBool (int address)
{
  int8_t value = 0;
  return eepromClass::readAll (address, value) ? 1 : 0;
}

size_t eepromClass::readString (int address, char* value, size_t maxLen)
{
  if (!value)
    return 0;

  if (address < 0 || address + maxLen > _size)
    return 0;

  uint16_t len;
  for (len = 0; len <= _size; len++)
    if (_data[address + len] == 0)
      break;

  if (address + len > _size)
    return 0;

  if (len > maxLen)
    return 0; //Maybe return part of the string instead?

  memcpy((uint8_t*) value, _data + address, len);
  value[len] = 0;
  return len;
}
/*
String eepromClass::readString (int address)
{
  if (address < 0 || address > _size)
    return String();

  uint16_t len;
  for (len = 0; len <= _size; len++)
    if (_data[address + len] == 0)
      break;

  if (address + len > _size)
    return String();

  char value[len+1];
  memcpy((uint8_t*) value, _data + address, len);
  value[len] = 0;
  return String(value);
}
*/
size_t eepromClass::readBytes (int address, void* value, size_t maxLen)
{
  if (!value || !maxLen)
    return 0;

  if (address < 0 || address + maxLen > _size)
    return 0;

  memcpy((void*) value, _data + address, maxLen);
  return maxLen;
}

template <class T> T eepromClass::readAll (int address, T &value)
{
  if (address < 0 || address + sizeof(T) > _size)
    return value;

  memcpy((uint8_t*) &value, _data + address, sizeof(T));
  return value;
}

/*
   Write 'value' to 'address'
*/
size_t eepromClass::writeByte (int address, uint8_t value)
{
  return eepromClass::writeAll (address, value);
}

size_t eepromClass::writeChar (int address, int8_t value)
{
  return eepromClass::writeAll (address, value);
}

size_t eepromClass::writeUChar (int address, uint8_t value)
{
  return eepromClass::writeAll (address, value);
}

size_t eepromClass::writeShort (int address, int16_t value)
{
  return eepromClass::writeAll (address, value);
}

size_t eepromClass::writeUShort (int address, uint16_t value)
{
  return eepromClass::writeAll (address, value);
}

size_t eepromClass::writeInt (int address, int32_t value)
{
  return eepromClass::writeAll (address, value);
}

size_t eepromClass::writeUInt (int address, uint32_t value)
{
  return eepromClass::writeAll (address, value);
}

size_t eepromClass::writeLong (int address, int32_t value)
{
  return eepromClass::writeAll (address, value);
}

size_t eepromClass::writeULong (int address, uint32_t value)
{
  return eepromClass::writeAll (address, value);
}

size_t eepromClass::writeLong64 (int address, int64_t value)
{
  return eepromClass::writeAll (address, value);
}

size_t eepromClass::writeULong64 (int address, uint64_t value)
{
  return eepromClass::writeAll (address, value);
}

size_t eepromClass::writeFloat (int address, float_t value)
{
  return eepromClass::writeAll (address, value);
}

size_t eepromClass::writeDouble (int address, double_t value)
{
  return eepromClass::writeAll (address, value);
}

size_t eepromClass::writeBool (int address, bool value)
{
  int8_t Bool;
  value ? Bool = 1 : Bool = 0;
  return eepromClass::writeAll (address, Bool);
}

size_t eepromClass::writeString (int address, const char* value)
{
  if (!value)
    return 0;

  if (address < 0 || address > _size)
    return 0;

  uint16_t len;
  for (len = 0; len <= _size; len++)
    if (value[len] == 0)
      break;

  if (address + len > _size)
    return 0;

  memcpy(_data + address, (const uint8_t*) value, len + 1);
  _dirty = true;
  return strlen(value);
}
/*
size_t eepromClass::writeString (int address, String value)
{
  return eepromClass::writeString (address, value.c_str());
}

size_t eepromClass::writeBytes (int address, const void* value, size_t len)
{
  if (!value || !len)
    return 0;

  if (address < 0 || address + len > _size)
    return 0;

  memcpy(_data + address, (const void*) value, len);
  _dirty = true;
  return len;
}
*/
template <class T> T eepromClass::writeAll (int address, const T &value)
{
  if (address < 0 || address + sizeof(T) > _size)
    return value;

  memcpy(_data + address, (const uint8_t*) &value, sizeof(T));
  _dirty = true;

  return sizeof (value);
}

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_eeprom)
eepromClass eeprom;
#endif
