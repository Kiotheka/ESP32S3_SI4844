#pragma once
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <driver/spi_master.h>
#include <driver/i2c_master.h>
#include <driver/gpio.h>
#include <freertos/task.h>
#include <GT911Constants.h>
#include <main.hpp>

class device
{
public:
  virtual ~device() = default;
  virtual void init() = 0;
  virtual auto is_display_touched() -> bool = 0;
  virtual auto asyncDMAIsBusy() -> bool = 0;
  virtual void asyncDMAWaitForCompletion() = 0;
  virtual void asyncDMAWriteBytes(uint8_t *data, uint32_t len) = 0;

  typedef struct
  {
    gpio_num_t _mosi;
    gpio_num_t _miso;
    gpio_num_t _sclk;
    gpio_num_t _cs;
    gpio_num_t _irq;
    spi_device_handle_t _handle;
    spi_host_device_t _host;
    int _frequency;
    bool _calibration;
    uint16_t _xp;
    uint16_t _yp;
    uint16_t _zp;
    int16_t _min_xp;    // Minimum xp calibration
    int16_t _min_yp;    // Minimum yp calibration
    int16_t _max_xp;    // Maximum xp calibration
    int16_t _max_yp;    // Maximum yp calibration
    int16_t _check1_xp; // increasing direction
    int16_t _check1_yp; // increasing direction
    int16_t _check2_xp; // increasing direction
    int16_t _check2_yp; // increasing direction
    int16_t _min_xc;    // Minimum x coordinate
    int16_t _min_yc;    // Minimum y coordinate
    int16_t _max_xc;    // Maximum x coordinate
    int16_t _max_yc;    // Maximum y coordinate
  } XPT2046_t;

  typedef struct
  {
    gpio_num_t _scl;
    gpio_num_t _sda;
    gpio_num_t _reset;
    gpio_num_t _int;
    i2c_master_bus_handle_t _bus_handle;
    i2c_master_dev_handle_t _device_handle;
    uint16_t _address_device;
    i2c_port_num_t _port;
    uint32_t _frequency;
    bool _calibration;
    uint16_t _xp;
    uint16_t _yp;
    uint16_t _zp;
    int16_t _min_xp;    // Minimum xp calibration
    int16_t _min_yp;    // Minimum yp calibration
    int16_t _max_xp;    // Maximum xp calibration
    int16_t _max_yp;    // Maximum yp calibration
    int16_t _check1_xp; // increasing direction
    int16_t _check1_yp; // increasing direction
    int16_t _check2_xp; // increasing direction
    int16_t _check2_yp; // increasing direction
    int16_t _min_xc;    // Minimum x coordinate
    int16_t _min_yc;    // Minimum y coordinate
    int16_t _max_xc;    // Maximum x coordinate
    int16_t _max_yc;    // Maximum y coordinate
  } GT911_t;
};

class TP_Point
{
public:
  TP_Point(void) {};
  ~TP_Point() {};
  TP_Point(uint8_t id, uint16_t x, uint16_t y, uint16_t size, uint8_t pressure, uint8_t state)
      : id(id), x(x), y(y), size(size), pressure(pressure), state(state) {}

  bool operator==(TP_Point point)
  {
    return ((point.x == x) && (point.y == y) && (point.size == size) && (point.pressure == pressure) && (point.state == state));
  }
  bool operator!=(TP_Point point)
  {
    return ((point.x != x) || (point.y != y) || (point.size != size) || (point.pressure != pressure) || (point.state != state));
  }

  uint8_t id;
  uint16_t x;
  uint16_t y;
  uint8_t size;
  uint8_t pressure;
  uint8_t state;
};