/*
12/04/2025 
teste display
 JC4827W543
 LVGL 9.2 
 Radio SI4844
 Testes com canvas para criar Dial
 Celio dos Santos Kiotheka
 celio@girotron.com.br
*/
#include <main.hpp>
//  #define LVGL_TASK

#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)                   \
  do                                         \
  {                                          \
    esp_err_t rc = (x);                      \
    if (rc != ESP_OK)                        \
    {                                        \
      ESP_LOGE("err", "esp_err_t = %d", rc); \
      assert(0 && #x);                       \
    }                                        \
  } while (0);

#define LVGL_TASK_MAX_DELAY_MS 500
#define LVGL_TASK_MIN_DELAY_MS 1
#define LVGL_TASK_STAK_SIZE (4 * 1024)
#define LVGL_TASK_PRIORITY 2
#define LVGL_TICK_PERIODO_MS 2
#define LOGIN_INI
#define EEPROM_SIZE 50
#define LOG_TAG "MAIN"

#define TOUCH_MIN_X 250
#define TOUCH_MAX_X 3850
#define TOUCH_MIN_Y 330
#define TOUCH_MAX_Y 3760

#define SI484_SDA GPIO_NUM_17
#define SI484_SCL GPIO_NUM_18
#define SI484_RST GPIO_NUM_7
#define SI484_IRQ GPIO_NUM_16
#define SI484_LNA GPIO_NUM_15

static JC4827W543R display{};
// display NV3041A—480 x 272
#if TOUCHRESISTIVE
device::XPT2046_t touchResistive;
#endif
#if TOUCHCAPACITIVE
device::GT911_t touchCapacitive;
#endif
uint32_t tickc = 0;
char stringInfoMem[100];
lv_obj_t *labelFrequency;
lv_obj_t *labelStereo;
lv_obj_t *labelTune;
lv_obj_t *scale;
lv_obj_t *screen;
lv_obj_t *line;
lv_style_t stylescreen;

enum
{
  TYPE_BAND_AM,
  TYPE_BAND_SW,
  TYPE_BAND_FM
};

typedef struct
{
  uint8_t total_tick_count;
  uint8_t major_tick_every;
  uint16_t rangeMin;
  uint16_t rangeMax;
  uint16_t mapMin;
  uint16_t mapMax;
} dial_t;

typedef struct
{
  uint8_t powerUp;
  union
  {
    struct
    {
      uint8_t band : 6;
      uint8_t xwait : 1;
      uint8_t xoscen : 1;
    } flag;
    uint8_t bandIndex;
  };
  uint16_t freqIncial;
  uint16_t freqFinal;
  uint8_t step;
  uint8_t feature;
  dial_t dial;
  uint8_t tipoBand;
} radio_t;

radio_t radio;
radio_t bands[30];
uint8_t bandIndex = 6;

#define LVGL_DRAW_BUF_LINES 272
#define LVGL_DUAL_BUFFERS 1

#if LVGL_DRAW_BUF_LINES > 34
void my_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
#define MAX_SPI_TRANSMIT 32768
  gpio_set_level(GPIO_NUM_44, true);
  uint16_t w = (area->x2 - area->x1 + 1);
  uint16_t h = (area->y2 - area->y1 + 1);
  uint32_t len = (w * h) * 2;
  uint8_t *p = px_map;
#if LVGL_DUAL_BUFFERS
  display.asyncDMAWaitForCompletion(); // 2 buffers
#endif
  display.set_write_address_window(area->x1, area->y1, w, h);
  while (len > MAX_SPI_TRANSMIT)
  {
    display.asyncDMAWriteBytes(p, MAX_SPI_TRANSMIT);
    p = p + MAX_SPI_TRANSMIT;
    len = len - MAX_SPI_TRANSMIT;
    display.asyncDMAWaitForCompletion();
  }
  if (len > 0)
  {
    display.asyncDMAWriteBytes(p, len);
  }
#if !LVGL_DUAL_BUFFERS
  display.asyncDMAWaitForCompletion(); // 1 buffer
#endif
  lv_display_flush_ready(disp);
  gpio_set_level(GPIO_NUM_44, false);
}
#else

void my_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
  gpio_set_level(GPIO_NUM_44, true);
  uint16_t w = (area->x2 - area->x1 + 1);
  uint16_t h = (area->y2 - area->y1 + 1);
  uint32_t len = (w * h) * 2;
  uint8_t *p = px_map;
#if LVGL_DUAL_BUFFERS
  display.asyncDMAWaitForCompletion(); // 2 buffers
#endif
  display.set_write_address_window(area->x1, area->y1, w, h);
  display.asyncDMAWriteBytes(p, len);
#if !LVGL_DUAL_BUFFERS
  display.asyncDMAWaitForCompletion(); // 1 buffer
#endif
  lv_disp_flush_ready(disp);
  gpio_set_level(GPIO_NUM_44, false);
}
#endif

void my_touchpad_read(lv_indev_t *indev_driver, lv_indev_data_t *data);
static void *disp_draw_buf1;
static void *disp_draw_buf2;
void lvgl_init()
{

  lv_display_t *disp;
  lv_init();
  lv_tick_set_cb([]
                 { return uint32_t(esp_timer_get_time() / 1000); });
  disp_draw_buf1 = heap_caps_malloc(TFT_WIDTH * LVGL_DRAW_BUF_LINES * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  disp_draw_buf2 = heap_caps_malloc(TFT_WIDTH * LVGL_DRAW_BUF_LINES * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

  if (!disp_draw_buf1 && !disp_draw_buf1)
  {
    ESP_LOGI(LOG_TAG, "Nao foi possivel alocar a memoria para disp_draw_buf");
    return;
  }
  // MALLOC_CAP_SPIRAM
  disp = lv_display_create(TFT_WIDTH, TFT_HEIGHT);
  lv_display_set_flush_cb(disp, my_disp_flush);
  lv_display_set_buffers(disp, disp_draw_buf1, disp_draw_buf2, TFT_WIDTH * LVGL_DRAW_BUF_LINES * 2, LV_DISPLAY_RENDER_MODE_PARTIAL);
  lv_indev_t *indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, my_touchpad_read);

  lv_style_init(&stylescreen);
  lv_style_set_bg_color(&stylescreen, lv_color_hex(0x000000));
}

//////////////////////////// end display //////////////////////////////
#ifdef LVGL_TASK
#if !LV_TICK_CUSTOM
static void increase_lvgl_tick(void *args)
{
  lv_tick_inc(LVGL_TICK_PERIODO_MS);
}
#endif
#endif
///// touch screen

static long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  const long run = in_max - in_min;
  if (run == 0)
  {
    return -1;
  }
  const long rise = out_max - out_min;
  const long delta = x - in_min;
  return (delta * rise) / run + out_min;
}

#if TOUCHRESISTIVE

void display_x_for_touch(const int16_t x, const int16_t y, int32_t *rx, int32_t *ry)
{

#if TFT_ORIENTATION == 1

  static float fact_y = (float)TFT_WIDTH / (touchResistive._max_yp - touchResistive._min_yp);
  *rx = float(touchResistive._max_yp - y) * fact_y;

  static float fact_x = (float)TFT_HEIGHT / (touchResistive._max_xp - touchResistive._min_xp);
  *ry = float(x - touchResistive._min_xp) * fact_x;

#elif TFT_ORIENTATION == 2

  static float fact_x = (float)TFT_WIDTH / (touchResistive._max_xp - touchResistive._min_xp);
  *rx = float(touchResistive._max_xp - x) * fact_x;

  static float fact_y = (float)TFT_HEIGHT / (touchResistive._max_yp - touchResistive._min_yp);
  *ry = float(touchResistive._max_yp - y) * fact_y;

#elif TFT_ORIENTATION == 3

  static float fact_y = (float)TFT_WIDTH / (touchResistive._max_yp - touchResistive._min_yp);
  *rx = float(y - touchResistive._min_yp) * fact_y;

  static float fact_x = (float)TFT_HEIGHT / (touchResistive._max_xp - touchResistive._min_xp);
  *ry = float(touchResistive._max_xp - x) * fact_x;

#else // 0

  static float fact_x = (float)TFT_WIDTH / (touchResistive._max_xp - touchResistive._min_xp);
  *rx = float(x - touchResistive._min_xp) * fact_x;

  static float fact_y = (float)TFT_HEIGHT / (touchResistive._max_yp - touchResistive._min_yp);
  *ry = float(y - touchResistive._min_yp) * fact_y;

#endif
}
#endif

#if TOUCHCAPACITIVE
bool touch_swap_xy = false;
int16_t touch_map_x1 = -1;
int16_t touch_map_x2 = -1;
int16_t touch_map_y1 = -1;
int16_t touch_map_y2 = -1;
int16_t touch_max_x = 0, touch_max_y = 0;
int16_t touch_raw_x = 0, touch_raw_y = 0;
int16_t touch_last_x = 0, touch_last_y = 0;

void translate_touch_raw()
{
  if (touch_swap_xy)
  {
    touch_last_x = map(touch_raw_y, touch_map_x1, touch_map_x2, 0, touch_max_x);
    touch_last_y = map(touch_raw_x, touch_map_y1, touch_map_y2, 0, touch_max_y);
  }
  else
  {
    touch_last_x = map(touch_raw_x, touch_map_x1, touch_map_x2, 0, touch_max_x);
    touch_last_y = map(touch_raw_y, touch_map_y1, touch_map_y2, 0, touch_max_y);
  }
}

void touch_capacitive_init(int16_t w, int16_t h, uint8_t r)
{
  touchCapacitive._scl = GPIO_NUM_4;
  touchCapacitive._sda = GPIO_NUM_8;
  touchCapacitive._int = GPIO_NUM_3;
  touchCapacitive._reset = GPIO_NUM_38;
  touchCapacitive._port = I2C_NUM_0;
  touchCapacitive._frequency = 100000;
  touchCapacitive._address_device = 0x5D;
  display.GT911Ini(&touchCapacitive);

  touch_max_x = w - 1;
  touch_max_y = h - 1;
  if (touch_map_x1 == -1)
  {
    switch (r)
    {
    case 1:
      touch_swap_xy = true;
      touch_map_x1 = 0;
      touch_map_x2 = touch_max_x;
      touch_map_y1 = touch_max_y;
      touch_map_y2 = 0;
      break;
    case 2:
      touch_swap_xy = false;
      touch_map_x1 = 0;
      touch_map_x2 = touch_max_x;
      touch_map_y1 = 0;
      touch_map_y2 = touch_max_y;
      break;
    case 3:
      touch_swap_xy = true;
      touch_map_x1 = touch_max_x;
      touch_map_x2 = 0;
      touch_map_y1 = 0;
      touch_map_y2 = touch_max_y;
      break;

    default: // case 0:
      touch_swap_xy = false;
      touch_map_x1 = touch_max_x;
      touch_map_x2 = 0;
      touch_map_y1 = touch_max_y;
      touch_map_y2 = 0;
      break;
    }
  }
}
#endif

#if TOUCHRESISTIVE
static void touch_resistive_init(void)
{

  touchResistive._host = SPI3_HOST;
  touchResistive._mosi = TOUCH_MOSI;
  touchResistive._miso = TOUCH_MISO;
  touchResistive._sclk = TOUCH_SCK;
  touchResistive._cs = TOUCH_CS;
  touchResistive._irq = GPIO_NUM_NC;   // TOUCH_IRQ;
  touchResistive._frequency = 1000000; // 1Mhz

  touchResistive._min_xp = TOUCH_MIN_X; // Minimum xp calibration
  touchResistive._min_yp = TOUCH_MIN_Y; // Minimum yp calibration
  touchResistive._max_xp = TOUCH_MAX_X; // Maximum xp calibration
  touchResistive._max_yp = TOUCH_MAX_Y; // Maximum yp calibration

  display.XPT2046Ini(&touchResistive);
}
#endif

void my_touchpad_read(lv_indev_t *indev_driver, lv_indev_data_t *data)
{

#if TOUCHCAPACITIVE
  if (display.GT911Read(&touchCapacitive))
  {
    TP_Point t = display.GT911GetPoint(&touchCapacitive, 0);
    touch_raw_x = t.x;
    touch_raw_y = t.y;
    touch_last_x = touch_raw_x;
    touch_last_y = touch_raw_y;

    translate_touch_raw();
    data->point.x = touch_last_x;
    data->point.y = touch_last_y;

    data->state = LV_INDEV_STATE_PR;
  }
  else
  {
    data->state = LV_INDEV_STATE_REL;
  }

#endif

#if TOUCHRESISTIVE

  if (display.touch_getxy(&touchResistive))
  {
    // data->point.y =
    display_x_for_touch(touchResistive._xp, touchResistive._yp, &data->point.x, &data->point.y);

    // data->point.x = display_y_for_touch(touchResistive._yp);

    data->state = LV_INDEV_STATE_PR;
  }
  else
  {
    data->state = LV_INDEV_STATE_REL;
  }
#endif

  if (data->state == LV_INDEV_STATE_PR)
  {
  }
}

/// end touch screen

/// radio

typedef struct
{
  gpio_num_t _scl;
  gpio_num_t _sda;
  gpio_num_t _reset;
  gpio_num_t _irq;
  gpio_num_t _lna;
  i2c_port_num_t _port;
  uint32_t _clock_frequency;
  uint16_t _address_device;
  i2c_master_bus_handle_t _bus_handle;
  i2c_master_dev_handle_t _device_handle;
} SI4844_t;

#define ATDD_POWER_DONW 0x11
#define ATDD_POWER_UP 0xE1
#define GET_REV 0x10
#define ATDD_COSEN 0x80
#define ATDD_XWAIT 0x40
#define ATDD_GET_STATUS 0xE0
#define CMD_SET_PROPERTY 0x12
SI4844_t SI4844_dev = {
    ._scl = SI484_SCL,
    ._sda = SI484_SDA,
    ._reset = SI484_RST,
    ._irq = SI484_IRQ,
    ._lna = SI484_LNA,
    ._port = I2C_NUM_1,
    ._clock_frequency = 100000,
    ._address_device = 0x11,
    ._bus_handle = NULL,
    ._device_handle = NULL,
};
static bool irqRadio = 0;
uint8_t data[50];
uint8_t comm[50];

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
  irqRadio = 1;
}

void SI4844_i2c_init(SI4844_t *dev)
{

  i2c_master_bus_config_t i2c_mst_config = {};
  i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
  i2c_mst_config.i2c_port = dev->_port;
  i2c_mst_config.scl_io_num = dev->_scl;
  i2c_mst_config.sda_io_num = dev->_sda;
  i2c_mst_config.glitch_ignore_cnt = 7;
  i2c_mst_config.flags.enable_internal_pullup = true;

  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &dev->_bus_handle));
  i2c_device_config_t dev_cfg = {};
  dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  dev_cfg.device_address = dev->_address_device; // 0x3C;
  dev_cfg.scl_speed_hz = dev->_clock_frequency;

  ESP_ERROR_CHECK(i2c_master_bus_add_device(dev->_bus_handle, &dev_cfg, &dev->_device_handle));

  if (dev->_irq > 0)
  {
    gpio_config_t int_SI4844 =
        {
            .pin_bit_mask = 1LLU << dev->_irq,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
            .intr_type = GPIO_INTR_POSEDGE,
        };
    gpio_config(&int_SI4844);
  }

  if (dev->_reset > 0)
  {
    gpio_config_t reset_SI4844 =
        {
            .pin_bit_mask = 1LLU << dev->_reset,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };

    gpio_config(&reset_SI4844);
    gpio_set_level(dev->_reset, false);
  }
  if (dev->_lna > 0)
  {
    gpio_config_t lna_SI4844 =
        {
            .pin_bit_mask = 1LLU << dev->_lna,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };

    gpio_config(&lna_SI4844);
    gpio_set_level(dev->_lna, false);
  }
  gpio_install_isr_service(0);
  gpio_isr_handler_add(dev->_irq, gpio_isr_handler, (void *)&dev->_irq);
}

void band_init(void)
{
  // am
  bands[0].powerUp = 0xE1;
  bands[0].flag.band = 24;
  bands[0].tipoBand = TYPE_BAND_AM;
  bands[0].flag.xwait = 1;
  bands[0].flag.xoscen = 1;
  bands[0].feature = 0;
  bands[0].freqIncial = 500;
  bands[0].freqFinal = 1700;
  bands[0].step = 10;
  bands[0].dial.total_tick_count = 61;
  bands[0].dial.major_tick_every = 5;
  bands[0].dial.rangeMin = 500;
  bands[0].dial.rangeMax = 1700;
  bands[0].dial.mapMin = 500;
  bands[0].dial.mapMax = 1700;

  bands[1].powerUp = 0xE1;
  bands[1].flag.band = 26;
  bands[1].tipoBand = TYPE_BAND_SW;
  bands[1].flag.xwait = 0;
  bands[1].flag.xoscen = 1;
  bands[1].feature = 0;
  bands[1].freqIncial = 5900;
  bands[1].freqFinal = 6200;
  bands[1].step = 5;
  bands[1].dial.total_tick_count = 31;
  bands[1].dial.major_tick_every = 5;
  bands[1].dial.rangeMin = 5900;
  bands[1].dial.rangeMax = 6200;
  bands[1].dial.mapMin = 5900;
  bands[1].dial.mapMax = 6200;

  bands[2].powerUp = 0xE1;
  bands[2].flag.band = 28;
  bands[2].tipoBand = TYPE_BAND_SW;
  bands[2].flag.xwait = 0;
  bands[2].flag.xoscen = 1;
  bands[2].feature = 0;
  bands[2].freqIncial = 7000;
  bands[2].freqFinal = 7600;
  bands[2].step = 5;
  bands[2].dial.total_tick_count = 31;
  bands[2].dial.major_tick_every = 5;
  bands[2].dial.rangeMin = 7000;
  bands[2].dial.rangeMax = 7600;
  bands[2].dial.mapMin = 7000;
  bands[2].dial.mapMax = 7600;

  bands[3].powerUp = 0xE1;
  bands[3].flag.band = 29;
  bands[3].tipoBand = TYPE_BAND_SW;
  bands[3].flag.xwait = 0;
  bands[3].flag.xoscen = 1;
  bands[3].feature = 0;
  bands[3].freqIncial = 9200;
  bands[3].freqFinal = 10000;
  bands[3].step = 5;
  bands[3].dial.total_tick_count = 41;
  bands[3].dial.major_tick_every = 5;
  bands[3].dial.rangeMin = 9200;
  bands[3].dial.rangeMax = 10000;
  bands[3].dial.mapMin = 9200;
  bands[3].dial.mapMax = 10000;

  bands[4].powerUp = 0xE1;
  bands[4].flag.band = 32;
  bands[4].tipoBand = TYPE_BAND_SW;
  bands[4].flag.xwait = 0;
  bands[4].flag.xoscen = 1;
  bands[4].feature = 0;
  bands[4].freqIncial = 11600;
  bands[4].freqFinal = 12200;
  bands[4].step = 5;
  bands[4].dial.total_tick_count = 31;
  bands[4].dial.major_tick_every = 5;
  bands[4].dial.rangeMin = 11600;
  bands[4].dial.rangeMax = 12200;
  bands[4].dial.mapMin = 11600;
  bands[4].dial.mapMax = 12200;

  bands[5].powerUp = 0xE1;
  bands[5].flag.band = 16;
  bands[5].tipoBand = TYPE_BAND_FM;
  bands[5].flag.xwait = 0;
  bands[5].flag.xoscen = 1;
  bands[5].feature = 0;
  bands[5].freqIncial = 7400;
  bands[5].freqFinal = 8800;
  bands[5].step = 10;
  bands[5].dial.total_tick_count = 26;
  bands[5].dial.major_tick_every = 5;
  bands[5].dial.rangeMin = 74;
  bands[5].dial.rangeMax = 88;
  bands[5].dial.mapMin = 7400;
  bands[5].dial.mapMax = 8800;

  bands[6].powerUp = 0xE1;
  bands[6].flag.band = 0;
  bands[6].tipoBand = TYPE_BAND_FM;
  bands[6].flag.xwait = 0;
  bands[6].flag.xoscen = 1;
  bands[6].feature = 0;
  bands[6].freqIncial = 8700;
  bands[6].freqFinal = 10800;
  bands[6].step = 10;
  bands[6].dial.total_tick_count = 43;
  bands[6].dial.major_tick_every = 6;
  bands[6].dial.rangeMin = 87;
  bands[6].dial.rangeMax = 108;
  bands[6].dial.mapMin = 8700; // 8700;
  bands[6].dial.mapMax = 10800;

  bands[7].powerUp = 0xE1;
  bands[7].flag.band = 40;
  bands[7].tipoBand = TYPE_BAND_SW;
  bands[7].flag.xwait = 0;
  bands[7].flag.xoscen = 1;
  bands[7].feature = 0;
  bands[7].freqIncial = 26900;
  bands[7].freqFinal = 27500;
  bands[7].step = 5;
  bands[7].dial.total_tick_count = 31;
  bands[7].dial.major_tick_every = 5;
  bands[7].dial.rangeMin = 26900;
  bands[7].dial.rangeMax = 27500;
  bands[7].dial.mapMin = 26900;
  bands[7].dial.mapMax = 27500;

  bands[8].powerUp = 0xE1;
  bands[8].flag.band = 0;
  bands[8].tipoBand = TYPE_BAND_FM;
  bands[8].flag.xwait = 0;
  bands[8].flag.xoscen = 1;
  bands[8].feature = 0;
  bands[8].freqIncial = 10800; // 8700;
  bands[8].freqFinal = 12800;
  bands[8].step = 10;
  bands[8].dial.total_tick_count = 43;
  bands[8].dial.major_tick_every = 6;
  bands[8].dial.rangeMin = 108; // 87;
  bands[8].dial.rangeMax = 128;
  bands[8].dial.mapMin = 10800; // 8700;
  bands[8].dial.mapMax = 12800;
}

void SI4844_powerOn(void)
{
  // bandIndex = 0;

  lv_scale_set_total_tick_count(scale, bands[bandIndex].dial.total_tick_count);
  lv_scale_set_major_tick_every(scale, bands[bandIndex].dial.major_tick_every);
  lv_scale_set_range(scale, bands[bandIndex].dial.rangeMin, bands[bandIndex].dial.rangeMax);

  if (bandIndex == 1)
  {
    static const char *custom_labels[] = {"5.90", "5.95", "6.00", "6.05", "6.10", "6.15", "6.30", NULL};
    lv_scale_set_text_src(scale, custom_labels);
  }
  else if (bandIndex == 2)
  {
    static const char *custom_labels[] = {"7.00", "7.10", "7.20", "7.30", "7.40", "7.50", "7.60", NULL};
    lv_scale_set_text_src(scale, custom_labels);
  }
  else if (bandIndex == 3)
  {
    static const char *custom_labels[] = {"9.20", "9.30", "9.40", "9.50", "9.60", "9.70", "9.80", "9.90", "10.00", NULL};
    lv_scale_set_text_src(scale, custom_labels);
  }
  else if (bandIndex == 4)
  {
    static const char *custom_labels[] = {"11.60", "11.70", "11.80", "11.90", "12.00", "12.10", "12.20", NULL};
    lv_scale_set_text_src(scale, custom_labels);
  }
  else
    lv_scale_set_text_src(scale, NULL);

  gpio_set_level(SI484_RST, false);
  vTaskDelay(1);
  gpio_set_level(SI484_RST, true);
  vTaskDelay(2);

  comm[0] = bands[bandIndex].powerUp;
  comm[1] = bands[bandIndex].bandIndex;
  comm[2] = bands[bandIndex].freqIncial >> 8;
  comm[3] = bands[bandIndex].freqIncial;
  comm[4] = bands[bandIndex].freqFinal >> 8;
  comm[5] = bands[bandIndex].freqFinal;
  comm[6] = bands[bandIndex].step;

  if (i2c_master_transmit(SI4844_dev._device_handle, comm, 7, 20) == ESP_OK)
  {
    irqRadio = 0;
  }
}

#define POS_X 40
#define POS_Y 20
void setPosDial(uint16_t pos)
{
  lv_obj_set_pos(line, POS_X - 1 + pos, POS_Y + 1);
}

ledc_channel_config_t

    ledc_channel = {
        .gpio_num = GPIO_NUM_6,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .hpoint = 0,
        .flags = 0,
};

void updateDial(uint16_t value)
{
  ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, value);
  ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
}

void dial_pwm_init(void)
{
  ledc_timer_config_t ledc_timer = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .duty_resolution = LEDC_TIMER_12_BIT,
      .timer_num = LEDC_TIMER_1,
      .freq_hz = 5000,
      .clk_cfg = LEDC_AUTO_CLK,
      .deconfigure = 0,
  };

  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
  ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

  updateDial(2500);
}

bool SI4844_get_status()
{

  i2c_master_transmit(SI4844_dev._device_handle, comm, 1, 20);
  vTaskDelay(1);
  if (i2c_master_receive(SI4844_dev._device_handle, data, 4, 10) == ESP_OK)
  {
    return true;
  }
  return false;
}
#define CMD_GET_PROPERTY 0x13
#define REFCLK_FREQ 0x0201
#define REFCLK_PRESCALE 0x0202
#define RX_VOLUME 0x4000
#define RX_BASS_TREBLE 0x4002
void SI4844_set_property(uint16_t property, uint16_t value)
{
  comm[0] = CMD_SET_PROPERTY;
  comm[1] = 0x00;
  comm[2] = property >> 8;
  comm[3] = property;
  comm[4] = value >> 8;
  comm[5] = value;
  i2c_master_transmit(SI4844_dev._device_handle, comm, 6, 20);
}

////////////////// gui

lv_obj_t *spinbox;

static void lv_spinbox_increment_event_cb(lv_event_t *e)
{

  lv_obj_t *target = (lv_obj_t *)lv_event_get_user_data(e);
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT)
  {

    lv_spinbox_increment(target);
    if (target == spinbox)
    {
    }
  }
}

static void lv_spinbox_decrement_event_cb(lv_event_t *e)
{
  lv_obj_t *target = (lv_obj_t *)lv_event_get_user_data(e);
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SHORT_CLICKED || code == LV_EVENT_LONG_PRESSED_REPEAT)
  {
    lv_spinbox_decrement(target);
    if (target == spinbox)
    {
    }
  }
}

static void event_cb_myButton(lv_event_t *e)
{

  comm[0] = ATDD_GET_STATUS;
  i2c_master_transmit(SI4844_dev._device_handle, comm, 1, 20);
  vTaskDelay(1);
  if (i2c_master_receive(SI4844_dev._device_handle, data, 4, 10) == ESP_OK)
  {
    ESP_LOGI(LOG_TAG, "dados recebidos %X %X %X %X", data[0], data[1], data[2], data[3]);
  }
}

static void event_cb_powerOn(lv_event_t *e)
{
  SI4844_powerOn();
}

static void event_cb_band(lv_event_t *e)
{
  bandIndex++;
  if (bandIndex > 8)
    bandIndex = 0;
  SI4844_powerOn();
}

static void event_cb_test(lv_event_t *e)
{
}

// screens
void create_screen(void)
{
  screen = lv_obj_create(NULL);
  lv_obj_add_style(screen, &stylescreen, 0);
  scale = lv_scale_create(screen);

  lv_obj_set_size(scale, 400, 20);
  lv_scale_set_mode(scale, LV_SCALE_MODE_HORIZONTAL_TOP);
  lv_obj_set_pos(scale, POS_X, POS_Y);
  lv_obj_set_style_text_font(scale, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_scale_set_label_show(scale, true);
  lv_obj_set_style_length(scale, 8, LV_PART_ITEMS);
  lv_obj_set_style_length(scale, 15, LV_PART_INDICATOR);

  static lv_point_precise_t line_points[] = {{0, 11}, {0, 27}};

  static lv_style_t style_line;
  lv_style_init(&style_line);
  lv_style_set_line_width(&style_line, 5);
  lv_style_set_line_color(&style_line, lv_palette_main(LV_PALETTE_RED));

  line = lv_line_create(screen);
  lv_line_set_points(line, line_points, 2);
  lv_obj_add_style(line, &style_line, 0);
  lv_obj_set_pos(line, POS_X - 1 + 0, POS_Y + 1);

  labelStereo = lv_label_create(screen);
  lv_label_set_text(labelStereo, "");
  lv_obj_align(labelStereo, LV_ALIGN_TOP_LEFT, 60, 50);

  labelTune = lv_label_create(screen);
  lv_label_set_text(labelTune, "");
  lv_obj_align(labelTune, LV_ALIGN_TOP_LEFT, 120, 50);

  labelFrequency = lv_label_create(screen);

  static lv_style_t style;
  lv_style_init(&style);
  lv_style_set_text_font(&style, &lv_font_montserrat_48);
  lv_style_set_text_color(&style, LV_COLOR_MAKE(0x24, 0x02, 0xF5));
  lv_obj_add_style(labelFrequency, &style, 0);
  lv_label_set_text(labelFrequency, "0");
  lv_obj_set_pos(labelFrequency, 250, 170);

////////////////////////////////////////////////// spinbox
#define POSDCL_H 150

  lv_obj_t *label_bit_63_32 = lv_label_create(screen);
  lv_obj_set_pos(label_bit_63_32, 0, POSDCL_H + 25 + 8);
  lv_label_set_text(label_bit_63_32, "FR");
  lv_obj_set_style_text_color(label_bit_63_32, lv_palette_main(LV_PALETTE_GREEN), 0);

  spinbox = lv_spinbox_create(screen);
  lv_spinbox_set_range(spinbox, 0, 65535);
  lv_spinbox_set_digit_format(spinbox, 5, 0);
  lv_obj_set_width(spinbox, 80);
  lv_obj_set_pos(spinbox, 40 + 8, POSDCL_H + 25);
  lv_spinbox_set_value(spinbox, 0);
  lv_obj_set_style_bg_opa(spinbox, 80, LV_PART_CURSOR);

  lv_obj_t *btn_minus;
  lv_obj_t *btn_plus;
  int32_t h1 = lv_obj_get_height(spinbox);
  btn_minus = lv_btn_create(screen);
  lv_obj_set_size(btn_minus, h1, h1);
  lv_obj_align_to(btn_minus, spinbox, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
  lv_obj_set_style_bg_img_src(btn_minus, LV_SYMBOL_MINUS, 0);
  lv_obj_add_event_cb(btn_minus, lv_spinbox_decrement_event_cb, LV_EVENT_ALL, spinbox);
  lv_obj_set_style_bg_color(btn_minus, lv_palette_main(LV_PALETTE_GREY), 0);

  btn_plus = lv_btn_create(screen);
  lv_obj_set_size(btn_plus, h1, h1);
  lv_obj_align_to(btn_plus, btn_minus, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
  lv_obj_set_style_bg_img_src(btn_plus, LV_SYMBOL_PLUS, 0);
  lv_obj_add_event_cb(btn_plus, lv_spinbox_increment_event_cb, LV_EVENT_ALL, spinbox);
  lv_obj_set_style_bg_color(btn_plus, lv_palette_main(LV_PALETTE_GREY), 0);
  lv_spinbox_set_value(spinbox, 32768);

  lv_obj_t *btn1 = lv_button_create(screen);
  lv_obj_add_event_cb(btn1, event_cb_myButton, LV_EVENT_CLICKED, NULL);
  lv_obj_set_size(btn1, 100, 50);
  lv_obj_set_pos(btn1, 0, 80);
  lv_obj_remove_flag(btn1, LV_OBJ_FLAG_PRESS_LOCK);

  lv_obj_t *btn1label = lv_label_create(btn1);
  lv_label_set_text(btn1label, "Status");
  lv_obj_center(btn1label);
  ////////////////////////////////////////////////////
  lv_obj_t *btn2 = lv_button_create(screen);
  lv_obj_add_event_cb(btn2, event_cb_powerOn, LV_EVENT_CLICKED, NULL);
  lv_obj_set_size(btn2, 100, 50);
  lv_obj_set_pos(btn2, 110, 80);
  lv_obj_remove_flag(btn2, LV_OBJ_FLAG_PRESS_LOCK);

  lv_obj_t *btn2label = lv_label_create(btn2);
  lv_label_set_text(btn2label, "Power");
  lv_obj_center(btn2label);
  ///

  lv_obj_t *btn3 = lv_button_create(screen);
  lv_obj_add_event_cb(btn3, event_cb_band, LV_EVENT_CLICKED, NULL);
  lv_obj_set_size(btn3, 100, 50);
  lv_obj_set_pos(btn3, 220, 80);
  lv_obj_remove_flag(btn3, LV_OBJ_FLAG_PRESS_LOCK);

  lv_obj_t *btn3label = lv_label_create(btn3);
  lv_label_set_text(btn3label, "Band");
  lv_obj_center(btn3label);

  lv_obj_t *btn4 = lv_button_create(screen);
  lv_obj_add_event_cb(btn4, event_cb_test, LV_EVENT_CLICKED, NULL);
  lv_obj_set_size(btn4, 100, 50);
  lv_obj_set_pos(btn4, 330, 80);
  lv_obj_remove_flag(btn4, LV_OBJ_FLAG_PRESS_LOCK);

  lv_obj_t *btn4label = lv_label_create(btn4);
  lv_label_set_text(btn4label, "test");
  lv_obj_center(btn4label);
}

#define CANVAS_WIDTH 480
#define CANVAS_HEIGHT 40

#define BUF_LEN CANVAS_WIDTH *CANVAS_HEIGHT * 4
static uint8_t *buf_draw_buf = (uint8_t *)heap_caps_malloc(BUF_LEN, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
static lv_draw_buf_t draw_buf = {
    .header = {
        .magic = LV_IMAGE_HEADER_MAGIC,
        .cf = LV_COLOR_FORMAT_ARGB8888,
        .flags = LV_IMAGE_FLAGS_MODIFIABLE,
        .w = CANVAS_WIDTH,
        .h = CANVAS_HEIGHT,
        .stride = LV_DRAW_BUF_STRIDE(CANVAS_WIDTH, LV_COLOR_FORMAT_ARGB8888),
        .reserved_2 = 0,
    },
    .data_size = BUF_LEN, // sizeof(buf_draw_buf),
    .data = buf_draw_buf,
    .unaligned_data = buf_draw_buf,
    .handlers = 0,
};

lv_obj_t *canvas;

void ini_dial_canvas(void)
{

  if (buf_draw_buf)
  {
    lv_image_header_t *header = &draw_buf.header;
    lv_draw_buf_init(&draw_buf, header->w, header->h, (lv_color_format_t)header->cf, header->stride, buf_draw_buf, BUF_LEN); // sizeof(buf_draw_buf));
    lv_draw_buf_set_flag(&draw_buf, LV_IMAGE_FLAGS_MODIFIABLE);
    // LV_DRAW_BUF_DEFINE_STATIC(draw_buf, CANVAS_WIDTH, CANVAS_HEIGHT, LV_COLOR_FORMAT_ARGB8888);
    // LV_DRAW_BUF_INIT_STATIC(draw_buf);
    canvas = lv_canvas_create(screen);
    lv_canvas_set_draw_buf(canvas, &draw_buf);
    lv_obj_set_pos(canvas, 0, 272 - CANVAS_HEIGHT);
  }
  else
    ESP_LOGI(LOG_TAG, "Nao foi possivel alocar memoria");
}

void dial_canvas(uint32_t fr, uint16_t fr_band_min, uint16_t fr_band_max, uint8_t step, uint8_t tipoBand)
{
#define STEP2 5
#define STEP3 10

  lv_canvas_fill_bg(canvas, LV_COLOR_MAKE(0x00, 0x00, 0x0), LV_OPA_COVER);
  lv_layer_t layer;
  lv_draw_line_dsc_t dsc;
  lv_draw_label_dsc_t dscl;
  lv_draw_rect_dsc_t dscr;
  lv_draw_triangle_dsc_t dsct;
  lv_canvas_init_layer(canvas, &layer);
  lv_draw_rect_dsc_init(&dscr);
  lv_draw_line_dsc_init(&dsc);
  lv_draw_label_dsc_init(&dscl);
  lv_draw_triangle_dsc_init(&dsct);
  char text[20];
  dsc.width = 1;
  dsc.color = LV_COLOR_MAKE(0xFF, 0xFF, 0xFF);
  dscl.color = LV_COLOR_MAKE(0xFF, 0xFF, 0xFF);
  dsct.bg_color = LV_COLOR_MAKE(0xFF, 0x00, 0x00);
  dscl.font = &lv_font_montserrat_14;
  dscl.decor = LV_TEXT_DECOR_NONE;
  dscl.align = LV_TEXT_ALIGN_CENTER;
  dscl.text_local = 1;
  dscl.text = text; // ponteiro do texto label

#define DIAL_LENGTH 60
#define DIAL_CENTER 30
#define DIAL_POS 0

  int pointerStep = (fr) - (DIAL_CENTER * step);

  for (int i = 0; i < DIAL_LENGTH + 1; i++)
  {
    if (!(pointerStep < fr_band_min || pointerStep > fr_band_max))
    {
      dsc.color = LV_COLOR_MAKE(0xFF, 0xFF, 0xFF);
      dsc.p1.x = i * 8;
      dsc.p2.x = i * 8;
      dsc.p2.y = CANVAS_HEIGHT;

      if ((pointerStep % (step * STEP3)) == 0 || i == DIAL_CENTER)
      {
        dsc.width = 2;
        dsc.p1.y = CANVAS_HEIGHT - 24;

        if ((pointerStep % (step * STEP3)) == 0)
        {
          if (tipoBand == TYPE_BAND_SW)
          {
            sprintf(text, "%u", pointerStep);
          }
          else if (tipoBand == TYPE_BAND_FM)
          {
            sprintf(text, "%u.0", pointerStep / 100);
          }
          else
          {
            sprintf(text, "%u", pointerStep);
          }
          lv_area_t coords = {(i * 8) - 25, CANVAS_HEIGHT - 40, (i * 8) + 25, CANVAS_HEIGHT - 20};
          lv_draw_label(&layer, &dscl, &coords);
        }
        if (i == DIAL_CENTER)
        {
          dsc.color = LV_COLOR_MAKE(0xFF, 0x0, 0x0);
#if 0 // triangulo no cursor
          dsct.p[0].x = (i * 8) - 5; 
          dsct.p[0].y = CANVAS_HEIGHT - 24;   
          dsct.p[1].x = (i * 8) + 5;
          dsct.p[1].y = CANVAS_HEIGHT - 24;
          dsct.p[2].x = (i * 8);         
          dsct.p[2].y = CANVAS_HEIGHT - 14;
          lv_draw_triangle(&layer, &dsct);
#endif
        }
      }
      else if (((pointerStep % (step * STEP2)) == 0) && ((pointerStep % (step * STEP3)) != 0))
      {
        dsc.width = 2;
        dsc.p1.y = CANVAS_HEIGHT - 16;
      }
      else
      {
        dsc.width = 1;
        dsc.p1.y = CANVAS_HEIGHT - 8;
      }
      lv_draw_line(&layer, &dsc);
    }
    
    pointerStep += step;
  }
#if 0 /// teste smeter
#define METER_POS_X 10
#define METER_POS_Y 30
  for (int i = 0; i < 17; i++)
  {
    if (i < 10)
    {
      dscr.bg_color = LV_COLOR_MAKE(0x00, 0xFF, 0x00);
      lv_area_t coords = {METER_POS_X + (i * 4), METER_POS_Y - 5 - i, METER_POS_X + 1 + (i * 4), METER_POS_Y};
      lv_draw_rect(&layer, &dscr, &coords);
    }
    else
    {
      dscr.bg_color = LV_COLOR_MAKE(0xFF, 0x00, 0x00);
      lv_area_t coords = {METER_POS_X + (i * 4), METER_POS_Y - 5 - i, METER_POS_X + 1 + (i * 4), METER_POS_Y};
      lv_draw_rect(&layer, &dscr, &coords);
    }
  }
#endif
  lv_canvas_finish_layer(canvas, &layer);
}

extern "C" void app_main(void)
{

  setup();
  while (1)
  {
    loop();
    static uint32_t yieldTick = 0;
    if (yieldTick < xTaskGetTickCount())
    {
      yieldTick = xTaskGetTickCount() + 200;
      vTaskDelay(5);
    }
  }
}

void setup(void)
{
  // multiplos pinos como saida
  gpio_config_t io_output =
      {
          .pin_bit_mask =
              1LLU << GPIO_NUM_38 | 1LLU << GPIO_NUM_44 | 1LLU << GPIO_NUM_43 | 1LLU << SI484_RST |
              1LLU << GPIO_NUM_9,
          .mode = GPIO_MODE_OUTPUT,
          .pull_up_en = GPIO_PULLUP_DISABLE,
          .pull_down_en = GPIO_PULLDOWN_DISABLE,
          .intr_type = GPIO_INTR_DISABLE,
      };

  gpio_config(&io_output);
  gpio_set_level(GPIO_NUM_38, true);
  eeprom.begin(EEPROM_SIZE);
  display.init();
#if TOUCHCAPACITIVE
  touch_capacitive_init(TFT_WIDTH, TFT_HEIGHT, TFT_ORIENTATION);
#endif

#if TOUCHRESISTIVE

  touch_resistive_init();

#endif

#ifdef LVGL_TASK
#if !LV_TICK_CUSTOM
  const esp_timer_create_args_t lvgl_tick_timer_args = {
      .callback = &increase_lvgl_tick,
      .arg = 0,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "lvgl_tick",
      .skip_unhandled_events = 0,
  };
  // cria um timer de 500hz e chama a função do lvgl para gerar temporização encrementando a variavel de tempo em 2ms
  esp_timer_handle_t lvgl_tick_timer = NULL;
  ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIODO_MS * 1000));
#endif
  xTaskCreate(lvgl_port_task, "LVGL", LVGL_TASK_STAK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);
#endif

  display.back_ligth(250);
  lvgl_init();
  create_screen();
  lv_scr_load(screen);

#if 0 // teste de tempo de escrita no buffer display em SPIRAM e RAM
#define LEN 64
    ESP_LOGI("TAG", "inicializando");
    uint8_t *bf;
    
    bf = (uint8_t*) heap_caps_malloc(4096 * LEN, MALLOC_CAP_INTERNAL); 
    
    if(bf){
    
    uint32_t time = esp_timer_get_time();
   
   
    for (uint32_t b = 0; b < 4096 * LEN; b++)
    {
        bf[b] = 0x1;
    }

    time = esp_timer_get_time() - time;
    ESP_LOGI(LOG_TAG, "tempo de gravacao para %dK %lu", (4096 * LEN) / 1000, time);
    uint32_t read = 0;
    time = esp_timer_get_time();
    for (uint32_t b = 0; b < 4096 * LEN; b++)
    {
        read += bf[b];
    }

    time = esp_timer_get_time() - time;
    ESP_LOGI(LOG_TAG, "tempo de leitura para %dK %lu %lu", (4096 * LEN) / 1000, time, read);

  } else{

    ESP_LOGI(LOG_TAG, "nao alocado");
  }
#endif

  SI4844_i2c_init(&SI4844_dev);
  band_init();
  SI4844_powerOn();
  dial_pwm_init();
  ini_dial_canvas();
  ESP_LOGI(LOG_TAG, "Setup ok");
}

uint32_t bf[10];

uint16_t freq = 0;
uint8_t status = 0;
uint8_t status_hold = 0;
uint8_t f5khz = 0;
void loop(void)
{

  lv_timer_handler();

  if (irqRadio)
  {
    irqRadio = 0;
    comm[0] = ATDD_GET_STATUS;

    if (SI4844_get_status())
    {
      freq = (uint16_t)(((data[2] >> 4) & 0x7) * 1000 + (data[2] & 0xf) * 100 + (data[3] >> 4) * 10 + (data[3] & 0xf));
      if (data[2] & 0x80)
      {
        f5khz = 5;
      }
      else
      {
        f5khz = 0;
      }
      status = data[0];
      if (status & 0x40)
      {
        SI4844_powerOn();
      }

      ESP_LOGI(LOG_TAG, "Status %s%s%s%s%s%s%s%s",
               (status & 0x80) ? " CTS" : "",
               (status & 0x40) ? " HOSTRST" : "",
               (status & 0x20) ? " HOSTPWRUP" : "",
               (status & 0x10) ? " INFORDY" : "",
               (status & 0x08) ? " STATION" : "",
               (status & 0x04) ? " STEREO" : "",
               (status & 0x02) ? " BCFG1" : "",
               (status & 0x01) ? " BCFG0" : "");

      if (bandIndex != 0)
      {
        freq *= 10;
        freq += f5khz;
      }

      ESP_LOGI(LOG_TAG, "dados recebidos %02X %02X %02X %02X %u", data[0], data[1], data[2], data[3], freq);
      setPosDial(map((uint32_t)freq, bands[bandIndex].dial.mapMin, bands[bandIndex].dial.mapMax, 0, 400));
      dial_canvas(freq, bands[bandIndex].freqIncial, bands[bandIndex].freqFinal, bands[bandIndex].step, bands[bandIndex].tipoBand);
    }
  }

  if (tickc < xTaskGetTickCount())
  {
    tickc = xTaskGetTickCount() + 99;
#if 0 // monitor  de memoria  
    multi_heap_info_t info;
    lv_mem_monitor_t mon;
    lv_mem_monitor(&mon);
    heap_caps_get_info(&info, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    sprintf(stringInfoMem, "used: %6d free: %6d free Heap: %ld heap used: %6d heap free: %6d", (int)(mon.total_size - mon.free_size), (int)mon.free_biggest_size, esp_get_free_heap_size(), (int)(info.total_allocated_bytes), (int)info.total_free_bytes);
    ESP_LOGI(LOG_TAG, "%s %lu", stringInfoMem, uxTaskGetStackHighWaterMark2(NULL));
   // sprintf(stringInfoMem, "heap used: %6d heap free: %6d", (int)(info.total_allocated_bytes), (int)info.total_free_bytes);
#endif
  }

  static uint32_t count = 0;
  if (count < xTaskGetTickCount())
  {
    count = xTaskGetTickCount() + 10;
    if (bandIndex == 6 || bandIndex == 5)
    {
      lv_label_set_text_fmt(labelFrequency, "%3u.%1u", freq / 100, (freq / 10) % 10);
    }
    else
    {
      lv_label_set_text_fmt(labelFrequency, "%5u", freq);
    }

    if (status_hold != status)
    {
      status_hold = status;
      if (status & 0x04)
      {
        lv_label_set_text(labelStereo, "Stereo");
      }
      else
      {
        lv_label_set_text(labelStereo, "");
      }
      if (status & 0x08)
      {
        lv_label_set_text(labelTune, "Tune");
      }
      else
      {
        lv_label_set_text(labelTune, "");
      }
    }
  }
}
