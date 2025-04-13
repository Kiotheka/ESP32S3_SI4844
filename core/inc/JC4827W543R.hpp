#pragma once

#include <device.hpp>
#include <driver/gpio.h>
#include <driver/ledc.h>

#define TFT_ORIENTATION 0
#define TFT_WIDTH 480
#define TFT_HEIGHT 272

#define TOUCH_IRQ GPIO_NUM_3
#define TOUCH_MOSI GPIO_NUM_11
#define TOUCH_MISO GPIO_NUM_13
#define TOUCH_SCK GPIO_NUM_12
#define TOUCH_CS GPIO_NUM_38

#define TFT_D0 GPIO_NUM_21
#define TFT_D1 GPIO_NUM_48
#define TFT_D2 GPIO_NUM_40
#define TFT_D3 GPIO_NUM_39
#define TFT_SCK GPIO_NUM_47
#define TFT_CS GPIO_NUM_45
#define TFT_BL GPIO_NUM_1
#define SPI_MODE0 0

#define NV3041A_MADCTL 0x36
#define NV3041A_COLMOD 0x3A

#define NV3041A_MADCTL_MY 0x80
#define NV3041A_MADCTL_MX 0x40
#define NV3041A_MADCTL_MV 0x20
#define NV3041A_MADCTL_ML 0x10
#define NV3041A_MADCTL_RGB 0x00 //

#define NV3041A_CASET 0x2A
#define NV3041A_RASET 0x2B
#define NV3041A_RAMWR 0x2C

class JC4827W543R final : public device
{
// maximum for this device
#define MAX_DMA_TRANSFER 32768

public:
  void init() override
  {
    gpio_config_t pGPIOConfig;
    pGPIOConfig.mode = GPIO_MODE_OUTPUT;
    pGPIOConfig.intr_type = GPIO_INTR_DISABLE;
    pGPIOConfig.pin_bit_mask = 1ULL << TFT_CS;
    pGPIOConfig.pull_down_en = GPIO_PULLDOWN_ENABLE;
    pGPIOConfig.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&pGPIOConfig);
    bus_chip_select_disable();
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    const spi_bus_config_t bus_cfg = {
        .data0_io_num = TFT_D0,
        .data1_io_num = TFT_D1,
        .sclk_io_num = TFT_SCK,
        .data2_io_num = TFT_D2,
        .data3_io_num = TFT_D3,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .max_transfer_sz = MAX_DMA_TRANSFER,
        .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_GPIO_PINS,
        .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
        .intr_flags = 0,
    };

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK)
    {
      ESP_ERROR_CHECK(ret);
      assert(ret);
    }

    const spi_device_interface_config_t dev_cfg = {
        .command_bits = 8,
        .address_bits = 24,
        .dummy_bits = 0,
        .mode = SPI_MODE0,
        .clock_source = SPI_CLK_SRC_DEFAULT,
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = 26800000UL,
        .input_delay_ns = 0,
        .spics_io_num = -1,
        .flags = SPI_DEVICE_HALFDUPLEX,
        .queue_size = 1,
        .pre_cb = pre_transaction_cb,
        .post_cb = post_transaction_cb,

    };
    ret = spi_bus_add_device(SPI2_HOST, &dev_cfg, &device_handle_);
    if (ret != ESP_OK)
    {
      ESP_ERROR_CHECK(ret);
      assert(ret);
    }
    // init values that will not change
    transaction_async_.cmd = 0x32;
    transaction_async_.addr = 0x003C00;
    transaction_async_.flags = SPI_TRANS_MODE_QIO;
    transaction_async_.user = this;

    constexpr uint8_t init_commands[] = {

        0xff,
        0xa5,

        0x36, // MACTL
        0xc0,

        0x3A, //
        0x01, // 01---565，00---666

        0x41,
        0x03, // 01--8bit, 03-16bit

        0x44, // VBP  ?????
        0x15, // 21

        0x45, // VFP  ?????
        0x15, // 21

        0x7d, // vdds_trim[2:0]
        0x03,

        0xc1, // avdd_clp_en avdd_clp[1:0] avcl_clp_en avcl_clp[1:0]
        0xab, // 0xbb	 88		  a2

        0xc2, // vgl_clp_en vgl_clp[2:0]
        0x17,

        0xc3, // vgl_clp_en vgl_clp[2:0]
        0x10,

        0xc6, // avdd_ratio_sel avcl_ratio_sel vgh_ratio_sel[1:0] vgl_ratio_sel[1:0]
        0x3a, // 35

        0xc7, // mv_clk_sel[1:0] avdd_clk_sel[1:0] avcl_clk_sel[1:0]
        0x25, // 2e

        0xc8, //	VGL_CLK_sel
        0x11, //

        0x7a, //	user_vgsp
        0x49, // 4f:0.8V		3f:1.04V	5f

        0x6f, //	user_gvdd
        0x2f, // 1C:5.61	  5f	 53		   2a	    3a

        0x78, //	user_gvcl
        0x4b, // 50:-3.22	  75			58	     	66

        0xc9, //
        0x00,

        0x67, //
        0x33,

        // gate_ed

        0x51, // gate_st_o[7:0]
        0x4b,

        0x52, // gate_ed_o[7:0]
        0x7c, // 76

        0x53, // gate_st_e[7:0]
        0x1c, // 76

        0x54, // gate_ed_e[7:0]
        0x77,
        ////sorce
        0x46, // fsm_hbp_o[5:0]
        0x0a,

        0x47, // fsm_hfp_o[5:0]
        0x2a,

        0x48, // fsm_hbp_e[5:0]
        0x0a,

        0x49, // fsm_hfp_e[5:0]
        0x1a,

        0x56, // src_ld_wd[1:0] src_ld_st[5:0]
        0x43,

        0x57, // pn_cs_en src_cs_st[5:0]
        0x42,

        0x58, // src_cs_p_wd[6:0]
        0x3c,

        0x59, // src_cs_n_wd[6:0]
        0x64,

        0x5a, // src_pchg_st_o[6:0]
        0x41, // 41

        0x5b, // src_pchg_wd_o[6:0]
        0x3c,

        0x5c, // src_pchg_st_e[6:0]
        0x02, // 02

        0x5d, // src_pchg_wd_e[6:0]
        0x3c, // 3c

        0x5e, // src_pol_sw[7:0]
        0x1f,

        0x60, // src_op_st_o[7:0]
        0x80,

        0x61, // src_op_st_e[7:0]
        0x3f,

        0x62, // src_op_ed_o[9:8] src_op_ed_e[9:8]
        0x21,

        0x63, // src_op_ed_o[7:0]
        0x07,

        0x64, // src_op_ed_e[7:0]
        0xe0,

        0x65, // chopper
        0x01,

        0xca, // avdd_mux_st_o[7:0]
        0x20,

        0xcb, // avdd_mux_ed_o[7:0]
        0x52, // 52

        0xcc, // avdd_mux_st_e[7:0]
        0x10,

        0xcD, // avdd_mux_ed_e[7:0]
        0x42,

        0xD0, // avcl_mux_st_o[7:0]
        0x20,

        0xD1, // avcl_mux_ed_o[7:0]
        0x52,

        0xD2, // avcl_mux_st_e[7:0]
        0x10,

        0xD3, // avcl_mux_ed_e[7:0]
        0x42,

        0xD4, // vgh_mux_st[7:0]
        0x0a,

        0xD5, // vgh_mux_ed[7:0]
        0x32,

        // 2-1
        ////gammma  weihuan pianguangpian 0913
        0x80, // gam_vrp0	0					6bit
        0x04,
        0xA0, // gam_VRN0		 0-
        0x00,

        0x81, // gam_vrp1	1				   6bit
        0x07,
        0xA1, // gam_VRN1		 1-
        0x05,

        0x82, // gam_vrp2	 2					6bit
        0x06,
        0xA2, // gam_VRN2		 2-
        0x04,

        0x86, // gam_prp0	 7bit	8			7bit
        0x2c, // 33
        0xA6, // gam_PRN0	 	8-
        0x2a, // 2a

        0x87, // gam_prp1	 7bit	 40			 7bit
        0x46, // 2d
        0xA7, // gam_PRN1	 	40-
        0x44, // 2d

        0x83, // gam_vrp3	 61				 6bit
        0x39,
        0xA3, // gam_VRN3		61-
        0x39,

        0x84, // gam_vrp4	  62			 6bit
        0x3a,
        0xA4, // gam_VRN4		62-
        0x3a,

        0x85, // gam_vrp5	  63			 6bit
        0x3f,
        0xA5, // gam_VRN5		63-
        0x3f,
        //

        0x88, // gam_pkp0	  	 4			   5bit
        0x08, // 0b
        0xA8, // gam_PKN0		4-
        0x08, // 0b

        0x89, // gam_pkp1	  5					5bit
        0x0f, // 14
        0xA9, // gam_PKN1		5-
        0x0f, // 14

        0x8a, // gam_pkp2	  7					 5bit
        0x17, // 1a
        0xAa, // gam_PKN2		7-
        0x17, // 1a

        0x8b, // gam_PKP3	  10				 5bit
        0x10,
        0xAb, // gam_PKN3		10-
        0x10,

        0x8c, // gam_PKP4	   16				 5bit
        0x16,
        0xAc, // gam_PKN4		16-
        0x16,

        0x8d, // gam_PKP5		22				 5bit
        0x14,
        0xAd, // gam_PKN5		22-
        0x14,

        0x8e, // gam_PKP6		28				 5bit
        0x11, // 16 change
        0xAe, // gam_PKN6		28-
        0x11, // 13change

        0x8f, // gam_PKP7		34				  5bit
        0x14,
        0xAf, // gam_PKN7		34-
        0x14,

        0x90, // gam_PKP8		 46				   5bit
        0x06,
        0xB0, // gam_PKN8		46-
        0x06,

        0x91, // gam_PKP9		 52					5bit
        0x0f,
        0xB1, // gam_PKN9		52-
        0x0f,

        0x92, // gam_PKP10		58					5bit
        0x16,
        0xB2, // gam_PKN10		58-
        0x16,

        0xff,
        0x00,

        0x11,
        0x00,
    };

    for (int i = 0; i < sizeof(init_commands); i += 2)
    {
      bus_write_c8d8(init_commands[i], init_commands[i + 1]);
    }
    vTaskDelay(pdMS_TO_TICKS(200));

    bus_write_c8(0x21);         // inversion off
    bus_write_c8d8(0x29, 0x00); // turn on display

    set_rotation(TFT_ORIENTATION);
    set_write_address_window(0, 0, TFT_WIDTH, TFT_HEIGHT);
    gpio_set_level(TFT_BL, 0);
  }

  void back_ligth(uint8_t value)
  {
    ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, value);
    ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
  }

  bool is_display_touched() override
  {
    return 1; // touch_screen.tirqTouched() && touch_screen.touched();
  }

  uint8_t comm[3];
  void GT911Ini(GT911_t *dev)
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
    dev_cfg.scl_speed_hz = dev->_frequency;

    ESP_ERROR_CHECK(i2c_master_bus_add_device(dev->_bus_handle, &dev_cfg, &dev->_device_handle));

    if (dev->_int > 0)
    {
      gpio_config_t int_GT911 =
          {
              .pin_bit_mask = 1LLU << dev->_int,
              .mode = GPIO_MODE_INPUT,
              .pull_up_en = GPIO_PULLUP_DISABLE,
              .pull_down_en = GPIO_PULLDOWN_DISABLE,
              .intr_type = GPIO_INTR_DISABLE,
          };

      gpio_config(&int_GT911);
    }
    if (dev->_reset > 0)
    {
      gpio_config_t reset_GT911 =
          {
              .pin_bit_mask = 1LLU << dev->_reset,
              .mode = GPIO_MODE_OUTPUT,
              .pull_up_en = GPIO_PULLUP_DISABLE,
              .pull_down_en = GPIO_PULLDOWN_DISABLE,
              .intr_type = GPIO_INTR_DISABLE,
          };

      gpio_config(&reset_GT911);
      gpio_set_level(dev->_reset, false);
      vTaskDelay(1);
      gpio_set_level(dev->_reset, true);
      vTaskDelay(2);
    }
    uint8_t data[3];
    comm[0] = GT911_COMMAND >> 8;
    comm[1] = (uint8_t)GT911_COMMAND;
    comm[2] = 0x02;
    i2c_master_transmit(dev->_device_handle, data, 3, 20);

    vTaskDelay(2);
  }

  uint8_t raw_data[40];
  bool GT911Read(GT911_t *dev)
  {
    comm[0] = GT911_POINT_INFO >> 8;
    comm[1] = (uint8_t)GT911_POINT_INFO;
    i2c_master_transmit_receive(dev->_device_handle, comm, 2, raw_data, 8, 30);
    comm[0] = GT911_POINT_INFO >> 8;
    comm[1] = (uint8_t)GT911_POINT_INFO;
    comm[2] = 0;
    i2c_master_transmit(dev->_device_handle, comm, 3, 20);
    return (raw_data[0] & 0xF) != 0 ? true : false;
  }

  TP_Point GT911GetPoint(GT911_t *dev, uint8_t n)
  {
    uint8_t rotation = 0;
    TP_Point t;
    uint16_t point_reg[5] = GT911_POINTS_REG;
    uint16_t offset = point_reg[n] - GT911_POINT_INFO;
    t.id = raw_data[offset];
    t.x = raw_data[offset + 1] + (raw_data[offset + 2] << 8);
    t.y = raw_data[offset + 3] + (raw_data[offset + 4] << 8);
    t.size = raw_data[offset + 5] | (raw_data[offset + 6] << 8);

    if (rotation == 0)
    {
    }
    else if (rotation == 1)
    {
      uint16_t tmp = t.x;
      t.x = t.y;
      t.y = tmp;
    }
    // ESP_LOGI(LOG_TAG, "x=%u y=%u", t.x, t.y);
    return t;
  }

////////////////////// touch

// XPT2046_t dev;
#define TAG "XPT2046"
#define MAX_LEN 3
#define XPT_START 0x80
#define XPT_Z2POS 0x40
#define XPT_Z1POS 0x30
#define XPT_XPOS 0x50
#define XPT_YPOS 0x10
#define XPT_8BIT 0x80
#define XPT_SER 0x04

  void
  XPT2046Ini(XPT2046_t *dev)
  {

    esp_err_t ret;
    spi_bus_config_t xpt_buscfg = {};
    xpt_buscfg.mosi_io_num = dev->_mosi;
    xpt_buscfg.miso_io_num = dev->_miso;
    xpt_buscfg.sclk_io_num = dev->_sclk;
    xpt_buscfg.quadwp_io_num = -1;
    xpt_buscfg.quadhd_io_num = -1;
    xpt_buscfg.data4_io_num = -1;
    xpt_buscfg.data5_io_num = -1;
    xpt_buscfg.data6_io_num = -1;
    xpt_buscfg.data7_io_num = -1;
    xpt_buscfg.max_transfer_sz = 3;
    xpt_buscfg.flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_GPIO_PINS;
    xpt_buscfg.isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO;
    xpt_buscfg.intr_flags = 0;

    ret = spi_bus_initialize(dev->_host, &xpt_buscfg, SPI_DMA_CH_AUTO);
    assert(ret == ESP_OK);

    gpio_reset_pin(dev->_cs);
    gpio_set_direction(dev->_cs, GPIO_MODE_OUTPUT);
    gpio_set_level(dev->_cs, true);
    if (dev->_irq != -1)
    {

      gpio_config_t io_conf = {};
      io_conf.intr_type = GPIO_INTR_DISABLE;
      io_conf.pin_bit_mask = (1ULL << dev->_irq);
      io_conf.mode = GPIO_MODE_INPUT;
      io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
      gpio_config(&io_conf);
    }

    spi_device_interface_config_t xpt_devcfg = {};
    xpt_devcfg.clock_speed_hz = dev->_frequency;
    xpt_devcfg.spics_io_num = dev->_cs;
    xpt_devcfg.flags = SPI_DEVICE_NO_DUMMY;
    xpt_devcfg.queue_size = 7;

    spi_device_handle_t xpt_handle;
    ret = spi_bus_add_device(dev->_host, &xpt_devcfg, &xpt_handle);
    dev->_handle = xpt_handle;
    dev->_calibration = true;
  }

  int xptGetit(XPT2046_t *dev, int cmd)
  {
    char rbuf[MAX_LEN];
    char wbuf[MAX_LEN];

    memset(wbuf, 0, sizeof(rbuf));
    memset(rbuf, 0, sizeof(rbuf));
    wbuf[0] = cmd;
    spi_transaction_t SPITransaction;
    esp_err_t ret;

    memset(&SPITransaction, 0, sizeof(spi_transaction_t));
    SPITransaction.length = MAX_LEN * 8;
    SPITransaction.tx_buffer = wbuf;
    SPITransaction.rx_buffer = rbuf;
#if 1
    ret = spi_device_transmit(dev->_handle, &SPITransaction);
#else
    ret = spi_device_polling_transmit(dev->_XPT_Handle, &SPITransaction);
#endif
    assert(ret == ESP_OK);
    int pos = ((rbuf[1] << 8) + rbuf[2]) >> 3;
    return (pos);
  }

  static int16_t besttwoavg(int16_t x, int16_t y, int16_t z)
  {
    int16_t da, db, dc;
    int16_t reta = 0;
    if (x > y)
      da = x - y;
    else
      da = y - x;
    if (x > z)
      db = x - z;
    else
      db = z - x;
    if (z > y)
      dc = z - y;
    else
      dc = y - z;

    if (da <= db && da <= dc)
      reta = (x + y) >> 1;
    else if (db <= da && db <= dc)
      reta = (x + z) >> 1;
    else
      reta = (y + z) >> 1;

    return (reta);
  }

  bool touch_getxy(XPT2046_t *dev)
  {
    int level = gpio_get_level(dev->_irq);
    if (level == 1)
      return false;
    uint16_t z, z1, z2;
    int16_t data[6];
    z1 = xptGetit(dev, (XPT_START | XPT_Z1POS));
    z = z1 + 4095;
    z2 = xptGetit(dev, (XPT_START | XPT_Z2POS));
    z -= z2;
    if (z > 300)
    {
      data[0] = xptGetit(dev, (XPT_START | XPT_XPOS));
      data[1] = xptGetit(dev, (XPT_START | XPT_YPOS));
      data[2] = xptGetit(dev, (XPT_START | XPT_XPOS));
      data[3] = xptGetit(dev, (XPT_START | XPT_YPOS));
      data[4] = xptGetit(dev, (XPT_START | XPT_XPOS));
      data[5] = xptGetit(dev, (XPT_START | XPT_YPOS));
      dev->_xp = besttwoavg(data[0], data[2], data[4]);
      dev->_yp = besttwoavg(data[1], data[3], data[5]);
      dev->_zp = z;
    }
    else
    {
      dev->_xp = 0;
      dev->_yp = 0;
      dev->_zp = 0;
      return false;
    }
    return true;
  }

  bool asyncDMAIsBusy() override
  {
    if (!async_busy_)
    {
      return false;
    }

    spi_transaction_t *t = nullptr;
    async_busy_ =
        spi_device_get_trans_result(device_handle_, &t, 0) == ESP_ERR_TIMEOUT;

    return async_busy_;
  }

  void asyncDMAWaitForCompletion() override
  {
    if (!async_busy_)
    {
      return;
    }

    spi_transaction_t *t = nullptr;
    assert(spi_device_get_trans_result(device_handle_, &t, portMAX_DELAY) == ESP_OK);

    async_busy_ = false;
  }

  void asyncDMAWriteBytes(uint8_t *data, uint32_t len) override
  {
    transaction_async_.tx_buffer = data;
    transaction_async_.length = len * 8;
    assert(spi_device_queue_trans(device_handle_, &transaction_async_, portMAX_DELAY) == ESP_OK);
    async_busy_ = true;
  }

  void set_write_address_window(int16_t x, int16_t y, uint16_t w, uint16_t h)
  {
    bus_write_c8d16d16(NV3041A_CASET, x, x + w - 1);
    bus_write_c8d16d16(NV3041A_RASET, y, y + h - 1);
    bus_write_c8(NV3041A_RAMWR);
  }

  void bus_write_c8d8(uint8_t cmd, uint8_t data)
  {
    transaction_.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_MULTILINE_CMD | SPI_TRANS_MULTILINE_ADDR;
    transaction_.cmd = 0x02;
    transaction_.addr = static_cast<uint32_t>(cmd) << 8;
    transaction_.tx_data[0] = data;
    transaction_.length = 8;
    assert(spi_device_polling_transmit(device_handle_, &transaction_) ==
           ESP_OK);
  }

private:
  ledc_timer_config_t ledc_timer = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .duty_resolution = LEDC_TIMER_8_BIT,
      .timer_num = LEDC_TIMER_0,
      .freq_hz = 1000,
      .clk_cfg = LEDC_AUTO_CLK,
      .deconfigure = 0,

  };
  ledc_channel_config_t

      ledc_channel = {
          .gpio_num = TFT_BL,
          .speed_mode = LEDC_LOW_SPEED_MODE,
          .channel = LEDC_CHANNEL_0,
          .intr_type = LEDC_INTR_DISABLE,
          .timer_sel = LEDC_TIMER_0,
          .duty = 0,
          .hpoint = 0,
          .flags = 0,
      };

  static void pre_transaction_cb(spi_transaction_t *trans)
  {
    JC4827W543R *dev = static_cast<JC4827W543R *>(trans->user);
    dev->bus_chip_select_enable();
  }

  static void post_transaction_cb(spi_transaction_t *trans)
  {
    JC4827W543R *dev = static_cast<JC4827W543R *>(trans->user);
    dev->bus_chip_select_disable();
  }

  void bus_chip_select_enable()
  {
    gpio_set_level(TFT_CS, 0);
  }
  void bus_chip_select_disable()
  {
    gpio_set_level(TFT_CS, 1);
  }

  void bus_write_c8(uint8_t cmd)
  {
    transaction_.flags = SPI_TRANS_MULTILINE_CMD | SPI_TRANS_MULTILINE_ADDR;
    transaction_.cmd = 0x02;
    transaction_.addr = static_cast<uint32_t>(cmd) << 8;
    transaction_.tx_buffer = NULL;
    transaction_.length = 0;
    assert(spi_device_polling_transmit(device_handle_, &transaction_) == ESP_OK);
  }

  void bus_write_c8d16d16(uint8_t cmd, uint16_t data1, uint16_t data2)
  {
    transaction_.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_MULTILINE_CMD | SPI_TRANS_MULTILINE_ADDR;
    transaction_.cmd = 0x02;
    transaction_.addr = static_cast<uint32_t>(cmd) << 8;
    transaction_.tx_data[0] = data1 >> 8;
    transaction_.tx_data[1] = data1;
    transaction_.tx_data[2] = data2 >> 8;
    transaction_.tx_data[3] = data2;
    transaction_.length = 32;
    assert(spi_device_polling_transmit(device_handle_, &transaction_) == ESP_OK);
  }

  void set_rotation(uint8_t r)
  {
    switch (r)
    {
    case 1:
      r = NV3041A_MADCTL_MY | NV3041A_MADCTL_MV | NV3041A_MADCTL_RGB;
      break;
    case 2:
      r = NV3041A_MADCTL_MX | NV3041A_MADCTL_MY | NV3041A_MADCTL_RGB;
      break;
    case 3:
      r = NV3041A_MADCTL_MX | NV3041A_MADCTL_MV | NV3041A_MADCTL_RGB;
      break;
    default:
      r = NV3041A_MADCTL_RGB;
      break;
    }
    bus_write_c8d8(NV3041A_MADCTL, r);
  }
  spi_host_device_t host_device_{};
  spi_device_handle_t device_handle_{};
  spi_transaction_t transaction_{};
  spi_transaction_t transaction_async_{};
  bool async_busy_ = false;
};