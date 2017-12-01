#include "mgos.h"
#include "mgos_spi.h"
#include "mgos_STMPE610.h"

static mgos_stmpe610_event_t s_event_handler = NULL;

static uint8_t readRegister8(uint8_t reg) {
  struct mgos_spi *spi = mgos_spi_get_global();
  if (!spi) {
    LOG(LL_ERROR, ("Cannot get global SPI bus"));
    return 0;
  }

  uint8_t tx_data = 0x80 | reg;
  uint8_t rx_data;

  struct mgos_spi_txn txn = {
      .cs = mgos_sys_config_get_stmpe610_cs_index(),
      .mode = 0,
      .freq = 1000000,
  };
  txn.hd.tx_len = 1;
  txn.hd.tx_data = &tx_data;
  txn.hd.dummy_len = 0;
  txn.hd.rx_len = 1;
  txn.hd.rx_data = &rx_data;
  if (!mgos_spi_run_txn(spi, false, &txn)) {
    LOG(LL_ERROR, ("SPI transaction failed"));
    return 0;
  }
  return rx_data;
}

static void writeRegister8(uint8_t reg, uint8_t val) {
  struct mgos_spi *spi = mgos_spi_get_global();
  if (!spi) {
    LOG(LL_ERROR, ("Cannot get global SPI bus"));
    return;
  }

  uint8_t tx_data[2] = {reg, val};

  struct mgos_spi_txn txn = {
      .cs = mgos_sys_config_get_stmpe610_cs_index(),
      .mode = 0,
      .freq = 1000000,
  };
  txn.hd.tx_data = tx_data;
  txn.hd.tx_len = sizeof(tx_data);
  txn.hd.dummy_len = 0;
  txn.hd.rx_len = 0;
  if (!mgos_spi_run_txn(spi, false, &txn)) {
    LOG(LL_ERROR, ("SPI transaction failed"));
    return;
  }
}

static uint8_t mgos_stmpe610_bufferLength(void) {
  return readRegister8(STMPE_FIFO_SIZE);
}


static uint8_t mgos_stmpe610_readData(uint16_t *x, uint16_t *y, uint8_t *z) {
  uint8_t data[4];
  uint8_t samples, cnt;
  uint32_t sum_sample_x = 0, sum_sample_y = 0;
  uint16_t sum_sample_z = 0;

  samples = cnt = mgos_stmpe610_bufferLength();
  LOG(LL_DEBUG, ("Touch sensed with %d samples", samples));
  if (samples == 0)
    return 0;
  
  while (cnt>0) {
    uint16_t sample_x, sample_y;
    uint8_t sample_z;
    for (uint8_t i=0; i<4; i++) {
      data[i] = readRegister8(0xD7);
    }
    sample_x = data[0];
    sample_x <<= 4;
    sample_x |= (data[1] >> 4);
    sample_y = data[1] & 0x0F; 
    sample_y <<= 8;
    sample_y |= data[2]; 
    sample_z = data[3];
    sum_sample_x += sample_x;
    sum_sample_y += sample_y;
    sum_sample_z += sample_z;
    LOG(LL_DEBUG, ("Sample at (%d,%d) pressure=%d, bufferLength=%d", sample_x, sample_y, sample_z, mgos_stmpe610_bufferLength()));
    cnt--;
  }
  *x = sum_sample_x / samples;
  *y = sum_sample_y / samples;
  *z = sum_sample_z / samples;

  writeRegister8(STMPE_FIFO_STA, STMPE_FIFO_STA_RESET); // clear FIFO
  writeRegister8(STMPE_FIFO_STA, 0);    // unreset

  return samples;
}

static uint16_t mgos_stmpe610_getVersion() {
  uint16_t version;

  version = readRegister8(0);
  version <<= 8;
  version |= readRegister8(1);

  LOG(LL_INFO, ("Read Version byte: 0x%04x", version));
  return version;
}

static void STMPE610_irq(int pin, void *arg) {
  struct mgos_stmpe610_event_data ed;

  if (mgos_stmpe610_bufferLength()==0) {
    uint8_t i;
    LOG(LL_DEBUG, ("Touch DOWN"));
    for (i=0; i<10; i++) {
      mgos_msleep(5);
      if (mgos_stmpe610_bufferLength()>0) {
        mgos_stmpe610_readData(&ed.x, &ed.y, &ed.z);
        ed.length=1;
        ed.direction = TOUCH_DOWN;
        LOG(LL_DEBUG, ("Touch DOWN at (%d,%d) pressure=%d, length=%d, iteration=%d", ed.x, ed.y, ed.z, ed.length, i));
        if (s_event_handler)
          s_event_handler(&ed);
        break;
      }
    }
    writeRegister8(STMPE_INT_STA, 0xFF); // reset all ints
    return;
  }

  ed.length = mgos_stmpe610_readData(&ed.x, &ed.y, &ed.z);
  ed.direction = TOUCH_UP;
  LOG(LL_DEBUG, ("Touch UP at (%d,%d) pressure=%d, length=%d", ed.x, ed.y, ed.z, ed.length));
  if (s_event_handler)
    s_event_handler(&ed);

  writeRegister8(STMPE_INT_STA, 0xFF); // reset all ints
  (void) pin;
  (void) arg;
}

void mgos_stmpe610_set_handler(mgos_stmpe610_event_t handler) {
  s_event_handler = handler;
}

bool mgos_stmpe610_spi_init(void) {
  uint16_t v;

  writeRegister8(STMPE_SYS_CTRL1, STMPE_SYS_CTRL1_RESET);
  mgos_msleep(10);

  v = mgos_stmpe610_getVersion();
  LOG(LL_INFO, ("STMPE610 init (CS%d, IRQ: %d)", mgos_sys_config_get_stmpe610_cs_index(), mgos_sys_config_get_stmpe610_irq_pin()));
  if (0x811 != v) {
    LOG(LL_ERROR, ("STMPE610 init failed (0x%04x), disabling", v));
    return true;
  }
  LOG(LL_INFO, ("STMPE610 init ok"));

  for (uint8_t i=0; i<65; i++)
    readRegister8(i);

  writeRegister8(STMPE_SYS_CTRL2, 0x0); // turn on clocks!
  writeRegister8(STMPE_TSC_CTRL, STMPE_TSC_CTRL_XYZ | STMPE_TSC_CTRL_EN); // XYZ and enable!
  writeRegister8(STMPE_INT_EN, STMPE_INT_EN_TOUCHDET);
  writeRegister8(STMPE_ADC_CTRL1, STMPE_ADC_CTRL1_10BIT | (0x6 << 4)); // 96 clocks per conversion
  writeRegister8(STMPE_ADC_CTRL2, STMPE_ADC_CTRL2_6_5MHZ);
  writeRegister8(STMPE_TSC_CFG, STMPE_TSC_CFG_4SAMPLE | STMPE_TSC_CFG_DELAY_1MS | STMPE_TSC_CFG_SETTLE_5MS);
  writeRegister8(STMPE_TSC_FRACTION_Z, 0x6);
  writeRegister8(STMPE_FIFO_TH, 1);
  writeRegister8(STMPE_FIFO_STA, STMPE_FIFO_STA_RESET);
  writeRegister8(STMPE_FIFO_STA, 0);    // unreset
  writeRegister8(STMPE_TSC_I_DRIVE, STMPE_TSC_I_DRIVE_50MA);
  writeRegister8(STMPE_INT_STA, 0xFF); // reset all ints
  writeRegister8(STMPE_INT_CTRL, STMPE_INT_CTRL_POL_LOW | STMPE_INT_CTRL_EDGE | STMPE_INT_CTRL_ENABLE);

  mgos_gpio_set_mode(mgos_sys_config_get_stmpe610_irq_pin(), MGOS_GPIO_MODE_INPUT);
  mgos_gpio_set_pull(mgos_sys_config_get_stmpe610_irq_pin(), MGOS_GPIO_PULL_UP);
  mgos_gpio_set_int_handler(mgos_sys_config_get_stmpe610_irq_pin(), MGOS_GPIO_INT_EDGE_NEG, STMPE610_irq, NULL);
  mgos_gpio_enable_int(mgos_sys_config_get_stmpe610_irq_pin());

  return true;
}
