#include "mgos.h"
#include "mgos_spi.h"
#include "mgos_stmpe610.h"

static mgos_stmpe610_event_t s_event_handler = NULL;
static enum mgos_stmpe610_rotation_t s_rotation = STMPE610_PORTRAIT;
struct mgos_stmpe610_event_data s_last_touch;

static uint8_t stmpe610_spi_read_register(uint8_t reg) {
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

static void stmpe610_spi_write_register(uint8_t reg, uint8_t val) {
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

static uint8_t stmpe610_get_bufferlength(void) {
  return stmpe610_spi_read_register(STMPE_FIFO_SIZE);
}


static uint8_t stmpe610_read_data(uint16_t *x, uint16_t *y, uint8_t *z) {
  uint8_t data[4];
  uint8_t samples, cnt;
  uint32_t sum_sample_x = 0, sum_sample_y = 0;
  uint16_t sum_sample_z = 0;

  samples = cnt = stmpe610_get_bufferlength();
  LOG(LL_DEBUG, ("Touch sensed with %d samples", samples));
  if (samples == 0)
    return 0;
  
  while (cnt>0) {
    uint16_t sample_x, sample_y;
    uint8_t sample_z;
    for (uint8_t i=0; i<4; i++) {
      data[i] = stmpe610_spi_read_register(0xD7);
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
    LOG(LL_DEBUG, ("Sample at (%d,%d) pressure=%d, bufferLength=%d", sample_x, sample_y, sample_z, stmpe610_get_bufferlength()));
    cnt--;
  }
  *x = sum_sample_x / samples;
  *y = sum_sample_y / samples;
  *z = sum_sample_z / samples;

  stmpe610_spi_write_register(STMPE_FIFO_STA, STMPE_FIFO_STA_RESET); // clear FIFO
  stmpe610_spi_write_register(STMPE_FIFO_STA, 0);    // unreset

  return samples;
}

static uint16_t smpe610_get_version() {
  uint16_t version;

  version = stmpe610_spi_read_register(0);
  version <<= 8;
  version |= stmpe610_spi_read_register(1);

  LOG(LL_INFO, ("Read Version byte: 0x%04x", version));
  return version;
}

static long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  if (x<in_min) x=in_min;
  if (x>in_max) x=in_max;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static void stmpe610_map_rotation(uint16_t x, uint16_t y, uint16_t *x_out, uint16_t *y_out) {
  switch(s_rotation) {
    case STMPE610_LANDSCAPE:
      *x_out = map(y, 150, 3800, 0, 4095);
      *y_out = map(x, 250, 3700, 0, 4095);
      break;
    case STMPE610_PORTRAIT_FLIP:
      *x_out = map(x, 250, 3800, 0, 4095);
      *y_out = 4095-map(y, 150, 3700, 0, 4095);
      break;
    case STMPE610_LANDSCAPE_FLIP:
      *x_out = 4095-map(y, 150, 3800, 0, 4095);
      *y_out = 4095-map(x, 250, 3700, 0, 4095);
      break;
    default: // STMPE610_PORTRAIT
      *x_out = 4095-map(x, 250, 3800, 0, 4095);
      *y_out = map(y, 150, 3700, 0, 4095);
  }
}

/* Each time a TOUCH_DOWN event occurs, s_last_touch is populated
   and a timer is started. When the timer fires, we checke to see
   if we've sent a TOUCH_UP event already. If not, we may still be
   pressing the screen. IF we're not pressing the screen STMP610
   bufferLength() will be 0. We've now detected a DOWN without a
   corresponding UP, so we send it ourselves.
*/
static void stmpe610_down_cb(void *arg) {
  if (s_last_touch.direction == TOUCH_UP)
    return;
  if (stmpe610_get_bufferlength() > 0)
    return;

  s_last_touch.direction=TOUCH_UP;
  if (s_event_handler) {
    LOG(LL_INFO, ("Touch DOWN not followed by UP -- sending phantom UP"));
    if (s_event_handler)
      s_event_handler(&s_last_touch);
  }
}

static void stmpe610_irq(int pin, void *arg) {
  struct mgos_stmpe610_event_data ed;
  uint16_t x, y;
  uint8_t z;

  if (stmpe610_get_bufferlength()==0) {
    uint8_t i;
    LOG(LL_DEBUG, ("Touch DOWN"));
    for (i=0; i<10; i++) {
      mgos_msleep(5);
      if (stmpe610_get_bufferlength()>0) {

        stmpe610_read_data(&x, &y, &z);
        LOG(LL_DEBUG, ("Touch DOWN at (%d,%d) pressure=%d, length=%d, iteration=%d", x, y, z, ed.length, i));

        ed.length=1;
        ed.direction = TOUCH_DOWN;
        stmpe610_map_rotation(x, y, &ed.x, &ed.y);
        ed.z = z;
        // To avoid DOWN events without an UP event, set a timer (see stmpe610_down_cb for details)
        memcpy((void *)&s_last_touch, (void *)&ed, sizeof(s_last_touch));
        mgos_set_timer(100, 0, stmpe610_down_cb, NULL);
        if (s_event_handler)
          s_event_handler(&ed);
        break;
      }
    }
    stmpe610_spi_write_register(STMPE_INT_STA, 0xFF); // reset all ints
    return;
  }

  ed.length = stmpe610_read_data(&x, &y, &z);
  LOG(LL_DEBUG, ("Touch UP at (%d,%d) pressure=%d, length=%d", x, y, z, ed.length));
  ed.direction = TOUCH_UP;
  stmpe610_map_rotation(x, y, &ed.x, &ed.y);
  ed.z = z;
  memcpy((void *)&s_last_touch, (void *)&ed, sizeof(s_last_touch));
  if (s_event_handler)
    s_event_handler(&ed);

  stmpe610_spi_write_register(STMPE_INT_STA, 0xFF); // reset all ints
  (void) pin;
  (void) arg;
}


void mgos_stmpe610_set_handler(mgos_stmpe610_event_t handler) {
  s_event_handler = handler;
}

void mgos_stmpe610_set_rotation(enum mgos_stmpe610_rotation_t rotation) {
  s_rotation = rotation;
}

bool mgos_stmpe610_is_touching() {
  return s_last_touch.direction == TOUCH_DOWN;
}

bool mgos_stmpe610_spi_init(void) {
  uint16_t v;

  stmpe610_spi_write_register(STMPE_SYS_CTRL1, STMPE_SYS_CTRL1_RESET);
  mgos_msleep(10);

  v = smpe610_get_version();
  LOG(LL_INFO, ("STMPE610 init (CS%d, IRQ: %d)", mgos_sys_config_get_stmpe610_cs_index(), mgos_sys_config_get_stmpe610_irq_pin()));
  if (0x811 != v) {
    LOG(LL_ERROR, ("STMPE610 init failed (0x%04x), disabling", v));
    return true;
  }
  LOG(LL_INFO, ("STMPE610 init ok"));

  for (uint8_t i=0; i<65; i++)
    stmpe610_spi_read_register(i);

  stmpe610_spi_write_register(STMPE_SYS_CTRL2, 0x0); // turn on clocks!
  stmpe610_spi_write_register(STMPE_TSC_CTRL, STMPE_TSC_CTRL_XYZ | STMPE_TSC_CTRL_EN); // XYZ and enable!
  stmpe610_spi_write_register(STMPE_INT_EN, STMPE_INT_EN_TOUCHDET);
  stmpe610_spi_write_register(STMPE_ADC_CTRL1, STMPE_ADC_CTRL1_10BIT | (0x6 << 4)); // 96 clocks per conversion
  stmpe610_spi_write_register(STMPE_ADC_CTRL2, STMPE_ADC_CTRL2_6_5MHZ);
  stmpe610_spi_write_register(STMPE_TSC_CFG, STMPE_TSC_CFG_4SAMPLE | STMPE_TSC_CFG_DELAY_1MS | STMPE_TSC_CFG_SETTLE_5MS);
  stmpe610_spi_write_register(STMPE_TSC_FRACTION_Z, 0x6);
  stmpe610_spi_write_register(STMPE_FIFO_TH, 1);
  stmpe610_spi_write_register(STMPE_FIFO_STA, STMPE_FIFO_STA_RESET);
  stmpe610_spi_write_register(STMPE_FIFO_STA, 0);    // unreset
  stmpe610_spi_write_register(STMPE_TSC_I_DRIVE, STMPE_TSC_I_DRIVE_50MA);
  stmpe610_spi_write_register(STMPE_INT_STA, 0xFF); // reset all ints
  stmpe610_spi_write_register(STMPE_INT_CTRL, STMPE_INT_CTRL_POL_LOW | STMPE_INT_CTRL_EDGE | STMPE_INT_CTRL_ENABLE);

  mgos_gpio_set_mode(mgos_sys_config_get_stmpe610_irq_pin(), MGOS_GPIO_MODE_INPUT);
  mgos_gpio_set_pull(mgos_sys_config_get_stmpe610_irq_pin(), MGOS_GPIO_PULL_UP);
  mgos_gpio_set_int_handler(mgos_sys_config_get_stmpe610_irq_pin(), MGOS_GPIO_INT_EDGE_NEG, stmpe610_irq, NULL);
  mgos_gpio_enable_int(mgos_sys_config_get_stmpe610_irq_pin());

  // Initialize the last touch to TOUCH_UP
  s_last_touch.direction=TOUCH_UP;
  s_last_touch.x=0;
  s_last_touch.y=0;
  s_last_touch.z=0;
  s_last_touch.length=0;

  return true;
}
