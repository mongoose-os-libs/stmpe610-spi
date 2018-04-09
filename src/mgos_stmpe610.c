/*
 * Copyright 2017 Google Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mgos.h"
#include "mgos_spi.h"
#include "mgos_stmpe610.h"

static mgos_stmpe610_event_t s_event_handler = NULL;
static uint8_t s_orientation = STMPE_ORIENTATION_X | STMPE_ORIENTATION_Y | STMPE_ORIENTATION_SWAP_NONE;
struct mgos_stmpe610_event_data s_last_touch;
static uint16_t s_max_x = 240;
static uint16_t s_max_y = 320;

static uint8_t stmpe610_spi_read_register(uint8_t reg) {
  struct mgos_spi *spi = mgos_spi_get_global();

  if (!spi) {
    LOG(LL_ERROR, ("Cannot get global SPI bus"));
    return 0;
  }

  uint8_t tx_data = 0x80 | reg;
  uint8_t rx_data;

  struct mgos_spi_txn txn = {
    .cs   = mgos_sys_config_get_stmpe610_cs_index(),
    .mode = 3,
    .freq = 1000000,
  };
  txn.hd.tx_len    = 1;
  txn.hd.tx_data   = &tx_data;
  txn.hd.dummy_len = 0;
  txn.hd.rx_len    = 1;
  txn.hd.rx_data   = &rx_data;
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

  uint8_t tx_data[2] = { reg, val };

  struct mgos_spi_txn txn = {
    .cs   = mgos_sys_config_get_stmpe610_cs_index(),
    .mode = 3,
    .freq = 1000000,
  };
  txn.hd.tx_data   = tx_data;
  txn.hd.tx_len    = sizeof(tx_data);
  txn.hd.dummy_len = 0;
  txn.hd.rx_len    = 0;
  if (!mgos_spi_run_txn(spi, false, &txn)) {
    LOG(LL_ERROR, ("SPI transaction failed"));
    return;
  }
}

static uint8_t stmpe610_get_bufferlength(void) {
  return stmpe610_spi_read_register(STMPE_FIFO_SIZE);
}

static long map(long x, long in_min, long in_max, long out_min, long out_max) {
  if (x < in_min) {
    x = in_min;
  }
  if (x > in_max) {
    x = in_max;
  }
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static uint8_t stmpe610_read_data(uint16_t *x, uint16_t *y, uint8_t *z) {
  uint8_t  data[4];
  uint8_t  samples, cnt;
  uint32_t sum_sample_x = 0, sum_sample_y = 0;
  uint16_t sum_sample_z = 0;

  samples = cnt = stmpe610_get_bufferlength();
  LOG(LL_DEBUG, ("Touch sensed with %d samples", samples));
  if (samples == 0) {
    return 0;
  }

  while (cnt > 0) {
    uint16_t sample_coord1, sample_coord2;
    uint8_t  sample_z;
    for (uint8_t i = 0; i < 4; i++) {
      data[i] = stmpe610_spi_read_register(0xD7);
    }
    sample_coord1   = data[0];
    sample_coord1 <<= 4;
    sample_coord1  |= (data[1] >> 4);
    if (s_orientation & STMPE_ORIENTATION_FLIP_X) {
      sample_coord1 = 4096 - sample_coord1;
    }
    sample_coord2   = data[1] & 0x0F;
    sample_coord2 <<= 8;
    sample_coord2  |= data[2];
    if (s_orientation & STMPE_ORIENTATION_FLIP_Y) {
      sample_coord2 = 4096 - sample_coord2;
    }
    if (s_orientation & STMPE_ORIENTATION_SWAP_XY) {
      uint32_t sample_swap = sample_coord1;
      sample_coord1 = sample_coord2;
      sample_coord2 = sample_swap;
    }
    sample_z      = data[3];
    sum_sample_x += sample_coord1;
    sum_sample_y += sample_coord2;
    sum_sample_z += sample_z;
    LOG(LL_DEBUG, ("Sample at (%d,%d) pressure=%d, bufferLength=%d", sample_coord1, sample_coord2, sample_z, stmpe610_get_bufferlength()));
    cnt--;
  }
  *x = map(sum_sample_x / samples, 150, 3800, 0, s_max_x);
  *y = map(sum_sample_y / samples, 150, 3800, 0, s_max_y);
  *z = sum_sample_z / samples;

  stmpe610_spi_write_register(STMPE_FIFO_STA, STMPE_FIFO_STA_RESET); // clear FIFO
  stmpe610_spi_write_register(STMPE_FIFO_STA, 0);                    // unreset

  return samples;
}

static uint16_t smpe610_get_version() {
  uint16_t version;

  version   = stmpe610_spi_read_register(0);
  version <<= 8;
  version  |= stmpe610_spi_read_register(1);

  LOG(LL_INFO, ("Read Version byte: 0x%04x", version));
  return version;
}

/* Each time a TOUCH_DOWN event occurs, s_last_touch is populated
 * and a timer is started. When the timer fires, we checke to see
 * if we've sent a TOUCH_UP event already. If not, we may still be
 * pressing the screen. IF we're not pressing the screen STMP610
 * bufferLength() will be 0. We've now detected a DOWN without a
 * corresponding UP, so we send it ourselves.
 */
static void stmpe610_down_cb(void *arg) {
  if (s_last_touch.direction == TOUCH_UP) {
    return;
  }
  if (stmpe610_get_bufferlength() > 0) {
    return;
  }

  s_last_touch.direction = TOUCH_UP;
  if (s_event_handler) {
    LOG(LL_INFO, ("Touch DOWN not followed by UP -- sending phantom UP"));
    if (s_event_handler) {
      s_event_handler(&s_last_touch);
    }
  }
  (void)arg;
}

static void stmpe610_irq(int pin, void *arg) {
  struct mgos_stmpe610_event_data ed;

  if (stmpe610_get_bufferlength() == 0) {
    uint8_t i;
    LOG(LL_DEBUG, ("Touch DOWN"));
    for (i = 0; i < 10; i++) {
      mgos_msleep(5);
      if (stmpe610_get_bufferlength() > 0) {
        stmpe610_read_data(&ed.x, &ed.y, &ed.z);
        LOG(LL_DEBUG, ("Touch DOWN at (%d,%d) pressure=%d, length=%d, iteration=%d", ed.x, ed.y, ed.z, ed.length, i));

        ed.length    = 1;
        ed.direction = TOUCH_DOWN;
        // To avoid DOWN events without an UP event, set a timer (see stmpe610_down_cb for details)
        memcpy((void *)&s_last_touch, (void *)&ed, sizeof(s_last_touch));
        mgos_set_timer(100, 0, stmpe610_down_cb, NULL);
        if (s_event_handler) {
          s_event_handler(&ed);
        }
        break;
      }
    }
    stmpe610_spi_write_register(STMPE_INT_STA, 0xFF); // reset all ints
    return;
  }

  ed.length = stmpe610_read_data(&ed.x, &ed.y, &ed.z);
  LOG(LL_DEBUG, ("Touch UP at (%d,%d) pressure=%d, length=%d", ed.x, ed.y, ed.z, ed.length));
  ed.direction = TOUCH_UP;
  memcpy((void *)&s_last_touch, (void *)&ed, sizeof(s_last_touch));
  if (s_event_handler) {
    s_event_handler(&ed);
  }

  stmpe610_spi_write_register(STMPE_INT_STA, 0xFF); // reset all ints
  (void)pin;
  (void)arg;
}

void mgos_stmpe610_set_handler(mgos_stmpe610_event_t handler) {
  s_event_handler = handler;
}

void mgos_stmpe610_set_orientation(uint8_t flags) {
  s_orientation = flags;
}

bool mgos_stmpe610_is_touching() {
  return s_last_touch.direction == TOUCH_DOWN;
}

void mgos_stmpe610_set_dimensions(uint16_t x, uint16_t y) {
  s_max_x = x;
  s_max_y = y;
}

bool mgos_stmpe610_spi_init(void) {
  uint16_t v;

  mgos_gpio_set_mode(mgos_sys_config_get_stmpe610_irq_pin(), MGOS_GPIO_MODE_INPUT);
  mgos_gpio_set_pull(mgos_sys_config_get_stmpe610_irq_pin(), MGOS_GPIO_PULL_UP);
  mgos_gpio_set_int_handler(mgos_sys_config_get_stmpe610_irq_pin(), MGOS_GPIO_INT_EDGE_NEG, stmpe610_irq, NULL);
  mgos_gpio_enable_int(mgos_sys_config_get_stmpe610_irq_pin());

  stmpe610_spi_write_register(STMPE_SYS_CTRL1, STMPE_SYS_CTRL1_RESET);
  mgos_msleep(10);

  v = smpe610_get_version();
  LOG(LL_INFO, ("STMPE610 init (CS%d, IRQ: %d)", mgos_sys_config_get_stmpe610_cs_index(), mgos_sys_config_get_stmpe610_irq_pin()));
  if (0x811 != v) {
    LOG(LL_ERROR, ("STMPE610 init failed (0x%04x), disabling", v));
    return true;
  }
  LOG(LL_INFO, ("STMPE610 init ok"));

  for (uint8_t i = 0; i < 65; i++) {
    stmpe610_spi_read_register(i);
  }

  stmpe610_spi_write_register(STMPE_SYS_CTRL2, 0x0);                                   // turn on clocks!
  stmpe610_spi_write_register(STMPE_TSC_CTRL, STMPE_TSC_CTRL_XYZ | STMPE_TSC_CTRL_EN); // XYZ and enable!
  stmpe610_spi_write_register(STMPE_INT_EN, STMPE_INT_EN_TOUCHDET);
  stmpe610_spi_write_register(STMPE_ADC_CTRL1, STMPE_ADC_CTRL1_10BIT | (0x6 << 4));    // 96 clocks per conversion
  stmpe610_spi_write_register(STMPE_ADC_CTRL2, STMPE_ADC_CTRL2_6_5MHZ);
  stmpe610_spi_write_register(STMPE_TSC_CFG, STMPE_TSC_CFG_4SAMPLE | STMPE_TSC_CFG_DELAY_1MS | STMPE_TSC_CFG_SETTLE_5MS);
  stmpe610_spi_write_register(STMPE_TSC_FRACTION_Z, 0x6);
  stmpe610_spi_write_register(STMPE_FIFO_TH, 1);
  stmpe610_spi_write_register(STMPE_FIFO_STA, STMPE_FIFO_STA_RESET);
  stmpe610_spi_write_register(STMPE_FIFO_STA, 0);   // unreset
  stmpe610_spi_write_register(STMPE_TSC_I_DRIVE, STMPE_TSC_I_DRIVE_50MA);
  stmpe610_spi_write_register(STMPE_INT_STA, 0xFF); // reset all ints
  stmpe610_spi_write_register(STMPE_INT_CTRL, STMPE_INT_CTRL_POL_LOW | STMPE_INT_CTRL_EDGE | STMPE_INT_CTRL_ENABLE);

  // Initialize the last touch to TOUCH_UP
  s_last_touch.direction = TOUCH_UP;
  s_last_touch.x         = 0;
  s_last_touch.y         = 0;
  s_last_touch.z         = 0;
  s_last_touch.length    = 0;

  // Set the orientation
  s_orientation = mgos_sys_config_get_stmpe610_orientation();

  return true;
}
