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

/***************************************************
 * This is a library for the Adafruit STMPE610 Resistive
 * touch screen controller breakout
 * ----> http://www.adafruit.com/products/1571
 *
 * Check out the links above for our tutorials and wiring diagrams
 * These breakouts use SPI or I2C to communicate
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 * MIT license, all text above must be included in any redistribution
 ****************************************************/

/* Adapted for native Mongoose by Pim van Pelt <pim@google.com> */

#ifndef __STMPE610H_
#define __STMPE610H_

#define STMPE_ADDR                     0x41

#define STMPE_SYS_CTRL1                0x03
#define STMPE_SYS_CTRL1_RESET          0x02
#define STMPE_SYS_CTRL2                0x04

#define STMPE_TSC_CTRL                 0x40
#define STMPE_TSC_CTRL_EN              0x01
#define STMPE_TSC_CTRL_XYZ             0x00
#define STMPE_TSC_CTRL_XY              0x02

#define STMPE_INT_CTRL                 0x09
#define STMPE_INT_CTRL_POL_HIGH        0x04
#define STMPE_INT_CTRL_POL_LOW         0x00
#define STMPE_INT_CTRL_EDGE            0x02
#define STMPE_INT_CTRL_LEVEL           0x00
#define STMPE_INT_CTRL_ENABLE          0x01
#define STMPE_INT_CTRL_DISABLE         0x00

#define STMPE_INT_EN                   0x0A
#define STMPE_INT_EN_TOUCHDET          0x01
#define STMPE_INT_EN_FIFOTH            0x02
#define STMPE_INT_EN_FIFOOF            0x04
#define STMPE_INT_EN_FIFOFULL          0x08
#define STMPE_INT_EN_FIFOEMPTY         0x10
#define STMPE_INT_EN_ADC               0x40
#define STMPE_INT_EN_GPIO              0x80

#define STMPE_INT_STA                  0x0B
#define STMPE_INT_STA_TOUCHDET         0x01

#define STMPE_ADC_CTRL1                0x20
#define STMPE_ADC_CTRL1_12BIT          0x08
#define STMPE_ADC_CTRL1_10BIT          0x00

#define STMPE_ADC_CTRL2                0x21
#define STMPE_ADC_CTRL2_1_625MHZ       0x00
#define STMPE_ADC_CTRL2_3_25MHZ        0x01
#define STMPE_ADC_CTRL2_6_5MHZ         0x02

#define STMPE_TSC_CFG                  0x41
#define STMPE_TSC_CFG_1SAMPLE          0x00
#define STMPE_TSC_CFG_2SAMPLE          0x40
#define STMPE_TSC_CFG_4SAMPLE          0x80
#define STMPE_TSC_CFG_8SAMPLE          0xC0
#define STMPE_TSC_CFG_DELAY_10US       0x00
#define STMPE_TSC_CFG_DELAY_50US       0x08
#define STMPE_TSC_CFG_DELAY_100US      0x10
#define STMPE_TSC_CFG_DELAY_500US      0x18
#define STMPE_TSC_CFG_DELAY_1MS        0x20
#define STMPE_TSC_CFG_DELAY_5MS        0x28
#define STMPE_TSC_CFG_DELAY_10MS       0x30
#define STMPE_TSC_CFG_DELAY_50MS       0x38
#define STMPE_TSC_CFG_SETTLE_10US      0x00
#define STMPE_TSC_CFG_SETTLE_100US     0x01
#define STMPE_TSC_CFG_SETTLE_500US     0x02
#define STMPE_TSC_CFG_SETTLE_1MS       0x03
#define STMPE_TSC_CFG_SETTLE_5MS       0x04
#define STMPE_TSC_CFG_SETTLE_10MS      0x05
#define STMPE_TSC_CFG_SETTLE_50MS      0x06
#define STMPE_TSC_CFG_SETTLE_100MS     0x07

#define STMPE_FIFO_TH                  0x4A

#define STMPE_FIFO_SIZE                0x4C

#define STMPE_FIFO_STA                 0x4B
#define STMPE_FIFO_STA_RESET           0x01
#define STMPE_FIFO_STA_OFLOW           0x80
#define STMPE_FIFO_STA_FULL            0x40
#define STMPE_FIFO_STA_EMPTY           0x20
#define STMPE_FIFO_STA_THTRIG          0x10

#define STMPE_TSC_I_DRIVE              0x58
#define STMPE_TSC_I_DRIVE_20MA         0x00
#define STMPE_TSC_I_DRIVE_50MA         0x01

#define STMPE_TSC_DATA_X               0x4D
#define STMPE_TSC_DATA_Y               0x4F
#define STMPE_TSC_FRACTION_Z           0x56

#define STMPE_GPIO_SET_PIN             0x10
#define STMPE_GPIO_CLR_PIN             0x11
#define STMPE_GPIO_DIR                 0x13
#define STMPE_GPIO_ALT_FUNCT           0x17

// Use these in mgos_stmpe610_set_orientation()
#define STMPE_ORIENTATION_X            0x00
#define STMPE_ORIENTATION_Y            0x00
#define STMPE_ORIENTATION_FLIP_X       0x01
#define STMPE_ORIENTATION_FLIP_Y       0x02
#define STMPE_ORIENTATION_SWAP_NONE    0x00
#define STMPE_ORIENTATION_SWAP_XY      0x04

enum mgos_stmpe610_touch_t {
  TOUCH_DOWN = 0,
  TOUCH_UP   = 1,
};

struct mgos_stmpe610_event_data {
  enum mgos_stmpe610_touch_t direction;
  uint16_t                   x;
  uint16_t                   y;
  uint8_t                    z;
  uint8_t                    length; // Amount of time the TOUCH_DOWN event lasted (always set to 1 for TOUCH_UP)
};

bool mgos_stmpe610_init(void);

typedef void (*mgos_stmpe610_event_t)(struct mgos_stmpe610_event_data *);
void mgos_stmpe610_set_handler(mgos_stmpe610_event_t handler);
void mgos_stmpe610_set_dimensions(uint16_t x, uint16_t y);
void mgos_stmpe610_set_orientation(uint8_t flags);
bool mgos_stmpe610_is_touching();

#endif // __STMPE610H_
