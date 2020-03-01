#include "driver_init.h"
#include "commands.h"
#include "subbus.h"

static void update_status(uint16_t *status, uint8_t pin, uint16_t bit) {
  if (gpio_get_pin_level(pin)) {
    *status |= bit;
  } else {
    *status &= ~bit;
  }
}

/**
 * This file should include a memory map. The current one is In Evernote.
 */
static subbus_cache_word_t cmd_cache[CMD_HIGH_ADDR-CMD_BASE_ADDR+1] = {
  { 0, 0, true,  false, true, false, false } // Offset 0: R: Command Status. W: Command value
};

static void cmd_poll(void) {
  uint16_t cmd;
  uint16_t status = cmd_cache[0].cache;
  if (subbus_cache_iswritten(&sb_cmd, CMD_BASE_ADDR, &cmd)) {
    switch (cmd) {
      case 0: // Pump enable
        if ((status & 0x6) == 0) {
          gpio_set_pin_level(PUMP_EN, true);
        }
        break;
      case 1: // Pump disable
        if ((status & 0x6) == 0) {
          gpio_set_pin_level(PUMP_EN, false);
        }
        break;
      case 2: // Set speed to 00
        gpio_set_pin_level(PUMP_SA, false); break;
        gpio_set_pin_level(PUMP_SB, false); break;
        break;
      case 3: // Set speed to 01
        if (status & 1) {
          gpio_set_pin_level(PUMP_SA,  true); break;
          gpio_set_pin_level(PUMP_SB, false); break;
        }
        break;
      case 4: // Set speed to 10
        if (status & 1) {
          gpio_set_pin_level(PUMP_SA, false); break;
          gpio_set_pin_level(PUMP_SB,  true); break;
        }
        break;
      case 5: // Set speed to 11
        if (status & 1) {
          gpio_set_pin_level(PUMP_SA,  true); break;
          gpio_set_pin_level(PUMP_SB,  true); break;
        }
        break;
      case 6:  gpio_set_pin_level(STLED1, true); break;
      case 7:  gpio_set_pin_level(STLED1, false); break;
      case 8:  gpio_set_pin_level(STLED2, true); break;
      case 9:  gpio_set_pin_level(STLED2, false); break;
      case 10: gpio_set_pin_level(STLED3, true); break;
      case 11: gpio_set_pin_level(STLED3, false); break;
      default:
        break;
    }
  }
  status = 0;
  update_status(&status, PUMP_EN, 0x01);
  update_status(&status, PUMP_SA, 0x02);
  update_status(&status, PUMP_SB, 0x04);
  update_status(&status, STLED1, 0x08);
  update_status(&status, STLED2, 0x10);
  update_status(&status, STLED3, 0x20);
  subbus_cache_update(&sb_cmd, CMD_BASE_ADDR, status);
}

static void cmd_reset(void) {
  if (!sb_cmd.initialized) {
    sb_cmd.initialized = true;
  }
}

subbus_driver_t sb_cmd = {
  CMD_BASE_ADDR, CMD_HIGH_ADDR, // address range
  cmd_cache,
  cmd_reset,
  cmd_poll,
  0, // dynamic driver
  false
};
