/** @file i2c.c */
#include "driver_init.h"
#include <utils.h>
#include <hal_init.h>
#include <hal_i2c_m_async.h>
#include <stdint.h>
#include "atmel_start_pins.h"
#include "i2c.h"
#include "subbus.h"

static bool i2c_enabled = I2C_ENABLE_DEFAULT;

/**
 * These addresses belong to the I2C module
 * 0x20 R:  I2C_Status
 * 0x21 R:  HeaterV
 * 0x22 R:  IRSetV
 * 0x23 R:  ADS_N
 * 0x24 R:  LTR: TBD
 * 0x25 R:  LTR: TBD
 * 0x26 R:  LTR: TBD
 * 0x27 R:  MPL: TBD
 * 0x28 R:  MPL: TBD
 * 0x29 R:  MPL: TBD
 */
static subbus_cache_word_t i2c_cache[I2C_HIGH_ADDR-I2C_BASE_ADDR+1] = {
  // Value, Wvalue, readable, was_read, writable, written, dynamic
  { 0, 0, true,  false, false, false, false }, // Offset 0:  R: I2C Status
  { 0, 0, true,  false, false, false, false }, // Offset 1:  R: ADC Status
  { 0, 0, true,  false, false, false, false }, // Offset 2:  R: ADC CH0
  { 0, 0, true,  false, false, false, false }, // Offset 3:  R: ADC CH1
  { 0, 0, true,  false, false, false, false }, // Offset 4:  R: ADC CH2
  { 0, 0, true,  false, false, false, false }, // Offset 5:  R: ADC CH3
  { 0, 0, true,  false, false, false, false }, // Offset 6:  R: ADC CH4 PumpT
  { 0, 0, true,  false, false, false, false }, // Offset 7:  R: ADC CH5 PMotT
  { 0, 0, true,  false, false, false, false }, // Offset 8:  R: ADC V80_V
  { 0, 0, true,  false, false, false, false }, // Offset 9:  R: ADC V80_I
  { 0, 0, true,  false, false, false, false }, // Offset 10: R: ADC V28C1_V
  { 0, 0, true,  false, false, false, false }, // Offset 11: R: ADC V28C2_V
};

enum ads_state_t {ads_init, ads_init_tx, ads_read_cfg,
                  ads_read_cfg_tx, ads_reg0, ads_read_adc,
                  ads_read_adc_tx, ads_next_chan};

/** Write to config register [01] 0x83 0x03:
 *   0x01: Pointer register value specifying config register
 *   0x8303:
 *     OS[15] = 1: Single Conversion
 *     MUX[14:12] = 000: AIN0/AIN1 => HeaterV
 *     PGA[11:9] = 001: FSR = +/- 4.096V
 *     MODE[8] = 1: Single shot conversion
 *     DR[7:5] = 000: 8 SPS
 *     COMP_MODE[4] = 0: Default/Don't Care
 *     COMP_POL[3] = 0: Default/Don't Care
 *     COMP_LAT[2] = 0: Default/Don't Care
 *     COMP_QUE[1:0] = 11: Disable comparator and set ALERT to high impedance
 */
/** Write to config register [01] 0xB3 0x03:
 *   0x01: Pointer register value specifying config register
 *   0xB303:
 *     OS[15] = 1: Single Conversion
 *     MUX[14:12] = 011: AIN2/AIN3 => IRSetV
 *     PGA[11:9] = 001: FSR = +/- 4.096V
 *     MODE[8] = 1: Single shot conversion
 *     DR[7:5] = 000: 8 SPS
 *     COMP_MODE[4] = 0: Default/Don't Care
 *     COMP_POL[3] = 0: Default/Don't Care
 *     COMP_LAT[2] = 0: Default/Don't Care
 *     COMP_QUE[1:0] = 11: Disable comparator and set ALERT to high impedance
 */
/** Write to config register [01] 0x81 0x03:
 *   0x01: Pointer register value specifying config register
 *   0x8303:
 *     OS[15] = 1: Single Conversion
 *     MUX[14:12] = 000: AIN0/AIN1 => HeaterV
 *     PGA[11:9] = 000: FSR = +/- 6.144V
 */
/** Write to config register [01] 0xBB 0x03:
 *   0x01: Pointer register value specifying config register
 *   0xB303:
 *     OS[15] = 1: Single Conversion
 *     MUX[14:12] = 011: AIN2/AIN3 => IRSetV
 *     PGA[11:9] = 101: FSR = +/- 0.256V
 */

typedef struct {
  uint16_t offset;
  uint16_t bit_mask;
  uint16_t i2c_address;
  uint16_t ads_config;
} ads_channel_t;

ads_channel_t ads_chans[10] = {
  { 0, 0x0001, 0x48, 0xB303 }, // CH0V, AIN2/AIN3, +/-4.096V
  { 1, 0x0002, 0x48, 0x8303 }, // CH1V, AIN0/AIN1, +/-4.096V
  { 2, 0x0004, 0x49, 0xB303 }, // CH2V, AIN2/AIN3, +/-4.096V
  { 3, 0x0008, 0x49, 0x8303 }, // CH3V, AIN0/AIN1, +/-4.096V
  { 4, 0x0010, 0x4A, 0xB303 }, // CH4V PMotT, AIN2/AIN3, +/-4.096V
  { 5, 0x0020, 0x4A, 0x8303 }, // CH5V PumpT, AIN0/AIN1, +/-4.096V
  { 6, 0x0040, 0x48, 0x8103 }, // V80_V, AIN0/AIN1, +/-6.144V
  { 7, 0x0080, 0x48, 0xBB03 }, // V80_I, AIN2/AIN3, +/-0.256V
  { 8, 0x0100, 0x49, 0x8303 }, // V28C1_V, AIN0/AIN1, +/-4.096V
  { 9, 0x0200, 0x49, 0xB303 }, // V28C2_V, AIN2/AIN3, +/-4.096V
};

/** i2c error codes are defined in hal/include/hpl_i2c_m_sync.h
 *  named I2C_ERR_* and I2C_OK
 */
typedef struct {
  struct i2c_m_async_desc *i2c;
  struct io_descriptor *io;
  ads_channel_t *defs;
  int n_defs;
  int cur_def;
  volatile bool txfr_complete;
  volatile bool error_seen;
  enum ads_state_t ads_state;
  uint16_t ads_n_reads;
  volatile int32_t I2C_error;
  uint8_t ads_obuf[3];
  uint8_t ads_ibuf[2];
} i2c_chain_t;

i2c_chain_t I2C_T_chain = { &I2C_T, 0, &ads_chans[0], 6, 0, true, false, ads_init, 0, I2C_OK, {0, 0, 0}, {0, 0}};
i2c_chain_t I2C_P_chain = { &I2C_P, 0, &ads_chans[6], 4, 0, true, false, ads_init, 0, I2C_OK, {0, 0, 0}, {0, 0}};

static void ads_obuf_setup(i2c_chain_t *i2c, ads_channel_t *def, bool cfg) {
  if (cfg) {
    i2c->ads_obuf[0] = 1;
    i2c->ads_obuf[1] = def->ads_config >> 8;
    i2c->ads_obuf[2] = def->ads_config & 0xFF;
  } else {
    i2c->ads_obuf[0] = 0;
  }
}

// This method is specific to this implementation of ads1115
static bool i2c_write(i2c_chain_t *i2c, ads_channel_t *def, bool cfg, enum ads_state_t next) {
  assert(i2c->txfr_complete, __FILE__, __LINE__);
  i2c->txfr_complete = false;
  i2c->error_seen = false;
  ads_obuf_setup(i2c, def, cfg);
  i2c_m_async_set_slaveaddr(i2c->i2c, def->i2c_address, I2C_M_SEVEN);
  io_write(i2c->io, i2c->ads_obuf, def ? 3 : 1);
  i2c->ads_state = next;
  return false;
}

// This method is specific to this implementation of ads1115
static bool i2c_read(i2c_chain_t *i2c, ads_channel_t *def, enum ads_state_t next) {
  assert(i2c->txfr_complete, __FILE__, __LINE__);
  i2c->txfr_complete = false;
  i2c->error_seen = false;
  i2c_m_async_set_slaveaddr(i2c->i2c, def->i2c_address, I2C_M_SEVEN);
  io_read(i2c->io, i2c->ads_ibuf, 2);
  i2c->ads_state = next;
  return false;
}

static bool ads_tx_complete(i2c_chain_t *i2c, enum ads_state_t next, enum ads_state_t errnext) {
  i2c->ads_state = i2c->error_seen ? errnext : next;
  return true;
}

/**
 * @return true if the bus is free and available for another device
 */
static bool ads1115_poll(i2c_chain_t *i2c) {
  ads_channel_t *def = &i2c->defs[i2c->cur_def];
  switch (i2c->ads_state) {
    case ads_init: // Start to convert HeaterV
      i2c->ads_n_reads = 0;
      return i2c_write(i2c, def, true, ads_init_tx);
    case ads_init_tx:
      return ads_tx_complete(i2c, ads_read_cfg, ads_next_chan);
    case ads_read_cfg: // Start read from config register
      return i2c_read(i2c, def, ads_read_cfg_tx);
    case ads_read_cfg_tx: // If high bit is set, conversion is complete
      if (i2c->error_seen)
        i2c->ads_state = ads_next_chan;
      else if (i2c->ads_ibuf[0] & 0x80) {
        i2c->ads_state = ads_reg0;
      } else {
        ++i2c->ads_n_reads;
        i2c->ads_state = ads_read_cfg;
      }
      return true;
    case ads_reg0: // Write pointer register to read from conversion reg [0]
      return i2c_write(i2c, def, false, ads_read_adc);
    case ads_read_adc: // Start read from conversion reg
      return i2c_read(i2c, def, ads_read_adc_tx);
    case ads_read_adc_tx: // Save converted value
      if (i2c->error_seen)
        i2c->ads_state = ads_next_chan;
      else {
        sb_cache_update(i2c_cache, I2C_ADC_OFFSET+def->offset, (i2c->ads_ibuf[0] << 8) | i2c->ads_ibuf[1]);
        // sb_cache_update(i2c_cache, def->offset, ads_n_reads);
        i2c->ads_state = ads_init;
      }
      return true;
    case ads_next_chan:
      if (++i2c->cur_def >= i2c->n_defs)
        i2c->cur_def = 0;
      return false;
    default:
      assert(false, __FILE__, __LINE__);
  }
  return true;
}

void i2c_enable(bool value) {
  i2c_enabled = value;
}

#define I2C_INTFLAG_ERROR (1<<7)

static void I2C_T_async_error(struct i2c_m_async_desc *const i2c, int32_t error) {
  I2C_T_chain.txfr_complete = true;
  I2C_T_chain.error_seen = true;
  I2C_T_chain.I2C_error = error;
  if (sb_cache_was_read(i2c_cache, I2C_STATUS_OFFSET)) {
    sb_cache_update(i2c_cache, I2C_STATUS_OFFSET, 0);
  }
  if (error >= -7 && error <= -2) {
    uint16_t val = i2c_cache[I2C_STATUS_OFFSET].cache;
    val |= (1 << (7+error));
    sb_cache_update(i2c_cache, I2C_STATUS_OFFSET, val);
  }
  if (error == I2C_ERR_BUS) {
    hri_sercomi2cm_write_STATUS_reg(I2C_T.device.hw, SERCOM_I2CM_STATUS_BUSERR);
    hri_sercomi2cm_clear_INTFLAG_reg(I2C_T.device.hw, I2C_INTFLAG_ERROR);
  }
}

static void I2C_P_async_error(struct i2c_m_async_desc *const i2c, int32_t error) {
  I2C_P_chain.txfr_complete = true;
  I2C_P_chain.error_seen = true;
  I2C_P_chain.I2C_error = error;
  if (sb_cache_was_read(i2c_cache, I2C_STATUS_OFFSET)) {
    sb_cache_update(i2c_cache, I2C_STATUS_OFFSET, 0);
  }
  if (error >= -7 && error <= -2) {
    uint16_t val = i2c_cache[I2C_STATUS_OFFSET].cache;
    val |= (1 << (8+7+error));
    sb_cache_update(i2c_cache, I2C_STATUS_OFFSET, val);
  }
  if (error == I2C_ERR_BUS) {
    hri_sercomi2cm_write_STATUS_reg(I2C_P.device.hw, SERCOM_I2CM_STATUS_BUSERR);
    hri_sercomi2cm_clear_INTFLAG_reg(I2C_P.device.hw, I2C_INTFLAG_ERROR);
  }
}

static void I2C_T_txfr_completed(struct i2c_m_async_desc *const i2c) {
  I2C_T_chain.txfr_complete = true;
}

static void I2C_P_txfr_completed(struct i2c_m_async_desc *const i2c) {
  I2C_P_chain.txfr_complete = true;
}

static void i2c_reset() {
  if (!sb_i2c.initialized) {
    // I2C_T_init(); // Called from driver_init
    i2c_m_async_get_io_descriptor(&I2C_T, &I2C_T_chain.io);
    i2c_m_async_enable(&I2C_T);
    i2c_m_async_register_callback(&I2C_T, I2C_M_ASYNC_ERROR, (FUNC_PTR)I2C_T_async_error);
    i2c_m_async_register_callback(&I2C_T, I2C_M_ASYNC_TX_COMPLETE, (FUNC_PTR)I2C_T_txfr_completed);
    i2c_m_async_register_callback(&I2C_T, I2C_M_ASYNC_RX_COMPLETE, (FUNC_PTR)I2C_T_txfr_completed);

    // I2C_P_init(); // Called from driver_init
    i2c_m_async_get_io_descriptor(&I2C_P, &I2C_P_chain.io);
    i2c_m_async_enable(&I2C_P);
    i2c_m_async_register_callback(&I2C_P, I2C_M_ASYNC_ERROR, (FUNC_PTR)I2C_P_async_error);
    i2c_m_async_register_callback(&I2C_P, I2C_M_ASYNC_TX_COMPLETE, (FUNC_PTR)I2C_P_txfr_completed);
    i2c_m_async_register_callback(&I2C_P, I2C_M_ASYNC_RX_COMPLETE, (FUNC_PTR)I2C_P_txfr_completed);

    sb_i2c.initialized = true;
  }
}

void i2c_poll(void) {
  ads1115_poll(&I2C_T_chain);
  ads1115_poll(&I2C_P_chain);
}

subbus_driver_t sb_i2c = {
  I2C_BASE_ADDR, I2C_HIGH_ADDR, // address range
  i2c_cache,
  i2c_reset,
  i2c_poll,
  0, // Dynamic function
  false // initialized
};
