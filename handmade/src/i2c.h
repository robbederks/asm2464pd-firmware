#ifndef __I2C_H__
#define __I2C_H__

#include "types.h"
#include "registers.h"

/*=== HW I2C master ===*/

/* Trigger the peripheral and wait for DONE. CSR is W1C on the status bits
 * (1-6): writing 0xFE clears any lingering flags from the prior transaction
 * in one shot without setting GO. Writing 0x01 then fires. Without the
 * pre-clear, status bits like RD_DONE (bit 3) leak into the next fire and
 * the peripheral silently drops every other transaction. */
static uint8_t i2c_fire_wait(void) {
  uint8_t csr = 0;
  uint16_t timeout = 0xFFFF;
  REG_I2C_CSR = 0xFF;
  REG_I2C_CSR = 0x01;
  do {
    csr = REG_I2C_CSR;
    if (csr & I2C_CSR_DONE) break;
  } while (--timeout);
  return csr;
}

static void i2c_init(void) {
  REG_GPIO_CTRL(9)  = I2C_GPIO_ALT_SCL;     /* 0x13 */
  REG_GPIO_CTRL(10) = I2C_GPIO_ALT_SDA_OD;  /* 0x34 — open-drain SDA */
  REG_I2C_MODE = 0xC4;
  REG_I2C_CLK_LO = 0x18;
  REG_I2C_CLK_HI = 0x4A;
}

/* Write a 16-bit big-endian register. The peripheral sends 3 data bytes
 * from 0xE800 after DATA0; the third lands on the slave's reg+1 as a
 * partial write and is discarded on STOP. */
static uint8_t i2c_write_reg16(uint8_t addr7, uint8_t reg, uint16_t val) {
  uint8_t csr;
  (&REG_I2C_XRAM)[0] = (val >> 8) & 0xFF;
  (&REG_I2C_XRAM)[1] = val & 0xFF;
  (&REG_I2C_XRAM)[2] = 0x00;
  REG_I2C_ADDR        = (addr7 << 1) | 0;
  REG_I2C_DATA0       = reg;
  REG_I2C_WLEN        = 0x03;
  REG_I2C_RLEN        = 0x00;
  REG_I2C_DMA_SRC_HI  = 0;
  REG_I2C_DMA_SRC_LO  = 0;
  REG_I2C_DMA_DEST_HI = 0;
  REG_I2C_DMA_DEST_LO = 0;
  csr = i2c_fire_wait();
  REG_I2C_CSR = I2C_CSR_DONE;
  return csr;
}

/* Park the slave's register pointer at `reg`. Writes three zero bytes that
 * the slave's read-only registers will ignore — don't use on writable regs. */
static uint8_t i2c_point(uint8_t addr7, uint8_t reg) {
  return i2c_write_reg16(addr7, reg, 0x0000);
}

/* Read `rlen` bytes at the slave's current register pointer. The peripheral
 * DMAs received bytes into the XRAM buffer at 0xE800 while C805 bit 4 is
 * held high. (C87A bit 0 is the combined-read DMA arm — only for the mode
 * where C870 carries the write direction byte. Setting it here, where C870
 * already carries addr+R, leaves the peripheral in a half-configured state
 * and every subsequent transaction silently drops.) */
static uint8_t i2c_read(uint8_t addr7, uint8_t rlen, uint8_t *out) {
  uint8_t csr, dma_en_save, i;
  REG_I2C_ADDR        = (addr7 << 1) | 1;
  REG_I2C_DATA0       = 0;
  REG_I2C_WLEN        = 0;
  REG_I2C_RLEN        = rlen;
  REG_I2C_DMA_SRC_HI  = 0;
  REG_I2C_DMA_SRC_LO  = 0;
  REG_I2C_DMA_DEST_HI = 0;
  REG_I2C_DMA_DEST_LO = 0;
  dma_en_save = REG_I2C_DMA_ENABLE;
  REG_I2C_DMA_ENABLE = (dma_en_save & 0xEF) | 0x10;
  csr = i2c_fire_wait();
  if ((csr & I2C_CSR_DONE) && csr != I2C_CSR_RD_NACK) {
    for (i = 0; i < rlen; i++) out[i] = (&REG_I2C_XRAM)[i];
  }
  REG_I2C_DMA_ENABLE = dma_en_save & 0xEF;
  REG_I2C_CSR = I2C_CSR_DONE;
  return csr;
}

/*=== INA231 power monitor (I2C addr 0x45) ===
 *
 * 1 LSB bus = 1.25 mV, 1 LSB shunt = 2.5 uV.
 * Stock CFG (0x4127) is AVG=16, CT=1.1 ms, continuous both channels.
 * Eval-board shunt calibrated against a known 2.242 A load. */
#define INA231_ADDR        0x45
#define INA231_REG_CFG     0x00
#define INA231_REG_SHUNT   0x01
#define INA231_REG_BUS     0x02
#define INA231_CFG_STOCK   0x4127
#define INA231_SHUNT_UOHM  2408   /* measured, not the nominal 2×1 mOhm */

static void ina231_init(void) {
  i2c_write_reg16(INA231_ADDR, INA231_REG_CFG, INA231_CFG_STOCK);
}

/* Read one 16-bit BE register. Returns 1 on success.  Both halves are
 * strictly checked: a silent-drop or NACK on the pointer write leaves the
 * slave's register pointer stale, and the subsequent read would return
 * data from the wrong register. */
static uint8_t ina231_read_u16(uint8_t reg, uint16_t *out) {
  uint8_t rx[2];
  if (i2c_point(INA231_ADDR, reg) != I2C_CSR_WR_DONE) return 0;
  if (i2c_read(INA231_ADDR, 2, rx) != I2C_CSR_RD_DONE) return 0;
  *out = ((uint16_t)rx[0] << 8) | rx[1];
  return 1;
}

#endif
