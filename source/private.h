#ifndef _WSEN_PADS_PRIVATE_H_
#define _WSEN_PADS_PRIVATE_H_

#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include "esp_log.h"
#include "wsen-pads.h"
#include "registers.h"

struct wsen_pads_t
{
    int flags;
    esp_err_t error;
    i2c_port_t port;
    wsen_pads_device_addr_t dev;
};

/**
 * Read from sensor registers.
 * @param port I2C port.
 * @param dev I2C device address.
 * @param addr Register address.
 * @param buffer Buffer to read into.
 * @param size Size of read in bytes.
 * @return Non zero when IO error.
 */
esp_err_t wsen_pads_read(i2c_port_t port, uint8_t dev, uint8_t addr, void *buffer, size_t size);

/**
 * Write to sensor registers.
 * @param port I2C port.
 * @param dev I2C device address.
 * @param addr Register address.
 * @param buffer Buffer to write from.
 * @param size Size of write in bytes.
 * @return Non zero when IO error.
 */
esp_err_t wsen_pads_write(i2c_port_t port, uint8_t dev, uint8_t addr, const void *buffer, size_t size);

/**
 * Do bit manipulation on sensor registers.
 * @param port I2C port.
 * @param dev I2C device address.
 * @param addr Register address.
 * @param set Bits to set.
 * @param reset Bits to reset.
 * @return Non zero when IO error.
 * @remarks Bits are reset before they are set.
 */
esp_err_t wsen_pads_setbits(i2c_port_t port, uint8_t dev, uint8_t addr, uint8_t set, uint8_t reset);

/**
 * Read bitfield registers.
 * @param port I2C port.
 * @param dev I2C device addrress.
 * @param addr Register address.
 * @param mask Bitfield mask.
 * @param err IO error.
 * @return Bits when successful, -1 on IO error.
 */
int wsen_pads_getbits(i2c_port_t port, uint8_t dev, uint8_t addr, uint8_t mask, esp_err_t* err);

#define LOGD(...) ESP_LOGD("wsen-pads", __VA_ARGS__)
#define LOGI(...) ESP_LOGI("wsen-pads", __VA_ARGS__)
#define LOGW(...) ESP_LOGW("wsen-pads", __VA_ARGS__)
#define LOGE(...) ESP_LOGE("wsen-pads", __VA_ARGS__)

#endif
