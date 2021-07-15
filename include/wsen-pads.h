/*
 * 
 */
#ifndef _ESP_IDF_WSEN_PADS_H_
#define _ESP_IDF_WSEN_PADS_H_

#include "wsen-pads/bits.h"
#include <driver/i2c.h>

/**
 * Create driver instance.
 * @param port I2C port to use.
 * @param address I2C device address.
 * @return Driver instance or NULL (failure).
 */
wsen_pads_t wsen_pads_create(i2c_port_t port, wsen_pads_device_addr_t address);

/**
 * Destroy driver instance.
 * @param driver Driver instance.
 */
void wsen_pads_destroy(wsen_pads_t driver);

/**
 * Get specific information on last IO error.
 * @param driver Driver instance.
 * @return Last IO error.
 */
esp_err_t wsen_pads_getLastError(wsen_pads_t driver);

/**
 * Initialize sensor.
 * @param driver Driver instance.
 * @return -1 on error.
 */
int wsen_pads_init(wsen_pads_t driver);

int wsen_pads_reset(wsen_pads_t driver, int config, int calibration);

/**
 * Configure sensor interface.
 * @param driver Driver instance.
 * @param sda_pu Enable internal SDA pull up resistor.
 * @param sao_pu Enable internal SAO pull up resistor.
 * @param int_pd Enable internal INT pull down resitor.
 * @param int_active Make interrupt pin active low or high. (pass 0 for low, 1 for high)
 * @param int_od Make interrupt pin open drain (open collector).
 */
int wsen_pads_configureInterface(
    wsen_pads_t driver,
    int sda_pu,
    int sao_pu,
    int int_pd,
    int int_active,
    int int_od
);

#endif
