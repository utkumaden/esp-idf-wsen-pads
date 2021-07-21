/*
 * 
 */
#ifndef _ESP_IDF_WSEN_PADS_H_
#define _ESP_IDF_WSEN_PADS_H_

#include "wsen-pads/bits.h"
#include <driver/i2c.h>
#include <stdint.h>

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

/**
 * Reset WSEN_PADS device.
 * @param driver Driver instance.
 * @param config Reset device configuration.
 * @param calibration Reset device calibration parameters.
 */
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

/**
 * Set or reset interrupts.
 * @param driver Driver instance.
 * @param mux Interrupt mux selection.
 * @param fifo_full Interrupt when FIFO full.
 * @param fifo_threshold Interrupt when FIFO threshold level reached.
 * @param fifo_overflow Interrupt when FIFO overflows.
 * @param data_ready Interrupt when pressure data ready.
 * @param latch Latch interrupt pin until read.
 * @return -1 on error.
 * @remarks
 * The interrupt mux determines which interrupts are propogated. When
 * differential pressure interrupts are selected, no other interrupt is sent.
 */
int wsen_pads_setInterrupt(
    wsen_pads_t driver,
    wsen_pads_interrupt_mux_t mux,
    int fifo_full,
    int fifo_threshold,
    int fifo_overflow,
    int data_ready,
    int latch
);

/**
 * Get device status bits.
 * @param driver Driver instance.
 * @param out_status Status bits.
 * @param opt_out_fifo_level (Optional) FIFO fill level.
 * @return -1 on error.
 *
 * @remarks Clears interrupt status. FIFO fill level is also read as an
 * optimization, which can be written to pointer if wanted.
 */
int wsen_pads_getStatus(wsen_pads_t driver, wsen_pads_status_t *out_status, uint8_t *opt_out_fifo_level);

/**
 * Configure data mode of device.
 * @param driver Driver instance.
 * @param dataRate Output data rate.
 * @param lowpass Low pass mode.
 * @param low_noise Enable low noise mode.
 * @param blockDataUpdate Data registers keep their values until read when set.
 * @return -1 on error.
 * @remarks Sensor enters powerdown mode before configuring data modes.
 */
int wsen_pads_configureData(
    wsen_pads_t driver,
    wsen_pads_odr_t dataRate,
    wsen_pads_lowpass_t lowpass,
    int low_noise,
    int blockDataUpdate
);

/**
 * Enable autofeatures of the sensor.
 * @param driver Driver instance.
 * @param autozero Enable or reset AUTOZERO mode.
 * @param autorefp Enable or reset AUTOREFP.
 * @return -1 on error.
 * @remarks Refer to sensor documentation for feature descriptions. AUTOZERO and
 * AUTOREFP cannot be set at the same time.
 */
int wsen_pads_autofeature(wsen_pads_t driver, int autozero, int autorefp);

/**
 * Convert numeric to reference pressure value.
 * @param value Value to convert.
 * @return Converted value.
 */
#define WSEN_PADS_TO_REFERENCE_PRESSURE(value) ((uint16_t)((value) / 16))

/**
 * Set pressure threshold used with differential pressure interrupts.
 * @param driver Driver instance.
 * @param pressure Pressure threshold value in kPa / 16.
 * @return -1 on error.
 * @remarks Pressure threshold value is written to sensor as fixed point value.
 * The value is in 62.5 Pa increments.
 */
int wsen_pads_setPressureThresholdR(wsen_pads_t driver, uint16_t pressure);

/**
 * Set pressure threshold used with differential pressure interrupts.
 * @param driver Driver instance.
 * @param pressure Pressure threshold value in Pa.
 * @return -1 on error.
 * @remarks Pressure threshold value is written to sensor as fixed point value.
 * The value is in 62.5 Pa increments.
 */
inline static int wsen_pads_setPressureThresholdI(wsen_pads_t driver, uint32_t pressure)
{
    return wsen_pads_setPressureThresholdR(driver, WSEN_PADS_TO_REFERENCE_PRESSURE(pressure / 1000));
}

/**
 * Set pressure threshold used with differential pressure interrupts.
 * @param driver Driver instance.
 * @param pressure Pressure threshold value in Pa.
 * @return -1 on error.
 * @remarks Pressure threshold value is written to sensor as fixed point value.
 * The value is in 62.5 Pa increments.
 */
inline static int wsen_pads_setPressureThresholdF(wsen_pads_t driver, float pressure)
{
    return wsen_pads_setPressureThresholdR(driver, WSEN_PADS_TO_REFERENCE_PRESSURE(pressure / 1000));
}

#endif
