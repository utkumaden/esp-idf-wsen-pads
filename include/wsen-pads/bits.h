#ifndef _ESP_IDF_WSEN_PADS_H_
#error Never include "wsen-pads/bits.h" by itself. Include "wsen-pads.h" instead.
#else 

/* Opaque pointer to driver sturcture. */
typedef struct wsen_pads_t *wsen_pads_t;

typedef enum wsen_pads_device_addr_t
{
    WSEN_PADS_I2C_ADDR_DETECT   = 0,
    WSEN_PADS_I2C_ADDR_LO       = 0xB8,
    WSEN_PADS_I2C_ADDR_HI       = 0xBA
} wsen_pads_device_addr_t;

typedef enum wsen_pads_odr_t 
{
    WSEN_PADS_DATARATE_OFF_OR_SINGLE    = 0,
    WSEN_PADS_DATARATE_1HZ              = 1,
    WSEN_PADS_DATARATE_10HZ             = 2,
    WSEN_PADS_DATARATE_25HZ             = 3,
    WSEN_PADS_DATARATE_50HZ             = 4,
    WSEN_PADS_DATARATE_75HZ             = 5,
    WSEN_PADS_DATARATE_100HZ            = 6,
    WSEN_PADS_DATARATE_200HZ            = 7,
    WSEN_PADS_DATARATE_MAX              = WSEN_PADS_DATARATE_200HZ
} wsen_pads_odr_t;

typedef enum wsen_pads_lowpass_t
{
    WSEN_PADS_LOWPASS_OFF       = 0,
    WSEN_PADS_LOWPASS_SINGLE    = 1,
    WSEN_PADS_LOWPASS_DUAL      = 2
} wsen_pads_lowpass_t;

typedef enum wsen_pdas_interrupt_mux_t
{
    WSEN_PADS_INTERRUPT_MUX_OTHER                   = 0,
    WSEN_PADS_INTERRUPT_MUX_PRESSURE_HIGH           = 1,
    WSEN_PADS_INTERRUPT_MUX_PRESSURE_LOW            = 2,
    WSEN_PADS_INTERRUPT_MUX_PRESSURE_HIGH_OR_LOW    = 3
} wsen_pads_interrupt_mux_t;

#define WSEN_PADS_FIFO_CAPACITY 128 /**< Maximum number of samples the FIFO can hold. */

typedef enum wsen_pads_status_t
{
    WSEN_PADS_STATUS_PRESSURE_READY         = (1 << 0),
    WSEN_PADS_STATUS_TEMPERATURE_READY      = (1 << 1),
    WSEN_PADS_STATUS_PRESSURE_OVERRUN       = (1 << 2),
    WSEN_PADS_STATUS_TEMPERATURE_OVERRUN    = (1 << 3),
    WSEN_PADS_STATUS_FIFO_THRESHOLD         = (1 << 4),
    WSEN_PADS_STATUS_FIFO_OVERRUN           = (1 << 5),
    WSEN_PADS_STATUS_FIFO_FULL              = (1 << 6),
    WSEN_PADS_STATUS_INT_ACTIVE             = (1 << 7),
    WSEN_PADS_STATUS_INT_PRESSURE_LOW       = (1 << 8),
    WSEN_PADS_STATUS_INT_PRESSURE_HIGH      = (1 << 9)
} wsen_pads_status_t;

#endif
