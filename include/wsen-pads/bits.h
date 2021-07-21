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

#define WSEN_PADS_FIFO_CAPACITY 128 /**< Maximum number of samples the FIFO can hold. */

#endif
