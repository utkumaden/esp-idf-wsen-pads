#include "private.h"

#define F_INIT 1

#define READ(addr, buffer, size) (driver->error = wsen_pads_read(driver->port, driver->dev, addr, buffer, size))
#define WRITE(addr, buffer, size) (driver->error = wsen_pads_write(driver->port, driver->dev, addr, buffer, size))
#define SET(addr, set, reset) (driver->error = wsen_pads_setbits(driver->port, driver->dev, addr, set, reset))
#define GET(addr, mask) wsen_pads_getbits(driver->port, driver->dev, addr, mask, &driver->error)
#define ASSERT_DRV() \
do {\
    if (!driver) { errno = EINVAL; return -1; }\
    else if (!(driver->flags & F_INIT)) { errno = EFAULT; return -1; }} \
while(0)

wsen_pads_t wsen_pads_create(i2c_port_t port, wsen_pads_device_addr_t address)
{
    wsen_pads_t driver;

    if (
        port > I2C_NUM_MAX ||
        (
            address != WSEN_PADS_I2C_ADDR_DETECT &&
            address != WSEN_PADS_I2C_ADDR_LO
            && address != WSEN_PADS_I2C_ADDR_HI
        )
    )
    {
        errno = EINVAL;
        return NULL;
    }

    driver = calloc(1, sizeof(struct wsen_pads_t));
    if (driver)
    {
        driver->port = port;
        driver->dev = address;
    }

    return driver;
}

void wsen_pads_destroy(wsen_pads_t driver)
{
    free(driver);
}

esp_err_t wsen_pads_getLastError(wsen_pads_t driver)
{
    if (driver)
    {
        return driver->error;
    }
    else
    {
        return ESP_ERR_INVALID_ARG;
    }
}

static int wsen_pads_detect(wsen_pads_t driver)
{
    if (driver->dev)
    {
        uint8_t id;

        LOGI("Probing for sensor at addr=%d", (int)driver->dev);
        if (READ(WSEN_PADS_DEVICE_ID, &id, 1))
        {
            if (errno != ENOMEM)
                LOGE("IO error detecting sensor.");
            return -1;
        }

        if (id == WSEN_PADS_DEVICE_ID_VALUE)
        {
            LOGI("ID mathces, sensor verified.");
            return 0;
        }
        else
        {
            LOGE("Unexpect value for ID: %d", (int)id);
            errno = EADDRINUSE;
            return -1;
        }
    }
    else
    {
        driver->dev = WSEN_PADS_I2C_ADDR_LO;

        if (wsen_pads_detect(driver))
        {
            driver->dev = WSEN_PADS_I2C_ADDR_HI;
            if (wsen_pads_detect(driver))
            {
                LOGE("Sensor not found.");
            }
        }

        return 0;
    }
}

int wsen_pads_init(wsen_pads_t driver)
{
    if (!driver)
    {
        errno = EINVAL;
        return -1;
    }

    if (wsen_pads_detect(driver))
    {
        return -1;
    }

    driver->flags |= F_INIT;
    if (wsen_pads_reset(driver, 1, 1))
    {
        driver->flags &= ~F_INIT;
        return -1;
    }

    return 0;
}

int wsen_pads_reset(wsen_pads_t driver, int config, int calibration)
{
    ASSERT_DRV();

    if (config)
    {
        SET(WSEN_PADS_CTRL_2, WSEN_PADS_CTRL_2_SWRESET, 0);
        do
        {
            vTaskDelay(pdMS_TO_TICKS(1));
        } while (GET(WSEN_PADS_CTRL_1, WSEN_PADS_CTRL_2_SWRESET) && !driver->error);
    }

    if (calibration)
    {
        SET(WSEN_PADS_CTRL_2, WSEN_PADS_CTRL_2_BOOT, 0);
        do
        {
            vTaskDelay(pdMS_TO_TICKS(1));
        } while (GET(WSEN_PADS_INT_SRC, WSEN_PADS_INT_SRC_BOOT_ON) && !driver->error);
    }

    return driver->error;
}

int wsen_pads_configureInterface(
    wsen_pads_t driver,
    int sda_pu,
    int sao_pu,
    int int_pd,
    int int_active,
    int int_od
)
{
    ASSERT_DRV();

    if (
        SET(
            WSEN_PADS_INTERFACE_CTRL,

            ((sda_pu) ? WSEN_PADS_INTERFACE_CTRL_SDA_PU_EN : 0 ) |
            ((sao_pu) ? WSEN_PADS_INTERFACE_CTRL_SAO_PU_EN : 0) |
            ((int_pd) ? 0 : WSEN_PADS_INTERFACE_CTRL_PD_DIS_INT),

            WSEN_PADS_INTERFACE_CTRL_SDA_PU_EN |
            WSEN_PADS_INTERFACE_CTRL_SAO_PU_EN |
            WSEN_PADS_INTERFACE_CTRL_PD_DIS_INT |
            1 /* I2C disable bit */
        )

        ||

        SET (
            WSEN_PADS_CTRL_2,

            ((int_active) ? 0 : WSEN_PADS_CTRL_2_INT_H_L) |
            ((int_od) ? WSEN_PADS_CTRL_2_PP_OD : 0) |
            WSEN_PADS_CTRL_2_IF_ADD_INC,

            WSEN_PADS_CTRL_2_INT_H_L |
            WSEN_PADS_CTRL_2_PP_OD
        )
    )
    {
        return -1;
    }

    return 0;
}