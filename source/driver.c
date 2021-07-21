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
        errno = EIO;
        return -1;
    }

    return 0;
}

int wsen_pads_setInterrupt(
    wsen_pads_t driver,
    wsen_pads_interrupt_mux_t mux,
    int fifo_full,
    int fifo_threshold,
    int fifo_overflow,
    int data_ready,
    int latch
)
{
    // Writes to two registers, INT_CFG and CTRL_3
    ASSERT_DRV();

    if (fifo_threshold > WSEN_PADS_FIFO_CAPACITY || mux > 3)
    {
        errno = EINVAL;
        return -1;
    }

    // Cannot have differential mode without these flags.
    if (mux && !GET(WSEN_PADS_INT_CFG, WSEN_PADS_INT_CFG_AUTOREFP | WSEN_PADS_INT_CFG_AUTOZERO))
    {
        errno = EFAULT;
        return -1;
    }

    uint8_t int_cfg, ctrl_3;

    int_cfg =
        (mux) ? WSEN_PADS_INT_CFG_DIFF_EN : 0 |
        (latch) ? WSEN_PADS_INT_CFG_LIR : 0 |
        (mux & WSEN_PADS_INTERRUPT_MUX_PRESSURE_HIGH) ? WSEN_PADS_INT_CFG_PHE : 0 |
        (mux & WSEN_PADS_INTERRUPT_MUX_PRESSURE_LOW) ? WSEN_PADS_INT_CFG_PLE : 0;

    ctrl_3 =
        (fifo_full) ? WSEN_PADS_CTRL_3_INT_F_FULL : 0 |
        (fifo_threshold) ? WSEN_PADS_CTRL_3_INT_F_WTM : 0 |
        (fifo_overflow) ? WSEN_PADS_CTRL_3_INT_F_OVR : 0 |
        (data_ready) ? WSEN_PADS_CTRL_3_DRDY : 0 |
        (uint8_t) mux;

    if (
        SET(
            WSEN_PADS_INT_CFG,
            int_cfg,
            WSEN_PADS_INT_CFG_DIFF_EN |
            WSEN_PADS_INT_CFG_LIR |
            WSEN_PADS_INT_CFG_PLE |
            WSEN_PADS_INT_CFG_PHE
        )

        ||

        SET(
            WSEN_PADS_CTRL_3,
            ctrl_3,
            WSEN_PADS_CTRL_3_INT_F_FULL |
            WSEN_PADS_CTRL_3_INT_F_WTM |
            WSEN_PADS_CTRL_3_INT_F_OVR |
            WSEN_PADS_CTRL_3_DRDY |
            WSEN_PADS_CTRL_3_INT_S_MASK
        )
    )
    {
        errno = EIO;
        return -1;
    }

    return 0;
}

int wsen_pads_getStatus(wsen_pads_t driver, wsen_pads_status_t *out_status, uint8_t *opt_out_fifo_level)
{
    // Read 4 registers, INT_SRC, FIFO_STATUS_1, FIFO_STATUS_2, STATUS
    ASSERT_DRV();

    if (out_status == NULL)
    {
        errno = EINVAL;
        return -1;
    }

    // It is more efficient time wise to read the FIFO_STATUS_1 as well as the others.
    uint8_t status[4];  // 0 INT_SRC
                        // 1 FIFO_STATUS_1
                        // 2 FIFO_STATUS_2
                        // 3 STATUS

    if ((driver->error = wsen_pads_read(driver->port, driver->dev, WSEN_PADS_INT_SRC, status, sizeof status)))
    {
        errno = EIO;
        return -1;
    }

    *out_status =
        (status[0] & WSEN_PADS_INT_SRC_IA) ? WSEN_PADS_STATUS_INT_ACTIVE : 0 |
        (status[0] & WSEN_PADS_INT_SRC_PH) ? WSEN_PADS_STATUS_INT_PRESSURE_HIGH : 0 |
        (status[0] & WSEN_PADS_INT_SRC_PL) ? WSEN_PADS_STATUS_INT_PRESSURE_LOW : 0 |
        (status[2] & WSEN_PADS_FIFO_STATUS_2_FIFO_FULL_IA) ? WSEN_PADS_STATUS_FIFO_FULL : 0 |
        (status[2] & WSEN_PADS_FIFO_STATUS_2_FIFO_OVER_IA) ? WSEN_PADS_STATUS_FIFO_OVERRUN : 0 |
        (status[2] & WSEN_PADS_FIFO_STATUS_2_FIFO_WTM_IA) ? WSEN_PADS_STATUS_FIFO_THRESHOLD : 0 |
        (status[3] & WSEN_PADS_STATUS_P_DA) ? WSEN_PADS_STATUS_PRESSURE_READY : 0 |
        (status[3] & WSEN_PADS_STATUS_P_OR) ? WSEN_PADS_STATUS_PRESSURE_OVERRUN : 0 |
        (status[3] & WSEN_PADS_STATUS_T_DA) ? WSEN_PADS_STATUS_TEMPERATURE_READY : 0 |
        (status[3] & WSEN_PADS_STATUS_T_OR) ? WSEN_PADS_STATUS_TEMPERATURE_OVERRUN : 0;

    if (opt_out_fifo_level)
    {
        *opt_out_fifo_level = status[1];
    }

    return 0;
}

int wsen_pads_configureData(
    wsen_pads_t driver,
    wsen_pads_odr_t dataRate,
    wsen_pads_lowpass_t lowpass,
    int low_noise,
    int blockDataUpdate
)
{
    // Writes to CTRL_1, CTRL_2
    ASSERT_DRV();

    if (
        dataRate > WSEN_PADS_DATARATE_MAX ||
        lowpass > WSEN_PADS_LOWPASS_DUAL ||
        (
            // Low noise does not offer these modes.
            low_noise &&
            (
                dataRate == WSEN_PADS_CTRL_1_ODR_100HZ ||
                dataRate == WSEN_PADS_DATARATE_200HZ
            )
        )
    )
    {
        errno = EINVAL;
        return -1;
    }

    // First, read control bits.
    uint8_t bits[2];
    if (driver->error = wsen_pads_read(driver->port, driver->dev, WSEN_PADS_CTRL_1, bits, sizeof(bits)))
    {
        errno = EIO;
        return -1;
    }

    bits[0] &= ~WSEN_PADS_CTRL_1_ODR_MASK;

    // If low noise state changes, first enter power down mode.
    // Conversion to 0,1 bools, don't remove bangs.
    if (!(bits[2] & WSEN_PADS_CTRL_2_LOW_NOISE_EN) != !low_noise)
    {
        if (WRITE(WSEN_PADS_CTRL_1, bits, 1))
        {
            errno = EIO;
            return -1;
        }
    }

    // Reset bits.
    // ODR masking alraedy achieved.
    bits[0] &= ~(WSEN_PADS_CTRL_1_EN_LPFP | WSEN_PADS_CTRL_1_LPFP_CFG | WSEN_PADS_CTRL_1_BDU);
    bits[1] &= ~WSEN_PADS_CTRL_2_LOW_NOISE_EN;

    // Set bits.
    bits[0] |=
        (dataRate << WSEN_PADS_CTRL_1_ODR_POS) |
        (blockDataUpdate ? WSEN_PADS_CTRL_1_BDU : 0);
    switch (lowpass)
    {
    // Yes the missing breaks are intentional.
    case WSEN_PADS_LOWPASS_DUAL:
        bits[0] |= WSEN_PADS_CTRL_1_LPFP_CFG;
    case WSEN_PADS_LOWPASS_SINGLE:
        bits[0] |= WSEN_PADS_CTRL_1_EN_LPFP;
    default:
    case WSEN_PADS_LOWPASS_OFF:
        break;
    }
    bits[1] = (low_noise) ? WSEN_PADS_CTRL_2_LOW_NOISE_EN : 0;

    // Write bits, in reverse order (LOW_NOISE must be set or reset first.)
    if (
        WRITE(WSEN_PADS_CTRL_2, &bits[1], 1)
        ||
        WRITE(WSEN_PADS_CTRL_1, &bits[0], 1)
    )
    {
        errno = EIO;
        return -1;
    }

    return 0;
}

int wsen_pads_autofeature(wsen_pads_t driver, int autozero, int autorefp)
{
    // Writes to INT_CFG
    ASSERT_DRV();

    if (autozero && autorefp)
    {
        errno = EINVAL;
        return -1;
    }

    if (
        SET(
            WSEN_PADS_INT_CFG,
            (autozero) ? WSEN_PADS_INT_CFG_AUTOZERO : WSEN_PADS_INT_CFG_RESET_AZ |
            (autorefp) ? WSEN_PADS_INT_CFG_AUTOREFP : WSEN_PADS_INT_CFG_RESET_ARP,
            WSEN_PADS_INT_CFG_AUTOZERO | WSEN_PADS_INT_CFG_AUTOREFP |
            WSEN_PADS_INT_CFG_RESET_AZ | WSEN_PADS_INT_CFG_RESET_ARP
        )
    )
    {
        errno = EIO;
        return -1;
    }

    return 0;
}

int wsen_pads_setPressureThresholdR(wsen_pads_t driver, uint16_t pressure)
{
    // Write to threshold pressure register.
    ASSERT_DRV();

    uint8_t val[2];
    val[0] = pressure & 0xFF;
    val[1] = pressure >> 8;

    if (WRITE(WSEN_PADS_THR_P_L, val, 2))
    {
        errno = EIO;
        return -1;
    }

    return 0;
}
