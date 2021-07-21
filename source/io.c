#include "private.h"
#include <assert.h>

esp_err_t wsen_pads_read(i2c_port_t port, uint8_t dev, uint8_t addr, void *buffer, size_t size)
{
    assert(port <= I2C_NUM_MAX);
    assert(buffer);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd)
    {
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, dev | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, addr, true);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, dev | I2C_MASTER_READ, true);
        i2c_master_read(cmd, (uint8_t*)buffer, size, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);

        esp_err_t err = i2c_master_cmd_begin(port, cmd, 20);

        i2c_cmd_link_delete(cmd);

        if (err)
            errno = EIO;

        return err;
    }
    else
    {
        return ESP_ERR_NO_MEM;
    }
}

esp_err_t wsen_pads_write(i2c_port_t port, uint8_t dev, uint8_t addr, const void *buffer, size_t size)
{
    i2c_cmd_handle_t cmd;
    esp_err_t err;

    assert(port <= I2C_NUM_MAX);
    assert(buffer);

    cmd = i2c_cmd_link_create();
    if (cmd)
    {
        for (int i = 0; i < size; ++i)
        {
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, dev | I2C_MASTER_WRITE, true);
            i2c_master_write_byte(cmd, addr, true);
            i2c_master_write_byte(cmd, ((uint8_t*)buffer)[i], true);
        }
        i2c_master_stop(cmd);

        err = i2c_master_cmd_begin(port, cmd, 20);

        i2c_cmd_link_delete(cmd);

        if (err)
            errno = EIO;

        return err;
    }
    else
    {
        return ESP_ERR_NO_MEM;
    }
}

esp_err_t wsen_pads_setbits(i2c_port_t port, uint8_t dev, uint8_t addr, uint8_t set, uint8_t reset)
{
    uint8_t bitfield;
    esp_err_t err;

    err = wsen_pads_read(port, dev, addr, &bitfield, 1);
    if (err) return err;

    bitfield = (bitfield & ~reset) | set;

    return wsen_pads_write(port, dev, addr, &bitfield, 1);
}

int wsen_pads_getbits(i2c_port_t port, uint8_t dev, uint8_t addr, uint8_t mask, esp_err_t* err)
{
    uint8_t bitfield;

    assert(err);

    *err = wsen_pads_read(port, dev, addr, &bitfield, 1);
    if (*err) return -1;

    return bitfield & mask;
}