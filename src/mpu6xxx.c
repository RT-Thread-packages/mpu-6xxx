/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-10-23     flybreak     the first version
 */

#include <mpu6xxx.h>
#include <mpu6xxx_reg.h>
#include <rtdevice.h>

#define DBG_ENABLE
#define DBG_SECTION_NAME "mpu6xxx"
#define DBG_LEVEL DBG_LOG
#define DBG_COLOR
#include <rtdbg.h>

/**
 * This function writes the value of the register for mpu6xxx
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for mpu6xxx
 * @param data value to write
 *
 * @return the writing status, RT_EOK reprensents  writing the value of the register successfully.
 */
static rt_err_t mpu6xxx_write_reg(struct mpu6xxx_device *dev, rt_uint8_t reg, rt_uint8_t data)
{
    rt_int8_t res = 0;
#ifdef RT_USING_I2C
    struct rt_i2c_msg msgs;
    rt_uint8_t buf[2] = {reg, data};
#endif
    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        msgs.addr  = dev->i2c_addr;    /* slave address */
        msgs.flags = RT_I2C_WR;        /* write flag */
        msgs.buf   = buf;              /* Send data pointer */
        msgs.len   = 2;

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, &msgs, 1) == 1)
        {
            res = RT_EOK;
        }
        else
        {
            res = -RT_ERROR;
        }
#endif
    }
    else
    {
#ifdef RT_USING_SPI
        res = rt_spi_send_then_send((struct rt_spi_device *)dev->bus, &reg, 1, &data, 1);
#endif
    }
    return res;
}

/**
 * This function reads the value of registers for mpu6xxx
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for mpu6xxx
 * @param len number of register
 * @param buf read data pointer
 *
 * @return the reading status, RT_EOK reprensents  reading the value of registers successfully.
 */
static rt_err_t mpu6xxx_read_regs(struct mpu6xxx_device *dev, rt_uint8_t reg, rt_uint8_t len, rt_uint8_t *buf)
{
    rt_int8_t res = 0;
#ifdef RT_USING_I2C
    struct rt_i2c_msg msgs[2];
#endif
#ifdef RT_USING_SPI
    rt_uint8_t tmp;
#endif
    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
#ifdef RT_USING_I2C
        msgs[0].addr  = dev->i2c_addr;    /* Slave address */
        msgs[0].flags = RT_I2C_WR;        /* Write flag */
        msgs[0].buf   = &reg;             /* Slave register address */
        msgs[0].len   = 1;                /* Number of bytes sent */

        msgs[1].addr  = dev->i2c_addr;    /* Slave address */
        msgs[1].flags = RT_I2C_RD;        /* Read flag */
        msgs[1].buf   = buf;              /* Read data pointer */
        msgs[1].len   = len;              /* Number of bytes read */

        if (rt_i2c_transfer((struct rt_i2c_bus_device *)dev->bus, msgs, 2) == 2)
        {
            res = RT_EOK;
        }
        else
        {
            res = -RT_ERROR;
        }
#endif
    }
    else
    {
#ifdef RT_USING_SPI
        //The first bit of the first byte contains the Read/Write bit and indicates the Read (1) or Write (0) operation.
        tmp = reg | 0x80;

        res = rt_spi_send_then_recv((struct rt_spi_device *)dev->bus, &tmp, 1, buf, len);
#endif
    }
    return res;
}

/**
 * This function writes a bit value of registers for mpu6xxx
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for mpu6xxx
 * @param bit the position of the register
 * @param data value to write
 *
 * @return the writing status, RT_EOK reprensents  writing a bit value of registers successfully.
 */
static rt_err_t mpu6xxx_write_bit(struct mpu6xxx_device *dev, rt_uint8_t reg, rt_uint8_t bit, rt_uint8_t data)
{
    rt_uint8_t byte;

    mpu6xxx_read_regs(dev, reg, 1, &byte);
    byte = (data != 0) ? (byte | (1 << bit)) : (byte & ~(1 << bit));

    return mpu6xxx_write_reg(dev, reg, byte);
}

/**
 * This function reads a bit value of registers for mpu6xxx
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for mpu6xxx
 * @param bit the position of the register
 * @param data read data pointer
 *
 * @return the reading status, RT_EOK reprensents  reading a bit value of registers successfully.
 */
static rt_err_t mpu6xxx_read_bit(struct mpu6xxx_device *dev, rt_uint8_t reg, rt_uint8_t bit, rt_uint8_t *data)
{
    rt_uint8_t byte;

    mpu6xxx_read_regs(dev, reg, 1, &byte);
    *data = byte & (1 << bit);

    return RT_EOK;
}

/**
 * This function writes multi-bit value of registers for mpu6xxx
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for mpu6xxx
 * @param start_bit the start position of the register
 * @param len number of bits to write
 * @param data value to write
 *
 * @return the writing status, RT_EOK reprensents  writing multi-bit value of registers successfully.
 */
static rt_err_t mpu6xxx_write_bits(struct mpu6xxx_device *dev, rt_uint8_t reg, rt_uint8_t start_bit, rt_uint8_t len, rt_uint8_t data)
{
    rt_uint8_t byte, mask;

    mpu6xxx_read_regs(dev, reg, 1, &byte);
    mask = ((1 << len) - 1) << (start_bit - len + 1);
    data <<= (start_bit - len + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    byte &= ~(mask); // zero all important bits in existing byte
    byte |= data; // combine data with existing byte

    return mpu6xxx_write_reg(dev, reg, byte);
}

/**
 * This function reads multi-bit value of registers for mpu6xxx
 *
 * @param dev the pointer of device driver structure
 * @param reg the register for mpu6xxx
 * @param start_bit the start position of the register
 * @param len number of bits to write
 * @param data read data pointer
 *
 * @return the reading status, RT_EOK reprensents  reading multi-bit value of registers successfully.
 */
static rt_err_t mpu6xxx_read_bits(struct mpu6xxx_device *dev, rt_uint8_t reg, rt_uint8_t start_bit, rt_uint8_t len, rt_uint8_t *data)
{
    rt_uint8_t byte, mask;

    mpu6xxx_read_regs(dev, reg, 1, &byte);
    mask = ((1 << len) - 1) << (start_bit - len + 1);
    byte &= mask;
    byte >>= (start_bit - len + 1);
    *data = byte;
    return RT_EOK;
}

/**
 * This function gets the raw data of the accelerometer
 *
 * @param dev the pointer of device driver structure
 * @param accel the pointer of 3axes structure for receive data
 *
 * @return the reading status, RT_EOK reprensents  reading the data successfully.
 */
static rt_err_t mpu6xxx_get_accel_raw(struct mpu6xxx_device *dev, struct mpu6xxx_3axes *accel)
{
    rt_uint8_t buffer[6];

    mpu6xxx_read_regs(dev, MPU6XXX_RA_ACCEL_XOUT_H, 6, buffer);

    accel->x = ((rt_uint16_t)buffer[0] << 8) + buffer[1];
    accel->y = ((rt_uint16_t)buffer[2] << 8) + buffer[3];
    accel->z = ((rt_uint16_t)buffer[4] << 8) + buffer[5];

    return RT_EOK;
}

/**
 * This function gets the raw data of the gyroscope
 *
 * @param dev the pointer of device driver structure
 * @param gyro the pointer of 3axes structure for receive data
 *
 * @return the reading status, RT_EOK reprensents  reading the data successfully.
 */
static rt_err_t mpu6xxx_get_gyro_raw(struct mpu6xxx_device *dev, struct mpu6xxx_3axes *gyro)
{
    rt_uint8_t buffer[6];

    mpu6xxx_read_regs(dev, MPU6XXX_RA_GYRO_XOUT_H, 6, buffer);

    gyro->x = ((rt_uint16_t)buffer[0] << 8) + buffer[1];
    gyro->y = ((rt_uint16_t)buffer[2] << 8) + buffer[3];
    gyro->z = ((rt_uint16_t)buffer[4] << 8) + buffer[5];

    return RT_EOK;
}

/**
 * This function gets the raw data of the temperature
 *
 * @param dev the pointer of device driver structure
 * @param temp read data pointer
 *
 * @return the reading status, RT_EOK reprensents  reading the data successfully.
 */
static rt_err_t mpu6xxx_get_temp_raw(struct mpu6xxx_device *dev, rt_int16_t *temp)
{
    rt_uint8_t buffer[2];

    mpu6xxx_read_regs(dev, MPU6XXX_RA_TEMP_OUT_H, 2, buffer);

    *temp = ((rt_uint16_t)buffer[0] << 8) + buffer[1];

    return RT_EOK;
}

/**
 * This function gets mpu6xxx parameters.
 *
 * @param dev the pointer of device driver structure
 * @param cmd Configuration item
 * @param param read data pointer
 *
 * @return the reading status, RT_EOK reprensents  reading the data successfully.
 */
static rt_err_t mpu6xxx_get_param(struct mpu6xxx_device *dev, enum mpu6xxx_cmd cmd, rt_uint16_t *param)
{
    rt_uint8_t data = 0;

    RT_ASSERT(dev);

    switch (cmd)
    {
    case MPU6XXX_GYRO_RANGE:  /* Gyroscope full scale range */
        mpu6xxx_read_bits(dev, MPU6XXX_RA_GYRO_CONFIG, MPU6XXX_GCONFIG_FS_SEL_BIT, MPU6XXX_GCONFIG_FS_SEL_LENGTH, &data);
        *param = data;
        break;
    case MPU6XXX_ACCEL_RANGE: /* Accelerometer full scale range */
        mpu6xxx_read_bits(dev, MPU6XXX_RA_ACCEL_CONFIG, MPU6XXX_ACONFIG_AFS_SEL_BIT, MPU6XXX_ACONFIG_AFS_SEL_LENGTH, &data);
        *param = data;
        break;
    case MPU6XXX_DLPF_CONFIG: /* Digital Low Pass Filter */
        mpu6xxx_read_bits(dev, MPU6XXX_RA_CONFIG, MPU6XXX_CFG_DLPF_CFG_BIT, MPU6XXX_CFG_DLPF_CFG_LENGTH, &data);
        *param = data;
        break;
    case MPU6XXX_SAMPLE_RATE: /* Sample Rate */
        /* Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV) */
        mpu6xxx_read_bits(dev, MPU6XXX_RA_CONFIG, MPU6XXX_CFG_DLPF_CFG_BIT, MPU6XXX_CFG_DLPF_CFG_LENGTH, &data);

        if (data == 0 || data == 7) /* dlpf is disable */
        {
            mpu6xxx_read_regs(dev, MPU6XXX_RA_SMPLRT_DIV, 1, &data);
            *param = 8000 / (data + 1);
        }
        else /* dlpf is enable */
        {
            mpu6xxx_read_regs(dev, MPU6XXX_RA_SMPLRT_DIV, 1, &data);
            *param = 1000 / (data + 1);
        }
        break;
    case MPU6XXX_SLEEP: /* sleep mode */
        mpu6xxx_read_bit(dev, MPU6XXX_RA_PWR_MGMT_1, MPU6XXX_PWR1_SLEEP_BIT, &data);
        *param = data;
        break;
    }

    return RT_EOK;
}
/**
 * This function set mpu6xxx parameters.
 *
 * @param dev the pointer of device driver structure
 * @param cmd Configuration item
 * @param param Configuration item parameter
 *
 * @return the setting status, RT_EOK reprensents  setting the parameter successfully.
 */
rt_err_t mpu6xxx_set_param(struct mpu6xxx_device *dev, enum mpu6xxx_cmd cmd, rt_uint16_t param)
{
    rt_uint8_t data = 0;

    RT_ASSERT(dev);

    switch (cmd)
    {
    case MPU6XXX_GYRO_RANGE:  /* Gyroscope full scale range */
        mpu6xxx_write_bits(dev, MPU6XXX_RA_GYRO_CONFIG, MPU6XXX_GCONFIG_FS_SEL_BIT, MPU6XXX_GCONFIG_FS_SEL_LENGTH, param);
        dev->config.gyro_range = param;
        break;
    case MPU6XXX_ACCEL_RANGE: /* Accelerometer full scale range */
        mpu6xxx_write_bits(dev, MPU6XXX_RA_ACCEL_CONFIG, MPU6XXX_ACONFIG_AFS_SEL_BIT, MPU6XXX_ACONFIG_AFS_SEL_LENGTH, param);
        dev->config.accel_range = param;
        break;
    case MPU6XXX_DLPF_CONFIG: /* Digital Low Pass Filter */
        mpu6xxx_write_bits(dev, MPU6XXX_RA_CONFIG, MPU6XXX_CFG_DLPF_CFG_BIT, MPU6XXX_CFG_DLPF_CFG_LENGTH, param);
        break;
    case MPU6XXX_SAMPLE_RATE: /* Sample Rate —— 16-bit unsigned value.
                                 Sample Rate = [1000 -  4]HZ when dlpf is enable
                                 Sample Rate = [8000 - 32]HZ when dlpf is disable */

        //Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
        mpu6xxx_read_bits(dev, MPU6XXX_RA_CONFIG, MPU6XXX_CFG_DLPF_CFG_BIT, MPU6XXX_CFG_DLPF_CFG_LENGTH, &data);

        if (data == 0 || data == 7) /* dlpf is disable */
        {
            if (param > 8000)
                data = 0;
            else if (param < 32)
                data = 0xFF;
            else
                data = 8000 / param - 1;
        }
        else /* dlpf is enable */
        {
            if (param > 1000)
                data = 0;
            else if (param < 4)
                data = 0xFF;
            else
                data = 1000 / param - 1;
        }
        mpu6xxx_write_reg(dev, MPU6XXX_RA_SMPLRT_DIV, data);
        break;
    case MPU6XXX_SLEEP: /* Configure sleep mode */
        mpu6xxx_write_bit(dev, MPU6XXX_RA_PWR_MGMT_1, MPU6XXX_PWR1_SLEEP_BIT, param);
        break;
    }

    return RT_EOK;
}

/**
* This function gets the data of the accelerometer, unit: mg
 *
 * @param dev the pointer of device driver structure
 * @param accel the pointer of 3axes structure for receive data
 *
 * @return the reading status, RT_EOK reprensents  reading the data successfully.
 */
rt_err_t mpu6xxx_get_accel(struct mpu6xxx_device *dev, struct mpu6xxx_3axes *accel)
{
    struct mpu6xxx_3axes tmp;
    rt_uint16_t sen;

    mpu6xxx_get_accel_raw(dev, &tmp);

    switch (dev->config.accel_range)
    {
    case MPU6XXX_ACCEL_RANGE_2G:
        sen = 16384;
        break;
    case MPU6XXX_ACCEL_RANGE_4G:
        sen = 8192;
        break;
    case MPU6XXX_ACCEL_RANGE_8G:
        sen = 4096;
        break;
    case MPU6XXX_ACCEL_RANGE_16G:
        sen = 2048;
        break;
    }

    accel->x = (rt_int32_t)tmp.x * 1000 / sen;
    accel->y = (rt_int32_t)tmp.y * 1000 / sen;
    accel->z = (rt_int32_t)tmp.z * 1000 / sen;

    return RT_EOK;
}

/**
* This function gets the data of the gyroscope, unit: deg/10s
 *
 * @param dev the pointer of device driver structure
 * @param gyro the pointer of 3axes structure for receive data
 *
 * @return the reading status, RT_EOK reprensents  reading the data successfully.
 */
rt_err_t mpu6xxx_get_gyro(struct mpu6xxx_device *dev, struct mpu6xxx_3axes *gyro)
{
    struct mpu6xxx_3axes tmp;
    rt_uint16_t sen;

    mpu6xxx_get_gyro_raw(dev, &tmp);

    switch (dev->config.gyro_range)
    {
    case MPU6XXX_GYRO_RANGE_250DPS:
        sen = 1310;
        break;
    case MPU6XXX_GYRO_RANGE_500DPS:
        sen = 655;
        break;
    case MPU6XXX_GYRO_RANGE_1000DPS:
        sen = 328;
        break;
    case MPU6XXX_GYRO_RANGE_2000DPS:
        sen = 164;
        break;
    }
    gyro->x = (rt_int32_t)tmp.x * 100 / sen;
    gyro->y = (rt_int32_t)tmp.y * 100 / sen;
    gyro->z = (rt_int32_t)tmp.z * 100 / sen;

    return RT_EOK;
}

/**
 * This function gets the data of the temperature, unit: Centigrade
 *
 * @param dev the pointer of device driver structure
 * @param temp read data pointer
 *
 * @return the reading status, RT_EOK reprensents  reading the data successfully.
 */
rt_err_t mpu6xxx_get_temp(struct mpu6xxx_device *dev, float *temp)
{
    rt_int16_t tmp;

    mpu6xxx_get_temp_raw(dev, &tmp);

    /* mpu60x0: Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53 */
    *temp = (double)tmp / 340 + 36.53;

    return RT_EOK;
}

/**
 * This function initialize the mpu6xxx device.
 *
 * @param dev_name the name of transfer device
 * @param param the i2c device address for i2c communication
 *
 * @return the pointer of device driver structure, RT_NULL reprensents  initialization failed.
 */
struct mpu6xxx_device *mpu6xxx_init(const char *dev_name, rt_uint8_t param)
{
    struct mpu6xxx_device *dev = RT_NULL;
    rt_uint8_t reg = 0xFF;
    RT_ASSERT(dev_name);

    dev = rt_calloc(1, sizeof(struct mpu6xxx_device));
    if (dev == RT_NULL)
    {
        LOG_E("Can't allocate memory for mpu6xxx device on '%s' ", dev_name);
        rt_free(dev);

        return RT_NULL;
    }

    dev->bus = rt_device_find(dev_name);
    if (dev->bus == RT_NULL)
    {
        LOG_E("Can't find device:'%s'", dev_name);
        rt_free(dev);

        return RT_NULL;
    }

    if (dev->bus->type == RT_Device_Class_I2CBUS)
    {
        dev->i2c_addr = param;
    }

    if (mpu6xxx_read_regs(dev, MPU6XXX_RA_WHO_AM_I, 1, &reg) != RT_EOK)
    {
        LOG_E("Failed to read device id!");
        return RT_NULL;
    }

    dev->id = reg;

    switch (dev->id)
    {
    case MPU6050_WHO_AM_I:
        LOG_I("Find device: mpu6050!");
        break;
    case MPU6500_WHO_AM_I:
        LOG_I("Find device: mpu6500!");
        break;
    case MPU9250_WHO_AM_I:
        LOG_I("Find device: mpu9250!");
        break;
    case ICM20608_WHO_AM_I:
        LOG_I("Find device: icm20608!");
        break;
    case 0xFF:
        LOG_E("No device connection!");
        return RT_NULL;
    default:
        LOG_W("Unknown device id: 0x%x!", reg);
    }

    mpu6xxx_get_param(dev, MPU6XXX_ACCEL_RANGE, &dev->config.accel_range);
    mpu6xxx_get_param(dev, MPU6XXX_GYRO_RANGE, &dev->config.gyro_range);

    mpu6xxx_write_bits(dev, MPU6XXX_RA_PWR_MGMT_1, MPU6XXX_PWR1_CLKSEL_BIT, MPU6XXX_PWR1_CLKSEL_LENGTH, MPU6XXX_CLOCK_PLL_XGYRO);
    mpu6xxx_set_param(dev, MPU6XXX_GYRO_RANGE, MPU6XXX_GYRO_RANGE_250DPS);
    mpu6xxx_set_param(dev, MPU6XXX_ACCEL_RANGE, MPU6XXX_ACCEL_RANGE_2G);
    mpu6xxx_set_param(dev, MPU6XXX_SLEEP, MPU6XXX_SLEEP_DISABLE);

    return dev;
}
