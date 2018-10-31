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
#include <rtthread.h>
#include <string.h>
#include <stdlib.h>

/* Default configuration, please change according to the actual situation */
#define MPU6XXX_BUS   "i2c2"
#define MPU6XXX_ADDR  0x68

#ifndef RT_USING_I2C
    #error Please open the i2c device!
#endif

static struct mpu6xxx_device *mpu6xxx_dev = RT_NULL;

/* Test function */
static int mpu6xxx_test(rt_uint16_t num)
{
    struct mpu6xxx_3axes accel, gyro;
    float temp;

    if (mpu6xxx_dev == RT_NULL)
    {
        rt_kprintf("Please probe mpu6xxx first!\n");
        return -1;
    }

    while (num --)
    {
        mpu6xxx_get_accel(mpu6xxx_dev, &accel);
        mpu6xxx_get_gyro(mpu6xxx_dev, &gyro);
        mpu6xxx_get_temp(mpu6xxx_dev, &temp);

        rt_kprintf("accel.x = %3d, accel.y = %3d, accel.z = %3d ", accel.x, accel.y, accel.z);
        rt_kprintf("gyro.x = %3d gyro.y = %3d, gyro.z = %3d, ", gyro.x, gyro.y, gyro.z);
        rt_kprintf("temp = %d.%d\n", (int)(temp * 100) / 100, (int)(temp * 100) % 100);

        rt_thread_mdelay(100);
    }
    return 0;
}

/* Initialize and configure mpu6xxx */
static int mpu6xxx_probe(void)
{
    /* Initialize mpu6xxx, i2c communication requires incoming device addresses */
    mpu6xxx_dev = mpu6xxx_init(MPU6XXX_BUS, MPU6XXX_ADDR);

    if (mpu6xxx_dev == RT_NULL)
    {
        rt_kprintf("mpu6xxx init failed\n");
        return -1;
    }

    /* To configure the frequency of the low pass filter is 188HZ */
    mpu6xxx_set_param(mpu6xxx_dev, MPU6XXX_DLPF_CONFIG, MPU6XXX_DLPF_188HZ);

    /* To configure the frequency of the sample rate is 100HZ */
    mpu6xxx_set_param(mpu6xxx_dev, MPU6XXX_SAMPLE_RATE, 100);

    rt_kprintf("mpu6xxx init succeed\n");

    return 0;
}

/* mpu6xxx control function */
void mpu6xxx_i2c(int argc, char **argv)
{
    /* If the number of arguments less than 2 */
    if (argc < 2)
    {
        rt_kprintf("\n");
        rt_kprintf("mpu6xxx_i2c [OPTION] [PARAM]\n");
        rt_kprintf("            probe                 Probe and init mpu6xxx\n");
        rt_kprintf("            sr <var>              Set sample rate to var\n");
        rt_kprintf("                                  var = [1000 -  4] when dlpf is enable\n");
        rt_kprintf("                                  var = [8000 - 32] when dlpf is disable\n");
        rt_kprintf("            gr <var>              Set gyro range to var\n");
        rt_kprintf("                                  var = [0 - 3] means [250 - 2000DPS]\n");
        rt_kprintf("            ar <var>              Set accel range to var\n");
        rt_kprintf("                                  var = [0 - 3] means [2 - 16G]\n");
        rt_kprintf("            sleep <var>           Set sleep status\n");
        rt_kprintf("                                  var = 0 means disable, = 1 means enable\n");
        rt_kprintf("            test [num]            Test [num] times mpu6xxx\n");
        rt_kprintf("                                  num default 5\n");
        return ;
    }
    else if (argc == 2)
    {
        if (!strcmp(argv[1], "probe"))
        {
            mpu6xxx_probe();
        }
        else if (!strcmp(argv[1], "test"))
        {
            mpu6xxx_test(5);
        }
    }
    else if (argc == 3)
    {
        if (!strcmp(argv[1], "sr"))
        {
            mpu6xxx_set_param(mpu6xxx_dev, MPU6XXX_SAMPLE_RATE, strtoul(argv[2], 0, 10));
        }
        else if (!strcmp(argv[1], "sleep"))
        {
            mpu6xxx_set_param(mpu6xxx_dev, MPU6XXX_SLEEP, strtoul(argv[2], 0, 10));
        }
        else if (!strcmp(argv[1], "gr"))
        {
            mpu6xxx_set_param(mpu6xxx_dev, MPU6XXX_GYRO_RANGE, strtoul(argv[2], 0, 10));
        }
        else if (!strcmp(argv[1], "ar"))
        {
            mpu6xxx_set_param(mpu6xxx_dev, MPU6XXX_ACCEL_RANGE, strtoul(argv[2], 0, 10));
        }
        else if (!strcmp(argv[1], "test"))
        {
            mpu6xxx_test(strtoul(argv[2], 0, 10));
        }
    }
}
MSH_CMD_EXPORT(mpu6xxx_i2c, mpu6xxx sensor function);