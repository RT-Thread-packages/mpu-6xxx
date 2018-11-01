# mpu6xxx 驱动库

## 简介

这是一个支持 IIC、SPI 的 mpu6xxx 驱动库的软件包，兼容 mpu6000、mpu6050、mpu6500、mpu9250、icm20608等芯片。

### 目录结构

```
mpu6xxx
│   README.md                       // 软件包说明
│   SConscript                      // RT-Thread 默认的构建脚本
│   LICENSE                         // 许可证文件
├───docs 
│   |   api.md                      // API 使用说明
├───samples                         // 示例代码
│   |   mpu6xxx_sample.c            // 软件包使用示例代码
└───src                             // 源文件
└───inc                             // 头文件
```

### 许可证

mpu6xxx 遵循 Apache-2.0 许可，详见 `LICENSE` 文件。

### 依赖

- RT_Thread 3.0+
- i2c/spi 设备驱动

## 获取方式

使用 `mpu6xxx package` 需要在 RT-Thread 的包管理中选中它，具体路径如下：

```
RT-Thread online packages
    peripheral libraries and drivers  --->
        mpu6xxx: Universal 6-axis sensor driver library  --->
```

进入 mpu6xxx 软件包的配置菜单按自己的需求进行具体的配置

```
    --- mpu6xxx: Universal 6-axis sensor driver library                           
        [*]   Enable mpu6xxx sample
           Version (latest)  --->
```

**Enable mpu6xxx sample** ：开启 mpu6xxx  使用示例

配置完成后让 RT-Thread 的包管理器自动更新，或者使用 pkgs --update 命令更新包到 BSP 中。

## 使用 mpu6xxx 软件包

mpu6xxx 软件包的使用流程一般如下：

1. 初始化 mpu6xxx 设备 `mpu6xxx_init`
2. 配置相关参数 `mpu6xxx_set_param` (也可以不调用，使用默认参数)
3. 读取传感器数据 `mpu6xxx_get_gyro/accel/temp`

详细的使用方法可以参考[mpu6xxx 示例程序](samples/mpu6xxx_sample.c) 和 [API 说明文档](doc/api.md)。

## 联系方式

- 维护：[guozhanxin](https://github.com/Guozhanxin)
- 主页：<https://github.com/RT-Thread-packages/mpu-6xxx>