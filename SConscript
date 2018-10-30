from building import *
Import('rtconfig')

src   = []
cwd   = GetCurrentDir()

# add mpu6xxx src files.
if GetDepend('PKG_USING_MPU6XXX'):
    src += Glob('src/mpu6xxx.c')

if GetDepend('PKG_USING_MPU6XXX_SAMPLE_I2C'):
    src += Glob('samples/mpu6xxx_sample_i2c.c')

if GetDepend('PKG_USING_MPU6XXX_SAMPLE_SPI'):
    src += Glob('samples/mpu6xxx_sample_spi.c')

# add mpu6xxx include path.
path  = [cwd + '/inc']

# add src and include to group.
group = DefineGroup('mpu6xxx', src, depend = ['PKG_USING_MPU6XXX'], CPPPATH = path)

Return('group')