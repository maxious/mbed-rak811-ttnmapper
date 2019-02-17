#sed -i 's/#define RCC_HSICALIBRATION_DEFAULT       (0x10U)/#define RCC_HSICALIBRATION_DEFAULT       (0x11U)/' mbed-os/targets/TARGET_STM/TARGET_STM32L1/device/stm32l1xx_hal_rcc.h
#sed -i 's/define symbol __size_heap__   = 0x800;/define symbol __size_heap__   = 0x1800;/' mbed-os/targets/TARGET_STM/TARGET_STM32L1/TARGET_MTB_RAK811/device/TOOLCHAIN_IAR/stm32l152xba.icf
mbed compile -m MTB_RAK811 -t GCC_ARM
