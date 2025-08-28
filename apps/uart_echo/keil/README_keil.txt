
Keil/uVision template notes:
- Device: STM32F407VG (adjust as needed)
- You MUST add vendor files:
  * CMSIS: core_cm4.h, startup_stm32f407xx.s, system_stm32f4xx.c
  * STM32CubeF4 LL/HAL includes as referenced by platform code
- Include paths:
  - ../../../../core/include
  - ../../../../services/at_engine/include
  - ../../../../platform/mcu/stm32f4/include
  - ../../../../platform/boards/stm32f407-custom
  - ../../../../platform/os/baremetal/include
  - (plus your CMSIS and Cube paths)
- Source groups already reference repo files; add vendor sources to a "Vendor" group.
