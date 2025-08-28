
# Placing UART Rings in a Custom Memory Region

## GCC / arm-none-eabi-gcc
1. Add `__attribute__((section(".uartbufs")))` to your override buffers.
2. Include `tooling/linker/gcc/uartbufs.ld` in your link command or merge it into your main linker script.
3. Map `.uartbufs` to your desired region (e.g., DTCM/CCRAM).

## Keil uVision (ARMCC/ARMCLANG)
1. Mark buffers with `__attribute__((section(".uartbufs")))`.
2. In your scatter file, add a region to collect `.uartbufs` (see `tooling/linker/armcc/uartbufs.sct`).
3. Point the project to use the modified scatter file (Options for Target → Linker → Use Scatter File).
