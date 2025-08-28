
/* Example GCC linker snippet: place .uartbufs in CCM/DTCM/etc. Include with: -T uartbufs.ld */
SECTIONS
{
  .uartbufs (NOLOAD) :
  {
    . = ALIGN(4);
    *(.uartbufs)
    . = ALIGN(4);
  } >RAM
}
