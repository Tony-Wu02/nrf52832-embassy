MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  FLASH : ORIGIN = 0x00000000, LENGTH = 512K
  RAM : ORIGIN = 0x20000000, LENGTH = 64K

  /* These values correspond to the NRF52832 with Softdevices S132 7.3.0 */
  /*
     FLASH : ORIGIN = 0x00027000, LENGTH = 456K
     RAM : ORIGIN = 0x20020000, LENGTH = 32K
  */
} 