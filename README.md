Bootloader for MKS Monster 8 V1/V2 using the onboard USB-A interface.

A bit smaller than the STM32 FATFS version.

The used USB stack is TinyUSB, instead of the ST's own USB middleware.
https://ejaaskel.dev/making-usb-device-with-stm32-tinyusb/

The steps to integrate the TinyUSB to the STM32CubeIde project follow this GitHub answer:
https://github.com/hathach/tinyusb/discussions/633#discussioncomment-342237
