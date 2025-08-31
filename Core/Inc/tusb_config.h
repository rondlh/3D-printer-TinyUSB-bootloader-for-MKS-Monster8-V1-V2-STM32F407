#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
 extern "C" {
#endif

// Common Configuration
#define CFG_TUSB_MCU         	    OPT_MCU_STM32F4 // MCU family, important!!!
#define CFG_TUSB_OS                 OPT_OS_NONE     // Operating system used (NONE/RTOS)

// USB DEVICE
//#define CFG_TUD_ENABLED           1 // Enable USB device
//#define BOARD_TUD_RHPORT  	    1 // Root hub port number
//#define BOARD_TUD_MAX_SPEED   	(OPT_MODE_FULL_SPEED)
//#define CFG_TUSB_RHPORT1_MODE     (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)

 // USB HOST
#define CFG_TUH_ENABLED             1 // Enable USB host
#define BOARD_TUH_RHPORT            1 // Root hub port number
#define BOARD_TUH_MAX_SPEED         (OPT_MODE_FULL_SPEED)
#define CFG_TUSB_RHPORT1_MODE       (OPT_MODE_HOST | OPT_MODE_FULL_SPEED)
#define CFG_TUH_MAX_SPEED           (OPT_MODE_FULL_SPEED) // Max speed the controller can use with internal PHY
#define CFG_TUH_MSC_MAXLUN          4 // MSC LUN, typical for most card reader

// SUPPORTED DEVICE TYPES
#define CFG_TUH_HUB                 0 // number of supported hubs
#define CFG_TUH_MSC                 1 // number of supported MSC
#define CFG_TUH_CDC                 0 // number of supported CDCs
#define CFG_TUH_HID                 0 // typical keyboard + mouse device can have 3-4 HID interfaces
#define CFG_TUH_VENDOR              0 // vendor specific devices

// max device support (excluding hub device): 1 hub typically has 4 ports
#define CFG_TUH_DEVICE_MAX          (3*CFG_TUH_HUB + 1)

/* USB DMA on some MCUs can only access a specific SRAM region with restriction on alignment.
 * Tinyusb use follows macros to declare transferring memory so that they can be put
 * into those specific section.
 * e.g
 * - CFG_TUSB_MEM SECTION : __attribute__ (( section(".usb_ram") ))
 * - CFG_TUSB_MEM_ALIGN   : __attribute__ ((aligned(4)))
 */
#ifndef CFG_TUH_MEM_SECTION
    #define CFG_TUH_MEM_SECTION
#endif
#ifndef CFG_TUH_MEM_ALIGN
    #define CFG_TUH_MEM_ALIGN     __attribute__ ((aligned(4)))
#endif

// DRIVER CONFIG
#define CFG_TUH_ENUMERATION_BUFSIZE 256 // Size of buffer to hold descriptors etc.

// DEBUG LEVEL 0-3
#define CFG_TUSB_DEBUG            0

#ifdef __cplusplus
 }
#endif

#endif /* _TUSB_CONFIG_H_ */
