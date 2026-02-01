/* gpio_led_ioctl.h
 *
 * Shared header between kernel driver and user-space application.
 * Defines IOCTL commands for GPIO LED control on Raspberry Pi 4.
 *
 * Hardware: LED on Physical Pin 16 (BCM GPIO 23)
 */

#ifndef _GPIO_LED_IOCTL_H
#define _GPIO_LED_IOCTL_H

#ifdef __KERNEL__
#include <linux/ioctl.h>
#else
#include <sys/ioctl.h>
#endif

/* Magic number for our driver - pick a unique one (check Documentation/ioctl/ioctl-number.txt) */
#define GPIO_LED_MAGIC  'L'

/* LED states */
#define LED_OFF         0
#define LED_ON          1

/* IOCTL Commands
 *
 * _IO    : No data transfer
 * _IOW   : Write data to driver (user -> kernel)
 * _IOR   : Read data from driver (kernel -> user)
 * _IOWR  : Read/Write data
 *
 * Arguments: (magic, command_number, data_type)
 */

/* Turn LED ON */
#define GPIO_LED_SET_ON         _IO(GPIO_LED_MAGIC, 0)

/* Turn LED OFF */
#define GPIO_LED_SET_OFF        _IO(GPIO_LED_MAGIC, 1)

/* Toggle LED state */
#define GPIO_LED_TOGGLE         _IO(GPIO_LED_MAGIC, 2)

/* Get current LED state: returns 0 (OFF) or 1 (ON) */
#define GPIO_LED_GET_STATE      _IOR(GPIO_LED_MAGIC, 3, int)

/* Set LED state with value: 0 = OFF, 1 = ON */
#define GPIO_LED_SET_STATE      _IOW(GPIO_LED_MAGIC, 4, int)

/* Set blink: pass period in milliseconds */
#define GPIO_LED_SET_BLINK      _IOW(GPIO_LED_MAGIC, 5, int)

/* Stop blinking */
#define GPIO_LED_STOP_BLINK     _IO(GPIO_LED_MAGIC, 6)

/* Maximum IOCTL command number (for validation) */
#define GPIO_LED_MAX_CMD        6

/* Device name and path */
#define DEVICE_NAME             "gpio_led"
#define DEVICE_PATH             "/dev/" DEVICE_NAME

/* BCM GPIO pin number */
#define LED_GPIO_BCM            23

#endif /* _GPIO_LED_IOCTL_H */
