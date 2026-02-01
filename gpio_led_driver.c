/*
 * gpio_led_driver.c
 *
 * Linux Kernel Device Driver for GPIO LED on Raspberry Pi 4
 *
 * Controls an LED connected to Physical Pin 16 (BCM GPIO 23).
 * Provides a character device interface with IOCTL support for:
 *   - ON / OFF / Toggle
 *   - Get / Set state
 *   - Blink with configurable period
 *
 * Target: Raspberry Pi 4 running a mainline-compatible kernel (5.x / 6.x)
 * Build:  Cross-compile or native build with kernel headers
 *
 * Author:  Anjaneya
 * License: GPL v2
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/gpio.h>          /* Legacy GPIO API (gpio_request, etc.) */
#include <linux/gpio/consumer.h> /* Descriptor-based GPIO API            */
#include <linux/uaccess.h>       /* copy_to_user / copy_from_user        */
#include <linux/timer.h>         /* Kernel timers for blink              */
#include <linux/mutex.h>
#include <linux/err.h>

#include "gpio_led_ioctl.h"

/* --------------------------------------------------------------------------
 * Module metadata
 * -------------------------------------------------------------------------- */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Anjaneya");
MODULE_DESCRIPTION("GPIO LED Driver for Raspberry Pi 4 (BCM GPIO 23) with IOCTL");
MODULE_VERSION("1.0");

/* --------------------------------------------------------------------------
 * Driver private data
 * -------------------------------------------------------------------------- */
struct gpio_led_dev {
    dev_t           devno;          /* Major:Minor device number          */
    struct cdev     cdev;           /* Character device structure          */
    struct class    *class;         /* Device class (for /sys/class entry) */
    struct device   *device;        /* Device (for /dev entry via udev)   */
    struct gpio_desc *gpio;         /* GPIO descriptor (new API)          */

    int             led_state;      /* Current LED state: 0=OFF, 1=ON    */
    struct mutex    lock;           /* Protects led_state and GPIO access */

    /* Blink support */
    struct timer_list blink_timer;  /* Kernel timer for blinking          */
    int             blinking;       /* 1 = blink active, 0 = stopped     */
    unsigned long   blink_period_jiffies; /* Half-period in jiffies       */
};

static struct gpio_led_dev *led_dev;

/* --------------------------------------------------------------------------
 * Low-level GPIO helpers (called with mutex held)
 * -------------------------------------------------------------------------- */
static void gpio_led_set(struct gpio_led_dev *dev, int state)
{
    dev->led_state = !!state;  /* Normalize to 0 or 1 */
    gpiod_set_value(dev->gpio, dev->led_state);
}

static int gpio_led_get(struct gpio_led_dev *dev)
{
    return dev->led_state;
}

static void gpio_led_toggle(struct gpio_led_dev *dev)
{
    gpio_led_set(dev, !dev->led_state);
}

/* --------------------------------------------------------------------------
 * Blink timer callback
 * -------------------------------------------------------------------------- */
static void blink_timer_callback(struct timer_list *t)
{
    struct gpio_led_dev *dev = from_timer(dev, t, blink_timer);

    /*
     * Timer callbacks run in softirq context — we cannot use mutex here.
     * Direct GPIO access is safe for simple output GPIOs on RPi.
     */
    dev->led_state = !dev->led_state;
    gpiod_set_value(dev->gpio, dev->led_state);

    /* Re-arm the timer if still blinking */
    if (dev->blinking)
        mod_timer(&dev->blink_timer, jiffies + dev->blink_period_jiffies);
}

static void start_blink(struct gpio_led_dev *dev, int period_ms)
{
    dev->blink_period_jiffies = msecs_to_jiffies(period_ms / 2);
    if (dev->blink_period_jiffies < 1)
        dev->blink_period_jiffies = 1;

    dev->blinking = 1;
    mod_timer(&dev->blink_timer, jiffies + dev->blink_period_jiffies);

    pr_info("gpio_led: Blink started, period = %d ms\n", period_ms);
}

static void stop_blink(struct gpio_led_dev *dev)
{
    dev->blinking = 0;
    del_timer_sync(&dev->blink_timer);

    pr_info("gpio_led: Blink stopped\n");
}

/* --------------------------------------------------------------------------
 * File operations: open / release / ioctl / read / write
 * -------------------------------------------------------------------------- */
static int gpio_led_open(struct inode *inode, struct file *filp)
{
    filp->private_data = led_dev;
    pr_info("gpio_led: Device opened\n");
    return 0;
}

static int gpio_led_release(struct inode *inode, struct file *filp)
{
    pr_info("gpio_led: Device closed\n");
    return 0;
}

/*
 * read() — returns '0' or '1' followed by newline (human-readable state).
 * Allows: cat /dev/gpio_led
 */
static ssize_t gpio_led_read(struct file *filp, char __user *buf,
                              size_t count, loff_t *ppos)
{
    struct gpio_led_dev *dev = filp->private_data;
    char state_str[4];
    int len;

    if (*ppos > 0)
        return 0;  /* EOF on subsequent reads (simple implementation) */

    mutex_lock(&dev->lock);
    len = snprintf(state_str, sizeof(state_str), "%d\n", dev->led_state);
    mutex_unlock(&dev->lock);

    if (count < len)
        return -EINVAL;

    if (copy_to_user(buf, state_str, len))
        return -EFAULT;

    *ppos += len;
    return len;
}

/*
 * write() — accepts '0' or '1' to set LED state.
 * Allows: echo 1 > /dev/gpio_led
 */
static ssize_t gpio_led_write(struct file *filp, const char __user *buf,
                               size_t count, loff_t *ppos)
{
    struct gpio_led_dev *dev = filp->private_data;
    char val;

    if (count < 1)
        return -EINVAL;

    if (copy_from_user(&val, buf, 1))
        return -EFAULT;

    mutex_lock(&dev->lock);
    if (val == '1')
        gpio_led_set(dev, LED_ON);
    else if (val == '0')
        gpio_led_set(dev, LED_OFF);
    else {
        mutex_unlock(&dev->lock);
        return -EINVAL;
    }
    mutex_unlock(&dev->lock);

    pr_info("gpio_led: LED set to %c via write()\n", val);
    return count;
}

/*
 * ioctl() — the main control interface
 */
static long gpio_led_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct gpio_led_dev *dev = filp->private_data;
    int state;
    int period_ms;
    int ret = 0;

    /* Validate magic number and command range */
    if (_IOC_TYPE(cmd) != GPIO_LED_MAGIC)
        return -ENOTTY;
    if (_IOC_NR(cmd) > GPIO_LED_MAX_CMD)
        return -ENOTTY;

    mutex_lock(&dev->lock);

    switch (cmd) {

    case GPIO_LED_SET_ON:
        stop_blink(dev);
        gpio_led_set(dev, LED_ON);
        pr_info("gpio_led: IOCTL SET_ON\n");
        break;

    case GPIO_LED_SET_OFF:
        stop_blink(dev);
        gpio_led_set(dev, LED_OFF);
        pr_info("gpio_led: IOCTL SET_OFF\n");
        break;

    case GPIO_LED_TOGGLE:
        stop_blink(dev);
        gpio_led_toggle(dev);
        pr_info("gpio_led: IOCTL TOGGLE -> %d\n", dev->led_state);
        break;

    case GPIO_LED_GET_STATE:
        state = gpio_led_get(dev);
        if (copy_to_user((int __user *)arg, &state, sizeof(int))) {
            ret = -EFAULT;
            break;
        }
        pr_info("gpio_led: IOCTL GET_STATE -> %d\n", state);
        break;

    case GPIO_LED_SET_STATE:
        if (copy_from_user(&state, (int __user *)arg, sizeof(int))) {
            ret = -EFAULT;
            break;
        }
        if (state != LED_ON && state != LED_OFF) {
            ret = -EINVAL;
            break;
        }
        stop_blink(dev);
        gpio_led_set(dev, state);
        pr_info("gpio_led: IOCTL SET_STATE -> %d\n", state);
        break;

    case GPIO_LED_SET_BLINK:
        if (copy_from_user(&period_ms, (int __user *)arg, sizeof(int))) {
            ret = -EFAULT;
            break;
        }
        if (period_ms < 50 || period_ms > 10000) {
            pr_err("gpio_led: Blink period out of range [50..10000] ms\n");
            ret = -EINVAL;
            break;
        }
        start_blink(dev, period_ms);
        break;

    case GPIO_LED_STOP_BLINK:
        stop_blink(dev);
        break;

    default:
        ret = -ENOTTY;
        break;
    }

    mutex_unlock(&dev->lock);
    return ret;
}

/* File operations structure */
static const struct file_operations gpio_led_fops = {
    .owner          = THIS_MODULE,
    .open           = gpio_led_open,
    .release        = gpio_led_release,
    .read           = gpio_led_read,
    .write          = gpio_led_write,
    .unlocked_ioctl = gpio_led_ioctl,
};

/* --------------------------------------------------------------------------
 * Module init / exit
 * -------------------------------------------------------------------------- */
static int __init gpio_led_init(void)
{
    int ret;

    pr_info("gpio_led: Initializing GPIO LED driver (BCM %d)\n", LED_GPIO_BCM);

    /* Allocate driver private data */
    led_dev = kzalloc(sizeof(*led_dev), GFP_KERNEL);
    if (!led_dev)
        return -ENOMEM;

    mutex_init(&led_dev->lock);

    /* --- Step 1: Request and configure GPIO (legacy API for explicit pin number) --- */
    if (!gpio_is_valid(LED_GPIO_BCM)) {
        pr_err("gpio_led: GPIO %d is not valid\n", LED_GPIO_BCM);
        ret = -ENODEV;
        goto err_free_dev;
    }

    ret = gpio_request(LED_GPIO_BCM, "led_gpio_23");
    if (ret) {
        pr_err("gpio_led: Failed to request GPIO %d (err %d)\n", LED_GPIO_BCM, ret);
        goto err_free_dev;
    }

    /* Set direction to output, initial value LOW (LED off) */
    ret = gpio_direction_output(LED_GPIO_BCM, 0);
    if (ret) {
        pr_err("gpio_led: Failed to set GPIO direction (err %d)\n", ret);
        goto err_free_gpio;
    }

    /* Convert to descriptor-based API for cleaner value set/get */
    led_dev->gpio = gpio_to_desc(LED_GPIO_BCM);
    if (!led_dev->gpio) {
        pr_err("gpio_led: Failed to get GPIO descriptor\n");
        ret = -ENODEV;
        goto err_free_gpio;
    }

    led_dev->led_state = LED_OFF;

    /* --- Step 2: Allocate a major:minor number dynamically --- */
    ret = alloc_chrdev_region(&led_dev->devno, 0, 1, DEVICE_NAME);
    if (ret < 0) {
        pr_err("gpio_led: Failed to allocate chrdev region (err %d)\n", ret);
        goto err_free_gpio;
    }

    pr_info("gpio_led: Registered with major=%d, minor=%d\n",
            MAJOR(led_dev->devno), MINOR(led_dev->devno));

    /* --- Step 3: Initialize and add the character device --- */
    cdev_init(&led_dev->cdev, &gpio_led_fops);
    led_dev->cdev.owner = THIS_MODULE;

    ret = cdev_add(&led_dev->cdev, led_dev->devno, 1);
    if (ret < 0) {
        pr_err("gpio_led: Failed to add cdev (err %d)\n", ret);
        goto err_unreg_chrdev;
    }

    /* --- Step 4: Create device class (visible in /sys/class/) --- */
    led_dev->class = class_create(DEVICE_NAME);
    if (IS_ERR(led_dev->class)) {
        ret = PTR_ERR(led_dev->class);
        pr_err("gpio_led: Failed to create class (err %d)\n", ret);
        goto err_cdev_del;
    }

    /* --- Step 5: Create device node (udev creates /dev/gpio_led) --- */
    led_dev->device = device_create(led_dev->class, NULL,
                                     led_dev->devno, NULL, DEVICE_NAME);
    if (IS_ERR(led_dev->device)) {
        ret = PTR_ERR(led_dev->device);
        pr_err("gpio_led: Failed to create device (err %d)\n", ret);
        goto err_class_destroy;
    }

    /* --- Step 6: Initialize blink timer --- */
    timer_setup(&led_dev->blink_timer, blink_timer_callback, 0);
    led_dev->blinking = 0;

    pr_info("gpio_led: Driver loaded successfully. Device: /dev/%s\n", DEVICE_NAME);
    return 0;

/* --- Error cleanup (reverse order) --- */
err_class_destroy:
    class_destroy(led_dev->class);
err_cdev_del:
    cdev_del(&led_dev->cdev);
err_unreg_chrdev:
    unregister_chrdev_region(led_dev->devno, 1);
err_free_gpio:
    gpio_free(LED_GPIO_BCM);
err_free_dev:
    kfree(led_dev);
    return ret;
}

static void __exit gpio_led_exit(void)
{
    pr_info("gpio_led: Unloading driver\n");

    /* Stop blink timer */
    stop_blink(led_dev);

    /* Turn LED off before unloading */
    gpiod_set_value(led_dev->gpio, 0);

    /* Tear down in reverse order of init */
    device_destroy(led_dev->class, led_dev->devno);
    class_destroy(led_dev->class);
    cdev_del(&led_dev->cdev);
    unregister_chrdev_region(led_dev->devno, 1);
    gpio_free(LED_GPIO_BCM);
    kfree(led_dev);

    pr_info("gpio_led: Driver unloaded\n");
}

module_init(gpio_led_init);
module_exit(gpio_led_exit);
