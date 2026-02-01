# GPIO LED Driver for Raspberry Pi 4

A Linux kernel character device driver that controls an LED connected to **Physical Pin 16 (BCM GPIO 23)** on the Raspberry Pi 4, with full IOCTL support and a user-space control application.

## Hardware Setup

```
Raspberry Pi 4 GPIO Header
     ┌─────────────────────────────┐
     │         (pin 1) 3.3V  5V (pin 2)         │
     │         (pin 3) SDA   5V (pin 4)         │
     │         (pin 5) SCL  GND (pin 6)         │
     │         ...                               │
     │  ┌──→  (pin 15) GPIO22  GPIO23 (pin 16) ◄── LED HERE
     │  │     (pin 17) 3.3V   GPIO24 (pin 18)  │
     │         ...                               │
     └─────────────────────────────┘

Physical Pin 16 (BCM GPIO 23) ──── 330Ω ────┤►│──── GND (any GND pin)
                                             LED (anode to resistor,
                                                  cathode to GND)
```

**Components needed:**
- 1x LED (any color)
- 1x 330Ω resistor (220Ω-1kΩ acceptable)
- Jumper wires

## Project Structure

```
gpio_led_driver/
├── gpio_led_ioctl.h     # Shared header (IOCTL command definitions)
├── gpio_led_driver.c    # Kernel module (character device driver)
├── gpio_led_app.c       # User-space application
├── Makefile             # Build system
└── README.md            # This file
```

## Building

### Prerequisites (on Raspberry Pi 4)

```bash
# Install kernel headers
sudo apt update
sudo apt install raspberrypi-kernel-headers build-essential
```

### Build the Kernel Module

```bash
make
```

### Build the User-Space App

```bash
make app
# or directly:
gcc -Wall -Wextra -O2 -o gpio_led_app gpio_led_app.c
```

### Cross-Compilation (from x86 Host)

```bash
# Install cross-compiler
sudo apt install gcc-aarch64-linux-gnu

# Build (adjust KERNEL_DIR to your RPi kernel source)
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- \
     KERNEL_DIR=/path/to/rpi-linux-source

# Build user app
aarch64-linux-gnu-gcc -Wall -O2 -o gpio_led_app gpio_led_app.c
```

## Usage

### Load the Driver

```bash
# Insert the module
sudo insmod gpio_led_driver.ko

# Verify it loaded
dmesg | tail -10
# Should show: gpio_led: Driver loaded successfully. Device: /dev/gpio_led

# Check device node was created
ls -la /dev/gpio_led

# Check permissions (change if needed)
sudo chmod 666 /dev/gpio_led    # Allow non-root access (optional)
```

### Interactive Mode

```bash
sudo ./gpio_led_app
```

This presents a menu:
```
╔══════════════════════════════════════════╗
║     GPIO LED Controller (BCM Pin 23)     ║
╠══════════════════════════════════════════╣
║  1. Turn LED ON          (ioctl)         ║
║  2. Turn LED OFF         (ioctl)         ║
║  3. Toggle LED           (ioctl)         ║
║  4. Get LED state        (ioctl)         ║
║  5. Set LED state        (ioctl)         ║
║  6. Start blink          (ioctl)         ║
║  7. Stop blink           (ioctl)         ║
║  8. Read state           (read syscall)  ║
║  9. Write state          (write syscall) ║
║  0. Exit                                 ║
╚══════════════════════════════════════════╝
```

### Command-Line Mode

```bash
sudo ./gpio_led_app on          # Turn LED on
sudo ./gpio_led_app off         # Turn LED off
sudo ./gpio_led_app toggle      # Toggle state
sudo ./gpio_led_app state       # Get current state
sudo ./gpio_led_app blink 500   # Blink with 500ms period
sudo ./gpio_led_app stop        # Stop blinking
```

### Shell Interface (Without the App)

```bash
# Read state
cat /dev/gpio_led

# Turn ON
echo 1 > /dev/gpio_led

# Turn OFF
echo 0 > /dev/gpio_led
```

### Unload the Driver

```bash
sudo rmmod gpio_led_driver
dmesg | tail -5
# Should show: gpio_led: Driver unloaded
```

## Architecture

```
┌──────────────────────────────────────────────────────────┐
│                     USER SPACE                           │
│                                                          │
│  gpio_led_app.c                   Shell commands         │
│  ┌────────────┐                   ┌──────────────┐      │
│  │ ioctl()    │                   │ echo/cat     │      │
│  │ read()     │                   │ > /dev/      │      │
│  │ write()    │                   │   gpio_led   │      │
│  └─────┬──────┘                   └──────┬───────┘      │
│        │                                  │              │
├────────┼──────────────────────────────────┼──────────────┤
│        │         KERNEL SPACE             │              │
│        ▼                                  ▼              │
│  ┌─────────────────────────────────────────────┐        │
│  │          gpio_led_driver.c                   │        │
│  │                                               │        │
│  │  file_operations:                             │        │
│  │    .open           → gpio_led_open()          │        │
│  │    .release        → gpio_led_release()       │        │
│  │    .read           → gpio_led_read()          │        │
│  │    .write          → gpio_led_write()         │        │
│  │    .unlocked_ioctl → gpio_led_ioctl()         │        │
│  │                                               │        │
│  │  IOCTL handlers:                              │        │
│  │    SET_ON / SET_OFF / TOGGLE                  │        │
│  │    GET_STATE / SET_STATE                       │        │
│  │    SET_BLINK / STOP_BLINK                     │        │
│  │                                               │        │
│  │  Blink: kernel timer (timer_list)             │        │
│  │  Sync:  mutex for shared state                │        │
│  └────────────────┬──────────────────────────────┘        │
│                   │                                       │
│                   ▼                                       │
│  ┌────────────────────────────────┐                      │
│  │    GPIO Subsystem              │                      │
│  │    gpiod_set_value()           │                      │
│  │    gpio_direction_output()     │                      │
│  └────────────────┬───────────────┘                      │
│                   │                                       │
├───────────────────┼───────────────────────────────────────┤
│                   ▼         HARDWARE                      │
│  ┌────────────────────────────────┐                      │
│  │  BCM2711 GPIO Controller       │                      │
│  │  Register: 0xFE200000          │                      │
│  │  Pin 23 → Physical Pin 16     │                      │
│  └────────────────┬───────────────┘                      │
│                   │                                       │
│                   ▼                                       │
│            ┌──────────┐                                   │
│            │   LED    │                                   │
│            └──────────┘                                   │
└──────────────────────────────────────────────────────────┘
```

## IOCTL Command Summary

| Command              | Direction | Data   | Description                    |
|----------------------|-----------|--------|--------------------------------|
| `GPIO_LED_SET_ON`    | None      | —      | Turn LED on                    |
| `GPIO_LED_SET_OFF`   | None      | —      | Turn LED off                   |
| `GPIO_LED_TOGGLE`    | None      | —      | Toggle current state           |
| `GPIO_LED_GET_STATE` | Read      | `int`  | Get state (0=OFF, 1=ON)        |
| `GPIO_LED_SET_STATE` | Write     | `int`  | Set state (0 or 1)             |
| `GPIO_LED_SET_BLINK` | Write     | `int`  | Start blink (period in ms)     |
| `GPIO_LED_STOP_BLINK`| None      | —      | Stop blinking                  |

## Key Concepts Demonstrated

1. **Character Device Driver**: Dynamic major/minor allocation, cdev, class/device creation for udev
2. **GPIO Subsystem**: Both legacy (`gpio_request`) and descriptor-based (`gpiod_set_value`) APIs
3. **IOCTL Interface**: Proper use of `_IO`, `_IOR`, `_IOW` macros with magic numbers
4. **Kernel Timers**: `timer_list` for periodic blink functionality
5. **Concurrency**: Mutex protection for shared state
6. **User-Kernel Data Transfer**: `copy_to_user` / `copy_from_user`
7. **Error Handling**: Goto-based cleanup chain in `__init`
8. **Multiple Interfaces**: IOCTL + read/write + shell echo/cat

## Troubleshooting

```bash
# Check if module is loaded
lsmod | grep gpio_led

# Check kernel log for errors
dmesg | grep gpio_led

# If GPIO is busy (another driver using it)
# Check /sys/kernel/debug/gpio or:
cat /sys/kernel/debug/pinctrl/fe200000.gpio-pinctrl-bcm2711/pins

# If /dev/gpio_led doesn't appear
# Check if udev is running, or create manually:
sudo mknod /dev/gpio_led c <MAJOR> 0
# (get MAJOR from dmesg output)

# Verify GPIO pin mapping
# Physical Pin 16 = BCM GPIO 23 = WiringPi Pin 4
pinout  # (if raspi-gpio is installed)
```

## License

GPL v2
