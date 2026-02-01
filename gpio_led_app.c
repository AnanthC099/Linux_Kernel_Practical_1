/*
 * gpio_led_app.c
 *
 * User-space application for controlling GPIO LED via IOCTL.
 * Communicates with /dev/gpio_led character device.
 *
 * Build:  gcc -Wall -Wextra -O2 -o gpio_led_app gpio_led_app.c
 * Usage:  sudo ./gpio_led_app          (interactive menu)
 *         sudo ./gpio_led_app on       (command-line: turn on)
 *         sudo ./gpio_led_app off      (command-line: turn off)
 *         sudo ./gpio_led_app toggle   (command-line: toggle)
 *         sudo ./gpio_led_app state    (command-line: get state)
 *         sudo ./gpio_led_app blink N  (command-line: blink with N ms period)
 *
 * Author: Anjaneya
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>

#include "gpio_led_ioctl.h"

/* --------------------------------------------------------------------------
 * Helper: open the device
 * -------------------------------------------------------------------------- */
static int open_device(void)
{
    int fd = open(DEVICE_PATH, O_RDWR);
    if (fd < 0) {
        perror("Failed to open " DEVICE_PATH);
        fprintf(stderr, "Make sure the driver is loaded: sudo insmod gpio_led_driver.ko\n");
        fprintf(stderr, "And that you have permission: run with sudo\n");
    }
    return fd;
}

/* --------------------------------------------------------------------------
 * IOCTL wrapper functions
 * -------------------------------------------------------------------------- */
static int led_set_on(int fd)
{
    if (ioctl(fd, GPIO_LED_SET_ON) < 0) {
        perror("ioctl GPIO_LED_SET_ON");
        return -1;
    }
    printf("LED turned ON\n");
    return 0;
}

static int led_set_off(int fd)
{
    if (ioctl(fd, GPIO_LED_SET_OFF) < 0) {
        perror("ioctl GPIO_LED_SET_OFF");
        return -1;
    }
    printf("LED turned OFF\n");
    return 0;
}

static int led_toggle(int fd)
{
    if (ioctl(fd, GPIO_LED_TOGGLE) < 0) {
        perror("ioctl GPIO_LED_TOGGLE");
        return -1;
    }
    printf("LED toggled\n");
    return 0;
}

static int led_get_state(int fd)
{
    int state;
    if (ioctl(fd, GPIO_LED_GET_STATE, &state) < 0) {
        perror("ioctl GPIO_LED_GET_STATE");
        return -1;
    }
    printf("LED state: %s (%d)\n", state ? "ON" : "OFF", state);
    return state;
}

static int led_set_state(int fd, int state)
{
    if (ioctl(fd, GPIO_LED_SET_STATE, &state) < 0) {
        perror("ioctl GPIO_LED_SET_STATE");
        return -1;
    }
    printf("LED state set to: %s\n", state ? "ON" : "OFF");
    return 0;
}

static int led_set_blink(int fd, int period_ms)
{
    if (ioctl(fd, GPIO_LED_SET_BLINK, &period_ms) < 0) {
        perror("ioctl GPIO_LED_SET_BLINK");
        fprintf(stderr, "Valid range: 50 - 10000 ms\n");
        return -1;
    }
    printf("LED blinking with period %d ms\n", period_ms);
    return 0;
}

static int led_stop_blink(int fd)
{
    if (ioctl(fd, GPIO_LED_STOP_BLINK) < 0) {
        perror("ioctl GPIO_LED_STOP_BLINK");
        return -1;
    }
    printf("LED blink stopped\n");
    return 0;
}

/* --------------------------------------------------------------------------
 * Read/Write interface (alternative to IOCTL)
 * -------------------------------------------------------------------------- */
static int led_read_state(int fd)
{
    char buf[4];
    ssize_t n;

    /* Reset file position */
    lseek(fd, 0, SEEK_SET);

    n = read(fd, buf, sizeof(buf) - 1);
    if (n < 0) {
        perror("read");
        return -1;
    }
    buf[n] = '\0';
    printf("LED state (via read): %s", buf);
    return 0;
}

static int led_write_state(int fd, const char *val)
{
    if (write(fd, val, 1) < 0) {
        perror("write");
        return -1;
    }
    printf("LED set to %s via write()\n", (*val == '1') ? "ON" : "OFF");
    return 0;
}

/* --------------------------------------------------------------------------
 * Command-line mode
 * -------------------------------------------------------------------------- */
static int handle_cli(int fd, int argc, char *argv[])
{
    if (strcmp(argv[1], "on") == 0) {
        return led_set_on(fd);
    } else if (strcmp(argv[1], "off") == 0) {
        return led_set_off(fd);
    } else if (strcmp(argv[1], "toggle") == 0) {
        return led_toggle(fd);
    } else if (strcmp(argv[1], "state") == 0) {
        return led_get_state(fd);
    } else if (strcmp(argv[1], "blink") == 0) {
        if (argc < 3) {
            fprintf(stderr, "Usage: %s blink <period_ms>\n", argv[0]);
            return -1;
        }
        int period = atoi(argv[2]);
        return led_set_blink(fd, period);
    } else if (strcmp(argv[1], "stop") == 0) {
        return led_stop_blink(fd);
    } else {
        fprintf(stderr, "Unknown command: %s\n", argv[1]);
        fprintf(stderr, "Valid: on, off, toggle, state, blink <ms>, stop\n");
        return -1;
    }
}

/* --------------------------------------------------------------------------
 * Interactive menu mode
 * -------------------------------------------------------------------------- */
static void print_menu(void)
{
    printf("\n");
    printf("╔══════════════════════════════════════════╗\n");
    printf("║     GPIO LED Controller (BCM Pin 23)     ║\n");
    printf("╠══════════════════════════════════════════╣\n");
    printf("║  1. Turn LED ON          (ioctl)         ║\n");
    printf("║  2. Turn LED OFF         (ioctl)         ║\n");
    printf("║  3. Toggle LED           (ioctl)         ║\n");
    printf("║  4. Get LED state        (ioctl)         ║\n");
    printf("║  5. Set LED state        (ioctl)         ║\n");
    printf("║  6. Start blink          (ioctl)         ║\n");
    printf("║  7. Stop blink           (ioctl)         ║\n");
    printf("║  8. Read state           (read syscall)  ║\n");
    printf("║  9. Write state          (write syscall) ║\n");
    printf("║  0. Exit                                 ║\n");
    printf("╚══════════════════════════════════════════╝\n");
    printf("Choice: ");
}

static void interactive_mode(int fd)
{
    int choice;
    int val;

    while (1) {
        print_menu();

        if (scanf("%d", &choice) != 1) {
            /* Clear invalid input */
            int c;
            while ((c = getchar()) != '\n' && c != EOF)
                ;
            printf("Invalid input. Enter a number 0-9.\n");
            continue;
        }

        switch (choice) {
        case 1:
            led_set_on(fd);
            break;

        case 2:
            led_set_off(fd);
            break;

        case 3:
            led_toggle(fd);
            break;

        case 4:
            led_get_state(fd);
            break;

        case 5:
            printf("Enter state (0=OFF, 1=ON): ");
            if (scanf("%d", &val) == 1)
                led_set_state(fd, val);
            else
                printf("Invalid input\n");
            break;

        case 6:
            printf("Enter blink period in ms (50-10000): ");
            if (scanf("%d", &val) == 1)
                led_set_blink(fd, val);
            else
                printf("Invalid input\n");
            break;

        case 7:
            led_stop_blink(fd);
            break;

        case 8:
            led_read_state(fd);
            break;

        case 9:
            printf("Enter value ('0' or '1'): ");
            if (scanf("%d", &val) == 1) {
                char v = (val == 1) ? '1' : '0';
                led_write_state(fd, &v);
            } else {
                printf("Invalid input\n");
            }
            break;

        case 0:
            printf("Exiting. LED state preserved.\n");
            return;

        default:
            printf("Invalid choice. Enter 0-9.\n");
            break;
        }
    }
}

/* --------------------------------------------------------------------------
 * Main
 * -------------------------------------------------------------------------- */
int main(int argc, char *argv[])
{
    int fd;
    int ret = 0;

    fd = open_device();
    if (fd < 0)
        return EXIT_FAILURE;

    if (argc > 1) {
        /* Command-line mode */
        ret = handle_cli(fd, argc, argv);
    } else {
        /* Interactive menu mode */
        interactive_mode(fd);
    }

    close(fd);
    return (ret < 0) ? EXIT_FAILURE : EXIT_SUCCESS;
}
