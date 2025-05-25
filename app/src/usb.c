#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

#include "usb.h"

LOG_MODULE_REGISTER(usb);

/* USB-UART device */
const static struct device *dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

/* Setup USB-UART console */
int enable_usb_uart(void)
{
	uint32_t dtr = 0;
	if (usb_enable(NULL)) {
		return -ENODEV;
	}

	while (!dtr) {
		uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
		k_sleep(K_MSEC(100));
	}

	LOG_INF("Logger initialized");
	return 0;
}
