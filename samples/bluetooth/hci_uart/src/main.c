/*
 * Copyright (c) 2016 Nordic Semiconductor ASA
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include <zephyr.h>
#include <arch/cpu.h>
#include <sys/byteorder.h>
#include <sys/reboot.h>
#include <logging/log.h>
#include <sys/util.h>

#include <device.h>
#include <init.h>
#include <drivers/uart.h>

#include <net/buf.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/l2cap.h>
#include <bluetooth/hci.h>
#include <bluetooth/buf.h>
#include <bluetooth/hci_raw.h>

#define LOG_MODULE_NAME hci_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

static const struct device *hci_uart_dev =
	DEVICE_DT_GET(DT_CHOSEN(zephyr_bt_c2h_uart));
static K_THREAD_STACK_DEFINE(tx_thread_stack, CONFIG_BT_HCI_TX_STACK_SIZE);
static struct k_thread tx_thread_data;
static struct k_mutex uart_mutex;
static K_FIFO_DEFINE(tx_queue);

/* RX in terms of bluetooth communication */
static K_FIFO_DEFINE(uart_tx_queue);

#define H4_CMD 0x01
#define H4_ACL 0x02
#define H4_SCO 0x03
#define H4_EVT 0x04
#define H4_ISO 0x05

/* Receiver states. */
#define ST_IDLE 0	/* Waiting for packet type. */
#define ST_HDR 1	/* Receiving packet header. */
#define ST_PAYLOAD 2	/* Receiving packet payload. */
#define ST_DISCARD 3	/* Dropping packet. */

#define HCI_VENDOR_EVT			0xff
#define HCI_VENDOR_LOG_LVL_LEN	1
#define HCI_VENDOR_LOG_FTR_LEN	1

#define LOG_INFO_LVL    	0x01
#define LOG_WARN_LVL    	0x02
#define LOG_ERR_LVL     	0x03
#define LOG_FATAL_LVL   	0x04

static bool hci_ready = false;

static void hci_log_send(uint8_t lvl, const char *msg);

void hci_log(uint8_t lvl, const char *log, ...)
{
	va_list args;
	char buf[255] = {0};
	int len;

	if(!hci_ready){
		return;
	}

    va_start(args, log);
    len = vsnprintf(buf, 255, log, args);
    va_end(args);

	k_mutex_lock(&uart_mutex, K_FOREVER);
	hci_log_send(lvl, buf);
	k_mutex_unlock(&uart_mutex);
}

#define HCI_UART_INFO(fmt, ...) { \
	LOG_INF(fmt, ##__VA_ARGS__); \
	hci_log(LOG_WARN_LVL, fmt, ##__VA_ARGS__); \
}

#define HCI_UART_WARN(fmt, ...) { \
	LOG_WRN(fmt, ##__VA_ARGS__); \
	hci_log(LOG_WARN_LVL, fmt, ##__VA_ARGS__); \
}

#define HCI_UART_ERR(fmt, ...) { \
	LOG_ERR(fmt, ##__VA_ARGS__); \
	hci_log(LOG_ERR_LVL, fmt, ##__VA_ARGS__); \
}

#define HCI_UART_FATAL(fmt, ...) { \
	LOG_ERR(fmt, ##__VA_ARGS__); \
	hci_log(LOG_FATAL_LVL, fmt, ##__VA_ARGS__); \
}

static void esf_dump(const z_arch_esf_t *esf)
{
	HCI_UART_FATAL("r0/a1:  0x%08x  r1/a2:  0x%08x  r2/a3:  0x%08x",
		esf->basic.a1, esf->basic.a2, esf->basic.a3);
	HCI_UART_FATAL("r3/a4:  0x%08x r12/ip:  0x%08x r14/lr:  0x%08x",
		esf->basic.a4, esf->basic.ip, esf->basic.lr);
	HCI_UART_FATAL(" xpsr:  0x%08x", esf->basic.xpsr);
#if defined(CONFIG_EXTRA_EXCEPTION_INFO)
	const struct _callee_saved *callee = esf->extra_info.callee;

	if (callee != NULL) {
		HCI_UART_FATAL("r4/v1:  0x%08x  r5/v2:  0x%08x  r6/v3:  0x%08x",
			callee->v1, callee->v2, callee->v3);
		HCI_UART_FATAL("r7/v4:  0x%08x  r8/v5:  0x%08x  r9/v6:  0x%08x",
			callee->v4, callee->v5, callee->v6);
		HCI_UART_FATAL("r10/v7: 0x%08x  r11/v8: 0x%08x    psp:  0x%08x",
			callee->v7, callee->v8, callee->psp);
	}

	HCI_UART_FATAL("EXC_RETURN: 0x%0x", esf->extra_info.exc_return);

#endif /* CONFIG_EXTRA_EXCEPTION_INFO */
	HCI_UART_FATAL("Faulting instruction address (r15/pc): 0x%08x",
		esf->basic.pc);
}

void k_sys_fatal_error_handler(unsigned int reason, const z_arch_esf_t *esf)
{
	HCI_UART_FATAL("BLE controller fault:");
	esf_dump(esf);
	sys_reboot(SYS_REBOOT_COLD);
}

/* Length of a discard/flush buffer.
 * This is sized to align with a BLE HCI packet:
 * 1 byte H:4 header + 32 bytes ACL/event data
 * Bigger values might overflow the stack since this is declared as a local
 * variable, smaller ones will force the caller to call into discard more
 * often.
 */
#define H4_DISCARD_LEN 33


static int h4_read(const struct device *uart, uint8_t *buf, size_t len)
{
	int rx = uart_fifo_read(uart, buf, len);

	LOG_DBG("read %d req %d", rx, len);

	return rx;
}

static bool valid_type(uint8_t type)
{
	return (type == H4_CMD) | (type == H4_ACL) | (type == H4_ISO);
}

/* Function expects that type is validated and only CMD, ISO or ACL will be used. */
static uint32_t get_len(const uint8_t *hdr_buf, uint8_t type)
{
	switch (type) {
	case H4_CMD:
		return ((const struct bt_hci_cmd_hdr *)hdr_buf)->param_len;
	case H4_ISO:
		return sys_le16_to_cpu(((const struct bt_hci_iso_data_hdr *)hdr_buf)->slen);
	case H4_ACL:
		return sys_le16_to_cpu(((const struct bt_hci_acl_hdr *)hdr_buf)->len);
	default:
		HCI_UART_ERR("Invalid type: %u", type);
		return 0;
	}
}

/* Function expects that type is validated and only CMD, ISO or ACL will be used. */
static int hdr_len(uint8_t type)
{
	switch (type) {
	case H4_CMD:
		return sizeof(struct bt_hci_cmd_hdr);
	case H4_ISO:
		return sizeof(struct bt_hci_iso_data_hdr);
	case H4_ACL:
		return sizeof(struct bt_hci_acl_hdr);
	default:
		HCI_UART_ERR("Invalid type: %u", type);
		return 0;
	}
}

static void rx_isr(void)
{
	static struct net_buf *buf;
	static int remaining;
	static uint8_t state;
	static uint8_t type;
	static uint8_t hdr_buf[MAX(sizeof(struct bt_hci_cmd_hdr),
			sizeof(struct bt_hci_acl_hdr))];
	int read;

	hci_ready = true;

	do {
		switch (state) {
		case ST_IDLE:
			/* Get packet type */
			read = h4_read(hci_uart_dev, &type, sizeof(type));
			/* since we read in loop until no data is in the fifo,
			 * it is possible that read = 0.
			 */
			if (read) {
				if (valid_type(type)) {
					/* Get expected header size and switch
					 * to receiving header.
					 */
					remaining = hdr_len(type);
					state = ST_HDR;
				} else {
					HCI_UART_WARN("Unknown header %d", type);
				}
			}
			break;
		case ST_HDR:
			read = h4_read(hci_uart_dev,
				       &hdr_buf[hdr_len(type) - remaining],
				       remaining);
			remaining -= read;
			if (remaining == 0) {
				/* Header received. Allocate buffer and get
				 * payload length. If allocation fails leave
				 * interrupt. On failed allocation state machine
				 * is reset.
				 */
				buf = bt_buf_get_tx(BT_BUF_H4, K_NO_WAIT,
						    &type, sizeof(type));
				if (!buf) {
					HCI_UART_ERR("No available command buffers!");
					state = ST_IDLE;
					return;
				}

				remaining = get_len(hdr_buf, type);

				net_buf_add_mem(buf, hdr_buf, hdr_len(type));
				if (remaining > net_buf_tailroom(buf)) {
					HCI_UART_ERR("Not enough space in buffer");
					net_buf_unref(buf);
					state = ST_DISCARD;
				} else {
					state = ST_PAYLOAD;
				}
			}
			break;
		case ST_PAYLOAD:
			read = h4_read(hci_uart_dev, net_buf_tail(buf),
				       remaining);
			buf->len += read;
			remaining -= read;
			if (remaining == 0) {
				/* Packet received */
				LOG_DBG("putting RX packet in queue.");
				net_buf_put(&tx_queue, buf);
				state = ST_IDLE;
			}
			break;
		case ST_DISCARD:
		{
			uint8_t discard[H4_DISCARD_LEN];
			size_t to_read = MIN(remaining, sizeof(discard));

			read = h4_read(hci_uart_dev, discard, to_read);
			remaining -= read;
			if (remaining == 0) {
				state = ST_IDLE;
			}

			break;

		}
		default:
			read = 0;
			__ASSERT_NO_MSG(0);
			break;

		}
	} while (read);
}

static void tx_isr(void)
{
	static struct net_buf *buf;
	int len;

	if (!buf) {
		buf = net_buf_get(&uart_tx_queue, K_NO_WAIT);
		if (!buf) {
			uart_irq_tx_disable(hci_uart_dev);
			return;
		}
	}

	len = uart_fifo_fill(hci_uart_dev, buf->data, buf->len);
	net_buf_pull(buf, len);
	if (!buf->len) {
		net_buf_unref(buf);
		buf = NULL;
	}
}

static void bt_uart_isr(const struct device *unused, void *user_data)
{
	ARG_UNUSED(unused);
	ARG_UNUSED(user_data);

	if (!(uart_irq_rx_ready(hci_uart_dev) ||
	      uart_irq_tx_ready(hci_uart_dev))) {
		LOG_DBG("spurious interrupt");
	}

	if (uart_irq_tx_ready(hci_uart_dev)) {
		tx_isr();
	}

	if (uart_irq_rx_ready(hci_uart_dev)) {
		rx_isr();
	}
}

static void tx_thread(void *p1, void *p2, void *p3)
{
	while (1) {
		struct net_buf *buf;
		int err;

		/* Wait until a buffer is available */
		buf = net_buf_get(&tx_queue, K_FOREVER);
		/* Pass buffer to the stack */
		err = bt_send(buf);
		if (err) {
			HCI_UART_ERR("Unable to send (err %d)", err);
			net_buf_unref(buf);
		}

		/* Give other threads a chance to run if tx_queue keeps getting
		 * new data all the time.
		 */
		k_yield();
	}
}

static int h4_send(struct net_buf *buf)
{
	LOG_DBG("buf %p type %u len %u", buf, bt_buf_get_type(buf),
		    buf->len);

	net_buf_put(&uart_tx_queue, buf);
	uart_irq_tx_enable(hci_uart_dev);

	return 0;
}

#if defined(CONFIG_BT_CTLR_ASSERT_HANDLER)
void bt_ctlr_assert_handle(char *file, uint32_t line)
{

	/* Disable interrupts, this is unrecoverable */
	(void)irq_lock();

	uart_irq_rx_disable(hci_uart_dev);
	uart_irq_tx_disable(hci_uart_dev);

	hci_log(LOG_FATAL_LVL, "[%s:%d] - assert", file, line);
	
	while (1) {
	}
}
#endif /* CONFIG_BT_CTLR_ASSERT_HANDLER */

static void hci_log_send(uint8_t lvl, const char *msg)
{
	uint32_t str_len = strlen(msg);

	uart_poll_out(hci_uart_dev, H4_EVT);
	uart_poll_out(hci_uart_dev, HCI_VENDOR_EVT);
	uart_poll_out(hci_uart_dev, str_len + HCI_VENDOR_LOG_LVL_LEN + HCI_VENDOR_LOG_FTR_LEN);
	uart_poll_out(hci_uart_dev, lvl);

	if (str_len) {
		while (*msg != '\0') {
			uart_poll_out(hci_uart_dev, *msg);
			msg++;
		}
		uart_poll_out(hci_uart_dev, 0x00);
	}
}

static int hci_uart_init(const struct device *unused)
{
	LOG_DBG("");

	if (!device_is_ready(hci_uart_dev)) {
		LOG_ERR("HCI UART %s is not ready", hci_uart_dev->name);
		return -EINVAL;
	}

	uart_irq_rx_disable(hci_uart_dev);
	uart_irq_tx_disable(hci_uart_dev);

	uart_irq_callback_set(hci_uart_dev, bt_uart_isr);

	uart_irq_rx_enable(hci_uart_dev);

	return 0;
}

SYS_DEVICE_DEFINE("hci_uart", hci_uart_init, NULL,
		  APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

void main(void)
{
	/* incoming events and data from the controller */
	static K_FIFO_DEFINE(rx_queue);
	int err;

	LOG_DBG("Start");
	__ASSERT(hci_uart_dev, "UART device is NULL");

	/* Enable the raw interface, this will in turn open the HCI driver */
	bt_enable_raw(&rx_queue);

	if (IS_ENABLED(CONFIG_BT_WAIT_NOP)) {
		/* Issue a Command Complete with NOP */
		int i;

		const struct {
			const uint8_t h4;
			const struct bt_hci_evt_hdr hdr;
			const struct bt_hci_evt_cmd_complete cc;
		} __packed cc_evt = {
			.h4 = H4_EVT,
			.hdr = {
				.evt = BT_HCI_EVT_CMD_COMPLETE,
				.len = sizeof(struct bt_hci_evt_cmd_complete),
			},
			.cc = {
				.ncmd = 1,
				.opcode = sys_cpu_to_le16(BT_OP_NOP),
			},
		};

		for (i = 0; i < sizeof(cc_evt); i++) {
			uart_poll_out(hci_uart_dev,
				      *(((const uint8_t *)&cc_evt)+i));
		}
	}

	k_mutex_init(&uart_mutex);
	/* Spawn the TX thread and start feeding commands and data to the
	 * controller
	 */
	k_thread_create(&tx_thread_data, tx_thread_stack,
			K_THREAD_STACK_SIZEOF(tx_thread_stack), tx_thread,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);
	k_thread_name_set(&tx_thread_data, "HCI uart TX");

	while (1) {
		struct net_buf *buf;
		buf = net_buf_get(&rx_queue, K_FOREVER);
		k_mutex_lock(&uart_mutex, K_FOREVER);
		err = h4_send(buf);
		k_mutex_unlock(&uart_mutex);
		if (err) {
			HCI_UART_ERR("Failed to send");
		}
	}
}
