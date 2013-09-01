/*
 * USB FTDI MPSSE SPI/GPIO driver
 *
 *	Copyright (C) 2013
 *          Ryan Press (ryan@presslab.us)
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or
 *	(at your option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/dmapool.h>
#include <linux/timer.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/gpio.h>

#include "ftdi_mpsse.h"

#define DRV_NAME "ftdi_mpsse"
#define DRIVER_AUTHOR "Ryan Press <ryan@presslab.us>"
#define DRIVER_DESC "USB FTDI MPSSE SPI/GPIO Driver"

#define GPIO_BUF_LEN 512
#define CONCURRENT_GPIO_WRITES 8
#define CONCURRENT_SPI_WRITES 6

static __u16 vendor = FTDI_VID;
static __u16 product;
static int latency_timer = 16;

static int spi_numcs = 1;
static int gpio_offset = 4;
static char *wait_on_l1 = NULL;

static int gpio_base = -1;
static __s16 spi_base = -1;
static atomic_t dyn_gpio_base = ATOMIC_INIT(-1);
static atomic_t dyn_spi_base = ATOMIC_INIT(-1);

static struct usb_device_id id_table_combined [] = {
	{ USB_DEVICE(FTDI_VID, FTDI_8U232AM_PID) },
	{ USB_DEVICE(FTDI_VID, FTDI_8U232AM_ALT_PID) },
	{ USB_DEVICE(FTDI_VID, FTDI_8U2232C_PID) },
	{ USB_DEVICE(FTDI_VID, FTDI_4232H_PID) },
	{ USB_DEVICE(FTDI_VID, FTDI_232H_PID) },
	{ USB_DEVICE(FTDI_VID, FTDI_FTX_PID) },
	{ },					/* Optional parameter entry */
	{ }					/* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, id_table_combined);

enum ftdi_types { TYPE_FT2232C, TYPE_FT2232H, TYPE_FT4232H, TYPE_FT232H };

enum ftdi_gpiocmds { GPIOCMD_WRITE_HIGH, GPIOCMD_WRITE_LOW, GPIOCMD_READ_HIGH,
		GPIOCMD_READ_LOW, GPIOCMD_WAIT_ON_L1HI, GPIOCMD_WAIT_ON_L1LO };

static const char *ftdi_chip_name[] = {
	[TYPE_FT2232C] = "FT2232C",
	[TYPE_FT2232H] = "FT2232H",
	[TYPE_FT4232H] = "FT4232H",
	[TYPE_FT232H]  = "FT232H",
};


struct ftdi_device {
	struct usb_device	*udev;
	enum ftdi_types		type;
	uint8_t			interface;
	uint8_t			in_ep;
	uint8_t			out_ep;
	uint16_t		max_tx_size;
	uint32_t		tclk;
	struct mutex		usb_lock;
	struct usb_anchor	urb_submitted;
	struct timer_list	timer;

	struct spi_bitbang	spi;
	struct semaphore	spi_limit_sem;
	uint16_t		spi_write_cmd;

	struct gpio_chip	gpio;
	struct semaphore	gpio_limit_sem;
	uint16_t		gpio_dirs;
	uint16_t		gpio_pins;

	spinlock_t		gpio_txlock;
	uint8_t			*gpio_buf;
	struct urb		*gpio_urb;
	uint16_t		gpio_idx;
	struct dma_pool		*gpio_pool;

};


/*
 * ***************************************************************************
 * GPIO specific functions
 * ***************************************************************************
 */

static void ftdi_gpio_bulk_callback(struct urb *urb)
{
	struct ftdi_device *dev = urb->context;

	switch(urb->status) {
	case -ENOENT:
	case -ECONNRESET:
	case -ESHUTDOWN:
		dev_warn(&dev->udev->dev, "%s - write bulk status %d", __func__, urb->status);
		break;
	default:
		break;
	}

	dma_pool_free(dev->gpio_pool, urb->transfer_buffer, urb->transfer_dma);
	up(&dev->gpio_limit_sem);
}

static int ftdi_gpio_tx(struct ftdi_device * dev)
{
	int err;
	unsigned long flags;

	spin_lock_irqsave(&dev->gpio_txlock, flags);

	if (dev->gpio_buf == NULL) {
		spin_unlock_irqrestore(&dev->gpio_txlock, flags);
		return 0;
	}

	del_timer(&dev->timer);

	usb_fill_bulk_urb(dev->gpio_urb, dev->udev, usb_sndbulkpipe(dev->udev, dev->out_ep),
		dev->gpio_buf, dev->gpio_idx, ftdi_gpio_bulk_callback, dev);
	dev->gpio_urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	dev->gpio_buf = NULL;
	dev->gpio_idx = 0;

	if ((err = usb_submit_urb(dev->gpio_urb, GFP_ATOMIC))) {
		dev_err(&dev->udev->dev, "%s - failed write urb, err %d", __func__, err);
		goto error;
	}

	usb_free_urb(dev->gpio_urb);
	spin_unlock_irqrestore(&dev->gpio_txlock, flags);
	return 0;

error:
	dma_pool_free(dev->gpio_pool, dev->gpio_urb->transfer_buffer, dev->gpio_urb->transfer_dma);
	usb_unanchor_urb(dev->gpio_urb);
	usb_free_urb(dev->gpio_urb);

	spin_unlock_irqrestore(&dev->gpio_txlock, flags);

	up(&dev->gpio_limit_sem);
	return err;
}

static void ftdi_gpio_timer(unsigned long data)
{
	struct ftdi_device *dev = (struct ftdi_device *)data;

	/* when timer expires write whatever is in the buffer */
	(void) ftdi_gpio_tx(dev);
}

static int ftdi_gpio_common(struct ftdi_device *dev, enum ftdi_gpiocmds cmd)
{
	int err = 0;
	uint8_t *buf = NULL;
	bool force = false;
	struct urb *urb;
	uint16_t idx;
	unsigned long flags;

	spin_lock_irqsave(&dev->gpio_txlock, flags);
	buf = dev->gpio_buf;
	idx = dev->gpio_idx;

	/* buffer is empty, so start the URB */
	if (buf == NULL) {
		spin_unlock_irqrestore(&dev->gpio_txlock, flags);

		if (down_interruptible(&dev->gpio_limit_sem) < 0)
			return -EINTR;

		if (!(urb = usb_alloc_urb(0, GFP_KERNEL))) {
			err = -ENOMEM;
			goto error_no_buf;
		}

		/* alloc from the pool, size is GPIO_BUF_LEN bytes */
		if (!(buf = dma_pool_alloc(dev->gpio_pool, GFP_KERNEL, &urb->transfer_dma))) {
			err = -ENOMEM;
			goto error_pool;
		}

		usb_anchor_urb(urb, &dev->urb_submitted);

		spin_lock_irqsave(&dev->gpio_txlock, flags);

		/* after 'latency_timer' ms, this timer will queue tx if it hasn't been written yet */
		dev->timer.expires = jiffies + msecs_to_jiffies(latency_timer);
		add_timer(&dev->timer);

		dev->gpio_urb = urb;
		dev->gpio_buf = buf;
	}

	/* add command to the buffer */
	switch (cmd) {
	case GPIOCMD_WRITE_HIGH:
		buf[idx++] = FTDI_MPSSE_WRITEHIGH;
		buf[idx++] = dev->gpio_pins >> 8;
		buf[idx++] = dev->gpio_dirs >> 8;
		break;
	case GPIOCMD_WRITE_LOW:
		buf[idx++] = FTDI_MPSSE_WRITELOW;
		buf[idx++] = dev->gpio_pins;
		buf[idx++] = dev->gpio_dirs;
		break;
	case GPIOCMD_READ_HIGH:
		buf[idx++] = FTDI_MPSSE_READHIGH;
		force = true;
		break;
	case GPIOCMD_READ_LOW:
		buf[idx++] = FTDI_MPSSE_READLOW;
		force = true;
		break;
	case GPIOCMD_WAIT_ON_L1HI:
		buf[idx++] = FTDI_MPSSE_WAIT_ON_HI;
		break;
	case GPIOCMD_WAIT_ON_L1LO:
		buf[idx++] = FTDI_MPSSE_WAIT_ON_LO;
		break;
	}

	dev->gpio_idx = idx;
	spin_unlock_irqrestore(&dev->gpio_txlock, flags);

	/* if the buffer is almost full, or a read command, queue it up */
	if (idx + 3 >= GPIO_BUF_LEN || force) {
		return ftdi_gpio_tx(dev);
	}

	return 0;
error_pool:
	usb_free_urb(urb);
error_no_buf:
	up(&dev->gpio_limit_sem);
	return err;
}

static void ftdi_gpio_set_block(struct gpio_chip *gc, unsigned long mask, unsigned long values)
{
	uint16_t old_pins;
	struct ftdi_device *dev = container_of(gc, struct ftdi_device, gpio);

	mutex_lock(&dev->usb_lock);

	old_pins = dev->gpio_pins;
	dev->gpio_pins &= ~(mask << gpio_offset);
	dev->gpio_pins |= (values & mask) << gpio_offset;

	/* only write the pins if they have changed */
	if ((old_pins ^ dev->gpio_pins) & 0xFF00)
		(void) ftdi_gpio_common(dev, GPIOCMD_WRITE_HIGH);
	if ((old_pins ^ dev->gpio_pins) & 0xFF)
		(void) ftdi_gpio_common(dev, GPIOCMD_WRITE_LOW);

	mutex_unlock(&dev->usb_lock);
}

static void ftdi_gpio_set(struct gpio_chip *gc, unsigned gpio_num, int val)
{
	ftdi_gpio_set_block(gc, 1 << gpio_num, val ? 0xFFFF : 0);
}

static int ftdi_gpio_dir_in(struct gpio_chip *gc, unsigned gpio_num)
{
	uint16_t old_dirs;
	int err = 0;
	struct ftdi_device *dev = container_of(gc, struct ftdi_device, gpio);

	mutex_lock(&dev->usb_lock);

	old_dirs = dev->gpio_dirs;
	dev->gpio_dirs &= ~1 << (gpio_num + gpio_offset);

	if (old_dirs != dev->gpio_dirs)
		err = ftdi_gpio_common(dev, gpio_num >= (8 - gpio_offset) ? GPIOCMD_WRITE_HIGH : GPIOCMD_WRITE_LOW);

	mutex_unlock(&dev->usb_lock);
	return err;
}

static int ftdi_gpio_dir_out(struct gpio_chip *gc, unsigned gpio_num, int val)
{
	uint16_t old_dirs, old_pins;
	int err = 0;
	struct ftdi_device *dev = container_of(gc, struct ftdi_device, gpio);

	mutex_lock(&dev->usb_lock);

	old_dirs = dev->gpio_dirs;
	old_pins = dev->gpio_pins;
	dev->gpio_dirs |= 1 << (gpio_num + gpio_offset);
	dev->gpio_pins &= ~1 << (gpio_num + gpio_offset);
	if (val) {
		dev->gpio_pins |= 1 << (gpio_num + gpio_offset);
	}

	if (old_dirs != dev->gpio_dirs || old_pins != dev->gpio_pins)
		err = ftdi_gpio_common(dev, gpio_num >= (8 - gpio_offset) ? GPIOCMD_WRITE_HIGH : GPIOCMD_WRITE_LOW);

	mutex_unlock(&dev->usb_lock);
	return err;
}

/* TODO: This sleeps */
static unsigned long ftdi_gpio_get_block(struct gpio_chip *gc, unsigned long mask)
{
	unsigned long result = 0;
	int i, err, cnt = 0;
	uint8_t *buf;

	struct ftdi_device *dev = container_of(gc, struct ftdi_device, gpio);

	if (!((mask << gpio_offset) & 0xFFFF))
		return -EINVAL;

	mutex_lock(&dev->usb_lock);

	/* get DMA memory */
	if ((buf = kzalloc(4, GFP_KERNEL)) == NULL)
		goto error_mem;

	/* request gpio pins */
	if ((err = ftdi_gpio_common(dev, GPIOCMD_READ_HIGH)))
		goto error;
	if ((err = ftdi_gpio_common(dev, GPIOCMD_READ_LOW)))
		goto error;

	/* wait for write to be processed */
	if (!usb_wait_anchor_empty_timeout(&dev->urb_submitted, WDR_ANCHOR_TIMEOUT)) {
		err = -ETIMEDOUT;
		goto error;
	}

	/* read back pins */
	/* some chips send back a couple status-only packets */
	/* first, so we loop until we get 4 bytes */
	for (i = 0; i < 10; i ++) {
		if ((err = usb_bulk_msg(dev->udev, usb_rcvbulkpipe(dev->udev, dev->in_ep),
		buf, 4, &cnt, WDR_SHORT_TIMEOUT)))
			goto error;
		if (cnt >= 4) break;
	}

	if (cnt < 4) {
		err = -ENODATA;
		goto error;
	}

	result = (buf[2] << 8) >> gpio_offset;
	result |= buf[3] >> gpio_offset;

	kfree(buf);
	mutex_unlock(&dev->usb_lock);
	return result;

error:
	kfree(buf);
	mutex_unlock(&dev->usb_lock);
	dev_warn(&dev->udev->dev, "%s - error %d", __func__, err);
	return err;
error_mem:
	mutex_unlock(&dev->usb_lock);
	return -ENOMEM;

}

static int ftdi_gpio_get(struct gpio_chip *gc, unsigned gpio_num)
{
	int result;
	struct ftdi_device *dev = container_of(gc, struct ftdi_device, gpio);

	/* If module parameter set, tell MPSSE state machine to wait for level on GPIO L1 */
	if (wait_on_l1 && gpio_num + gpio_offset == 5) {
		mutex_lock(&dev->usb_lock);
		if ((result = ftdi_gpio_common(dev, wait_on_l1[0] == 'l' ? GPIOCMD_WAIT_ON_L1LO : GPIOCMD_WAIT_ON_L1HI)))
			return result;

		mutex_unlock(&dev->usb_lock);

		/* return "ready" even though we told the MPSSE to wait */
		if (wait_on_l1[0] == 'l')
			return 0;
		else
			return 1;
	}

	result = ftdi_gpio_get_block(gc, 1 << gpio_num);
	if (result < 0) return result;

	return (result & (1 << gpio_num)) ? 1 : 0;
}


/*
 * ***************************************************************************
 * SPI specific functions
 * ***************************************************************************
 */

static void ftdi_spi_bulk_callback(struct urb *urb)
{
	struct ftdi_device *dev = urb->context;

	switch(urb->status) {
	case -ENOENT:
	case -ECONNRESET:
	case -ESHUTDOWN:
		dev_warn(&dev->udev->dev, "%s - write bulk status %d", __func__, urb->status);
		break;
	default:
		break;
	}

	kfree(urb->transfer_buffer);
	up(&dev->spi_limit_sem);
}


static void ftdi_spi_chipsel(struct spi_device *spi, int value)
{
	struct spi_bitbang *bang = spi_master_get_devdata(spi->master);
	struct ftdi_device *dev = container_of(bang, struct ftdi_device, spi);

	mutex_lock(&dev->usb_lock);

	if ((spi->mode & SPI_CS_HIGH && value == BITBANG_CS_ACTIVE) ||
	  (!(spi->mode & SPI_CS_HIGH) && value == BITBANG_CS_INACTIVE)) {
		dev->gpio_pins |= 1 << (spi->chip_select + 3);
	} else {
		dev->gpio_pins &= ~(1 << (spi->chip_select + 3));
	}

	ftdi_gpio_common(dev, GPIOCMD_WRITE_LOW);

	mutex_unlock(&dev->usb_lock);
}

static int ftdi_spi_setup(struct spi_device *spi)
{
	int err, cnt = 0;
	int buf_idx = 0;
	uint16_t divisor;
	struct spi_bitbang *bang = spi_master_get_devdata(spi->master);
	struct ftdi_device *dev = container_of(bang, struct ftdi_device, spi);
	uint8_t * buf;

	if (spi->bits_per_word && spi->bits_per_word != 8) {
		dev_err(&spi->dev, "only 8 bits per word supported\n");
		return -EINVAL;
	}

	/* get DMA memory, enough for all commands */
	if ((buf = kzalloc(16, GFP_KERNEL)) == NULL)
		goto error_mem;

	mutex_lock(&dev->usb_lock);

	/* default to write */
	dev->spi_write_cmd = FTDI_MPSSE_SPI_WRITE;
	if (!(spi->mode & SPI_CPOL))
		dev->spi_write_cmd |= FTDI_MPSSE_SPI_CPOL;
	if (spi->mode & SPI_LSB_FIRST)
		dev->spi_write_cmd |= FTDI_MPSSE_SPI_LSB;

	/* loopback mode enable */
	if (spi->mode & SPI_LOOP)
		buf[buf_idx++] = FTDI_MPSSE_LOOPON;
	else
		buf[buf_idx++] = FTDI_MPSSE_LOOPOFF;

	if (spi->max_speed_hz) {
		divisor = DIV_ROUND_UP(dev->tclk, spi->max_speed_hz) - 1;
		/* disable divide-by-5 */
		if (dev->tclk == 30000000)
			buf[buf_idx++] = FTDI_MPSSE_DIVBY5OFF;
		/* set divisor, low byte, high byte */
		buf[buf_idx++] = FTDI_MPSSE_DIVISOR;
		/* set buf[x] and buf[x+1] with length */
		*((uint16_t *)&buf[buf_idx]) = cpu_to_le16(divisor);
		buf_idx += 2;

		if ((err = usb_bulk_msg(dev->udev, usb_sndbulkpipe(dev->udev, dev->out_ep),
	            buf, buf_idx, &cnt, WDR_TIMEOUT)))
			goto error;
	}

	mutex_unlock(&dev->usb_lock);
	kfree(buf);

	ftdi_spi_chipsel(spi, 0);

	return 0;

error:
	mutex_unlock(&dev->usb_lock);
	kfree(buf);
	return err;
error_mem:
	return -ENOMEM;
}

static int ftdi_spi_setupxfer(struct spi_device *spi, struct spi_transfer *t)
{
	return 0;
}

static int ftdi_spi_txrx(struct spi_device *spi, struct spi_transfer *t)
{
	int err;
	struct urb *urb = NULL;
	char *buf = NULL;

	struct spi_bitbang *bang = spi_master_get_devdata(spi->master);
	struct ftdi_device *dev = container_of(bang, struct ftdi_device, spi);

	mutex_lock(&dev->usb_lock);

	if (down_interruptible(&dev->spi_limit_sem) < 0)
		return -EINTR;

	if (!(urb = usb_alloc_urb(0, GFP_KERNEL))) {
		err = -ENOMEM;
		goto error_no_buf;
	}

	/* allocate 3 more bytes for header */
	if (!(buf = kmalloc(t->len + 3, GFP_KERNEL))) {
		err = -ENOMEM;
		goto error_kmalloc;
	}

	buf[0] = dev->spi_write_cmd;
	/* set buf[1] and buf[2] with length */
	*((uint16_t *)&buf[1]) = cpu_to_le16(t->len - 1);

	if (t->tx_buf == NULL) {
		memset(&buf[3], 0, t->len);
	} else if (copy_from_user(&buf[3], t->tx_buf, t->len)) {
		err = -EFAULT;
		goto error_copy;
	}

	if (t->rx_buf != NULL)
		buf[0] |= FTDI_MPSSE_SPI_READ;

	/* finish tx of GPIO, if any */
	ftdi_gpio_tx(dev);

	usb_fill_bulk_urb(urb, dev->udev, usb_sndbulkpipe(dev->udev, dev->out_ep),
			buf, t->len + 3, ftdi_spi_bulk_callback, dev);

	usb_anchor_urb(urb, &dev->urb_submitted);

	if ((err = usb_submit_urb(urb, GFP_KERNEL))) {
		dev_err(&dev->udev->dev, "%s - failed write urb, err %d\n", __func__, err);
		goto error_unanchor;
	}

	usb_free_urb(urb);

	if (t->rx_buf != NULL) {
		int cnt;

		/* wait for all writes to be processed */
		if (!usb_wait_anchor_empty_timeout(&dev->urb_submitted, WDR_ANCHOR_TIMEOUT)) {
			err = -ETIMEDOUT;
			goto error_read;
		}

		/* read rx data from pipe, first 2 bytes are status */
		if ((err = usb_bulk_msg(dev->udev, usb_rcvbulkpipe(dev->udev, dev->in_ep),
			buf, t->len + 2, &cnt, WDR_TIMEOUT)))

		if (cnt < 2) {
			err = -ENODATA;
			goto error_read;
		}
		if (copy_to_user(t->rx_buf, &buf[2], cnt - 2)) {
			err = -EFAULT;
			goto error_read;
		}
		dev_warn(&dev->udev->dev, "%s - reading from SPI\n", __func__);
	}

	mutex_unlock(&dev->usb_lock);
	return t->len;

error_unanchor:
	usb_unanchor_urb(urb);
error_copy:
	kfree(buf);
error_kmalloc:
	usb_free_urb(urb);
error_no_buf:
	up(&dev->spi_limit_sem);
error_read:
	mutex_unlock(&dev->usb_lock);
	return err;
}


/*
 * ***************************************************************************
 * FTDI driver specific functions
 * ***************************************************************************
 */

/* Probe function to check for special devices */
static int ftdi_probe(struct usb_interface * interface,
			const struct usb_device_id *id)
{
	int i, err = 0;
	unsigned version, interfaces;
	struct usb_endpoint_descriptor *endpoint;
	struct usb_host_interface *altf;
	struct usb_device *udev = interface_to_usbdev(interface);
	struct ftdi_device *dev = NULL;
	struct spi_master *master = NULL;

	dev = kzalloc(sizeof(struct ftdi_device), GFP_KERNEL);
	if (dev == NULL) {
		dev_err(&interface->dev, "out of memory\n");
		goto error_mem;
	}

	sema_init(&dev->gpio_limit_sem, CONCURRENT_GPIO_WRITES);
	sema_init(&dev->spi_limit_sem, CONCURRENT_SPI_WRITES);

	/* coherent DMA pool for gpio writes */
	dev->gpio_pool = dma_pool_create(DRV_NAME, NULL, GPIO_BUF_LEN, 0, 0);
	if (dev->gpio_pool == NULL) {
		dev_err(&interface->dev, "can't request coherent pool\n");
		goto error_pool;
	}

	init_usb_anchor(&dev->urb_submitted);

	dev->udev = usb_get_dev(udev);
	dev->interface = interface->altsetting->desc.bInterfaceNumber;
	mutex_init(&dev->usb_lock);
	init_timer(&dev->timer);
	dev->timer.function = ftdi_gpio_timer;
	dev->timer.data = (unsigned long)dev;

	usb_set_intfdata(interface, dev);

	version = le16_to_cpu(udev->descriptor.bcdDevice);
	interfaces = udev->actconfig->desc.bNumInterfaces;

	switch (version) {
	case 0x500:
		dev->type = TYPE_FT2232C;
		dev->tclk = 6000000;
		break;
	case 0x700:
		dev->type = TYPE_FT2232H;
		dev->tclk = 30000000;
		break;
	case 0x800:
		dev->type = TYPE_FT4232H;
		dev->tclk = 30000000;
		break;
	case 0x900:
		dev->type = TYPE_FT232H;
		dev->tclk = 30000000;
		break;
	default:
		err = -EINVAL;
		dev_err(&interface->dev, "unsupported FTDI chip type: 0x%04x\n", version);
		goto error;
	}

	altf = interface->cur_altsetting;
	for (i = 0; i < altf->desc.bNumEndpoints; i ++) {
		endpoint = &altf->endpoint[i].desc;
		if (usb_endpoint_is_bulk_in(endpoint)) {
			dev->in_ep = endpoint->bEndpointAddress;
			dev->max_tx_size = usb_endpoint_maxp(endpoint);
			if (dev->max_tx_size > 512)
				dev->max_tx_size = 512;
		} else if (usb_endpoint_is_bulk_out(endpoint)) {
			dev->out_ep = endpoint->bEndpointAddress;
		}
	}

	if (dev->in_ep == 0 || dev->out_ep == 0) {
		err = -EINVAL;
		dev_err(&interface->dev, "unrecognized device descriptor\n");
		goto error;
	}

	if ((err = usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
			FTDI_SIO_SET_LATENCY_TIMER_REQUEST, FTDI_SIO_SET_LATENCY_TIMER_TYPE,
			latency_timer,
			dev->interface, NULL, 0, WDR_TIMEOUT)) < 0) {
		goto error_usb;
	}

	if ((err = usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
			FTDI_SIO_RESET_REQUEST, FTDI_SIO_RESET_TYPE,
			FTDI_SIO_RESET_SIO,
			dev->interface, NULL, 0, WDR_TIMEOUT)) < 0) {
		goto error_usb;
	}

	if ((err = usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
			FTDI_SIO_BITMODE_REQUEST, FTDI_SIO_BITMODE_TYPE,
			(FTDI_DEFAULT_DIRS & 0xFF) | (FTDI_SIO_BITMODE_MPSSE << 8),
			dev->interface, NULL, 0, WDR_TIMEOUT)) < 0) {
		goto error_usb;
	}


	gpio_offset = 4 + spi_numcs - 1;

	dev->gpio.base = atomic_read(&dyn_gpio_base);
	if (gpio_base >= 0)
		atomic_add(16 - gpio_offset, &dyn_gpio_base);

	dev->gpio.dev = &interface->dev;
	dev->gpio.ngpio = 16 - gpio_offset;
	dev->gpio.label = DRV_NAME;
	dev->gpio.owner = THIS_MODULE;
//	dev->gpio.can_sleep = 1;
	dev->gpio.direction_input = ftdi_gpio_dir_in;
	dev->gpio.direction_output = ftdi_gpio_dir_out;
	dev->gpio.get = ftdi_gpio_get;
	dev->gpio.set = ftdi_gpio_set;
//	dev->gpio.set_block = ftdi_gpio_set_block;
//	dev->gpio.get_block = ftdi_gpio_get_block;

	/* Set dir for SPI port, all others inputs */
	dev->gpio_dirs = FTDI_DEFAULT_DIRS;
	/* Everything low except CS is default high */
	dev->gpio_pins = FTDI_DEFAULT_PINS;

	/* Send dirs and pins to chip, low and high byte */
	ftdi_gpio_common(dev, GPIOCMD_WRITE_HIGH);
	ftdi_gpio_common(dev, GPIOCMD_WRITE_LOW);

	if ((err = gpiochip_add(&dev->gpio)) < 0) {
		goto error;
	}

	master = spi_alloc_master(&interface->dev, 0);
	if (master == NULL) {
		err = -ENOMEM;
		dev_err(&interface->dev, "can't allocate spi master\n");
		goto error;
	}

	spi_master_set_devdata(master, &dev->spi);
	master->bus_num = atomic_read(&dyn_spi_base);
	if (spi_base >= 0)
		atomic_inc(&dyn_spi_base);
	master->num_chipselect = spi_numcs;
	master->mode_bits = SPI_CS_HIGH;
	master->flags = SPI_MASTER_NO_RX;
	master->setup = ftdi_spi_setup;

	dev->spi.master = spi_master_get(master);

	dev->spi.setup_transfer = ftdi_spi_setupxfer;
	dev->spi.chipselect = ftdi_spi_chipsel;
	dev->spi.txrx_bufs = ftdi_spi_txrx;

	if ((err = spi_bitbang_start(&dev->spi)) != 0) {
		dev_err(&udev->dev, "can't start SPI\n");
		spi_master_put(dev->spi.master);
		goto error;
	}

	dev_info(&interface->dev, "driver loaded SPI bus %d, chip type %s, intf %d\n",
		master->bus_num, ftdi_chip_name[dev->type], dev->interface);

	return 0;

error_usb:
	dev_err(&interface->dev, "error configuring device\n");
	goto error;
error:
	usb_set_intfdata(interface, NULL);
	dma_pool_destroy(dev->gpio_pool);
	kfree(dev);

	return err;
error_pool:
	kfree(dev);
error_mem:
	return -ENOMEM;
}

static void ftdi_disconnect(struct usb_interface *interface)
{
	struct ftdi_device *dev;

	dev = usb_get_intfdata(interface);

	if (dev) {
		/* shutdown SPI and GPIO first */
		if (dev->spi.master) {
			spi_bitbang_stop(&dev->spi);
			spi_master_put(dev->spi.master);
		}

		if (gpiochip_remove(&dev->gpio) < 0) {
			dev_err(&interface->dev, "error removing GPIO\n");
		}

		/* wait for all writes to be processed */
		if (!usb_wait_anchor_empty_timeout(&dev->urb_submitted, WDR_ANCHOR_TIMEOUT)) {
			usb_poison_anchored_urbs(&dev->urb_submitted);
			usb_wait_anchor_empty_timeout(&dev->urb_submitted, WDR_ANCHOR_TIMEOUT);
			usb_kill_anchored_urbs(&dev->urb_submitted);
		}

		usb_set_intfdata(interface, NULL);

		dma_pool_destroy(dev->gpio_pool);
		kfree(dev);
	}

	dev_info(&interface->dev, "driver now disconnected\n");
}

/* module stuff */
static struct usb_driver ftdi_driver = {
	.name = 	DRV_NAME,
	.probe = 	ftdi_probe,
	.disconnect = 	ftdi_disconnect,
	.id_table = 	id_table_combined,
};

static int __init ftdi_init(void)
{
	int ret;

	if (spi_numcs < 1 || spi_numcs > 5) {
		pr_err("spi_numcs is invalid");
		return -EINVAL;
	}

	if (spi_numcs > 2 && wait_on_l1) {
		pr_err("max two CS with wait_on_l1");
		return -EINVAL;
	}

	if (wait_on_l1 && ((wait_on_l1[0] != 'l' && wait_on_l1[0] != 'h') || wait_on_l1[1] != 0)) {
		pr_err("wait_on_l1 must be unset, \'l\', or \'h\'");
		return -EINVAL;
	}

	/* set atomic variables to module parameters */
	atomic_set(&dyn_gpio_base, gpio_base);
	atomic_set(&dyn_spi_base, spi_base);

	if (vendor > 0 && product > 0) {
		/* Add user specified VID/PID to reserved element of table. */
		int i;
		for (i = 0; id_table_combined[i].idVendor; i++)
			;
		id_table_combined[i].match_flags = USB_DEVICE_ID_MATCH_DEVICE;
		id_table_combined[i].idVendor = vendor;
		id_table_combined[i].idProduct = product;
	}

	ret = usb_register(&ftdi_driver);

	if (ret < 0)
		goto err;

	return 0;

err:
	pr_debug("%s: failed=%d\n", __func__, ret);

	return ret;
}

static void __exit ftdi_exit(void)
{
	usb_deregister(&ftdi_driver);
}

module_init(ftdi_init);
module_exit(ftdi_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

module_param(vendor, ushort, 0);
MODULE_PARM_DESC(vendor, "User specified vendor ID (default="
		__MODULE_STRING(FTDI_VID)")");
module_param(product, ushort, 0);
MODULE_PARM_DESC(product, "User specified product ID");

module_param(gpio_base, int, 0);
MODULE_PARM_DESC(gpio_base, "GPIO base number");

module_param(spi_base, short, 0);
MODULE_PARM_DESC(spi_base, "SPI base number");

module_param(spi_numcs, int, 0);
MODULE_PARM_DESC(spi_numcs, "SPI number of chip selects");

module_param(wait_on_l1, charp, 0);
MODULE_PARM_DESC(wait_on_l1, "MPSSE wait with get request on L1 pin.  Set to 'l' for wait on LOW, 'h' for wait on HIGH");

module_param(latency_timer, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(latency_timer, "device latency timer override in milliseconds");

