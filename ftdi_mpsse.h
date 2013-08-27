/*
 * Driver definitions for the USB FTDI SPI / GPIO driver
 */

#define FTDI_DEFAULT_DIRS       0x000B /* set SPI pin directions, and all GPIO pins to input */
#define FTDI_DEFAULT_PINS       0x0008 /* set CS default high */

/* URB timeouts */
#define WDR_TIMEOUT		500  /* default urb timeout */
#define WDR_SHORT_TIMEOUT	100  /* shorter urb timeout */
#define WDR_ANCHOR_TIMEOUT	5000 /* timeout for urb anchor wait */

/* MPSSE command set */
#define FTDI_MPSSE_SPI_CPOL     0x01 /* bitmask for SPI clock polarity */
#define FTDI_MPSSE_SPI_LSB      0x08 /* bitmask for SPI LSB first */
#define FTDI_MPSSE_SPI_WRITE    0x10 /* bitmask for write to SPI */
#define FTDI_MPSSE_SPI_READ     0x20 /* bitmask for read from SPI */

#define FTDI_MPSSE_WRITELOW     0x80 /* write the low byte to gpio */
#define FTDI_MPSSE_READLOW      0x81 /* read the low byte from gpio */
#define FTDI_MPSSE_WRITEHIGH    0x82 /* write the high byte to gpio */
#define FTDI_MPSSE_READHIGH     0x83 /* read the high byte from gpio */
#define FTDI_MPSSE_LOOPON       0x84 /* turn on TDO to TDI loopback */
#define FTDI_MPSSE_LOOPOFF      0x85 /* turn off TDO to TDI loopback */
#define FTDI_MPSSE_DIVISOR      0x86 /* set clock divisor */
#define FTDI_MPSSE_WAIT_ON_HI   0x88 /* wait for high on GPIOL1 */
#define FTDI_MPSSE_WAIT_ON_LO   0x89 /* wait for low on GPIOL1 */

#define FTDI_MPSSE_DIVBY5OFF    0x8A /* turn off divide-by-5 */

/*** "original" FTDI device VID/PIDs ***/
#define FTDI_VID		0x0403 /* Vendor Id */

#define FTDI_8U232AM_PID	0x6001 /* Similar device to SIO above */
#define FTDI_8U232AM_ALT_PID	0x6006 /* FTDI's alternate PID for above */
#define FTDI_8U2232C_PID	0x6010 /* Dual channel device */
#define FTDI_4232H_PID		0x6011 /* Quad channel hi-speed device */
#define FTDI_232H_PID		0x6014 /* Single channel hi-speed device */
#define FTDI_FTX_PID		0x6015 /* FT-X series (FT201X, FT230X, FT231X, etc) */

/* Commands */

/* FTDI_SIO_RESET */
#define FTDI_SIO_RESET_REQUEST	0x00
#define FTDI_SIO_RESET_TYPE	0x40
#define FTDI_SIO_RESET_SIO	0x00
#define FTDI_SIO_RESET_PURGE_RX	0x01
#define FTDI_SIO_RESET_PURGE_TX	0x02

/*
 * BmRequestType:  0100 0000B
 * bRequest:       FTDI_SIO_RESET
 * wValue:         Control Value
 *                   0 = Reset SIO
 *                   1 = Purge RX buffer
 *                   2 = Purge TX buffer
 * wIndex:         Port
 * wLength:        0
 * Data:           None
 *
 * The Reset SIO command has this effect:
 *
 *    Sets flow control set to 'none'
 *    Event char = $0D
 *    Event trigger = disabled
 *    Purge RX buffer
 *    Purge TX buffer
 *    Clear DTR
 *    Clear RTS
 *    baud and data format not reset
 *
 * The Purge RX and TX buffer commands affect nothing except the buffers
 *
 */

/*
 * FTDI_SIO_SET_LATENCY_TIMER
 *
 * Set the timeout interval. The FTDI collects data from the slave
 * device, transmitting it to the host when either A) 62 bytes are
 * received, or B) the timeout interval has elapsed and the buffer
 * contains at least 1 byte.  Setting this value to a small number
 * can dramatically improve performance for applications which send
 * small packets, since the default value is 16ms.
 */
#define  FTDI_SIO_SET_LATENCY_TIMER_REQUEST 0x09
#define  FTDI_SIO_SET_LATENCY_TIMER_TYPE    0x40

/*
 *  BmRequestType:   0100 0000b
 *  bRequest:        FTDI_SIO_SET_LATENCY_TIMER
 *  wValue:          Latency (milliseconds)
 *  wIndex:          Port
 *  wLength:         0
 *  Data:            None
 *
 * wValue:
 *   B0..7   Latency timer
 *   B8..15  0
 *
 */


/* FTDI_SIO_BITMODE */
#define FTDI_SIO_BITMODE_REQUEST   0x0B
#define FTDI_SIO_BITMODE_TYPE      0x40
#define FTDI_SIO_BITMODE_MPSSE     0x02

/* Descriptors returned by the device
 *
 *  Device Descriptor
 *
 * Offset	Field		Size	Value	Description
 * 0	bLength		1	0x12	Size of descriptor in bytes
 * 1	bDescriptorType	1	0x01	DEVICE Descriptor Type
 * 2	bcdUSB		2	0x0110	USB Spec Release Number
 * 4	bDeviceClass	1	0x00	Class Code
 * 5	bDeviceSubClass	1	0x00	SubClass Code
 * 6	bDeviceProtocol	1	0x00	Protocol Code
 * 7	bMaxPacketSize0 1	0x08	Maximum packet size for endpoint 0
 * 8	idVendor	2	0x0403	Vendor ID
 * 10	idProduct	2	0x8372	Product ID (FTDI_SIO_PID)
 * 12	bcdDevice	2	0x0001	Device release number
 * 14	iManufacturer	1	0x01	Index of man. string desc
 * 15	iProduct	1	0x02	Index of prod string desc
 * 16	iSerialNumber	1	0x02	Index of serial nmr string desc
 * 17	bNumConfigurations 1    0x01	Number of possible configurations
 *
 * Configuration Descriptor
 *
 * Offset	Field			Size	Value
 * 0	bLength			1	0x09	Size of descriptor in bytes
 * 1	bDescriptorType		1	0x02	CONFIGURATION Descriptor Type
 * 2	wTotalLength		2	0x0020	Total length of data
 * 4	bNumInterfaces		1	0x01	Number of interfaces supported
 * 5	bConfigurationValue	1	0x01	Argument for SetCOnfiguration() req
 * 6	iConfiguration		1	0x02	Index of config string descriptor
 * 7	bmAttributes		1	0x20	Config characteristics Remote Wakeup
 * 8	MaxPower		1	0x1E	Max power consumption
 *
 * Interface Descriptor
 *
 * Offset	Field			Size	Value
 * 0	bLength			1	0x09	Size of descriptor in bytes
 * 1	bDescriptorType		1	0x04	INTERFACE Descriptor Type
 * 2	bInterfaceNumber	1	0x00	Number of interface
 * 3	bAlternateSetting	1	0x00	Value used to select alternate
 * 4	bNumEndpoints		1	0x02	Number of endpoints
 * 5	bInterfaceClass		1	0xFF	Class Code
 * 6	bInterfaceSubClass	1	0xFF	Subclass Code
 * 7	bInterfaceProtocol	1	0xFF	Protocol Code
 * 8	iInterface		1	0x02	Index of interface string description
 *
 * IN Endpoint Descriptor
 *
 * Offset	Field			Size	Value
 * 0	bLength			1	0x07	Size of descriptor in bytes
 * 1	bDescriptorType		1	0x05	ENDPOINT descriptor type
 * 2	bEndpointAddress	1	0x82	Address of endpoint
 * 3	bmAttributes		1	0x02	Endpoint attributes - Bulk
 * 4	bNumEndpoints		2	0x0040	maximum packet size
 * 5	bInterval		1	0x00	Interval for polling endpoint
 *
 * OUT Endpoint Descriptor
 *
 * Offset	Field			Size	Value
 * 0	bLength			1	0x07	Size of descriptor in bytes
 * 1	bDescriptorType		1	0x05	ENDPOINT descriptor type
 * 2	bEndpointAddress	1	0x02	Address of endpoint
 * 3	bmAttributes		1	0x02	Endpoint attributes - Bulk
 * 4	bNumEndpoints		2	0x0040	maximum packet size
 * 5	bInterval		1	0x00	Interval for polling endpoint
 *
 * DATA FORMAT
 *
 * IN Endpoint
 *
 * The device reserves the first two bytes of data on this endpoint to contain
 * the current values of the modem and line status registers. In the absence of
 * data, the device generates a message consisting of these two status bytes
 * every 40 ms
 *
 * Byte 0: Modem Status
 *
 * Offset	Description
 * B0	Reserved - must be 1
 * B1	Reserved - must be 0
 * B2	Reserved - must be 0
 * B3	Reserved - must be 0
 * B4	Clear to Send (CTS)
 * B5	Data Set Ready (DSR)
 * B6	Ring Indicator (RI)
 * B7	Receive Line Signal Detect (RLSD)
 *
 * Byte 1: Line Status
 *
 * Offset	Description
 * B0	Data Ready (DR)
 * B1	Overrun Error (OE)
 * B2	Parity Error (PE)
 * B3	Framing Error (FE)
 * B4	Break Interrupt (BI)
 * B5	Transmitter Holding Register (THRE)
 * B6	Transmitter Empty (TEMT)
 * B7	Error in RCVR FIFO
 *
 */
#define FTDI_RS0_CTS	(1 << 4)
#define FTDI_RS0_DSR	(1 << 5)
#define FTDI_RS0_RI	(1 << 6)
#define FTDI_RS0_RLSD	(1 << 7)

#define FTDI_RS_DR	1
#define FTDI_RS_OE	(1<<1)
#define FTDI_RS_PE	(1<<2)
#define FTDI_RS_FE	(1<<3)
#define FTDI_RS_BI	(1<<4)
#define FTDI_RS_THRE	(1<<5)
#define FTDI_RS_TEMT	(1<<6)
#define FTDI_RS_FIFO	(1<<7)

/*
 * OUT Endpoint
 *
 * This device reserves the first bytes of data on this endpoint contain the
 * length and port identifier of the message. For the FTDI USB Serial converter
 * the port identifier is always 1.
 *
 * Byte 0: Line Status
 *
 * Offset	Description
 * B0	Reserved - must be 1
 * B1	Reserved - must be 0
 * B2..7	Length of message - (not including Byte 0)
 *
 */

