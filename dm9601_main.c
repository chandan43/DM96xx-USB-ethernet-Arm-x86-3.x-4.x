/*
 * Davicom DM9601 USB 1.1 10/100Mbps ethernet devices
 *
 * Peter Korsgaard <jacmet@sunsite.dk>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

//#define DEBUG

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/stddef.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/usb.h>
#include <linux/crc32.h>
#include <linux/usb/usbnet.h>
#include <linux/slab.h>

/* datasheet:
 * https://github.com/chandan43/DM96xx-USB-ethernet-Arm-x86-3.x-4.x/blob/master/DM9601_datasheet.pdf
 */
 /* control requests : Page 08*/
enum REQ {
	DM_READ_REGS	= 0x00,
	DM_WRITE_REGS	= 0x01,
	DM_READ_MEMS	= 0x02,
	DM_WRITE_REG	= 0x03,
	DM_WRITE_MEMS	= 0x05,
	DM_WRITE_MEM	= 0x07,
 };

/* registers */
enum Reg {
	 DM_NET_CTRL	= 0x00,
 	 DM_RX_CTRL	= 0x05,
	 DM_SHARED_CTRL	= 0x0b,
	 DM_SHARED_ADDR	= 0x0c,
	 DM_SHARED_DATA	= 0x0d,	/* low + high */
	 DM_PHY_ADDR	= 0x10,	/* 6 bytes */
	 DM_MCAST_ADDR	= 0x16,	/* 8 bytes */
	 DM_GPR_CTRL	= 0x1e,
	 DM_GPR_DATA	= 0x1f,
	 DM_CHIP_ID	= 0x2c,
	 DM_MODE_CTRL	= 0x91,	/* only on dm9620 */

};

/* chip id values */
#define ID_DM9601	0
#define ID_DM9620	1

enum Limit {

	DM_MAX_MCAST	= 64,
	DM_MCAST_SIZE	= 8,
	DM_EEPROM_LEN	= 256,
	DM_TX_OVERHEAD	= 2,	/* 2 byte header */
	DM_RX_OVERHEAD	= 7,	/* 3 byte header + 4 byte crc tail */
	DM_TIMEOUT	= 1000,

};

static const struct usb_device_id products[] = {
	{
	 USB_DEVICE(0x0fe6, 0x9700),	/* DM9601 USB to Fast Ethernet Adapter */
//	 .driver_info = (unsigned long)&dm9601_info,
	 },
	{}, //END
};


MODULE_DEVICE_TABLE(usb, products);

static struct usb_driver dm9601_driver = {
		.name 	    =  "dm9601",
		.id_table   =  products,
		.probe 	    =  usbnet_probe,
		.disconnect =  usbnet_disconnect, 
		.suspend    =  usbnet_suspend, 
		.resume     =  usbnet_resume,
};

module_usb_driver(dm9601_driver);

MODULE_AUTHOR("Chandan Jha <beingchandanjha@gmail.com>");
MODULE_DESCRIPTION("Davicom DM9601 USB 1.1 ethernet devices");
MODULE_LICENSE("GPL");

