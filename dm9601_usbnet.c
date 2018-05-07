#include <linux/module.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ctype.h>
#include <linux/ethtool.h>
#include <linux/workqueue.h>
#include <linux/mii.h>
#include <linux/usb.h>
#include <linux/usb/usbnet.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/pm_runtime.h>

/*-------------------------------------------------------------------------*/
/* work that cannot be done in interrupt context uses keventd.
 *
 * NOTE:  with 2.5 we could do more of this using completion callbacks,
 * especially now that control transfers can be queued.
 */

/**
 * usb_clear_halt - tells device to clear endpoint halt/stall condition
 * @dev: device whose endpoint is halted
 * @pipe: endpoint "pipe" being cleared
 * Context: !in_interrupt ()
 *
 * This is used to clear halt conditions for bulk and interrupt endpoints,
 * as reported by URB completion status.  Endpoints that are halted are
 * sometimes referred to as being "stalled".  Such endpoints are unable
 * to transmit or receive data until the halt status is cleared.  Any URBs
 * queued for such an endpoint should normally be unlinked by the driver
 * before clearing the halt condition, as described in sections 5.7.5
 * and 5.8.5 of the USB 2.0 spec.
 *
 * Note that control and isochronous endpoints don't halt, although control
 * endpoints report "protocol stall" (for unsupported requests) using the
 * same status code used to report a true stall.
 *
 * This call is synchronous, and may not be used in an interrupt context.
 *
 * Return: Zero on success, or else the status code returned by the
 * underlying usb_control_msg() call.
 */
static void
kevent (struct work_struct *work)
{
	struct usbnet	*dev = 
		container_of(work, struct usbnet, kevent);
	int status;
	/* usb_clear_halt() needs a thread context */
	if (test_bit (EVENT_TX_HALT, &dev->flags)) {
		unlink_urbs (dev, &dev->txq);
		status = usb_autopm_get_interface(dev->intf);
		if(status < 0)
			goto fail_pipe;
		status = usb_clear_halt (dev->udev, dev->out);
		usb_autopm_put_interface(dev->intf); 
		if (status < 0 && 
			status != -EPIPE &&
				status != -ESHUTDOWN ) {
			if (netif_msg_tx_err (dev))
fail_pipe:
		 netdev_err(dev->net, "can't clear tx halt, status %d\n",
				status);
		}else {
			clear_bit(EVENT_TX_HALT, &dev->flags);
			if (status != -ESHUTDOWN)
				netif_wake_queue (dev->net);
		}
	}
	if (test_bit (EVENT_RX_HALT, &dev->flags)) {
		unlink_urbs (dev, &dev->rxq);
		status = usb_autopm_get_interface(dev->intf);
		if (status < 0)
			goto fail_halt;
		status = usb_clear_halt (dev->udev, dev->in);
		usb_autopm_put_interface(dev->intf);
		if (status < 0 &&
		    status != -EPIPE &&
		    status != -ESHUTDOWN) {
			if (netif_msg_rx_err (dev))
fail_halt:
				netdev_err(dev->net, "can't clear rx halt, status %d\n",
					   status);
		} else {
			clear_bit (EVENT_RX_HALT, &dev->flags);
			tasklet_schedule (&dev->bh);
		}
	}

	/* tasklet could resubmit itself forever if memory is tight */
	if (test_bit (EVENT_RX_MEMORY, &dev->flags)) {
		struct urb	*urb = NULL;
		int resched = 1;

		if (netif_running (dev->net))
			urb = usb_alloc_urb (0, GFP_KERNEL);
		else
			clear_bit (EVENT_RX_MEMORY, &dev->flags);
		if (urb != NULL) {
			clear_bit (EVENT_RX_MEMORY, &dev->flags);
			status = usb_autopm_get_interface(dev->intf);
			if (status < 0) {
				usb_free_urb(urb);
				goto fail_lowmem;
			}
			if (rx_submit (dev, urb, GFP_KERNEL) == -ENOLINK)
				resched = 0;
			usb_autopm_put_interface(dev->intf);
fail_lowmem:
			if (resched)
				tasklet_schedule (&dev->bh);
		}
	}

	if (test_bit (EVENT_LINK_RESET, &dev->flags)) {
		struct driver_info	*info = dev->driver_info;
		int			retval = 0;

		clear_bit (EVENT_LINK_RESET, &dev->flags);
		status = usb_autopm_get_interface(dev->intf);
		if (status < 0)
			goto skip_reset;
		if(info->link_reset && (retval = info->link_reset(dev)) < 0) {
			usb_autopm_put_interface(dev->intf);
skip_reset:
			netdev_info(dev->net, "link reset failed (%d) usbnet usb-%s-%s, %s\n",
				    retval,
				    dev->udev->bus->bus_name,
				    dev->udev->devpath,
				    info->description);
		} else {
			usb_autopm_put_interface(dev->intf);
		}

		/* handle link change from link resetting */
		__handle_link_change(dev); //TODO:
	}
	if (test_bit (EVENT_LINK_CHANGE, &dev->flags))
		__handle_link_change(dev);

	if (dev->flags)
		netdev_dbg(dev->net, "kevent done, flags = 0x%lx\n", dev->flags);	

}  


/*-------------------------------------------------------------------------*/
/**
 * mod_timer - modify a timer's timeout
 * @timer: the timer to be modified
 * @expires: new timeout in jiffies
 *
 * mod_timer() is a more efficient way to update the expire field of an
 * active timer (if the timer is inactive it will be activated)
 *
 * mod_timer(timer, expires) is equivalent to:
 *
 *     del_timer(timer); timer->expires = expires; add_timer(timer);
 *
 * Note that if there are multiple unserialized concurrent users of the
 * same timer, then mod_timer() is the only safe way to modify the timeout,
 * since add_timer() cannot modify an already running timer.
 *
 * The function returns whether it has modified a pending timer or not.
 * (ie. mod_timer() of an inactive timer returns 0, mod_timer() of an
 * active timer returns 1.)
 */

static void tx_complete (struct urb *urb)
{
	struct sk_buff		*skb = (struct sk_buff *) urb->context;
	struct skb_data		*entry = (struct skb_data *) skb->cb;
	struct usbnet		*dev = entry->dev;
	if (urb->status == 0) {
		if (!(dev->driver_info->flags & FLAG_MULTI_PACKET))
			dev->net->stats.tx_packets++;
		dev->net->stats.tx_bytes += entry->length;
	}else {
		dev->net->stats.tx_errors++;
		switch (urb->status) {
			case -EPIPE:         /* Broken pipe */
				usbnet_defer_kevent (dev, EVENT_TX_HALT);
				break;
			/* software-driven interface shutdown */
			case -ECONNRESET:		// async unlink
			case -ESHUTDOWN:		// hardware gone
				break;

			// like rx, tx gets controller i/o faults during khubd delays
			// and so it uses the same throttling mechanism.
			case -EPROTO: /* Protocol error */
			case -ETIME:  /* Timer expired */
			case -EILSEQ: /* Illegal byte sequence */
				usb_mark_last_busy(dev->udev);
				if (!timer_pending (&dev->delay)) {
					mod_timer (&dev->delay,
					jiffies + THROTTLE_JIFFIES);
					netif_dbg(dev, link, dev->net,
					 	 "tx throttle %d\n", urb->status);
				}
				netif_stop_queue (dev->net);
				break;
			case default:
				netif_dbg(dev, tx_err, dev->net,
				  "tx err %d\n", entry->urb->status);
				break; 
		} 
	}
	usb_autopm_put_interface_async(dev->intf);
	(void) defer_bh(dev, skb, &dev->txq, tx_done); //TODO
}

/*-------------------------------------------------------------------------*/

void usbnet_tx_timeout (struct net_device *net)
{
	struct usbnet		*dev = netdev_priv(net);

	unlink_urbs (dev, &dev->txq);
	tasklet_schedule (&dev->bh);

	// FIXME: device recovery -- reset?
}

/*-------------------------------------------------------------------------*/

/**
 * skb_tx_timestamp() - Driver hook for transmit timestamping
 *
 * Ethernet MAC Drivers should call this function in their hard_xmit()
 * function immediately before giving the sk_buff to the MAC hardware.
 *
 * Specifically, one should make absolutely sure that this function is
 * called before TX completion of this packet can trigger.  Otherwise
 * the packet could potentially already be freed.
 *
 * @skb: A socket buffer.
 */
 /* If I understand well cdc allows back-to-back sending of maximumlength 
    packets and you need a "stop" signal to end the transfer.
    HID stops after either a short or a maximumlength packet.
 */
/**
 *	skb_tailroom - bytes at buffer end
 *	@skb: buffer to check
 *
 *	Return the number of bytes of free space at the tail of an sk_buff
 */
/**
 * usb_autopm_get_interface_async - increment a USB interface's PM-usage counter
 * @intf: the usb_interface whose counter should be incremented
 *
 * This routine does much the same thing as
 * usb_autopm_get_interface(): It increments @intf's usage counter and
 * queues an autoresume request if the device is suspended.  The
 * differences are that it does not perform any synchronization (callers
 * should hold a private lock and handle all synchronization issues
 * themselves), and it does not autoresume the device directly (it only
 * queues a request).  After a successful call, the device may not yet be
 * resumed.
 *
 * This routine can run in atomic context.
 *
 * Return: 0 on success. A negative error code otherwise.
 */
/**
 * usb_anchor_urb - anchors an URB while it is processed
 * @urb: pointer to the urb to anchor
 * @anchor: pointer to the anchor
 *
 * This can be called to have access to URBs which are to be executed
 * without bothering to track them
 */
/**
 *	netif_stop_queue - stop transmitted packets
 *	@dev: network device
 *
 *	Stop upper layers calling the device hard_start_xmit routine.
 *	Used for flow control when transmit resources are unavailable.
 */
/**
 * usb_free_urb - frees the memory used by a urb when all users of it are finished
 * @urb: pointer to the urb to free, may be NULL
 *
 * Must be called when a user of a urb is finished with it.  When the last user
 * of the urb calls this function, the memory of the urb is freed.
 *
 * Note: The transfer buffer associated with the urb is not freed unless the
 * URB_FREE_BUFFER transfer flag is set.
 */
/**
 * usb_get_urb - increments the reference count of the urb
 * @urb: pointer to the urb to modify, may be NULL
 *
 * This must be  called whenever a urb is transferred from a device driver to a
 * host controller driver.  This allows proper reference counting to happen
 * for urbs.
 *
 * Return: A pointer to the urb with the incremented reference counter.
 */
netdev_tx_t usbnet_start_xmit (struct sk_buff *skb,
				     struct net_device *net)
{
	struct usbnet		*dev = netdev_priv(net);
	int			length;
	struct urb		*urb = NULL;
	struct skb_data		*entry;
	struct driver_info	*info = dev->driver_info;
	unsigned long		flags;
	int retval;	
	
	if (skb)
		skb_tx_timestamp(skb);
	
	// some devices want funky USB-level framing, for
	// win32 driver (usually) and/or hardware quirks
	if (info->tx_fixup) {
		skb = info->tx_fixup (dev, skb, GFP_ATOMIC);
		if (!skb) {
			/* packet collected; minidriver waiting for more */
			if (info->flags & FLAG_MULTI_PACKET)
				goto not_drop;
			netif_dbg(dev, tx_err, dev->net, "can't tx_fixup skb\n");
			goto drop;
		}	
	}
	length = skb->len;

	if (!(urb = usb_alloc_urb (0, GFP_ATOMIC))) {
		netif_dbg(dev, tx_err, dev->net, "no urb\n");
		goto drop;
	}
	
	entry = (struct skb_data *) skb->cb;
	entry->urb = urb;
	entry->dev = dev;
	entry->length = length;
	
	usb_fill_bulk_urb (urb, dev->udev, dev->out,
			skb->data, skb->len, tx_complete, skb);
	
	/* don't assume the hardware handles USB_ZERO_PACKET
	 * NOTE:  strictly conforming cdc-ether devices should expect
	 * the ZLP here, but ignore the one-byte packet.
	 * NOTE2: CDC NCM specification is different from CDC ECM when
	 * handling ZLP/short packets, so cdc_ncm driver will make short
	 * packet itself if needed.
	 */

	 if (length % dev->maxpacket == 0) {
		if (!(info->flags & FLAG_SEND_ZLP)) {
			if (!(info->flags & FLAG_MULTI_PACKET)) {
				urb->transfer_buffer_length++;
				if (skb_tailroom(skb)) {
					skb->data[skb->len] = 0;
					__skb_put(skb, 1);
				}
			}
		} else
			urb->transfer_flags |= URB_ZERO_PACKET;
	}
	spin_lock_irqsave(&dev->txq.lock, flags);
	retval = usb_autopm_get_interface_async(dev->intf);
	if (retval < 0) {
		spin_unlock_irqrestore(&dev->txq.lock, flags);
		goto drop;
	}
#ifdef CONFIG_PM
	/* if this triggers the device is still a sleep */
	if (test_bit(EVENT_DEV_ASLEEP, &dev->flags)) {
		/* transmission will be done in resume */
		usb_anchor_urb(urb, &dev->deferred);
		/* no use to process more packets */
		netif_stop_queue(net);
		usb_put_urb(urb);
		spin_unlock_irqrestore(&dev->txq.lock, flags);
		netdev_dbg(dev->net, "Delaying transmission for resumption\n");
		goto deferred;
	}
#endif

	switch ((retval = usb_submit_urb (urb, GFP_ATOMIC))) {
		case -EPIPE:
			netif_stop_queue (net);
			usbnet_defer_kevent (dev, EVENT_TX_HALT);
			usb_autopm_put_interface_async(dev->intf);
			break;
		default:
			usb_autopm_put_interface_async(dev->intf);
			netif_dbg(dev, tx_err, dev->net,
			  	"tx: submit urb err %d\n", retval);
			break;
		case 0:
			net->trans_start = jiffies;
			__usbnet_queue_skb(&dev->txq, skb, tx_start);
			if (dev->txq.qlen >= TX_QLEN (dev))
				netif_stop_queue (net);
	}
	spin_unlock_irqrestore (&dev->txq.lock, flags);
	if (retval) {
		netif_dbg(dev, tx_err, dev->net, "drop, code %d\n", retval);
drop:
		dev->net->stats.tx_dropped++;
not_drop:
		if (skb)
			dev_kfree_skb_any (skb);
		usb_free_urb (urb);
	} else
		netif_dbg(dev, tx_queued, dev->net,
			  "> tx, len %d, type 0x%x\n", length, skb->protocol);
#ifdef CONFIG_PM
deferred:
#endif
	return NETDEV_TX_OK;	
}

/**
 * usb_alloc_urb - creates a new urb for a USB driver to use
 * @iso_packets: number of iso packets for this urb
 * @mem_flags: the type of memory to allocate, see kmalloc() for a list of
 *	valid options for this.
 *
 * Creates an urb for the USB driver to use, initializes a few internal
 * structures, increments the usage counter, and returns a pointer to it.
 *
 * If the driver want to use this urb for interrupt, control, or bulk
 * endpoints, pass '0' as the number of iso packets.
 *
 * The driver must call usb_free_urb() when it is finished with the urb.
 *
 * Return: A pointer to the new urb, or %NULL if no memory is available.
 */
static int rx_alloc_submit(struct usbnet *dev, gfp_t flags)
{
	
	struct urb	*urb;
	int		i;
	int		ret = 0;

	/* don't refill the queue all at once */
	for (i = 0; i < 10 && dev->rxq.qlen < RX_QLEN(dev); i++) {
		urb = usb_alloc_urb(0, flags);
		if (urb != NULL) {
			ret = rx_submit(dev, urb, flags); //TODO
			if (ret)
				goto err;
		} else {
			ret = -ENOMEM;
			goto err;
		}	
	}
err:
	return ret;	
}

/*-------------------------------------------------------------------------*/

// tasklet (work deferred from completions, in_irq) or timer

/**
 *	__skb_dequeue - remove from the head of the queue
 *	@list: list to dequeue from
 *
 *	Remove the head of the list. This function does not take any locks
 *	so must be used with appropriate locks held only. The head item is
 *	returned or %NULL if the list is empty.
 */
/**
 *	netif_carrier_ok - test if carrier present
 *	@dev: network device
 *
 * Check if carrier is present on device
 */

static void usbnet_bh(unsigned long param)
{
	struct usbnet *dev = (struct usbnet *) param;
	struct sk_buff		*skb;
	struct skb_data		*entry;
	
	while ((skb = skb_dequeue(&dev->done)))	{
		entry = (struct skb_data *) skb->cb;
		switch (entry->state) {
			case rx_done:
				entry->state = rx_done;
				rx_process (dev, skb);
				continue;
			case tx_done:
			case rx_cleanup:
				usb_free_urb(entry->urb);
				dev_kfree_skb (skb);
				continue;
			default:
				netdev_dbg(dev->net, "bogus skb state %d\n", entry->state); 		
		}
	}
	/* restart RX again after disabling due to high error rate */
	clear_bit(EVENT_RX_KILL, &dev->flags);
	// waiting for all pending urbs to complete?
	if (dev->wait) {
		if ((dev->txq.qlen + dev->rxq.qlen + dev->done.qlen) == 0) {
			wake_up (dev->wait);
		// or are we maybe short a few urbs?
		}else if (netif_running (dev->net) &&
		   netif_device_present (dev->net) &&
		   netif_carrier_ok(dev->net) &&
		   !timer_pending (&dev->delay) &&
		   !test_bit (EVENT_RX_HALT, &dev->flags))  {
			int	temp = dev->rxq.qlen;
			if (temp < RX_QLEN(dev)) {
				if (rx_alloc_submit(dev, GFP_ATOMIC) == -ENOLINK) //TODO
					return;
				if (temp != dev->rxq.qlen)
					netif_dbg(dev, link, dev->net,
					  	"rxqlen %d --> %d\n",
					  	temp, dev->rxq.qlen);
				if (dev->rxq.qlen < RX_QLEN(dev))
					tasklet_schedule (&dev->bh);			
			}
			if (dev->txq.qlen < TX_QLEN (dev))
				netif_wake_queue (dev->net);
		}

	}	
}


static const struct net_device_ops usbnet_netdev_ops = {
	.ndo_open		= usbnet_open,
	.ndo_stop		= usbnet_stop,
	.ndo_start_xmit		= usbnet_start_xmit,
	.ndo_tx_timeout		= usbnet_tx_timeout,
	.ndo_change_mtu		= usbnet_change_mtu,
	.ndo_set_mac_address 	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
};

/*-------------------------------------------------------------------------*/

// precondition: never called in_interrupt

static struct device_type wlan_type = {
	.name	= "wlan",
};

static struct device_type wwan_type = {
	.name	= "wwan",
};

/**
 * pm_runtime_enable - Enable runtime PM of a device.
 * @dev: Device to handle.
 */
/* Set the sysfs physical device reference for the network logical device
 * if set prior to registration will cause a symlink during initialization.
 */
/**
 *	netdev_priv - access network device private data
 *	@dev: network device
 *
 * Get network device private data
 */
/*
 * This function creates a split out lock class for each invocation;
 * this is needed for now since a whole lot of users of the skb-queue
 * infrastructure in drivers have different locking usage (in hardirq)
 * than the networking core (in softirq only). In the long run either the
 * network layer or drivers should need annotation to consolidate the
 * main types of usage into 3 classes.
 */
/**
 * usb_set_interface - Makes a particular alternate setting be current
 * @dev: the device whose interface is being updated
 * @interface: the interface being updated
 * @alternate: the setting being chosen.
 * Context: !in_interrupt ()
 *
 * This is used to enable data transfers on interfaces that may not
 * be enabled by default.  Not all devices support such configurability.
 * Only the driver bound to an interface may change its setting.
 *
 * Within any given configuration, each interface may have several
 * alternative settings.  These are often used to control levels of
 * bandwidth consumption.  For example, the default setting for a high
 * speed interrupt endpoint may not send more than 64 bytes per microframe,
 * while interrupt transfers of up to 3KBytes per microframe are legal.
 * Also, isochronous endpoints may never be part of an
 * interface's default setting.  To access such bandwidth, alternate
 * interface settings must be made current.
 *
 * Note that in the Linux USB subsystem, bandwidth associated with
 * an endpoint in a given alternate setting is not reserved until an URB
 * is submitted that needs that bandwidth.  Some other operating systems
 * allocate bandwidth early, when a configuration is chosen.
 *
 * This call is synchronous, and may not be used in an interrupt context.
 * Also, drivers must not change altsettings while urbs are scheduled for
 * endpoints in that interface; all such urbs must first be completed
 * (perhaps forced by unlinking).
 *
 * Return: Zero on success, or else the status code returned by the
 * underlying usb_control_msg() call.
 */
/**
 *	register_netdev	- register a network device
 *	@dev: device to register
 *
 *	Take a completed network device structure and add it to the kernel
 *	interfaces. A %NETDEV_REGISTER message is sent to the netdev notifier
 *	chain. 0 is returned on success. A negative errno code is returned
 *	on a failure to set up the device, or if the name is a duplicate.
 *
 *	This is a wrapper around register_netdevice that takes the rtnl semaphore
 *	and expands the device name if you passed a format string to
 *	alloc_netdev.
 */
/*-------------------------------------------------------------------------
 *
 * USB Device Driver support
 *
 *-------------------------------------------------------------------------*/

// precondition: never called in_interrupt
int usbnet_probe (struct usb_interface *udev,		 
			const struct usb_device_id *prod)
{
	struct usbnet			*dev;
	struct net_device		*net;
	struct usb_host_interface	*interface;
	struct driver_info		*info;
	struct usb_device		*xdev;
	int				status;
	const char			*name;
	struct usb_driver *driver       = to_usb_driver(udev->dev.driver);
	
	/* usbnet already took usb runtime pm, so have to enable the feature
	 * for usb interface, otherwise usb_autopm_get_interface may return
	 * failure if RUNTIME_PM is enabled.
	 */
	if (!driver->supports_autosuspend) {
		driver->supports_autosuspend = 1;
		pm_runtime_enable(&udev->dev);
	}
	name = udev->dev.driver->name;
	info = (struct driver_info *) prod->driver_info;	
		
	if (!info) {
		dev_dbg (&udev->dev, "blacklisted by %s\n", name);
		return -ENODEV;
	}
	
	xdev = interface_to_usbdev(udev);
	interface = udev->cur_altsetting;
	
	status = -ENOMEM;
	// set up our own records
	net = alloc_etherdev(sizeof(*dev));
	if (!net)
		goto out;
	/* netdev_printk() needs this so do it as early as possible */
	SET_NETDEV_DEV(net, &udev->dev);
	
	dev = netdev_priv(net);
	dev->udev = xdev;
	dev->intf = udev;
	dev->driver_info = info;
	dev->driver_name = name;
	dev->msg_enable = netif_msg_init (msg_level, NETIF_MSG_DRV
				| NETIF_MSG_PROBE | NETIF_MSG_LINK);
	skb_queue_head_init (&dev->rxq);
	skb_queue_head_init (&dev->txq);
	skb_queue_head_init (&dev->done);
	skb_queue_head_init(&dev->rxq_pause);
	dev->bh.func = usbnet_bh; //TODO
	dev->bh.data = (unsigned long) dev;
	INIT_WORK (&dev->kevent, kevent); //TODO
	init_usb_anchor(&dev->deferred);
	dev->delay.function = usbnet_bh;
	dev->delay.data = (unsigned long) dev;
	init_timer (&dev->delay);
	mutex_init (&dev->phy_mutex);
	mutex_init (&dev->interrupt_mutex);
	dev->interrupt_count = 0;

	dev->net = net;
	strcpy (net->name, "usb%d");
	memcpy (net->dev_addr, node_id, sizeof node_id); //TODO
	
	
	/* rx and tx sides can use different message sizes;
	 * bind() should set rx_urb_size in that case.
	 */
	dev->hard_mtu = net->mtu + net->hard_header_len;
#if 0
// dma_supported() is deeply broken on almost all architectures
	// possible with some EHCI controllers
	if (dma_supported (&udev->dev, DMA_BIT_MASK(64)))
		net->features |= NETIF_F_HIGHDMA;
#endif
	net->netdev_ops = &usbnet_netdev_ops; //TODO
	net->watchdog_timeo = TX_TIMEOUT_JIFFIES;
	net->ethtool_ops = &usbnet_ethtool_ops; //TODO
	// allow device-specific bind/init procedures
	// NOTE net->name still not usable ...

	if (info->bind){
		status = info->bind (dev, udev);
		if (status < 0)
			goto out1;
		// heuristic:  "usb%d" for links we know are two-host,
		// else "eth%d" when there's reasonable doubt.  userspace
		// can rename the link if it knows better.
		if ((dev->driver_info->flags & FLAG_ETHER) != 0 &&
		    ((dev->driver_info->flags & FLAG_POINTTOPOINT) == 0 ||
		     (net->dev_addr [0] & 0x02) == 0))
			strcpy (net->name, "eth%d");
		/* WLAN devices should always be named "wlan%d" */
		if ((dev->driver_info->flags & FLAG_WLAN) != 0)
			strcpy(net->name, "wlan%d");
		/* WWAN devices should always be named "wwan%d" */
		if ((dev->driver_info->flags & FLAG_WWAN) != 0)
			strcpy(net->name, "wwan%d");
		/* devices that cannot do ARP */
		if ((dev->driver_info->flags & FLAG_NOARP) != 0)
			net->flags |= IFF_NOARP;
		/* maybe the remote can't receive an Ethernet MTU */
		if (net->mtu > (dev->hard_mtu - net->hard_header_len))
			net->mtu = dev->hard_mtu - net->hard_header_len;	
	}else if (!info->in || !info->out)
		status = usbnet_get_endpoints (dev, udev); //TODO
	else {
		dev->in = usb_rcvbulkpipe (xdev, info->in);
		dev->out = usb_sndbulkpipe (xdev, info->out);
		if (!(info->flags & FLAG_NO_SETINT))
			status = usb_set_interface (xdev,
				interface->desc.bInterfaceNumber,
				interface->desc.bAlternateSetting);
		else
			status = 0;
	}
	//TODO:::
	if(status >= 0 && dev->status)
		status = init_status (dev,udev); //TODO: 
	if (status < 0)
		goto out3;
	if(!dev->rx_urb_size)
		dev->rx_urb_size = dev->hard_mtu;
	dev->maxpacket =  usb_maxpacket (dev->udev, dev->out, 1);
	if ((dev->driver_info->flags & FLAG_WLAN) != 0)
		SET_NETDEV_DEVTYPE(net, &wlan_type);
	if ((dev->driver_info->flags & FLAG_WWAN) != 0)
		SET_NETDEV_DEVTYPE(net, &wwan_type);

	status = register_netdev (net);
	if (status)
		goto out4;
	netif_info(dev, probe, dev->net,
		   "register '%s' at usb-%s-%s, %s, %pM\n",
		   udev->dev.driver->name,
		   xdev->bus->bus_name, xdev->devpath,
		   dev->driver_info->description,
		   net->dev_addr);
	/* ok, it's ready to go.*/
	usb_set_intfdata (udev, dev);
	
	netif_device_attach (net);

	if (dev->driver_info->flags & FLAG_LINK_INTR)
		usbnet_link_change(dev, 0, 0);
	return 0;
out4:
	usb_free_urb(dev->interrupt);
out3:
	if (info->unbind)
		info->unbind (dev, udev);
out1:
	free_netdev(net);
out:
	return status;

}

/**
 *	unregister_netdev - remove device from the kernel
 *	@dev: device
 *
 *	This function shuts down a device interface and removes it
 *	from the kernel tables.
 *
 *	This is just a wrapper for unregister_netdevice that takes
 *	the rtnl semaphore.  In general you want to use this and not
 *	unregister_netdevice.
 */
/**
 * cancel_work_sync - cancel a work and wait for it to finish
 * @work: the work to cancel
 *
 * Cancel @work and wait for its execution to finish.  This function
 * can be used even if the work re-queues itself or migrates to
 * another workqueue.  On return from this function, @work is
 * guaranteed to be not pending or executing on any CPU.
 *
 * cancel_work_sync(&delayed_work->work) must not be used for
 * delayed_work's.  Use cancel_delayed_work_sync() instead.
 *
 * The caller must ensure that the workqueue on which @work was last
 * queued can't be destroyed before this function returns.
 *
 * RETURNS:
 * %true if @work was pending, %false otherwise.
 */

/**
 * usb_scuttle_anchored_urbs - unanchor all an anchor's urbs
 * @anchor: the anchor whose urbs you want to unanchor
 *
 * use this to get rid of all an anchor's urbs
 */

/*-------------------------------------------------------------------------
 *
 * USB NET Disconnect : Only Invoked during Driver remove  
 *
 *-------------------------------------------------------------------------*/
void usbnet_disconnect (struct usb_interface *intf)
{
	struct usbnet		*dev;
	struct usb_device	*xdev;
	struct net_device	*net;
	dev = usb_get_intfdata(intf);
	usb_set_intfdata(intf, NULL);
	if (!dev)
		return;
	xdev = interface_to_usbdev (intf);
	netif_info(dev, probe, dev->net, "unregister '%s' usb-%s-%s, %s\n",
		   intf->dev.driver->name,
		   xdev->bus->bus_name, xdev->devpath,
		   dev->driver_info->description);
	net = dev->net;
	unregister_netdev (net);
	
	cancel_work_sync(&dev->kevent);
	
	usb_scuttle_anchored_urbs(&dev->deferred);
	if (dev->driver_info->unbind)
		dev->driver_info->unbind (dev, intf); //:TODO
	
	usb_kill_urb(dev->interrupt);
	usb_free_urb(dev->interrupt);

	free_netdev(net);
}

/*-------------------------------------------------------------------------*/

/*
 *  suspend the whole driver as soon as the first interface is suspended
 *  resume only when the last interface is resumed
 */
 /*  raw_spin_lock_irq() and raw_spin_lock_irqsave() disable local interrupts,
  *  however interrupts on other CPUs still may occur. raw_spin_lock() 
  *  does not.*/
int usbnet_suspend (struct usb_interface *intf, pm_message_t message)
{
	
	static usbnet		*dev = usb_get_intfdata(intf);
	
	if(!dev->suspended_count++) {
		spin_lock_irq(&dev->txq.lock);
		
		/* don't autosuspend while transmitting */
		if (dev->txq.qlen && PMSG_IS_AUTO(message)) {
			dev->suspend_count--;
			spin_unlock_irq(&dev->txq.lock);
			return -EBUSY;
		} else {
			set_bit(EVENT_DEV_ASLEEP, &dev->flags);
			spin_unlock_irq(&dev->txq.lock);
		}
		netif_device_detach (dev->net);
		usbnet_terminate_urbs(dev); //TODO
		__usbnet_status_stop_force(dev); //TODO

		/*
		 * reattach so runtime management can use and
		 * wake the device
		 */
		netif_device_attach (dev->net);
	}
	return 0;
}
/*
For reference, here is the full set of allocation flags, from the most restrictive to the least::

1.    GFP_ATOMIC: a high-priority allocation which will not sleep; this is the flag to use in interrupt handlers and other non-blocking situations.

2.    GFP_NOIO: blocking is possible, but no I/O will be performed.

3.    GFP_NOFS: no filesystem operations will be performed.

4.   GFP_KERNEL: a regular, blocking allocation.

5.    GFP_USER: a blocking allocation for user-space pages.

6.    GFP_HIGHUSER: for allocating user-space pages where high memory may be used. 

*/
/**
 * usb_autopm_put_interface_async - decrement a USB interface's PM-usage counter
 * @intf: the usb_interface whose counter should be decremented
 *
 * This routine does much the same thing as usb_autopm_put_interface():
 * It decrements @intf's usage counter and schedules a delayed
 * autosuspend request if the counter is <= 0.  The difference is that it
 * does not perform any synchronization; callers should hold a private
 * lock and handle all synchronization issues themselves.
 *
 * Typically a driver would call this routine during an URB's completion
 * handler, if no more URBs were pending.
 *
 * This routine can run in atomic context.
 */
/**
 *	__skb_queue_tail - queue a buffer at the list tail
 *	@list: list to use
 *	@newsk: buffer to queue
 *
 *	Queue a buffer at the end of a list. This function takes no locks
 *	and you must therefore hold required locks before calling it.
 *
 *	A buffer cannot be placed on two lists at the same time.
 */
/**
 * timer_pending - is a timer pending?
 * @timer: the timer in question
 *
 * timer_pending will tell whether a given timer is currently pending,
 * or not. Callers must ensure serialization wrt. other operations done
 * to this timer, eg. interrupt contexts, or other CPUs on SMP.
 *
 * return value: 1 if the timer is pending, 0 if not.
 */
/**
 * usb_autopm_get_interface_no_resume - increment a USB interface's PM-usage counter
 * @intf: the usb_interface whose counter should be incremented
 *
 * This routine increments @intf's usage counter but does not carry out an
 * autoresume.
 *
 * This routine can run in atomic context.
 */

int usbnet_resume (struct usb_interface *intf)
{
	struct usbnet		*dev = usb_get_intfdata(intf);
	struct sk_buff          *skb;
	struct urb              *res;
	int                     retval;
	
	if (!--dev->suspend_count) {
		/* resume interrupt URB if it was previously submitted */
		__usbnet_status_start_force(dev, GFP_NOIO); //TODO
		
		spin_lock_irq(&dev->txq.lock);
		while ((res = usb_get_from_anchor(&dev->deferred))){
			skb = (struct sk_buff *)res->context;
			retval = usb_submit_urb(res, GFP_ATOMIC);
			if (retval < 0) {
				dev_kfree_skb_any(skb);
				usb_free_urb(res);
				usb_autopm_put_interface_async(dev->intf);
			} else {
				dev->net->trans_start = jiffies;
				__skb_queue_tail(&dev->txq, skb);
			}	
		}
		smp_mb();
		clear_bit(EVENT_DEV_ASLEEP, &dev->flags);
		spin_unlock_irq(&dev->txq.lock);
	
		if (test_bit(EVENT_DEV_OPEN, &dev->flags)) {
			/* handle remote wakeup ASAP */
			if (!dev->wait &&
				netif_device_present(dev->net) &&
				!timer_pending(&dev->delay) &&
				!test_bit(EVENT_RX_HALT, &dev->flags))
					rx_alloc_submit(dev, GFP_NOIO); //TODO 

			if (!(dev->txq.qlen >= TX_QLEN(dev)))
				netif_tx_wake_all_queues(dev->net);
			tasklet_schedule (&dev->bh);
		}			
	}
	if (test_and_clear_bit(EVENT_DEVICE_REPORT_IDLE, &dev->flags))
				usb_autopm_get_interface_no_resume(intf);

	return 0;
}
