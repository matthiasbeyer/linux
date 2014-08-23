#include "headers.h"
#include <linux/usb/ch9.h>
static struct usb_device_id interface_usb_table[] = {
	{ USB_DEVICE(BCM_USB_VENDOR_ID_T3, BCM_USB_PRODUCT_ID_T3) },
	{ USB_DEVICE(BCM_USB_VENDOR_ID_T3, BCM_USB_PRODUCT_ID_T3B) },
	{ USB_DEVICE(BCM_USB_VENDOR_ID_T3, BCM_USB_PRODUCT_ID_T3L) },
	{ USB_DEVICE(BCM_USB_VENDOR_ID_T3, BCM_USB_PRODUCT_ID_SYM) },
	{ USB_DEVICE(BCM_USB_VENDOR_ID_ZTE, BCM_USB_PRODUCT_ID_226) },
	{ USB_DEVICE(BCM_USB_VENDOR_ID_FOXCONN, BCM_USB_PRODUCT_ID_1901) },
	{ USB_DEVICE(BCM_USB_VENDOR_ID_ZTE, BCM_USB_PRODUCT_ID_ZTE_TU25) },
	{ USB_DEVICE(BCM_USB_VENDOR_ID_ZTE, BCM_USB_PRODUCT_ID_ZTE_226) },
	{ USB_DEVICE(BCM_USB_VENDOR_ID_ZTE, BCM_USB_PRODUCT_ID_ZTE_326) },
	{ }
};
MODULE_DEVICE_TABLE(usb, interface_usb_table);

static int debug = -1;
module_param(debug, uint, 0600);
MODULE_PARM_DESC(debug, "Debug level (0=none,...,16=all)");

static const u32 default_msg =
	NETIF_MSG_DRV | NETIF_MSG_PROBE | NETIF_MSG_LINK
	| NETIF_MSG_TIMER | NETIF_MSG_TX_ERR | NETIF_MSG_RX_ERR
	| NETIF_MSG_IFUP | NETIF_MSG_IFDOWN;

static int InterfaceAdapterInit(struct bcm_interface_adapter *ad);

static void InterfaceAdapterFree(struct bcm_interface_adapter *intf_ad)
{
	int i = 0;
	struct bcm_mini_adapter *ps_ad = intf_ad->psAdapter;

	/* Wake up the wait_queue... */
	if (ps_ad->LEDInfo.led_thread_running &
			BCM_LED_THREAD_RUNNING_ACTIVELY) {
		ps_ad->DriverState = DRIVER_HALT;
		wake_up(&ps_ad->LEDInfo.notify_led_event);
	}
	reset_card_proc(ps_ad);

	/*
	 * worst case time taken by the RDM/WRM will be 5 sec. will check after
	 * every 100 ms to accertain the device is not being accessed. After
	 * this No RDM/WRM should be made.
	 */
	while (ps_ad->DeviceAccess) {
		BCM_DEBUG_PRINT(ps_ad, DBG_TYPE_INITEXIT, DRV_ENTRY,
				DBG_LVL_ALL, "Device is being accessed.\n");
		msleep(100);
	}
	/* Free interrupt URB */
	/* ps_ad->device_removed = TRUE; */
	usb_free_urb(intf_ad->psInterruptUrb);

	/* Free transmit URBs */
	for (i = 0; i < MAXIMUM_USB_TCB; i++) {
		if (intf_ad->asUsbTcb[i].urb  != NULL) {
			usb_free_urb(intf_ad->asUsbTcb[i].urb);
			intf_ad->asUsbTcb[i].urb = NULL;
		}
	}
	/* Free receive URB and buffers */
	for (i = 0; i < MAXIMUM_USB_RCB; i++) {
		if (intf_ad->asUsbRcb[i].urb != NULL) {
			kfree(intf_ad->asUsbRcb[i].urb->transfer_buffer);
			usb_free_urb(intf_ad->asUsbRcb[i].urb);
			intf_ad->asUsbRcb[i].urb = NULL;
		}
	}
	AdapterFree(ps_ad);
}

static void ConfigureEndPointTypesThroughEEPROM(
		struct bcm_mini_adapter *ad)
{
	u32 reg;
	int bytes;
	struct bcm_interface_adapter *interf_ad;

	/* Program EP2 MAX_PKT_SIZE */
	reg = ntohl(EP2_MPS_REG);
	BeceemEEPROMBulkWrite(ad, (PUCHAR)&reg, 0x128, 4, TRUE);
	reg = ntohl(EP2_MPS);
	BeceemEEPROMBulkWrite(ad, (PUCHAR)&reg, 0x12C, 4, TRUE);

	reg = ntohl(EP2_CFG_REG);
	BeceemEEPROMBulkWrite(ad, (PUCHAR)&reg, 0x132, 4, TRUE);
	interf_ad =
		(struct bcm_interface_adapter *)(ad->pvInterfaceAdapter);
	if (interf_ad->bHighSpeedDevice) {
		reg = ntohl(EP2_CFG_INT);
		BeceemEEPROMBulkWrite(ad, (PUCHAR)&reg, 0x136, 4, TRUE);
	} else {
		/* USE BULK EP as TX in FS mode. */
		reg = ntohl(EP2_CFG_BULK);
		BeceemEEPROMBulkWrite(ad, (PUCHAR)&reg, 0x136, 4, TRUE);
	}

	/* Program EP4 MAX_PKT_SIZE. */
	reg = ntohl(EP4_MPS_REG);
	BeceemEEPROMBulkWrite(ad, (PUCHAR)&reg, 0x13C, 4, TRUE);
	reg = ntohl(EP4_MPS);
	BeceemEEPROMBulkWrite(ad, (PUCHAR)&reg, 0x140, 4, TRUE);

	/* Program TX EP as interrupt(Alternate Setting) */
	bytes = rdmalt(ad, 0x0F0110F8, &reg, sizeof(u32));
	if (bytes < 0) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_INITEXIT, DRV_ENTRY,
				DBG_LVL_ALL, "reading of Tx EP failed\n");
		return;
	}
	reg |= 0x6;

	reg = ntohl(reg);
	BeceemEEPROMBulkWrite(ad, (PUCHAR)&reg, 0x1CC, 4, TRUE);

	reg = ntohl(EP4_CFG_REG);
	BeceemEEPROMBulkWrite(ad, (PUCHAR)&reg, 0x1C8, 4, TRUE);
	/* Program ISOCHRONOUS EP size to zero. */
	reg = ntohl(ISO_MPS_REG);
	BeceemEEPROMBulkWrite(ad, (PUCHAR)&reg, 0x1D2, 4, TRUE);
	reg = ntohl(ISO_MPS);
	BeceemEEPROMBulkWrite(ad, (PUCHAR)&reg, 0x1D6, 4, TRUE);

	/*
	 * Update EEPROM Version.
	 * Read 4 bytes from 508 and modify 511 and 510.
	 */
	ReadBeceemEEPROM(ad, 0x1FC, &reg);
	reg &= 0x0101FFFF;
	BeceemEEPROMBulkWrite(ad, (PUCHAR)&reg, 0x1FC, 4, TRUE);

	/*
	 * Update length field if required.
	 * Also make the string NULL terminated.
	 */

	ReadBeceemEEPROM(ad, 0xA8, &reg);
	if ((reg&0x00FF0000)>>16 > 0x30) {
		reg = (reg&0xFF00FFFF)|(0x30<<16);
		BeceemEEPROMBulkWrite(ad, (PUCHAR)&reg, 0xA8, 4, TRUE);
	}
	ReadBeceemEEPROM(ad, 0x148, &reg);
	if ((reg&0x00FF0000)>>16 > 0x30) {
		reg = (reg&0xFF00FFFF)|(0x30<<16);
		BeceemEEPROMBulkWrite(ad, (PUCHAR)&reg, 0x148, 4, TRUE);
	}
	reg = 0;
	BeceemEEPROMBulkWrite(ad, (PUCHAR)&reg, 0x122, 4, TRUE);
	reg = 0;
	BeceemEEPROMBulkWrite(ad, (PUCHAR)&reg, 0x1C2, 4, TRUE);
}

static int usbbcm_device_probe(struct usb_interface *intf,
			       const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	int retval;
	struct bcm_mini_adapter *adapter;
	struct bcm_interface_adapter *intf_ad;
	struct net_device *ndev;

	/* Reserve one extra queue for the bit-bucket */
	ndev = alloc_etherdev_mq(sizeof(struct bcm_mini_adapter),
			NO_OF_QUEUES + 1);
	if (ndev == NULL) {
		dev_err(&udev->dev, DRV_NAME ": no memory for device\n");
		return -ENOMEM;
	}

	SET_NETDEV_DEV(ndev, &intf->dev);

	adapter = netdev_priv(ndev);
	adapter->dev = ndev;
	adapter->msg_enable = netif_msg_init(debug, default_msg);

	/* Init default driver debug state */

	adapter->stDebugState.debug_level = DBG_LVL_CURR;
	adapter->stDebugState.type = DBG_TYPE_INITEXIT;

	/*
	 * Technically, one can start using BCM_DEBUG_PRINT after this point.
	 * However, realize that by default the Type/Subtype bitmaps are all
	 * zero now; so no prints will actually appear until the TestApp turns
	 * on debug paths via the ioctl(); so practically speaking, in early
	 * init, no logging happens.
	 *
	 * A solution (used below): we explicitly set the bitmaps to 1 for
	 * Type=DBG_TYPE_INITEXIT and ALL subtype's of the same. Now all bcm
	 * debug statements get logged, enabling debug during early init.
	 * Further, we turn this OFF once init_module() completes.
	 */

	adapter->stDebugState.subtype[DBG_TYPE_INITEXIT] = 0xff;
	BCM_SHOW_DEBUG_BITMAP(adapter);

	retval = InitAdapter(adapter);
	if (retval) {
		dev_err(&udev->dev, DRV_NAME ": InitAdapter Failed\n");
		AdapterFree(adapter);
		return retval;
	}

	/* Allocate interface adapter structure */
	intf_ad = kzalloc(sizeof(struct bcm_interface_adapter),
			GFP_KERNEL);
	if (intf_ad == NULL) {
		AdapterFree(adapter);
		return -ENOMEM;
	}

	adapter->pvInterfaceAdapter = intf_ad;
	intf_ad->psAdapter = adapter;

	/* Store usb interface in Interface Adapter */
	intf_ad->interface = intf;
	usb_set_intfdata(intf, intf_ad);

	BCM_DEBUG_PRINT(adapter, DBG_TYPE_INITEXIT, DRV_ENTRY, DBG_LVL_ALL,
			"intf_ad 0x%p\n", intf_ad);
	retval = InterfaceAdapterInit(intf_ad);
	if (retval) {
		/* If the Firmware/Cfg File is not present
		 * then return success, let the application
		 * download the files.
		 */
		if (-ENOENT == retval) {
			BCM_DEBUG_PRINT(adapter, DBG_TYPE_INITEXIT, DRV_ENTRY,
					DBG_LVL_ALL,
					"File Not Found.  Use app to download.\n");
			return STATUS_SUCCESS;
		}
		BCM_DEBUG_PRINT(adapter, DBG_TYPE_INITEXIT, DRV_ENTRY,
				DBG_LVL_ALL, "InterfaceAdapterInit failed.\n");
		usb_set_intfdata(intf, NULL);
		udev = interface_to_usbdev(intf);
		usb_put_dev(udev);
		InterfaceAdapterFree(intf_ad);
		return retval;
	}
	if (adapter->chip_id > T3) {
		uint32_t uiNackZeroLengthInt = 4;

		retval =
			wrmalt(adapter, DISABLE_USB_ZERO_LEN_INT,
					&uiNackZeroLengthInt,
					sizeof(uiNackZeroLengthInt));
		if (retval)
			return retval;
	}

	/* Check whether the USB-Device Supports remote Wake-Up */
	if (USB_CONFIG_ATT_WAKEUP & udev->actconfig->desc.bmAttributes) {
		/* If Suspend then only support dynamic suspend */
		if (adapter->bDoSuspend) {
#ifdef CONFIG_PM
			pm_runtime_set_autosuspend_delay(&udev->dev, 0);
			intf->needs_remote_wakeup = 1;
			usb_enable_autosuspend(udev);
			device_init_wakeup(&intf->dev, 1);
			INIT_WORK(&intf_ad->usbSuspendWork,
					putUsbSuspend);
			BCM_DEBUG_PRINT(adapter, DBG_TYPE_INITEXIT, DRV_ENTRY,
					DBG_LVL_ALL,
					"Enabling USB Auto-Suspend\n");
#endif
		} else {
			intf->needs_remote_wakeup = 0;
			usb_disable_autosuspend(udev);
		}
	}

	adapter->stDebugState.subtype[DBG_TYPE_INITEXIT] = 0x0;
	return retval;
}

static void usbbcm_disconnect(struct usb_interface *intf)
{
	struct bcm_interface_adapter *intf_ad = usb_get_intfdata(intf);
	struct bcm_mini_adapter *adapter;
	struct usb_device  *udev = interface_to_usbdev(intf);

	if (intf_ad == NULL)
		return;

	adapter = intf_ad->psAdapter;
	netif_device_detach(adapter->dev);

	if (adapter->bDoSuspend)
		intf->needs_remote_wakeup = 0;

	adapter->device_removed = TRUE;
	usb_set_intfdata(intf, NULL);
	InterfaceAdapterFree(intf_ad);
	usb_put_dev(udev);
}

static int AllocUsbCb(struct bcm_interface_adapter *intf_ad)
{
	int i = 0;

	for (i = 0; i < MAXIMUM_USB_TCB; i++) {
		intf_ad->asUsbTcb[i].urb = usb_alloc_urb(0, GFP_KERNEL);

		if (intf_ad->asUsbTcb[i].urb == NULL) {
			BCM_DEBUG_PRINT(intf_ad->psAdapter,
					DBG_TYPE_PRINTK, 0, 0,
					"Can't allocate Tx urb for index %d\n",
					i);
			return -ENOMEM;
		}
	}

	for (i = 0; i < MAXIMUM_USB_RCB; i++) {
		intf_ad->asUsbRcb[i].urb = usb_alloc_urb(0, GFP_KERNEL);

		if (intf_ad->asUsbRcb[i].urb == NULL) {
			BCM_DEBUG_PRINT(intf_ad->psAdapter,
					DBG_TYPE_PRINTK, 0, 0,
					"Can't allocate Rx urb for index %d\n",
					i);
			return -ENOMEM;
		}

		intf_ad->asUsbRcb[i].urb->transfer_buffer =
			kmalloc(MAX_DATA_BUFFER_SIZE, GFP_KERNEL);

		if (intf_ad->asUsbRcb[i].urb->transfer_buffer == NULL) {
			BCM_DEBUG_PRINT(intf_ad->psAdapter,
					DBG_TYPE_PRINTK, 0, 0,
					"Can't allocate Rx buffer for index %d\n",
					i);
			return -ENOMEM;
		}
		intf_ad->asUsbRcb[i].urb->transfer_buffer_length =
			MAX_DATA_BUFFER_SIZE;
	}
	return 0;
}

static int device_run(struct bcm_interface_adapter *intf_ad)
{
	int value = 0;
	UINT status = STATUS_SUCCESS;
	struct bcm_mini_adapter *psAd = intf_ad->psAdapter;

	status = InitCardAndDownloadFirmware(psAd);
	if (status != STATUS_SUCCESS) {
		pr_err(DRV_NAME "InitCardAndDownloadFirmware failed.\n");
		return status;
	}
	if (psAd->fw_download_done) {
		if (StartInterruptUrb(intf_ad)) {
			BCM_DEBUG_PRINT(psAd, DBG_TYPE_INITEXIT, DRV_ENTRY,
					DBG_LVL_ALL,
					"Cannot send interrupt in URB\n");
		}

		/*
		 * now register the cntrl interface.  after downloading the f/w
		 * waiting for 5 sec to get the mailbox interrupt.
		 */
		psAd->waiting_to_fw_download_done = false;
		value = wait_event_timeout(psAd->ioctl_fw_dnld_wait_queue,
					   psAd->waiting_to_fw_download_done,
					   5 * HZ);

		if (value == 0)
			pr_err(DRV_NAME ": Timeout waiting for mailbox interrupt.\n");

		if (register_control_device_interface(psAd) < 0) {
			pr_err(DRV_NAME ": Register Control Device failed.\n");
			return -EIO;
		}
	}
	return 0;
}

static int select_alternate_setting_for_highspeed_modem(
		struct bcm_interface_adapter *intf_ad,
		struct usb_endpoint_descriptor **endpoint,
		const struct usb_host_interface *iface_desc,
		int *usedIntOutForBulkTransfer)
{
	int retval = 0;
	struct bcm_mini_adapter *psAd = intf_ad->psAdapter;

	/* selecting alternate setting one as a default setting
	 * for High Speed  modem. */
	if (intf_ad->bHighSpeedDevice)
		retval = usb_set_interface(intf_ad->udev,
					   DEFAULT_SETTING_0,
					   ALTERNATE_SETTING_1);
	BCM_DEBUG_PRINT(psAd, DBG_TYPE_INITEXIT, DRV_ENTRY, DBG_LVL_ALL,
			"BCM16 is applicable on this dongle\n");
	if (retval || !intf_ad->bHighSpeedDevice) {
		*usedIntOutForBulkTransfer = EP2;
		*endpoint = &iface_desc->endpoint[EP2].desc;
		BCM_DEBUG_PRINT(psAd, DBG_TYPE_INITEXIT, DRV_ENTRY, DBG_LVL_ALL,
				"Interface altsetting failed or modem is configured to Full Speed, hence will work on default setting 0\n");
		/*
		 * If Modem is high speed device EP2 should be
		 * INT OUT End point
		 *
		 * If Mode is FS then EP2 should be bulk end
		 * point
		 */
		if ((intf_ad->bHighSpeedDevice &&
					!usb_endpoint_is_int_out(*endpoint)) ||
				(!intf_ad->bHighSpeedDevice &&
				 !usb_endpoint_is_bulk_out(*endpoint))) {
			BCM_DEBUG_PRINT(psAd, DBG_TYPE_INITEXIT, DRV_ENTRY,
					DBG_LVL_ALL,
					"Configuring the EEPROM\n");
			/* change the EP2, EP4 to INT OUT end point */
			ConfigureEndPointTypesThroughEEPROM(
					psAd);

			/*
			 * It resets the device and if any thing
			 * gets changed in USB descriptor it
			 * will show fail and re-enumerate the
			 * device
			 */
			retval = usb_reset_device(intf_ad->udev);
			if (retval) {
				BCM_DEBUG_PRINT(psAd, DBG_TYPE_INITEXIT,
						DRV_ENTRY, DBG_LVL_ALL,
						"reset failed.  Re-enumerating the device.\n");
				return retval;
			}

		}
		if (!intf_ad->bHighSpeedDevice &&
		    usb_endpoint_is_bulk_out(*endpoint)) {
			/*
			 * Once BULK is selected in FS mode.
			 * Revert it back to INT.
			 * Else USB_IF will fail.
			 */
			UINT _uiData = ntohl(EP2_CFG_INT);

			BCM_DEBUG_PRINT(psAd, DBG_TYPE_INITEXIT, DRV_ENTRY,
					DBG_LVL_ALL,
					"Reverting Bulk to INT as it is in Full Speed mode.\n");
			BeceemEEPROMBulkWrite(psAd, (PUCHAR) & _uiData, 0x136,
					      4, TRUE);
		}
	} else {
		*usedIntOutForBulkTransfer = EP4;
		*endpoint = &iface_desc->endpoint[EP4].desc;
		BCM_DEBUG_PRINT(psAd, DBG_TYPE_INITEXIT, DRV_ENTRY, DBG_LVL_ALL,
				"Choosing AltSetting as a default setting.\n");
		if (!usb_endpoint_is_int_out(*endpoint)) {
			BCM_DEBUG_PRINT(psAd, DBG_TYPE_INITEXIT, DRV_ENTRY,
					DBG_LVL_ALL,
					"Dongle does not have BCM16 Fix.\n");
			/*
			 * change the EP2, EP4 to INT OUT end point and use EP4
			 * in altsetting
			 */
			ConfigureEndPointTypesThroughEEPROM(psAd);

			/*
			 * It resets the device and if any thing
			 * gets changed in USB descriptor it
			 * will show fail and re-enumerate the
			 * device
			 */
			retval = usb_reset_device(intf_ad->udev);
			if (retval) {
				BCM_DEBUG_PRINT(psAd, DBG_TYPE_INITEXIT,
						DRV_ENTRY, DBG_LVL_ALL,
						"reset failed.  Re-enumerating the device.\n");
				return retval;
			}
		}
	}

	return 0;
}

static int InterfaceAdapterInit(struct bcm_interface_adapter *intf_ad)
{
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	size_t buffer_size;
	unsigned long value;
	int retval = 0;
	int usedIntOutForBulkTransfer = 0;
	bool bBcm16 = false;
	UINT uiData = 0;
	int bytes;
	struct bcm_mini_adapter *psAd = intf_ad->psAdapter;

	/* Store the usb dev into interface adapter */
	intf_ad->udev =
		usb_get_dev(interface_to_usbdev(intf_ad->interface));

	intf_ad->bHighSpeedDevice =
		(intf_ad->udev->speed == USB_SPEED_HIGH);
	psAd->interface_rdm = BcmRDM;
	psAd->interface_wrm = BcmWRM;

	bytes = rdmalt(psAd, CHIP_ID_REG, (u32 *) &(psAd->chip_id),
		       sizeof(u32));
	if (bytes < 0) {
		retval = bytes;
		BCM_DEBUG_PRINT(psAd, DBG_TYPE_PRINTK, 0, 0,
				"CHIP ID Read Failed\n");
		return retval;
	}

	if (0xbece3200 == (psAd->chip_id & ~(0xF0)))
		psAd->chip_id &= ~0xF0;

	dev_info(&intf_ad->udev->dev, "RDM Chip ID 0x%lx\n",
		 psAd->chip_id);

	iface_desc = intf_ad->interface->cur_altsetting;

	if (psAd->chip_id == T3B) {
		/* T3B device will have EEPROM, check if EEPROM is proper and
		 * BCM16 can be done or not. */
		BeceemEEPROMBulkRead(psAd, &uiData, 0x0, 4);
		if (uiData == BECM)
			bBcm16 = TRUE;

		dev_info(&intf_ad->udev->dev,
			 "number of alternate setting %d\n",
			 intf_ad->interface->num_altsetting);

		if (bBcm16 == TRUE) {
			retval = select_alternate_setting_for_highspeed_modem(
					intf_ad, &endpoint, iface_desc,
					&usedIntOutForBulkTransfer);
			if (retval)
				return retval;
		}
	}

	iface_desc = intf_ad->interface->cur_altsetting;

	for (value = 0; value < iface_desc->desc.bNumEndpoints; ++value) {
		endpoint = &iface_desc->endpoint[value].desc;

		if (!intf_ad->sBulkIn.bulk_in_endpointAddr &&
				usb_endpoint_is_bulk_in(endpoint)) {
			buffer_size = le16_to_cpu(endpoint->wMaxPacketSize);
			intf_ad->sBulkIn.bulk_in_size = buffer_size;
			intf_ad->sBulkIn.bulk_in_endpointAddr =
				endpoint->bEndpointAddress;
			intf_ad->sBulkIn.bulk_in_pipe = usb_rcvbulkpipe(
					intf_ad->udev,
					intf_ad->sBulkIn.bulk_in_endpointAddr);
		}

		if (!intf_ad->sBulkOut.bulk_out_endpointAddr &&
				usb_endpoint_is_bulk_out(endpoint)) {
			intf_ad->sBulkOut.bulk_out_endpointAddr =
				endpoint->bEndpointAddress;
			intf_ad->sBulkOut.bulk_out_pipe = usb_sndbulkpipe(
					intf_ad->udev,
					intf_ad->sBulkOut.bulk_out_endpointAddr);
		}

		if (!intf_ad->sIntrIn.int_in_endpointAddr &&
				usb_endpoint_is_int_in(endpoint)) {
			buffer_size = le16_to_cpu(endpoint->wMaxPacketSize);
			intf_ad->sIntrIn.int_in_size = buffer_size;
			intf_ad->sIntrIn.int_in_endpointAddr =
				endpoint->bEndpointAddress;
			intf_ad->sIntrIn.int_in_interval =
				endpoint->bInterval;
			intf_ad->sIntrIn.int_in_buffer =
				kmalloc(buffer_size, GFP_KERNEL);
			if (!intf_ad->sIntrIn.int_in_buffer)
				return -EINVAL;
		}

		if (!intf_ad->sIntrOut.int_out_endpointAddr &&
				usb_endpoint_is_int_out(endpoint)) {
			if (!intf_ad->sBulkOut.bulk_out_endpointAddr &&
					(psAd->chip_id == T3B) &&
					(value == usedIntOutForBulkTransfer)) {
				/*
				 * use first intout end point as a bulk out end
				 * point
				 */
				buffer_size =
					le16_to_cpu(endpoint->wMaxPacketSize);
				intf_ad->sBulkOut.bulk_out_size =
					buffer_size;
				intf_ad->sBulkOut.bulk_out_endpointAddr =
					endpoint->bEndpointAddress;
				intf_ad->sBulkOut.bulk_out_pipe =
					usb_sndintpipe(intf_ad->udev,
							intf_ad->sBulkOut
							.bulk_out_endpointAddr);
				intf_ad->sBulkOut.int_out_interval =
					endpoint->bInterval;
			} else if (value == EP6) {
				buffer_size =
					le16_to_cpu(endpoint->wMaxPacketSize);
				intf_ad->sIntrOut.int_out_size =
					buffer_size;
				intf_ad->sIntrOut.int_out_endpointAddr =
					endpoint->bEndpointAddress;
				intf_ad->sIntrOut.int_out_interval =
					endpoint->bInterval;
				intf_ad->sIntrOut.int_out_buffer =
					kmalloc(buffer_size, GFP_KERNEL);
				if (!intf_ad->sIntrOut.int_out_buffer)
					return -EINVAL;
			}
		}
	}

	usb_set_intfdata(intf_ad->interface, intf_ad);

	psAd->bcm_file_download = InterfaceFileDownload;
	psAd->bcm_file_readback_from_chip = InterfaceFileReadbackFromChip;
	psAd->interface_transmit = InterfaceTransmitPacket;

	retval = CreateInterruptUrb(intf_ad);

	if (retval) {
		BCM_DEBUG_PRINT(psAd, DBG_TYPE_PRINTK, 0, 0,
				"Cannot create interrupt urb\n");
		return retval;
	}

	retval = AllocUsbCb(intf_ad);
	if (retval)
		return retval;

	return device_run(intf_ad);
}

static int InterfaceSuspend(struct usb_interface *intf, pm_message_t message)
{
	struct bcm_interface_adapter *intf_ad = usb_get_intfdata(intf);

	intf_ad->bSuspended = TRUE;

	if (intf_ad->bPreparingForBusSuspend) {
		intf_ad->bPreparingForBusSuspend = false;

		if (intf_ad->psAdapter->LinkStatus == LINKUP_DONE) {
			intf_ad->psAdapter->IdleMode = TRUE;
			BCM_DEBUG_PRINT(intf_ad->psAdapter,
					DBG_TYPE_INITEXIT, DRV_ENTRY,
					DBG_LVL_ALL,
					"Host Entered in PMU Idle Mode.\n");
		} else {
			intf_ad->psAdapter->bShutStatus = TRUE;
			BCM_DEBUG_PRINT(intf_ad->psAdapter,
					DBG_TYPE_INITEXIT, DRV_ENTRY,
					DBG_LVL_ALL,
					"Host Entered in PMU Shutdown Mode.\n");
		}
	}
	intf_ad->psAdapter->bPreparingForLowPowerMode = false;

	/* Signaling the control pkt path */
	wake_up(&intf_ad->psAdapter->lowpower_mode_wait_queue);

	return 0;
}

static int InterfaceResume(struct usb_interface *intf)
{
	struct bcm_interface_adapter *intf_ad = usb_get_intfdata(intf);

	mdelay(100);
	intf_ad->bSuspended = false;

	StartInterruptUrb(intf_ad);
	InterfaceRx(intf_ad);
	return 0;
}

static struct usb_driver usbbcm_driver = {
	.name = "usbbcm",
	.probe = usbbcm_device_probe,
	.disconnect = usbbcm_disconnect,
	.suspend = InterfaceSuspend,
	.resume = InterfaceResume,
	.id_table = interface_usb_table,
	.supports_autosuspend = 1,
};

struct class *bcm_class;

static __init int bcm_init(void)
{
	int retval;

	pr_info("%s: %s, %s\n", DRV_NAME, DRV_DESCRIPTION, DRV_VERSION);
	pr_info("%s\n", DRV_COPYRIGHT);

	bcm_class = class_create(THIS_MODULE, DRV_NAME);
	if (IS_ERR(bcm_class)) {
		pr_err(DRV_NAME ": could not create class\n");
		return PTR_ERR(bcm_class);
	}

	retval = usb_register(&usbbcm_driver);
	if (retval < 0) {
		pr_err(DRV_NAME ": could not register usb driver\n");
		class_destroy(bcm_class);
		return retval;
	}
	return 0;
}

static __exit void bcm_exit(void)
{
	usb_deregister(&usbbcm_driver);
	class_destroy(bcm_class);
}

module_init(bcm_init);
module_exit(bcm_exit);

MODULE_DESCRIPTION(DRV_DESCRIPTION);
MODULE_VERSION(DRV_VERSION);
MODULE_LICENSE("GPL");
