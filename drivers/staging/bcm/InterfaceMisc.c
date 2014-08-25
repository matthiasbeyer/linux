#include "headers.h"

static int adapter_err_occurred(const struct bcm_interface_adapter *ad)
{
	if (ad->psAdapter->device_removed == TRUE) {
		BCM_DEBUG_PRINT(ad->psAdapter, DBG_TYPE_PRINTK, 0, 0,
				"Device got removed");
		return -ENODEV;
	}

	if ((ad->psAdapter->StopAllXaction == TRUE) &&
	    (ad->psAdapter->chip_id >= T3LPB)) {
		BCM_DEBUG_PRINT(ad->psAdapter, DBG_TYPE_OTHERS, RDM,
				DBG_LVL_ALL,
				"Currently Xaction is not allowed on the bus");
		return -EACCES;
	}

	if (ad->bSuspended == TRUE || ad->bPreparingForBusSuspend == TRUE) {
		BCM_DEBUG_PRINT(ad->psAdapter, DBG_TYPE_OTHERS, RDM,
				DBG_LVL_ALL,
				"Bus is in suspended states hence RDM not allowed..");
		return -EACCES;
	}

	return 0;
}

int InterfaceRDM(struct bcm_interface_adapter *intf_ad,
		unsigned int addr,
		void *buff,
		int len)
{
	int bytes;
	int err = 0;

	if (!intf_ad)
		return -EINVAL;

	err = adapter_err_occurred(intf_ad);
	if (err)
		return err;

	intf_ad->psAdapter->DeviceAccess = TRUE;

	bytes = usb_control_msg(intf_ad->udev,
				usb_rcvctrlpipe(intf_ad->udev, 0),
				0x02,
				0xC2,
				(addr & 0xFFFF),
				((addr >> 16) & 0xFFFF),
				buff,
				len,
				5000);

	if (-ENODEV == bytes)
		intf_ad->psAdapter->device_removed = TRUE;

	if (bytes < 0)
		BCM_DEBUG_PRINT(intf_ad->psAdapter, DBG_TYPE_OTHERS, RDM,
				DBG_LVL_ALL, "RDM failed status :%d", bytes);
	else
		BCM_DEBUG_PRINT(intf_ad->psAdapter, DBG_TYPE_OTHERS, RDM,
				DBG_LVL_ALL, "RDM sent %d", bytes);

	intf_ad->psAdapter->DeviceAccess = false;
	return bytes;
}

int InterfaceWRM(struct bcm_interface_adapter *intf_ad,
		unsigned int addr,
		void *buff,
		int len)
{
	int retval = 0;
	int err = 0;

	if (!intf_ad)
		return -EINVAL;

	err = adapter_err_occurred(intf_ad);
	if (err)
		return err;

	intf_ad->psAdapter->DeviceAccess = TRUE;

	retval = usb_control_msg(intf_ad->udev,
				usb_sndctrlpipe(intf_ad->udev, 0),
				0x01,
				0x42,
				(addr & 0xFFFF),
				((addr >> 16) & 0xFFFF),
				buff,
				len,
				5000);

	if (-ENODEV == retval)
		intf_ad->psAdapter->device_removed = TRUE;

	if (retval < 0)	{
		BCM_DEBUG_PRINT(intf_ad->psAdapter, DBG_TYPE_OTHERS, WRM,
				DBG_LVL_ALL, "WRM failed status :%d", retval);
		intf_ad->psAdapter->DeviceAccess = false;
		return retval;
	} else {
		intf_ad->psAdapter->DeviceAccess = false;
		BCM_DEBUG_PRINT(intf_ad->psAdapter, DBG_TYPE_OTHERS, WRM,
				DBG_LVL_ALL, "WRM sent %d", retval);
		return STATUS_SUCCESS;
	}
}

int BcmRDM(void *arg,
	unsigned int addr,
	void *buff,
	int len)
{
	return InterfaceRDM((struct bcm_interface_adapter *)arg, addr, buff,
			    len);
}

int BcmWRM(void *arg,
	unsigned int addr,
	void *buff,
	int len)
{
	return InterfaceWRM((struct bcm_interface_adapter *)arg, addr, buff,
			    len);
}

int Bcm_clear_halt_of_endpoints(struct bcm_mini_adapter *ad)
{
	struct bcm_interface_adapter *intf_ad =
		(struct bcm_interface_adapter *)(ad->pvInterfaceAdapter);
	int status = STATUS_SUCCESS;

	/*
	 * usb_clear_halt - tells device to clear endpoint halt/stall condition
	 * @dev: device whose endpoint is halted
	 * @pipe: endpoint "pipe" being cleared
	 * @ Context: !in_interrupt ()
	 *
	 * usb_clear_halt is the synchrnous call and returns 0 on success else
	 * returns with error code.
	 * This is used to clear halt conditions for bulk and interrupt
	 * endpoints only.
	 * Control and isochronous endpoints never halts.
	 *
	 * Any URBs  queued for such an endpoint should normally be unlinked by
	 * the driver before clearing the halt condition.
	 *
	 */

	/* Killing all the submitted urbs to different end points. */
	Bcm_kill_all_URBs(intf_ad);

	/* clear the halted/stalled state for every end point */
	status = usb_clear_halt(intf_ad->udev,
				intf_ad->sIntrIn.int_in_pipe);
	if (status != STATUS_SUCCESS)
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, INTF_INIT,
				DBG_LVL_ALL,
				"Unable to Clear Halt of Interrupt IN end point. :%d ",
				status);

	status = usb_clear_halt(intf_ad->udev,
				intf_ad->sBulkIn.bulk_in_pipe);
	if (status != STATUS_SUCCESS)
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, INTF_INIT,
				DBG_LVL_ALL,
				"Unable to Clear Halt of Bulk IN end point. :%d ",
				status);

	status = usb_clear_halt(intf_ad->udev,
				intf_ad->sBulkOut.bulk_out_pipe);
	if (status != STATUS_SUCCESS)
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, INTF_INIT,
				DBG_LVL_ALL,
				"Unable to Clear Halt of Bulk OUT end point. :%d ",
				status);

	return status;
}

void Bcm_kill_all_URBs(struct bcm_interface_adapter *intf_ad)
{
	struct urb *tempUrb = NULL;
	unsigned int i;

	/*
	 * usb_kill_urb - cancel a transfer request and wait for it to finish
	 * @urb: pointer to URB describing a previously submitted request,
	 * returns nothing as it is void returned API.
	 *
	 * This routine cancels an in-progress request. It is guaranteed that
	 * upon return all completion handlers will have finished and the URB
	 * will be totally idle and available for reuse
	 *
	 * This routine may not be used in an interrupt context (such as a
	 * bottom half or a completion handler), or when holding a spinlock, or
	 * in other situations where the caller can't schedule().
	 *
	 */

	/* Cancel submitted Interrupt-URB's */
	if (intf_ad->psInterruptUrb) {
		if (intf_ad->psInterruptUrb->status == -EINPROGRESS)
			usb_kill_urb(intf_ad->psInterruptUrb);
	}

	/* Cancel All submitted TX URB's */
	for (i = 0; i < MAXIMUM_USB_TCB; i++) {
		tempUrb = intf_ad->asUsbTcb[i].urb;
		if (tempUrb) {
			if (tempUrb->status == -EINPROGRESS)
				usb_kill_urb(tempUrb);
		}
	}

	for (i = 0; i < MAXIMUM_USB_RCB; i++) {
		tempUrb = intf_ad->asUsbRcb[i].urb;
		if (tempUrb) {
			if (tempUrb->status == -EINPROGRESS)
				usb_kill_urb(tempUrb);
		}
	}

	atomic_set(&intf_ad->uNumTcbUsed, 0);
	atomic_set(&intf_ad->uCurrTcb, 0);

	atomic_set(&intf_ad->uNumRcbUsed, 0);
	atomic_set(&intf_ad->uCurrRcb, 0);
}

void putUsbSuspend(struct work_struct *work)
{
	struct bcm_interface_adapter *intf_ad = NULL;
	struct usb_interface *intf = NULL;

	intf_ad = container_of(work, struct bcm_interface_adapter,
				     usbSuspendWork);
	intf = intf_ad->interface;

	if (intf_ad->bSuspended == false)
		usb_autopm_put_interface(intf);
}

