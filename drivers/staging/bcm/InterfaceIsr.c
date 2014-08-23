#include "headers.h"


static void read_int_callback(struct urb *urb/*, struct pt_regs *regs*/)
{
	int		status = urb->status;
	struct bcm_interface_adapter *intf_ad =
		(struct bcm_interface_adapter *)urb->context;
	struct bcm_mini_adapter *ad = intf_ad->psAdapter;

	if (netif_msg_intr(ad))
		pr_info(PFX "%s: interrupt status %d\n",
				ad->dev->name, status);

	if (ad->device_removed) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, INTF_INIT,
				DBG_LVL_ALL, "Device has Got Removed.");
		return;
	}

	if ((ad->bPreparingForLowPowerMode && ad->bDoSuspend) ||
			intf_ad->bSuspended ||
			intf_ad->bPreparingForBusSuspend) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, INTF_INIT,
				DBG_LVL_ALL,
				"Interrupt call back is called while suspending the device");
		return;
	}

	switch (status) {
	/* success */
	case STATUS_SUCCESS:
		if (urb->actual_length) {

			if (intf_ad->ulInterruptData[1] & 0xFF) {
				BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS,
						INTF_INIT, DBG_LVL_ALL,
						"Got USIM interrupt");
			}

			if (intf_ad->ulInterruptData[1] & 0xFF00) {
				atomic_set(&ad->CurrNumFreeTxDesc,
					(intf_ad->ulInterruptData[1] &
					 0xFF00) >> 8);
				atomic_set(&ad->uiMBupdate, TRUE);
				BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS,
					INTF_INIT, DBG_LVL_ALL,
					"TX mailbox contains %d",
					atomic_read(&ad->CurrNumFreeTxDesc));
			}
			if (intf_ad->ulInterruptData[1] >> 16) {
				ad->CurrNumRecvDescs =
					(intf_ad->ulInterruptData[1]  >> 16);
				BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS,
						INTF_INIT, DBG_LVL_ALL,
						"RX mailbox contains %d",
						ad->CurrNumRecvDescs);
				InterfaceRx(intf_ad);
			}
			if (ad->fw_download_done &&
				!ad->downloadDDR &&
				atomic_read(&ad->CurrNumFreeTxDesc)) {

				intf_ad->psAdapter->downloadDDR += 1;
				wake_up(&ad->tx_packet_wait_queue);
			}
			if (!ad->waiting_to_fw_download_done) {
				ad->waiting_to_fw_download_done = TRUE;
				wake_up(&ad->ioctl_fw_dnld_wait_queue);
			}
			if (!atomic_read(&ad->TxPktAvail)) {
				atomic_set(&ad->TxPktAvail, 1);
				wake_up(&ad->tx_packet_wait_queue);
			}
			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, INTF_INIT,
					DBG_LVL_ALL, "Firing interrupt in URB");
		}
		break;
	case -ENOENT:
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, INTF_INIT,
				DBG_LVL_ALL, "URB has got disconnected....");
		return;
	case -EINPROGRESS:
		/*
		 * This situation may happened when URBunlink is used.  for
		 * detail check usb_unlink_urb documentation.
		 */
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, INTF_INIT,
				DBG_LVL_ALL,
				"Impossibe condition has occurred... something very bad is going on");
		break;
		/* return; */
	case -EPIPE:
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, INTF_INIT,
				DBG_LVL_ALL,
				"Interrupt IN endPoint has got halted/stalled...need to clear this");
		ad->bEndPointHalted = TRUE;
		wake_up(&ad->tx_packet_wait_queue);
		urb->status = STATUS_SUCCESS;
		return;
	/* software-driven interface shutdown */
	case -ECONNRESET:	/* URB got unlinked */
	case -ESHUTDOWN:	/* hardware gone. this is the serious problem */
		/*
		 * Occurs only when something happens with the
		 * host controller device
		 */
	case -ENODEV: /* Device got removed */
	case -EINVAL:
		/*
		 * Some thing very bad happened with the URB. No
		 * description is available.
		 */
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, INTF_INIT,
				DBG_LVL_ALL, "interrupt urb error %d", status);
		urb->status = STATUS_SUCCESS;
		break;
		/* return; */
	default:
		/*
		 * This is required to check what is the defaults conditions
		 * when it occurs..
		 */
		BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, NEXT_SEND, DBG_LVL_ALL,
				"GOT DEFAULT INTERRUPT URB STATUS :%d..Please Analyze it...",
				status);
		break;
	}

	StartInterruptUrb(intf_ad);


}

int CreateInterruptUrb(struct bcm_interface_adapter *intf_ad)
{
	intf_ad->psInterruptUrb = usb_alloc_urb(0, GFP_KERNEL);
	if (!intf_ad->psInterruptUrb) {
		BCM_DEBUG_PRINT(intf_ad->psAdapter, DBG_TYPE_OTHERS,
				INTF_INIT, DBG_LVL_ALL,
				"Cannot allocate interrupt urb");
		return -ENOMEM;
	}
	intf_ad->psInterruptUrb->transfer_buffer =
		intf_ad->ulInterruptData;
	intf_ad->psInterruptUrb->transfer_buffer_length =
		sizeof(intf_ad->ulInterruptData);

	intf_ad->sIntrIn.int_in_pipe = usb_rcvintpipe(intf_ad->udev,
			intf_ad->sIntrIn.int_in_endpointAddr);

	usb_fill_int_urb(intf_ad->psInterruptUrb, intf_ad->udev,
			intf_ad->sIntrIn.int_in_pipe,
			intf_ad->psInterruptUrb->transfer_buffer,
			intf_ad->psInterruptUrb->transfer_buffer_length,
			read_int_callback, intf_ad,
			intf_ad->sIntrIn.int_in_interval);

	BCM_DEBUG_PRINT(intf_ad->psAdapter, DBG_TYPE_OTHERS, INTF_INIT,
			DBG_LVL_ALL, "Interrupt Interval: %d\n",
			intf_ad->sIntrIn.int_in_interval);
	return 0;
}


INT StartInterruptUrb(struct bcm_interface_adapter *intf_ad)
{
	INT status = 0;

	if (!(intf_ad->psAdapter->device_removed ||
				intf_ad->psAdapter->bEndPointHalted ||
				intf_ad->bSuspended ||
				intf_ad->bPreparingForBusSuspend ||
				intf_ad->psAdapter->StopAllXaction)) {
		status =
			usb_submit_urb(intf_ad->psInterruptUrb, GFP_ATOMIC);
		if (status) {
			BCM_DEBUG_PRINT(intf_ad->psAdapter,
					DBG_TYPE_OTHERS, INTF_INIT, DBG_LVL_ALL,
					"Cannot send inturb %d\n", status);
			if (status == -EPIPE) {
				intf_ad->psAdapter->bEndPointHalted =
					TRUE;
				wake_up(&intf_ad->psAdapter->tx_packet_wait_queue);
			}
		}
	}
	return status;
}

