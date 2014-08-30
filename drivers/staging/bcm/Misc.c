#include "headers.h"

static int BcmFileDownload(struct bcm_mini_adapter *ad, const char *path, unsigned int loc);
static void doPowerAutoCorrection(struct bcm_mini_adapter *ps_ad);
static void HandleShutDownModeRequest(struct bcm_mini_adapter *ad, PUCHAR buffer);
static int bcm_parse_target_params(struct bcm_mini_adapter *ad);
static void beceem_protocol_reset(struct bcm_mini_adapter *ad);

static void default_wimax_protocol_initialize(struct bcm_mini_adapter *ad)
{
	unsigned int i;

	for (i = 0; i < NO_OF_QUEUES-1; i++) {
		ad->PackInfo[i].uiThreshold = TX_PACKET_THRESHOLD;
		ad->PackInfo[i].uiMaxAllowedRate = MAX_ALLOWED_RATE;
		ad->PackInfo[i].uiMaxBucketSize = 20*1024*1024;
	}

	ad->BEBucketSize = BE_BUCKET_SIZE;
	ad->rtPSBucketSize = rtPS_BUCKET_SIZE;
	ad->LinkStatus = SYNC_UP_REQUEST;
	ad->TransferMode = IP_PACKET_ONLY_MODE;
	ad->usBestEffortQueueIndex = -1;
}

int InitAdapter(struct bcm_mini_adapter *ps_ad)
{
	int i = 0;
	int status = STATUS_SUCCESS;

	BCM_DEBUG_PRINT(ps_ad, DBG_TYPE_INITEXIT, MP_INIT, DBG_LVL_ALL, "Initialising Adapter = %p", ps_ad);

	if (ps_ad == NULL) {
		BCM_DEBUG_PRINT(ps_ad, DBG_TYPE_INITEXIT, MP_INIT, DBG_LVL_ALL, "Adapter is NULL");
		return -EINVAL;
	}

	sema_init(&ps_ad->NVMRdmWrmLock, 1);
	sema_init(&ps_ad->rdmwrmsync, 1);
	spin_lock_init(&ps_ad->control_queue_lock);
	spin_lock_init(&ps_ad->txtransmitlock);
	sema_init(&ps_ad->RxAppControlQueuelock, 1);
	sema_init(&ps_ad->fw_download_sema, 1);
	sema_init(&ps_ad->LowPowerModeSync, 1);

	for (i = 0; i < NO_OF_QUEUES; i++)
		spin_lock_init(&ps_ad->PackInfo[i].SFQueueLock);
	i = 0;

	init_waitqueue_head(&ps_ad->process_rx_cntrlpkt);
	init_waitqueue_head(&ps_ad->tx_packet_wait_queue);
	init_waitqueue_head(&ps_ad->process_read_wait_queue);
	init_waitqueue_head(&ps_ad->ioctl_fw_dnld_wait_queue);
	init_waitqueue_head(&ps_ad->lowpower_mode_wait_queue);
	ps_ad->waiting_to_fw_download_done = TRUE;
	ps_ad->fw_download_done = false;

	default_wimax_protocol_initialize(ps_ad);
	for (i = 0; i < MAX_CNTRL_PKTS; i++) {
		ps_ad->txctlpacket[i] = kmalloc(MAX_CNTL_PKT_SIZE, GFP_KERNEL);
		if (!ps_ad->txctlpacket[i]) {
			BCM_DEBUG_PRINT(ps_ad, DBG_TYPE_INITEXIT, MP_INIT, DBG_LVL_ALL, "No More Cntl pkts got, max got is %d", i);
			return -ENOMEM;
		}
	}

	if (AllocAdapterDsxBuffer(ps_ad)) {
		BCM_DEBUG_PRINT(ps_ad, DBG_TYPE_INITEXIT, MP_INIT, DBG_LVL_ALL, "Failed to allocate DSX buffers");
		return -EINVAL;
	}

	/* Initialize PHS interface */
	if (phs_init(&ps_ad->stBCMPhsContext, ps_ad) != 0) {
		BCM_DEBUG_PRINT(ps_ad, DBG_TYPE_INITEXIT, MP_INIT, DBG_LVL_ALL, "%s:%s:%d:Error PHS Init Failed=====>\n", __FILE__, __func__, __LINE__);
		return -ENOMEM;
	}

	status = BcmAllocFlashCSStructure(ps_ad);
	if (status) {
		BCM_DEBUG_PRINT(ps_ad, DBG_TYPE_INITEXIT, MP_INIT, DBG_LVL_ALL, "Memory Allocation for Flash structure failed");
		return status;
	}

	status = vendorextnInit(ps_ad);

	if (STATUS_SUCCESS != status) {
		BCM_DEBUG_PRINT(ps_ad, DBG_TYPE_INITEXIT, MP_INIT, DBG_LVL_ALL, "Vendor Init Failed");
		return status;
	}

	BCM_DEBUG_PRINT(ps_ad, DBG_TYPE_INITEXIT, MP_INIT, DBG_LVL_ALL, "Adapter initialised");

	return STATUS_SUCCESS;
}

void AdapterFree(struct bcm_mini_adapter *ad)
{
	int count;

	beceem_protocol_reset(ad);
	vendorextnExit(ad);

	if (ad->control_packet_handler && !IS_ERR(ad->control_packet_handler))
		kthread_stop(ad->control_packet_handler);

	if (ad->transmit_packet_thread && !IS_ERR(ad->transmit_packet_thread))
		kthread_stop(ad->transmit_packet_thread);

	wake_up(&ad->process_read_wait_queue);

	if (ad->LEDInfo.led_thread_running & (BCM_LED_THREAD_RUNNING_ACTIVELY | BCM_LED_THREAD_RUNNING_INACTIVELY))
		kthread_stop(ad->LEDInfo.led_cntrl_threadid);

	unregister_networkdev(ad);

	/* FIXME: use proper wait_event and refcounting */
	while (atomic_read(&ad->ApplicationRunning)) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_INITEXIT, MP_INIT, DBG_LVL_ALL, "Waiting for Application to close.. %d\n", atomic_read(&ad->ApplicationRunning));
		msleep(100);
	}
	unregister_control_device_interface(ad);
	kfree(ad->pstargetparams);

	for (count = 0; count < MAX_CNTRL_PKTS; count++)
		kfree(ad->txctlpacket[count]);

	FreeAdapterDsxBuffer(ad);
	kfree(ad->pvInterfaceAdapter);

	/* Free the PHS Interface */
	PhsCleanup(&ad->stBCMPhsContext);

	BcmDeAllocFlashCSStructure(ad);

	free_netdev(ad->dev);
}

static int create_worker_threads(struct bcm_mini_adapter *ps_ad)
{
	/* Rx Control Packets Processing */
	ps_ad->control_packet_handler = kthread_run((int (*)(void *))
							control_packet_handler, ps_ad, "%s-rx", DRV_NAME);
	if (IS_ERR(ps_ad->control_packet_handler)) {
		pr_notice(DRV_NAME ": could not create control thread\n");
		return PTR_ERR(ps_ad->control_packet_handler);
	}

	/* Tx Thread */
	ps_ad->transmit_packet_thread = kthread_run((int (*)(void *))
							tx_pkt_handler, ps_ad, "%s-tx", DRV_NAME);
	if (IS_ERR(ps_ad->transmit_packet_thread)) {
		pr_notice(DRV_NAME ": could not creat transmit thread\n");
		kthread_stop(ps_ad->control_packet_handler);
		return PTR_ERR(ps_ad->transmit_packet_thread);
	}
	return 0;
}

static struct file *open_firmware_file(struct bcm_mini_adapter *ad, const char *path)
{
	struct file *flp = filp_open(path, O_RDONLY, S_IRWXU);

	if (IS_ERR(flp)) {
		pr_err(DRV_NAME "Unable To Open File %s, err %ld", path, PTR_ERR(flp));
		flp = NULL;
	}

	if (ad->device_removed)
		flp = NULL;

	return flp;
}

/* Arguments:
 * Logical Adapter
 * Path to image file
 * Download Address on the chip
 */
static int BcmFileDownload(struct bcm_mini_adapter *ad, const char *path, unsigned int loc)
{
	int errorno = 0;
	struct file *flp = NULL;
	struct timeval tv = {0};

	flp = open_firmware_file(ad, path);
	if (!flp) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_INITEXIT, MP_INIT, DBG_LVL_ALL, "Unable to Open %s\n", path);
		return -ENOENT;
	}
	BCM_DEBUG_PRINT(ad, DBG_TYPE_INITEXIT, MP_INIT, DBG_LVL_ALL, "Opened file is = %s and length =0x%lx to be downloaded at =0x%x", path, (unsigned long)file_inode(flp)->i_size, loc);
	do_gettimeofday(&tv);

	BCM_DEBUG_PRINT(ad, DBG_TYPE_INITEXIT, MP_INIT, DBG_LVL_ALL, "download start %lx", ((tv.tv_sec * 1000) + (tv.tv_usec / 1000)));
	if (ad->bcm_file_download(ad->pvInterfaceAdapter, flp, loc)) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_INITEXIT, MP_INIT, DBG_LVL_ALL, "Failed to download the firmware with error %x!!!", -EIO);
		errorno = -EIO;
		goto exit_download;
	}
	vfs_llseek(flp, 0, 0);
	if (ad->bcm_file_readback_from_chip(ad->pvInterfaceAdapter, flp, loc)) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_INITEXIT, MP_INIT, DBG_LVL_ALL, "Failed to read back firmware!");
		errorno = -EIO;
		goto exit_download;
	}

exit_download:
	filp_close(flp, NULL);
	return errorno;
}

/**
 * @ingroup ctrl_pkt_functions
 * This function copies the contents of given buffer
 * to the control packet and queues it for transmission.
 * @note Do not acquire the spinlock, as it it already acquired.
 * @return  SUCCESS/FAILURE.
 * Arguments:
 * Logical Adapter
 * Control Packet Buffer
 */
int CopyBufferToControlPacket(struct bcm_mini_adapter *ad, void *ioBuffer)
{
	struct bcm_leader *leader = NULL;
	int status = 0;
	unsigned char *ctrl_buff;
	unsigned int pktlen = 0;
	struct bcm_link_request *link_req = NULL;
	PUCHAR add_indication = NULL;

	BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_CONTROL, DBG_LVL_ALL, "======>");
	if (!ioBuffer) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_CONTROL, DBG_LVL_ALL, "Got Null Buffer\n");
		return -EINVAL;
	}

	link_req = (struct bcm_link_request *)ioBuffer;
	leader = (struct bcm_leader *)ioBuffer; /* ioBuffer Contains sw_Status and Payload */

	if (ad->bShutStatus == TRUE &&
		link_req->szData[0] == LINK_DOWN_REQ_PAYLOAD &&
		link_req->szData[1] == LINK_SYNC_UP_SUBTYPE) {

		/* Got sync down in SHUTDOWN..we could not process this. */
		BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_CONTROL, DBG_LVL_ALL, "SYNC DOWN Request in Shut Down Mode..\n");
		return STATUS_FAILURE;
	}

	if ((leader->Status == LINK_UP_CONTROL_REQ) &&
		((link_req->szData[0] == LINK_UP_REQ_PAYLOAD &&
			(link_req->szData[1] == LINK_SYNC_UP_SUBTYPE)) || /* Sync Up Command */
			link_req->szData[0] == NETWORK_ENTRY_REQ_PAYLOAD)) /* Net Entry Command */ {

		if (ad->LinkStatus > PHY_SYNC_ACHIVED) {
			BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_CONTROL, DBG_LVL_ALL, "LinkStatus is Greater than PHY_SYN_ACHIEVED");
			return STATUS_FAILURE;
		}

		if (ad->bShutStatus == TRUE) {
			BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_CONTROL, DBG_LVL_ALL, "SYNC UP IN SHUTDOWN..Device WakeUp\n");
			if (ad->bTriedToWakeUpFromlowPowerMode == false) {
				BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_CONTROL, DBG_LVL_ALL, "Waking up for the First Time..\n");
				ad->usIdleModePattern = ABORT_SHUTDOWN_MODE; /* change it to 1 for current support. */
				ad->bWakeUpDevice = TRUE;
				wake_up(&ad->process_rx_cntrlpkt);
				status = wait_event_interruptible_timeout(ad->lowpower_mode_wait_queue, !ad->bShutStatus, (5 * HZ));

				if (status == -ERESTARTSYS)
					return status;

				if (ad->bShutStatus) {
					BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_CONTROL, DBG_LVL_ALL, "Shutdown Mode Wake up Failed - No Wake Up Received\n");
					return STATUS_FAILURE;
				}
			} else {
				BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_CONTROL, DBG_LVL_ALL, "Wakeup has been tried already...\n");
			}
		}
	}

	if (ad->IdleMode == TRUE) {
		/* BCM_DEBUG_PRINT(ad,DBG_TYPE_PRINTK, 0, 0,"Device is in Idle mode ... hence\n"); */
		if (leader->Status == LINK_UP_CONTROL_REQ || leader->Status == 0x80 ||
			leader->Status == CM_CONTROL_NEWDSX_MULTICLASSIFIER_REQ) {

			if ((leader->Status == LINK_UP_CONTROL_REQ) && (link_req->szData[0] == LINK_DOWN_REQ_PAYLOAD))	{
				if (link_req->szData[1] == LINK_SYNC_DOWN_SUBTYPE) {
					BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_CONTROL, DBG_LVL_ALL, "Link Down Sent in Idle Mode\n");
					ad->usIdleModePattern = ABORT_IDLE_SYNCDOWN; /* LINK DOWN sent in Idle Mode */
				} else {
					BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_CONTROL, DBG_LVL_ALL, "ABORT_IDLE_MODE pattern is being written\n");
					ad->usIdleModePattern = ABORT_IDLE_REG;
				}
			} else {
				BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_CONTROL, DBG_LVL_ALL, "ABORT_IDLE_MODE pattern is being written\n");
				ad->usIdleModePattern = ABORT_IDLE_MODE;
			}

			/*Setting bIdleMode_tx_from_host to TRUE to indicate LED control thread to represent
			 *  the wake up from idlemode is from host
			 */
			/* ad->LEDInfo.bIdleMode_tx_from_host = TRUE; */
			ad->bWakeUpDevice = TRUE;
			wake_up(&ad->process_rx_cntrlpkt);

			/* We should not send DREG message down while in idlemode. */
			if (LINK_DOWN_REQ_PAYLOAD == link_req->szData[0])
				return STATUS_SUCCESS;

			status = wait_event_interruptible_timeout(ad->lowpower_mode_wait_queue, !ad->IdleMode, (5 * HZ));

			if (status == -ERESTARTSYS)
				return status;

			if (ad->IdleMode) {
				BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_CONTROL, DBG_LVL_ALL, "Idle Mode Wake up Failed - No Wake Up Received\n");
				return STATUS_FAILURE;
			}
		} else {
			return STATUS_SUCCESS;
		}
	}

	/* The Driver has to send control messages with a particular VCID */
	leader->Vcid = VCID_CONTROL_PACKET; /* VCID for control packet. */

	/* Allocate skb for Control Packet */
	pktlen = leader->PLength;
	ctrl_buff = (char *)ad->txctlpacket[atomic_read(&ad->index_wr_txcntrlpkt)%MAX_CNTRL_PKTS];

	if (!ctrl_buff) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_CONTROL, DBG_LVL_ALL, "mem allocation Failed");
		return -ENOMEM;
	}

	BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_CONTROL, DBG_LVL_ALL, "Control packet to be taken =%d and address is =%pincoming address is =%p and packet len=%x",
			atomic_read(&ad->index_wr_txcntrlpkt), ctrl_buff, ioBuffer, pktlen);

	if (leader) {
		if ((leader->Status == 0x80) ||
			(leader->Status == CM_CONTROL_NEWDSX_MULTICLASSIFIER_REQ)) {
			/*
			 * Restructure the DSX message to handle Multiple classifier Support
			 * Write the Service Flow param Structures directly to the target
			 * and embed the pointers in the DSX messages sent to target.
			 */
			/* Lets store the current length of the control packet we are transmitting */
			add_indication = (PUCHAR)ioBuffer + LEADER_SIZE;
			pktlen = leader->PLength;
			status = StoreCmControlResponseMessage(ad, add_indication, &pktlen);
			if (status != 1) {
				ClearTargetDSXBuffer(ad, ((struct bcm_add_indication_alt *)add_indication)->u16TID, false);
				BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_CONTROL, DBG_LVL_ALL, " Error Restoring The DSX Control Packet. Dsx Buffers on Target may not be Setup Properly ");
				return STATUS_FAILURE;
			}
			/*
			 * update the leader to use the new length
			 * The length of the control packet is length of message being sent + Leader length
			 */
			leader->PLength = pktlen;
		}
	}

	if (pktlen + LEADER_SIZE > MAX_CNTL_PKT_SIZE)
		return -EINVAL;

	memset(ctrl_buff, 0, pktlen+LEADER_SIZE);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_CONTROL, DBG_LVL_ALL, "Copying the Control Packet Buffer with length=%d\n", leader->PLength);
	*(struct bcm_leader *)ctrl_buff = *leader;
	memcpy(ctrl_buff + LEADER_SIZE, ((PUCHAR)ioBuffer + LEADER_SIZE), leader->PLength);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_CONTROL, DBG_LVL_ALL, "Enqueuing the Control Packet");

	/* Update the statistics counters */
	spin_lock_bh(&ad->PackInfo[HiPriority].SFQueueLock);
	ad->PackInfo[HiPriority].uiCurrentBytesOnHost += leader->PLength;
	ad->PackInfo[HiPriority].uiCurrentPacketsOnHost++;
	atomic_inc(&ad->TotalPacketCount);
	spin_unlock_bh(&ad->PackInfo[HiPriority].SFQueueLock);
	ad->PackInfo[HiPriority].bValid = TRUE;

	BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_CONTROL, DBG_LVL_ALL, "CurrBytesOnHost: %x bValid: %x",
			ad->PackInfo[HiPriority].uiCurrentBytesOnHost,
			ad->PackInfo[HiPriority].bValid);
	status = STATUS_SUCCESS;
	/*Queue the packet for transmission */
	atomic_inc(&ad->index_wr_txcntrlpkt);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_CONTROL, DBG_LVL_ALL, "Calling transmit_packets");
	atomic_set(&ad->TxPktAvail, 1);
	wake_up(&ad->tx_packet_wait_queue);

	BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_CONTROL, DBG_LVL_ALL, "<====");
	return status;
}

/******************************************************************
* Function    - LinkMessage()
*
* Description - This function builds the Sync-up and Link-up request
* packet messages depending on the device Link status.
*
* Parameters  - ad:	Pointer to the Adapter structure.
*
* Returns     - None.
*******************************************************************/
void LinkMessage(struct bcm_mini_adapter *ad)
{
	struct bcm_link_request *link_request = NULL;

	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, LINK_UP_MSG, DBG_LVL_ALL, "=====>");
	if (ad->LinkStatus == SYNC_UP_REQUEST && ad->AutoSyncup) {
		link_request = kzalloc(sizeof(struct bcm_link_request), GFP_ATOMIC);
		if (!link_request) {
			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, LINK_UP_MSG, DBG_LVL_ALL, "Can not allocate memory for Link request!");
			return;
		}
		/* sync up request... */
		ad->LinkStatus = WAIT_FOR_SYNC; /* current link status */
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, LINK_UP_MSG, DBG_LVL_ALL, "Requesting For SyncUp...");
		link_request->szData[0] = LINK_UP_REQ_PAYLOAD;
		link_request->szData[1] = LINK_SYNC_UP_SUBTYPE;
		link_request->Leader.Status = LINK_UP_CONTROL_REQ;
		link_request->Leader.PLength = sizeof(ULONG);
		ad->bSyncUpRequestSent = TRUE;

	} else if (ad->LinkStatus == PHY_SYNC_ACHIVED && ad->AutoLinkUp) {
		link_request = kzalloc(sizeof(struct bcm_link_request), GFP_ATOMIC);
		if (!link_request) {
			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, LINK_UP_MSG, DBG_LVL_ALL, "Can not allocate memory for Link request!");
			return;
		}
		/* LINK_UP_REQUEST */
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, LINK_UP_MSG, DBG_LVL_ALL, "Requesting For LinkUp...");
		link_request->szData[0] = LINK_UP_REQ_PAYLOAD;
		link_request->szData[1] = LINK_NET_ENTRY;
		link_request->Leader.Status = LINK_UP_CONTROL_REQ;
		link_request->Leader.PLength = sizeof(ULONG);
	}
	if (link_request) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, LINK_UP_MSG, DBG_LVL_ALL, "Calling CopyBufferToControlPacket");
		CopyBufferToControlPacket(ad, link_request);
		kfree(link_request);
	}
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, LINK_UP_MSG, DBG_LVL_ALL, "LinkMessage <=====");
	return;
}

/**********************************************************************
* Function    - StatisticsResponse()
*
* Description - This function handles the Statistics response packet.
*
* Parameters  - ad: Pointer to the Adapter structure.
* - buff: Starting address of Statistic response data.
*
* Returns     - None.
************************************************************************/
void StatisticsResponse(struct bcm_mini_adapter *ad, void *buff)
{
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "%s====>", __func__);
	ad->StatisticsPointer = ntohl(*(__be32 *)buff);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "Stats at %x", (unsigned int)ad->StatisticsPointer);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "%s <====", __func__);
}

/**********************************************************************
* Function    - LinkControlResponseMessage()
*
* Description - This function handles the Link response packets.
*
* Parameters  - ad: Pointer to the Adapter structure.
* - buffer: Starting address of Link response data.
*
* Returns     - None.
***********************************************************************/
void LinkControlResponseMessage(struct bcm_mini_adapter *ad, PUCHAR buffer)
{
	BCM_DEBUG_PRINT(ad, DBG_TYPE_RX, RX_DPC, DBG_LVL_ALL, "=====>");

	if (*buffer == LINK_UP_ACK) {
		switch (*(buffer+1)) {
		case PHY_SYNC_ACHIVED: /* SYNCed UP */
			BCM_DEBUG_PRINT(ad, DBG_TYPE_PRINTK, 0, 0, "PHY_SYNC_ACHIVED");

				if (ad->LinkStatus == LINKUP_DONE)
					beceem_protocol_reset(ad);

				ad->usBestEffortQueueIndex = INVALID_QUEUE_INDEX;
				ad->LinkStatus = PHY_SYNC_ACHIVED;

				if (ad->LEDInfo.led_thread_running & BCM_LED_THREAD_RUNNING_ACTIVELY) {
					ad->DriverState = NO_NETWORK_ENTRY;
					wake_up(&ad->LEDInfo.notify_led_event);
				}

				LinkMessage(ad);
				break;

		case LINKUP_DONE:
			BCM_DEBUG_PRINT(ad, DBG_TYPE_RX, RX_DPC, DBG_LVL_ALL, "LINKUP_DONE");
			ad->LinkStatus = LINKUP_DONE;
			ad->bPHSEnabled = *(buffer+3);
			ad->bETHCSEnabled = *(buffer+4) & ETH_CS_MASK;
			BCM_DEBUG_PRINT(ad, DBG_TYPE_PRINTK, 0, 0, "PHS Support Status Received In LinkUp Ack : %x\n", ad->bPHSEnabled);

			if ((false == ad->bShutStatus) && (false == ad->IdleMode)) {
				if (ad->LEDInfo.led_thread_running & BCM_LED_THREAD_RUNNING_ACTIVELY) {
					ad->DriverState = NORMAL_OPERATION;
					wake_up(&ad->LEDInfo.notify_led_event);
				}
			}
			LinkMessage(ad);
			break;

		case WAIT_FOR_SYNC:
			/*
			 * Driver to ignore the DREG_RECEIVED
			 * WiMAX Application should handle this Message
			 */
			/* ad->liTimeSinceLastNetEntry = 0; */
			ad->LinkUpStatus = 0;
			ad->LinkStatus = 0;
			ad->usBestEffortQueueIndex = INVALID_QUEUE_INDEX;
			ad->bTriedToWakeUpFromlowPowerMode = false;
			ad->IdleMode = false;
			beceem_protocol_reset(ad);

			break;
		case LINK_SHUTDOWN_REQ_FROM_FIRMWARE:
		case COMPLETE_WAKE_UP_NOTIFICATION_FRM_FW:
		{
			HandleShutDownModeRequest(ad, buffer);
		}
		break;
		default:
			BCM_DEBUG_PRINT(ad, DBG_TYPE_PRINTK, 0, 0, "default case:LinkResponse %x", *(buffer + 1));
			break;
		}
	} else if (SET_MAC_ADDRESS_RESPONSE == *buffer) {
		PUCHAR puMacAddr = (buffer + 1);

		ad->LinkStatus = SYNC_UP_REQUEST;
		BCM_DEBUG_PRINT(ad, DBG_TYPE_RX, RX_DPC, DBG_LVL_ALL, "MAC address response, sending SYNC_UP");
		LinkMessage(ad);
		memcpy(ad->dev->dev_addr, puMacAddr, MAC_ADDRESS_SIZE);
	}
	BCM_DEBUG_PRINT(ad, DBG_TYPE_RX, RX_DPC, DBG_LVL_ALL, "%s <=====", __func__);
}

void SendIdleModeResponse(struct bcm_mini_adapter *ad)
{
	int status = 0, nvm_access = 0, low_pwr_abort_msg = 0;
	struct timeval tv;
	struct bcm_link_request idle_response = {{0} };

	memset(&tv, 0, sizeof(tv));
	idle_response.Leader.Status = IDLE_MESSAGE;
	idle_response.Leader.PLength = IDLE_MODE_PAYLOAD_LENGTH;
	idle_response.szData[0] = GO_TO_IDLE_MODE_PAYLOAD;
	BCM_DEBUG_PRINT(ad, DBG_TYPE_RX, RX_DPC, DBG_LVL_ALL, " ============>");

	/*********************************
	 *down_trylock -
	 * if [ semaphore is available ]
	 *		 acquire semaphone and return value 0 ;
	 *   else
	 *		 return non-zero value ;
	 *
	 ***********************************/

	nvm_access = down_trylock(&ad->NVMRdmWrmLock);
	low_pwr_abort_msg = down_trylock(&ad->LowPowerModeSync);


	if ((nvm_access || low_pwr_abort_msg || atomic_read(&ad->TotalPacketCount)) &&
		(ad->ulPowerSaveMode != DEVICE_POWERSAVE_MODE_AS_PROTOCOL_IDLE_MODE)) {

		if (!nvm_access)
			up(&ad->NVMRdmWrmLock);

		if (!low_pwr_abort_msg)
			up(&ad->LowPowerModeSync);

		idle_response.szData[1] = TARGET_CAN_NOT_GO_TO_IDLE_MODE; /* NACK- device access is going on. */
		BCM_DEBUG_PRINT(ad, DBG_TYPE_RX, RX_DPC, DBG_LVL_ALL, "HOST IS NACKING Idle mode To F/W!!!!!!!!");
		ad->bPreparingForLowPowerMode = false;
	} else {
		idle_response.szData[1] = TARGET_CAN_GO_TO_IDLE_MODE; /* 2; Idle ACK */
		ad->StatisticsPointer = 0;

		/* Wait for the LED to TURN OFF before sending ACK response */
		if (ad->LEDInfo.led_thread_running & BCM_LED_THREAD_RUNNING_ACTIVELY) {
			int iRetVal = 0;

			/* Wake the LED Thread with IDLEMODE_ENTER State */
			ad->DriverState = LOWPOWER_MODE_ENTER;
			BCM_DEBUG_PRINT(ad, DBG_TYPE_RX, RX_DPC, DBG_LVL_ALL, "LED Thread is Running..Hence Setting LED Event as IDLEMODE_ENTER jiffies:%ld", jiffies);
			wake_up(&ad->LEDInfo.notify_led_event);

			/* Wait for 1 SEC for LED to OFF */
			iRetVal = wait_event_timeout(ad->LEDInfo.idleModeSyncEvent, ad->LEDInfo.bIdle_led_off, msecs_to_jiffies(1000));

			/* If Timed Out to Sync IDLE MODE Enter, do IDLE mode Exit and Send NACK to device */
			if (iRetVal <= 0) {
				idle_response.szData[1] = TARGET_CAN_NOT_GO_TO_IDLE_MODE; /* NACK- device access is going on. */
				ad->DriverState = NORMAL_OPERATION;
				wake_up(&ad->LEDInfo.notify_led_event);
				BCM_DEBUG_PRINT(ad, DBG_TYPE_RX, RX_DPC, DBG_LVL_ALL, "NACKING Idle mode as time out happen from LED side!!!!!!!!");
			}
		}

		if (idle_response.szData[1] == TARGET_CAN_GO_TO_IDLE_MODE) {
			BCM_DEBUG_PRINT(ad, DBG_TYPE_RX, RX_DPC, DBG_LVL_ALL, "ACKING IDLE MODE !!!!!!!!!");
			down(&ad->rdmwrmsync);
			ad->bPreparingForLowPowerMode = TRUE;
			up(&ad->rdmwrmsync);
			/* Killing all URBS. */
			if (ad->bDoSuspend == TRUE)
				Bcm_kill_all_URBs((struct bcm_interface_adapter *)(ad->pvInterfaceAdapter));
		} else {
			ad->bPreparingForLowPowerMode = false;
		}

		if (!nvm_access)
			up(&ad->NVMRdmWrmLock);

		if (!low_pwr_abort_msg)
			up(&ad->LowPowerModeSync);
	}

	status = CopyBufferToControlPacket(ad, &idle_response);
	if (status != STATUS_SUCCESS) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_PRINTK, 0, 0, "fail to send the Idle mode Request\n");
		ad->bPreparingForLowPowerMode = false;
		StartInterruptUrb((struct bcm_interface_adapter *)(ad->pvInterfaceAdapter));
	}
	do_gettimeofday(&tv);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_RX, RX_DPC, DBG_LVL_ALL, "IdleMode Msg submitter to Q :%ld ms", tv.tv_sec * 1000 + tv.tv_usec / 1000);
}

/******************************************************************
* Function    - DumpPackInfo()
*
* Description - This function dumps the all Queue(PackInfo[]) details.
*
* Parameters  - ad: Pointer to the Adapter structure.
*
* Returns     - None.
*******************************************************************/
void DumpPackInfo(struct bcm_mini_adapter *ad)
{
	unsigned int i = 0;
	unsigned int index = 0;
	unsigned int clsfr_index = 0;
	struct bcm_classifier_rule *classif_entry = NULL;

	for (i = 0; i < NO_OF_QUEUES; i++) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "*********** Showing Details Of Queue %d***** ******", i);
		if (false == ad->PackInfo[i].bValid) {
			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "bValid is false for %X index\n", i);
			continue;
		}

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, " Dumping	SF Rule Entry For SFID %lX\n", ad->PackInfo[i].ulSFID);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, " ucDirection %X\n", ad->PackInfo[i].ucDirection);

		if (ad->PackInfo[i].ucIpVersion == IPV6)
			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "Ipv6 Service Flow\n");
		else
			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "Ipv4 Service Flow\n");

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "SF Traffic Priority %X\n", ad->PackInfo[i].u8TrafficPriority);

		for (clsfr_index = 0; clsfr_index < MAX_CLASSIFIERS; clsfr_index++) {
			classif_entry = &ad->astClassifierTable[clsfr_index];
			if (!classif_entry->bUsed)
				continue;

			if (classif_entry->ulSFID != ad->PackInfo[i].ulSFID)
				continue;

			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "\tDumping Classifier Rule Entry For Index: %X Classifier Rule ID : %X\n", clsfr_index, classif_entry->uiClassifierRuleIndex);
			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "\tDumping Classifier Rule Entry For Index: %X usVCID_Value : %X\n", clsfr_index, classif_entry->usVCID_Value);
			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "\tDumping Classifier Rule Entry For Index: %X bProtocolValid : %X\n", clsfr_index, classif_entry->bProtocolValid);
			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "\tDumping Classifier Rule Entry For Index: %X bTOSValid : %X\n", clsfr_index, classif_entry->bTOSValid);
			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "\tDumping Classifier Rule Entry For Index: %X bDestIpValid : %X\n", clsfr_index, classif_entry->bDestIpValid);
			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "\tDumping Classifier Rule Entry For Index: %X bSrcIpValid : %X\n", clsfr_index, classif_entry->bSrcIpValid);

			for (index = 0; index < MAX_PORT_RANGE; index++) {
				BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "\tusSrcPortRangeLo:%X\n", classif_entry->usSrcPortRangeLo[index]);
				BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "\tusSrcPortRangeHi:%X\n", classif_entry->usSrcPortRangeHi[index]);
				BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "\tusDestPortRangeLo:%X\n", classif_entry->usDestPortRangeLo[index]);
				BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "\tusDestPortRangeHi:%X\n", classif_entry->usDestPortRangeHi[index]);
			}

			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "\tucIPSourceAddressLength : 0x%x\n", classif_entry->ucIPSourceAddressLength);
			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "\tucIPDestinationAddressLength : 0x%x\n", classif_entry->ucIPDestinationAddressLength);
			for (index = 0; index < classif_entry->ucIPSourceAddressLength; index++) {
				if (ad->PackInfo[i].ucIpVersion == IPV6)	{
					BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "\tIpv6 ulSrcIpAddr :\n");
					DumpIpv6Address(classif_entry->stSrcIpAddress.ulIpv6Addr);
					BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "\tIpv6 ulSrcIpMask :\n");
					DumpIpv6Address(classif_entry->stSrcIpAddress.ulIpv6Mask);
				} else {
					BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "\tulSrcIpAddr:%lX\n", classif_entry->stSrcIpAddress.ulIpv4Addr[index]);
					BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "\tulSrcIpMask:%lX\n", classif_entry->stSrcIpAddress.ulIpv4Mask[index]);
				}
			}

			for (index = 0; index < classif_entry->ucIPDestinationAddressLength; index++) {
				if (ad->PackInfo[i].ucIpVersion == IPV6) {
					BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "\tIpv6 ulDestIpAddr :\n");
					DumpIpv6Address(classif_entry->stDestIpAddress.ulIpv6Addr);
					BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "\tIpv6 ulDestIpMask :\n");
					DumpIpv6Address(classif_entry->stDestIpAddress.ulIpv6Mask);
				} else {
					BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "\tulDestIpAddr:%lX\n", classif_entry->stDestIpAddress.ulIpv4Addr[index]);
					BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "\tulDestIpMask:%lX\n", classif_entry->stDestIpAddress.ulIpv4Mask[index]);
				}
			}
			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "\tucProtocol:0x%X\n", classif_entry->ucProtocol[0]);
			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "\tu8ClassifierRulePriority:%X\n", classif_entry->u8ClassifierRulePriority);
		}
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "ulSFID:%lX\n", ad->PackInfo[i].ulSFID);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "usVCID_Value:%X\n", ad->PackInfo[i].usVCID_Value);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "PhsEnabled: 0x%X\n", ad->PackInfo[i].bHeaderSuppressionEnabled);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "uiThreshold:%X\n", ad->PackInfo[i].uiThreshold);

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "bValid:%X\n", ad->PackInfo[i].bValid);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "bActive:%X\n", ad->PackInfo[i].bActive);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "ActivateReqSent: %x", ad->PackInfo[i].bActivateRequestSent);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "u8QueueType:%X\n", ad->PackInfo[i].u8QueueType);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "uiMaxBucketSize:%X\n", ad->PackInfo[i].uiMaxBucketSize);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "uiPerSFTxResourceCount:%X\n", atomic_read(&ad->PackInfo[i].uiPerSFTxResourceCount));
		/* DumpDebug(DUMP_INFO,("bCSSupport:%X\n",ad->PackInfo[i].bCSSupport)); */
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "CurrQueueDepthOnTarget: %x\n", ad->PackInfo[i].uiCurrentQueueDepthOnTarget);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "uiCurrentBytesOnHost:%X\n", ad->PackInfo[i].uiCurrentBytesOnHost);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "uiCurrentPacketsOnHost:%X\n", ad->PackInfo[i].uiCurrentPacketsOnHost);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "uiDroppedCountBytes:%X\n", ad->PackInfo[i].uiDroppedCountBytes);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "uiDroppedCountPackets:%X\n", ad->PackInfo[i].uiDroppedCountPackets);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "uiSentBytes:%X\n", ad->PackInfo[i].uiSentBytes);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "uiSentPackets:%X\n", ad->PackInfo[i].uiSentPackets);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "uiCurrentDrainRate:%X\n", ad->PackInfo[i].uiCurrentDrainRate);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "uiThisPeriodSentBytes:%X\n", ad->PackInfo[i].uiThisPeriodSentBytes);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "liDrainCalculated:%llX\n", ad->PackInfo[i].liDrainCalculated);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "uiCurrentTokenCount:%X\n", ad->PackInfo[i].uiCurrentTokenCount);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "liLastUpdateTokenAt:%llX\n", ad->PackInfo[i].liLastUpdateTokenAt);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "uiMaxAllowedRate:%X\n", ad->PackInfo[i].uiMaxAllowedRate);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "uiPendedLast:%X\n", ad->PackInfo[i].uiPendedLast);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "NumOfPacketsSent:%X\n", ad->PackInfo[i].NumOfPacketsSent);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "Direction: %x\n", ad->PackInfo[i].ucDirection);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "CID: %x\n", ad->PackInfo[i].usCID);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "ProtocolValid: %x\n", ad->PackInfo[i].bProtocolValid);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "TOSValid: %x\n", ad->PackInfo[i].bTOSValid);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "DestIpValid: %x\n", ad->PackInfo[i].bDestIpValid);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "SrcIpValid: %x\n", ad->PackInfo[i].bSrcIpValid);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "ActiveSet: %x\n", ad->PackInfo[i].bActiveSet);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "AdmittedSet: %x\n", ad->PackInfo[i].bAdmittedSet);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "AuthzSet: %x\n", ad->PackInfo[i].bAuthorizedSet);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "ClassifyPrority: %x\n", ad->PackInfo[i].bClassifierPriority);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "uiMaxLatency: %x\n", ad->PackInfo[i].uiMaxLatency);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO,
				DBG_LVL_ALL, "ServiceClassName: %*ph\n",
				4, ad->PackInfo[i].
					    ucServiceClassName);
/* BCM_DEBUG_PRINT (ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "bHeaderSuppressionEnabled :%X\n", ad->PackInfo[i].bHeaderSuppressionEnabled);
 * BCM_DEBUG_PRINT (ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "uiTotalTxBytes:%X\n", ad->PackInfo[i].uiTotalTxBytes);
 * BCM_DEBUG_PRINT (ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "uiTotalRxBytes:%X\n", ad->PackInfo[i].uiTotalRxBytes);
 *		DumpDebug(DUMP_INFO,("				uiRanOutOfResCount:%X\n",ad->PackInfo[i].uiRanOutOfResCount));
 */
	}

	for (i = 0; i < MIBS_MAX_HIST_ENTRIES; i++)
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "ad->aRxPktSizeHist[%x] = %x\n", i, ad->aRxPktSizeHist[i]);

	for (i = 0; i < MIBS_MAX_HIST_ENTRIES; i++)
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_INFO, DBG_LVL_ALL, "ad->aTxPktSizeHist[%x] = %x\n", i, ad->aTxPktSizeHist[i]);
}

int reset_card_proc(struct bcm_mini_adapter *ps_adapter)
{
	int retval = STATUS_SUCCESS;
	struct bcm_mini_adapter *ad = GET_BCM_ADAPTER(gblpnetdev);
	struct bcm_interface_adapter *intf_ad = NULL;
	unsigned int value = 0, reset_val = 0;
	int bytes;

	intf_ad = ((struct bcm_interface_adapter *)(ps_adapter->pvInterfaceAdapter));
	ps_adapter->bDDRInitDone = false;

	if (ps_adapter->chip_id >= T3LPB) {
		/* SYS_CFG register is write protected hence for modifying this reg value, it should be read twice before */
		rdmalt(ps_adapter, SYS_CFG, &value, sizeof(value));
		rdmalt(ps_adapter, SYS_CFG, &value, sizeof(value));

		/* making bit[6...5] same as was before f/w download. this setting force the h/w to */
		/* re-populated the SP RAM area with the string descriptor. */
		value = value | (ps_adapter->syscfgBefFwDld & 0x00000060);
		wrmalt(ps_adapter, SYS_CFG, &value, sizeof(value));
	}

	/* killing all submitted URBs. */
	intf_ad->psAdapter->StopAllXaction = TRUE;
	Bcm_kill_all_URBs(intf_ad);
	/* Reset the UMA-B Device */
	if (ps_adapter->chip_id >= T3LPB) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_PRINTK, 0, 0, "Resetting UMA-B\n");
		retval = usb_reset_device(intf_ad->udev);
		intf_ad->psAdapter->StopAllXaction = false;

		if (retval != STATUS_SUCCESS) {
			BCM_DEBUG_PRINT(ad, DBG_TYPE_PRINTK, 0, 0, "Reset failed with ret value :%d", retval);
			goto err_exit;
		}

		if (ps_adapter->chip_id == BCS220_2 ||
			ps_adapter->chip_id == BCS220_2BC ||
			ps_adapter->chip_id == BCS250_BC ||
			ps_adapter->chip_id == BCS220_3) {

			bytes = rdmalt(ps_adapter, HPM_CONFIG_LDO145, &value, sizeof(value));
			if (bytes < 0) {
				retval = bytes;
				BCM_DEBUG_PRINT(ad, DBG_TYPE_PRINTK, 0, 0, "read failed with status :%d", retval);
				goto err_exit;
			}
			/* setting 0th bit */
			value |= (1<<0);
			retval = wrmalt(ps_adapter, HPM_CONFIG_LDO145, &value, sizeof(value));
			if (retval < 0) {
				BCM_DEBUG_PRINT(ad, DBG_TYPE_PRINTK, 0, 0, "write failed with status :%d", retval);
				goto err_exit;
			}
		}
	} else {
		bytes = rdmalt(ps_adapter, 0x0f007018, &value, sizeof(value));
		if (bytes < 0) {
			retval = bytes;
			BCM_DEBUG_PRINT(ad, DBG_TYPE_PRINTK, 0, 0, "read failed with status :%d", retval);
			goto err_exit;
		}
		value &= (~(1<<16));
		retval = wrmalt(ps_adapter, 0x0f007018, &value, sizeof(value));
		if (retval < 0) {
			BCM_DEBUG_PRINT(ad, DBG_TYPE_PRINTK, 0, 0, "write failed with status :%d", retval);
			goto err_exit;
		}

		/* Toggling the GPIO 8, 9 */
		value = 0;
		retval = wrmalt(ps_adapter, GPIO_OUTPUT_REGISTER, &value, sizeof(value));
		if (retval < 0) {
			BCM_DEBUG_PRINT(ad, DBG_TYPE_PRINTK, 0, 0, "write failed with status :%d", retval);
			goto err_exit;
		}
		value = 0x300;
		retval = wrmalt(ps_adapter, GPIO_MODE_REGISTER, &value, sizeof(value));
		if (retval < 0) {
			BCM_DEBUG_PRINT(ad, DBG_TYPE_PRINTK, 0, 0, "write failed with status :%d", retval);
			goto err_exit;
		}
		mdelay(50);
	}

	/* ps_adapter->downloadDDR = false; */
	if (ps_adapter->bFlashBoot) {
		/* In flash boot mode MIPS state register has reverse polarity.
		 * So just or with setting bit 30.
		 * Make the MIPS in Reset state.
		 */
		rdmalt(ps_adapter, CLOCK_RESET_CNTRL_REG_1, &reset_val, sizeof(reset_val));
		reset_val |= (1<<30);
		wrmalt(ps_adapter, CLOCK_RESET_CNTRL_REG_1, &reset_val, sizeof(reset_val));
	}

	if (ps_adapter->chip_id >= T3LPB) {
		reset_val = 0;
		/*
		 * WA for SYSConfig Issue.
		 * Read SYSCFG Twice to make it writable.
		 */
		rdmalt(ps_adapter, SYS_CFG, &reset_val, sizeof(reset_val));
		if (reset_val & (1<<4)) {
			reset_val = 0;
			rdmalt(ps_adapter, SYS_CFG, &reset_val, sizeof(reset_val)); /* 2nd read to make it writable. */
			reset_val &= (~(1<<4));
			wrmalt(ps_adapter, SYS_CFG, &reset_val, sizeof(reset_val));
		}
	}
	reset_val = 0;
	wrmalt(ps_adapter, 0x0f01186c, &reset_val, sizeof(reset_val));

err_exit:
	intf_ad->psAdapter->StopAllXaction = false;
	return retval;
}

int run_card_proc(struct bcm_mini_adapter *ps_adapter)
{
	int status = STATUS_SUCCESS;
	int bytes;

	unsigned int value = 0;
	{
		bytes = rdmalt(ps_adapter, CLOCK_RESET_CNTRL_REG_1, &value, sizeof(value));
		if (bytes < 0) {
			status = bytes;
			BCM_DEBUG_PRINT(ps_adapter, DBG_TYPE_INITEXIT, MP_INIT, DBG_LVL_ALL, "%s:%d\n", __func__, __LINE__);
			return status;
		}

		if (ps_adapter->bFlashBoot)
			value &= (~(1<<30));
		else
			value |= (1<<30);

		if (wrmalt(ps_adapter, CLOCK_RESET_CNTRL_REG_1, &value, sizeof(value)) < 0) {
			BCM_DEBUG_PRINT(ps_adapter, DBG_TYPE_INITEXIT, MP_INIT, DBG_LVL_ALL, "%s:%d\n", __func__, __LINE__);
			return STATUS_FAILURE;
		}
	}
	return status;
}

int InitCardAndDownloadFirmware(struct bcm_mini_adapter *ps_adapter)
{
	int status;
	unsigned int value = 0;
	/*
	 * Create the threads first and then download the
	 * Firm/DDR Settings..
	 */
	status = create_worker_threads(ps_adapter);
	if (status < 0)
		return status;

	status = bcm_parse_target_params(ps_adapter);
	if (status)
		return status;

	if (ps_adapter->chip_id >= T3LPB) {
		rdmalt(ps_adapter, SYS_CFG, &value, sizeof(value));
		ps_adapter->syscfgBefFwDld = value;

		if ((value & 0x60) == 0)
			ps_adapter->bFlashBoot = TRUE;
	}

	reset_card_proc(ps_adapter);

	/* Initializing the NVM. */
	BcmInitNVM(ps_adapter);
	status = ddr_init(ps_adapter);
	if (status) {
		pr_err(DRV_NAME "ddr_init Failed\n");
		return status;
	}

	/* Download cfg file */
	status = buffDnldVerify(ps_adapter,
				(PUCHAR)ps_adapter->pstargetparams,
				sizeof(struct bcm_target_params),
				CONFIG_BEGIN_ADDR);
	if (status) {
		BCM_DEBUG_PRINT(ps_adapter, DBG_TYPE_INITEXIT, MP_INIT, DBG_LVL_ALL, "Error downloading CFG file");
		goto OUT;
	}

	if (register_networkdev(ps_adapter)) {
		BCM_DEBUG_PRINT(ps_adapter, DBG_TYPE_INITEXIT, MP_INIT, DBG_LVL_ALL, "Register Netdevice failed. Cleanup needs to be performed.");
		return -EIO;
	}

	if (false == ps_adapter->AutoFirmDld) {
		BCM_DEBUG_PRINT(ps_adapter, DBG_TYPE_INITEXIT, MP_INIT, DBG_LVL_ALL, "AutoFirmDld Disabled in CFG File..\n");
		/* If Auto f/w download is disable, register the control interface, */
		/* register the control interface after the mailbox. */
		if (register_control_device_interface(ps_adapter) < 0) {
			BCM_DEBUG_PRINT(ps_adapter, DBG_TYPE_INITEXIT, MP_INIT, DBG_LVL_ALL, "Register Control Device failed. Cleanup needs to be performed.");
			return -EIO;
		}
		return STATUS_SUCCESS;
	}

	/*
	 * Do the LED Settings here. It will be used by the Firmware Download
	 * Thread.
	 */

	/*
	 * 1. If the LED Settings fails, do not stop and do the Firmware download.
	 * 2. This init would happened only if the cfg file is present, else
	 *    call from the ioctl context.
	 */

	status = InitLedSettings(ps_adapter);
	if (status) {
		BCM_DEBUG_PRINT(ps_adapter, DBG_TYPE_PRINTK, 0, 0, "INIT LED FAILED\n");
		return status;
	}

	if (ps_adapter->LEDInfo.led_thread_running & BCM_LED_THREAD_RUNNING_ACTIVELY) {
		ps_adapter->DriverState = DRIVER_INIT;
		wake_up(&ps_adapter->LEDInfo.notify_led_event);
	}

	if (ps_adapter->LEDInfo.led_thread_running & BCM_LED_THREAD_RUNNING_ACTIVELY) {
		ps_adapter->DriverState = FW_DOWNLOAD;
		wake_up(&ps_adapter->LEDInfo.notify_led_event);
	}

	value = 0;
	wrmalt(ps_adapter, EEPROM_CAL_DATA_INTERNAL_LOC - 4, &value, sizeof(value));
	wrmalt(ps_adapter, EEPROM_CAL_DATA_INTERNAL_LOC - 8, &value, sizeof(value));

	if (ps_adapter->eNVMType == NVM_FLASH) {
		status = PropagateCalParamsFromFlashToMemory(ps_adapter);
		if (status) {
			BCM_DEBUG_PRINT(ps_adapter, DBG_TYPE_INITEXIT, MP_INIT, DBG_LVL_ALL, "Propagation of Cal param failed ..");
			goto OUT;
		}
	}

	/* Download Firmare */
	status = BcmFileDownload(ps_adapter, BIN_FILE, FIRMWARE_BEGIN_ADDR);
	if (status != 0) {
		BCM_DEBUG_PRINT(ps_adapter, DBG_TYPE_INITEXIT, MP_INIT, DBG_LVL_ALL, "No Firmware File is present...\n");
		goto OUT;
	}

	status = run_card_proc(ps_adapter);
	if (status) {
		BCM_DEBUG_PRINT(ps_adapter, DBG_TYPE_INITEXIT, MP_INIT, DBG_LVL_ALL, "run_card_proc Failed\n");
		goto OUT;
	}

	ps_adapter->fw_download_done = TRUE;
	mdelay(10);

OUT:
	if (ps_adapter->LEDInfo.led_thread_running & BCM_LED_THREAD_RUNNING_ACTIVELY) {
		ps_adapter->DriverState = FW_DOWNLOAD_DONE;
		wake_up(&ps_adapter->LEDInfo.notify_led_event);
	}

	return status;
}

static int bcm_parse_target_params(struct bcm_mini_adapter *ad)
{
	struct file *flp = NULL;
	char *buff;
	int len = 0;

	buff = kmalloc(BUFFER_1K, GFP_KERNEL);
	if (!buff)
		return -ENOMEM;

	ad->pstargetparams = kmalloc(sizeof(struct bcm_target_params), GFP_KERNEL);
	if (ad->pstargetparams == NULL) {
		kfree(buff);
		return -ENOMEM;
	}

	flp = open_firmware_file(ad, CFG_FILE);
	if (!flp) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_INITEXIT, MP_INIT, DBG_LVL_ALL, "NOT ABLE TO OPEN THE %s FILE\n", CFG_FILE);
		kfree(buff);
		kfree(ad->pstargetparams);
		ad->pstargetparams = NULL;
		return -ENOENT;
	}
	len = kernel_read(flp, 0, buff, BUFFER_1K);
	filp_close(flp, NULL);

	if (len != sizeof(struct bcm_target_params)) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_INITEXIT, MP_INIT, DBG_LVL_ALL, "Mismatch in Target Param Structure!\n");
		kfree(buff);
		kfree(ad->pstargetparams);
		ad->pstargetparams = NULL;
		return -ENOENT;
	}

	/* Check for autolink in config params */
	/*
	 * Values in ad->pstargetparams are in network byte order
	 */
	memcpy(ad->pstargetparams, buff, sizeof(struct bcm_target_params));
	kfree(buff);
	beceem_parse_target_struct(ad);
	return STATUS_SUCCESS;
}

void beceem_parse_target_struct(struct bcm_mini_adapter *ad)
{
	unsigned int host_drvr_cfg6 = 0, eeprom_flag = 0;

	if (ntohl(ad->pstargetparams->m_u32PhyParameter2) & AUTO_SYNC_DISABLE) {
		pr_info(DRV_NAME ": AutoSyncup is Disabled\n");
		ad->AutoSyncup = false;
	} else {
		pr_info(DRV_NAME ": AutoSyncup is Enabled\n");
		ad->AutoSyncup = TRUE;
	}

	if (ntohl(ad->pstargetparams->HostDrvrConfig6) & AUTO_LINKUP_ENABLE) {
		pr_info(DRV_NAME ": Enabling autolink up");
		ad->AutoLinkUp = TRUE;
	} else {
		pr_info(DRV_NAME ": Disabling autolink up");
		ad->AutoLinkUp = false;
	}
	/* Setting the DDR Setting.. */
	ad->DDRSetting = (ntohl(ad->pstargetparams->HostDrvrConfig6) >> 8)&0x0F;
	ad->ulPowerSaveMode = (ntohl(ad->pstargetparams->HostDrvrConfig6)>>12)&0x0F;
	pr_info(DRV_NAME ": DDR Setting: %x\n", ad->DDRSetting);
	pr_info(DRV_NAME ": Power Save Mode: %lx\n", ad->ulPowerSaveMode);
	if (ntohl(ad->pstargetparams->HostDrvrConfig6) & AUTO_FIRM_DOWNLOAD) {
		pr_info(DRV_NAME ": Enabling Auto Firmware Download\n");
		ad->AutoFirmDld = TRUE;
	} else {
		pr_info(DRV_NAME ": Disabling Auto Firmware Download\n");
		ad->AutoFirmDld = false;
	}
	host_drvr_cfg6 = ntohl(ad->pstargetparams->HostDrvrConfig6);
	ad->bMipsConfig = (host_drvr_cfg6>>20)&0x01;
	pr_info(DRV_NAME ": MIPSConfig   : 0x%X\n", ad->bMipsConfig);
	/* used for backward compatibility. */
	ad->bDPLLConfig = (host_drvr_cfg6>>19)&0x01;
	ad->PmuMode = (host_drvr_cfg6 >> 24) & 0x03;
	pr_info(DRV_NAME ": PMU MODE: %x", ad->PmuMode);

	if ((host_drvr_cfg6 >> HOST_BUS_SUSPEND_BIT) & (0x01)) {
		ad->bDoSuspend = TRUE;
		pr_info(DRV_NAME ": Making DoSuspend TRUE as per configFile");
	}

	eeprom_flag = ntohl(ad->pstargetparams->m_u32EEPROMFlag);
	pr_info(DRV_NAME ": eeprom_flag  : 0x%X\n", eeprom_flag);
	ad->eNVMType = (enum bcm_nvm_type)((eeprom_flag>>4)&0x3);
	ad->bStatusWrite = (eeprom_flag>>6)&0x1;
	ad->uiSectorSizeInCFG = 1024*(0xFFFF & ntohl(ad->pstargetparams->HostDrvrConfig4));
	ad->bSectorSizeOverride = (bool) ((ntohl(ad->pstargetparams->HostDrvrConfig4))>>16)&0x1;

	if (ntohl(ad->pstargetparams->m_u32PowerSavingModeOptions) & 0x01)
		ad->ulPowerSaveMode = DEVICE_POWERSAVE_MODE_AS_PROTOCOL_IDLE_MODE;

	if (ad->ulPowerSaveMode != DEVICE_POWERSAVE_MODE_AS_PROTOCOL_IDLE_MODE)
		doPowerAutoCorrection(ad);
}

static void doPowerAutoCorrection(struct bcm_mini_adapter *ps_ad)
{
	unsigned int reporting_mode;

	reporting_mode = ntohl(ps_ad->pstargetparams->m_u32PowerSavingModeOptions) & 0x02;
	ps_ad->bIsAutoCorrectEnabled = !((char)(ps_ad->ulPowerSaveMode >> 3) & 0x1);

	if (reporting_mode) {
		BCM_DEBUG_PRINT(ps_ad, DBG_TYPE_INITEXIT, MP_INIT, DBG_LVL_ALL, "can't do suspen/resume as reporting mode is enable");
		ps_ad->bDoSuspend = false;
	}

	if (ps_ad->bIsAutoCorrectEnabled && (ps_ad->chip_id >= T3LPB)) {
		/* If reporting mode is enable, switch PMU to PMC */
		{
			ps_ad->ulPowerSaveMode = DEVICE_POWERSAVE_MODE_AS_PMU_CLOCK_GATING;
			ps_ad->bDoSuspend = false;
		}

		/* clearing space bit[15..12] */
		ps_ad->pstargetparams->HostDrvrConfig6 &= ~(htonl((0xF << 12)));
		/* placing the power save mode option */
		ps_ad->pstargetparams->HostDrvrConfig6 |= htonl((ps_ad->ulPowerSaveMode << 12));
	} else if (ps_ad->bIsAutoCorrectEnabled == false) {
		/* remove the autocorrect disable bit set before dumping. */
		ps_ad->ulPowerSaveMode &= ~(1 << 3);
		ps_ad->pstargetparams->HostDrvrConfig6 &= ~(htonl(1 << 15));
		BCM_DEBUG_PRINT(ps_ad, DBG_TYPE_INITEXIT, MP_INIT, DBG_LVL_ALL, "Using Forced User Choice: %lx\n", ps_ad->ulPowerSaveMode);
	}
}

static void convertEndian(unsigned char rw_flag, unsigned int *buffer, unsigned int byte_count)
{
	unsigned int index = 0;

	if (RWM_WRITE == rw_flag) {
		for (index = 0; index < (byte_count/sizeof(unsigned int)); index++)
			buffer[index] = htonl(buffer[index]);
	} else {
		for (index = 0; index < (byte_count/sizeof(unsigned int)); index++)
			buffer[index] = ntohl(buffer[index]);
	}
}

int rdm(struct bcm_mini_adapter *ad, unsigned int addr, PCHAR pucBuff, size_t sSize)
{
	return ad->interface_rdm(ad->pvInterfaceAdapter,
				addr, pucBuff, sSize);
}

int wrm(struct bcm_mini_adapter *ad, unsigned int addr, PCHAR pucBuff, size_t sSize)
{
	int iRetVal;

	iRetVal = ad->interface_wrm(ad->pvInterfaceAdapter,
					addr, pucBuff, sSize);
	return iRetVal;
}

int wrmalt(struct bcm_mini_adapter *ad, unsigned int addr, unsigned int *pucBuff, size_t size)
{
	convertEndian(RWM_WRITE, pucBuff, size);
	return wrm(ad, addr, (PUCHAR)pucBuff, size);
}

int rdmalt(struct bcm_mini_adapter *ad, unsigned int addr, unsigned int *pucBuff, size_t size)
{
	int uiRetVal = 0;

	uiRetVal = rdm(ad, addr, (PUCHAR)pucBuff, size);
	convertEndian(RWM_READ, (unsigned int *)pucBuff, size);

	return uiRetVal;
}

int wrmWithLock(struct bcm_mini_adapter *ad, unsigned int addr, PCHAR pucBuff, size_t sSize)
{
	int status = STATUS_SUCCESS;

	down(&ad->rdmwrmsync);

	if ((ad->IdleMode == TRUE) ||
		(ad->bShutStatus == TRUE) ||
		(ad->bPreparingForLowPowerMode == TRUE)) {

		status = -EACCES;
		goto exit;
	}

	status = wrm(ad, addr, pucBuff, sSize);
exit:
	up(&ad->rdmwrmsync);
	return status;
}

int wrmaltWithLock(struct bcm_mini_adapter *ad, unsigned int addr, unsigned int *pucBuff, size_t size)
{
	int iRetVal = STATUS_SUCCESS;

	down(&ad->rdmwrmsync);

	if ((ad->IdleMode == TRUE) ||
		(ad->bShutStatus == TRUE) ||
		(ad->bPreparingForLowPowerMode == TRUE)) {

		iRetVal = -EACCES;
		goto exit;
	}

	iRetVal = wrmalt(ad, addr, pucBuff, size);
exit:
	up(&ad->rdmwrmsync);
	return iRetVal;
}

int rdmaltWithLock(struct bcm_mini_adapter *ad, unsigned int addr, unsigned int *pucBuff, size_t size)
{
	int uiRetVal = STATUS_SUCCESS;

	down(&ad->rdmwrmsync);
	if ((ad->IdleMode == TRUE) ||
		(ad->bShutStatus == TRUE) ||
		(ad->bPreparingForLowPowerMode == TRUE)) {

		uiRetVal = -EACCES;
		goto exit;
	}

	uiRetVal = rdmalt(ad, addr, pucBuff, size);
exit:
	up(&ad->rdmwrmsync);
	return uiRetVal;
}

static void HandleShutDownModeWakeup(struct bcm_mini_adapter *ad)
{
	int clear_abort_pattern = 0, status = 0;

	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, MP_SHUTDOWN, DBG_LVL_ALL, "====>\n");
	/* target has woken up From Shut Down */
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, MP_SHUTDOWN, DBG_LVL_ALL, "Clearing Shut Down Software abort pattern\n");
	status = wrmalt(ad, SW_ABORT_IDLEMODE_LOC, (unsigned int *)&clear_abort_pattern, sizeof(clear_abort_pattern));
	if (status) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, MP_SHUTDOWN, DBG_LVL_ALL, "WRM to SW_ABORT_IDLEMODE_LOC failed with err:%d", status);
		return;
	}

	if (ad->ulPowerSaveMode != DEVICE_POWERSAVE_MODE_AS_PROTOCOL_IDLE_MODE) {
		msleep(100);
		InterfaceHandleShutdownModeWakeup(ad);
		msleep(100);
	}

	if (ad->LEDInfo.led_thread_running & BCM_LED_THREAD_RUNNING_ACTIVELY) {
		ad->DriverState = NO_NETWORK_ENTRY;
		wake_up(&ad->LEDInfo.notify_led_event);
	}

	ad->bTriedToWakeUpFromlowPowerMode = false;
	ad->bShutStatus = false;
	wake_up(&ad->lowpower_mode_wait_queue);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, MP_SHUTDOWN, DBG_LVL_ALL, "<====\n");
}

static void SendShutModeResponse(struct bcm_mini_adapter *ad)
{
	struct bcm_link_request stShutdownResponse;
	unsigned int nvm_access = 0, low_pwr_abort_msg = 0;
	unsigned int status = 0;

	memset(&stShutdownResponse, 0, sizeof(struct bcm_link_request));
	stShutdownResponse.Leader.Status  = LINK_UP_CONTROL_REQ;
	stShutdownResponse.Leader.PLength = 8; /* 8 bytes; */
	stShutdownResponse.szData[0] = LINK_UP_ACK;
	stShutdownResponse.szData[1] = LINK_SHUTDOWN_REQ_FROM_FIRMWARE;

	/*********************************
	 * down_trylock -
	 * if [ semaphore is available ]
	 *		 acquire semaphone and return value 0 ;
	 *   else
	 *		 return non-zero value ;
	 *
	 ***********************************/

	nvm_access = down_trylock(&ad->NVMRdmWrmLock);
	low_pwr_abort_msg = down_trylock(&ad->LowPowerModeSync);

	if (nvm_access || low_pwr_abort_msg || atomic_read(&ad->TotalPacketCount)) {
		if (!nvm_access)
			up(&ad->NVMRdmWrmLock);

		if (!low_pwr_abort_msg)
			up(&ad->LowPowerModeSync);

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, MP_SHUTDOWN, DBG_LVL_ALL, "Device Access is going on NACK the Shut Down MODE\n");
		stShutdownResponse.szData[2] = SHUTDOWN_NACK_FROM_DRIVER; /* NACK- device access is going on. */
		ad->bPreparingForLowPowerMode = false;
	} else {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, MP_SHUTDOWN, DBG_LVL_ALL, "Sending SHUTDOWN MODE ACK\n");
		stShutdownResponse.szData[2] = SHUTDOWN_ACK_FROM_DRIVER; /* ShutDown ACK */

		/* Wait for the LED to TURN OFF before sending ACK response */
		if (ad->LEDInfo.led_thread_running & BCM_LED_THREAD_RUNNING_ACTIVELY) {
			int iRetVal = 0;

			/* Wake the LED Thread with LOWPOWER_MODE_ENTER State */
			ad->DriverState = LOWPOWER_MODE_ENTER;
			wake_up(&ad->LEDInfo.notify_led_event);

			/* Wait for 1 SEC for LED to OFF */
			iRetVal = wait_event_timeout(ad->LEDInfo.idleModeSyncEvent, ad->LEDInfo.bIdle_led_off, msecs_to_jiffies(1000));

			/* If Timed Out to Sync IDLE MODE Enter, do IDLE mode Exit and Send NACK to device */
			if (iRetVal <= 0) {
				stShutdownResponse.szData[1] = SHUTDOWN_NACK_FROM_DRIVER; /* NACK- device access is going on. */
				ad->DriverState = NO_NETWORK_ENTRY;
				wake_up(&ad->LEDInfo.notify_led_event);
			}
		}

		if (stShutdownResponse.szData[2] == SHUTDOWN_ACK_FROM_DRIVER) {
			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, MP_SHUTDOWN, DBG_LVL_ALL, "ACKING SHUTDOWN MODE !!!!!!!!!");
			down(&ad->rdmwrmsync);
			ad->bPreparingForLowPowerMode = TRUE;
			up(&ad->rdmwrmsync);
			/* Killing all URBS. */
			if (ad->bDoSuspend == TRUE)
				Bcm_kill_all_URBs((struct bcm_interface_adapter *)(ad->pvInterfaceAdapter));
		} else {
			ad->bPreparingForLowPowerMode = false;
		}

		if (!nvm_access)
			up(&ad->NVMRdmWrmLock);

		if (!low_pwr_abort_msg)
			up(&ad->LowPowerModeSync);
	}

	status = CopyBufferToControlPacket(ad, &stShutdownResponse);
	if (status != STATUS_SUCCESS) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, MP_SHUTDOWN, DBG_LVL_ALL, "fail to send the Idle mode Request\n");
		ad->bPreparingForLowPowerMode = false;
		StartInterruptUrb((struct bcm_interface_adapter *)(ad->pvInterfaceAdapter));
	}
}

static void HandleShutDownModeRequest(struct bcm_mini_adapter *ad, PUCHAR buffer)
{
	unsigned int reset_val = 0;

	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, MP_SHUTDOWN, DBG_LVL_ALL, "====>\n");

	if (*(buffer+1) ==  COMPLETE_WAKE_UP_NOTIFICATION_FRM_FW) {
		HandleShutDownModeWakeup(ad);
	} else if (*(buffer+1) ==  LINK_SHUTDOWN_REQ_FROM_FIRMWARE) {
		/* Target wants to go to Shut Down Mode */
		/* InterfacePrepareForShutdown(ad); */
		if (ad->chip_id == BCS220_2 ||
			ad->chip_id == BCS220_2BC ||
			ad->chip_id == BCS250_BC ||
			ad->chip_id == BCS220_3) {

			rdmalt(ad, HPM_CONFIG_MSW, &reset_val, 4);
			reset_val |= (1<<17);
			wrmalt(ad, HPM_CONFIG_MSW, &reset_val, 4);
		}

		SendShutModeResponse(ad);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, MP_SHUTDOWN, DBG_LVL_ALL, "ShutDownModeResponse:Notification received: Sending the response(Ack/Nack)\n");
	}

	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, MP_SHUTDOWN, DBG_LVL_ALL, "<====\n");
}

void ResetCounters(struct bcm_mini_adapter *ad)
{
	beceem_protocol_reset(ad);
	ad->CurrNumRecvDescs = 0;
	ad->PrevNumRecvDescs = 0;
	ad->LinkUpStatus = 0;
	ad->LinkStatus = 0;
	atomic_set(&ad->cntrlpktCnt, 0);
	atomic_set(&ad->TotalPacketCount, 0);
	ad->fw_download_done = false;
	ad->LinkStatus = 0;
	ad->AutoLinkUp = false;
	ad->IdleMode = false;
	ad->bShutStatus = false;
}

struct bcm_classifier_rule *GetFragIPClsEntry(struct bcm_mini_adapter *ad, USHORT usIpIdentification, ULONG SrcIP)
{
	unsigned int index = 0;

	for (index = 0; index < MAX_FRAGMENTEDIP_CLASSIFICATION_ENTRIES; index++) {
		if ((ad->astFragmentedPktClassifierTable[index].bUsed) &&
			(ad->astFragmentedPktClassifierTable[index].usIpIdentification == usIpIdentification) &&
			(ad->astFragmentedPktClassifierTable[index].ulSrcIpAddress == SrcIP) &&
			!ad->astFragmentedPktClassifierTable[index].bOutOfOrderFragment)

			return ad->astFragmentedPktClassifierTable[index].pstMatchedClassifierEntry;
	}
	return NULL;
}

void AddFragIPClsEntry(struct bcm_mini_adapter *ad, struct bcm_fragmented_packet_info *psFragPktInfo)
{
	unsigned int index = 0;

	for (index = 0; index < MAX_FRAGMENTEDIP_CLASSIFICATION_ENTRIES; index++) {
		if (!ad->astFragmentedPktClassifierTable[index].bUsed) {
			memcpy(&ad->astFragmentedPktClassifierTable[index], psFragPktInfo, sizeof(struct bcm_fragmented_packet_info));
			break;
		}
	}
}

void DelFragIPClsEntry(struct bcm_mini_adapter *ad, USHORT usIpIdentification, ULONG SrcIp)
{
	unsigned int index = 0;

	for (index = 0; index < MAX_FRAGMENTEDIP_CLASSIFICATION_ENTRIES; index++) {
		if ((ad->astFragmentedPktClassifierTable[index].bUsed) &&
			(ad->astFragmentedPktClassifierTable[index].usIpIdentification == usIpIdentification) &&
			(ad->astFragmentedPktClassifierTable[index].ulSrcIpAddress == SrcIp))

			memset(&ad->astFragmentedPktClassifierTable[index], 0, sizeof(struct bcm_fragmented_packet_info));
	}
}

void update_per_cid_rx(struct bcm_mini_adapter *ad)
{
	unsigned int qindex = 0;

	if ((jiffies - ad->liDrainCalculated) < XSECONDS)
		return;

	for (qindex = 0; qindex < HiPriority; qindex++) {
		if (ad->PackInfo[qindex].ucDirection == 0) {
			ad->PackInfo[qindex].uiCurrentRxRate =
				(ad->PackInfo[qindex].uiCurrentRxRate +
					ad->PackInfo[qindex].uiThisPeriodRxBytes) / 2;

			ad->PackInfo[qindex].uiThisPeriodRxBytes = 0;
		} else {
			ad->PackInfo[qindex].uiCurrentDrainRate =
				(ad->PackInfo[qindex].uiCurrentDrainRate +
					ad->PackInfo[qindex].uiThisPeriodSentBytes) / 2;
			ad->PackInfo[qindex].uiThisPeriodSentBytes = 0;
		}
	}
	ad->liDrainCalculated = jiffies;
}

void update_per_sf_desc_cnts(struct bcm_mini_adapter *ad)
{
	int iIndex = 0;
	u32 uibuff[MAX_TARGET_DSX_BUFFERS];
	int bytes;

	if (!atomic_read(&ad->uiMBupdate))
		return;

	bytes = rdmaltWithLock(ad, TARGET_SFID_TXDESC_MAP_LOC, (unsigned int *)uibuff, sizeof(unsigned int) * MAX_TARGET_DSX_BUFFERS);
	if (bytes < 0) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_PRINTK, 0, 0, "rdm failed\n");
		return;
	}

	for (iIndex = 0; iIndex < HiPriority; iIndex++) {
		if (ad->PackInfo[iIndex].bValid && ad->PackInfo[iIndex].ucDirection) {
			if (ad->PackInfo[iIndex].usVCID_Value < MAX_TARGET_DSX_BUFFERS)
				atomic_set(&ad->PackInfo[iIndex].uiPerSFTxResourceCount, uibuff[ad->PackInfo[iIndex].usVCID_Value]);
			else
				BCM_DEBUG_PRINT(ad, DBG_TYPE_PRINTK, 0, 0, "Invalid VCID : %x\n", ad->PackInfo[iIndex].usVCID_Value);
		}
	}
	atomic_set(&ad->uiMBupdate, false);
}

void flush_queue(struct bcm_mini_adapter *ad, unsigned int iQIndex)
{
	struct sk_buff *PacketToDrop = NULL;
	struct net_device_stats *netstats = &ad->dev->stats;

	spin_lock_bh(&ad->PackInfo[iQIndex].SFQueueLock);

	while (ad->PackInfo[iQIndex].FirstTxQueue && atomic_read(&ad->TotalPacketCount)) {
		PacketToDrop = ad->PackInfo[iQIndex].FirstTxQueue;
		if (PacketToDrop && PacketToDrop->len) {
			netstats->tx_dropped++;
			DEQUEUEPACKET(ad->PackInfo[iQIndex].FirstTxQueue, ad->PackInfo[iQIndex].LastTxQueue);
			ad->PackInfo[iQIndex].uiCurrentPacketsOnHost--;
			ad->PackInfo[iQIndex].uiCurrentBytesOnHost -= PacketToDrop->len;

			/* Adding dropped statistics */
			ad->PackInfo[iQIndex].uiDroppedCountBytes += PacketToDrop->len;
			ad->PackInfo[iQIndex].uiDroppedCountPackets++;
			dev_kfree_skb(PacketToDrop);
			atomic_dec(&ad->TotalPacketCount);
		}
	}
	spin_unlock_bh(&ad->PackInfo[iQIndex].SFQueueLock);
}

static void beceem_protocol_reset(struct bcm_mini_adapter *ad)
{
	int i;

	if (netif_msg_link(ad))
		pr_notice(PFX "%s: protocol reset\n", ad->dev->name);

	netif_carrier_off(ad->dev);
	netif_stop_queue(ad->dev);

	ad->IdleMode = false;
	ad->LinkUpStatus = false;
	ClearTargetDSXBuffer(ad, 0, TRUE);
	/* Delete All Classifier Rules */

	for (i = 0; i < HiPriority; i++)
		DeleteAllClassifiersForSF(ad, i);

	flush_all_queues(ad);

	if (ad->TimerActive == TRUE)
		ad->TimerActive = false;

	memset(ad->astFragmentedPktClassifierTable, 0, sizeof(struct bcm_fragmented_packet_info) * MAX_FRAGMENTEDIP_CLASSIFICATION_ENTRIES);

	for (i = 0; i < HiPriority; i++) {
		/* resetting only the first size (S_MIBS_SERVICEFLOW_TABLE) for the SF. */
		/* It is same between MIBs and SF. */
		memset(&ad->PackInfo[i].stMibsExtServiceFlowTable, 0, sizeof(struct bcm_mibs_parameters));
	}
}
