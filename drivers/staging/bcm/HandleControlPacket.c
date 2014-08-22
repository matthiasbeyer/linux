/**
 * @file HandleControlPacket.c
 * This file contains the routines to deal with
 * sending and receiving of control packets.
 */
#include "headers.h"

/**
 * When a control packet is received, analyze the
 * "status" and call appropriate response function.
 * Enqueue the control packet for Application.
 * @return None
 */
static VOID handle_rx_control_packet(struct bcm_mini_adapter *ad,
				     struct sk_buff *skb)
{
	struct bcm_tarang_data *tarang = NULL;
	bool high_prio_msg = false;
	struct sk_buff *new_packet = NULL;
	CHAR cntrl_msg_mask_bit = 0;
	bool drop_pkt_flag = TRUE;
	USHORT status = *(PUSHORT)(skb->data);

	if (netif_msg_pktdata(ad))
		print_hex_dump(KERN_DEBUG, PFX "rx control: ", DUMP_PREFIX_NONE,
			       16, 1, skb->data, skb->len, 0);

	switch (status) {
	case CM_RESPONSES:               /* 0xA0 */
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CP_CTRL_PKT,
			DBG_LVL_ALL,
			"MAC Version Seems to be Non Multi-Classifier, rejected by Driver");
		high_prio_msg = TRUE;
		break;
	case CM_CONTROL_NEWDSX_MULTICLASSIFIER_RESP:
		high_prio_msg = TRUE;
		if (ad->LinkStatus == LINKUP_DONE)
			CmControlResponseMessage(ad,
				(skb->data + sizeof(USHORT)));
		break;
	case LINK_CONTROL_RESP:          /* 0xA2 */
	case STATUS_RSP:                 /* 0xA1 */
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CP_CTRL_PKT,
			DBG_LVL_ALL, "LINK_CONTROL_RESP");
		high_prio_msg = TRUE;
		LinkControlResponseMessage(ad,
			(skb->data + sizeof(USHORT)));
		break;
	case STATS_POINTER_RESP:         /* 0xA6 */
		high_prio_msg = TRUE;
		StatisticsResponse(ad, (skb->data + sizeof(USHORT)));
		break;
	case IDLE_MODE_STATUS:           /* 0xA3 */
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CP_CTRL_PKT,
			DBG_LVL_ALL,
			"IDLE_MODE_STATUS Type Message Got from F/W");
		InterfaceIdleModeRespond(ad, (PUINT)(skb->data +
					sizeof(USHORT)));
		high_prio_msg = TRUE;
		break;

	case AUTH_SS_HOST_MSG:
		high_prio_msg = TRUE;
		break;

	default:
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CP_CTRL_PKT,
			DBG_LVL_ALL, "Got Default Response");
		/* Let the Application Deal with This Packet */
		break;
	}

	/* Queue The Control Packet to The Application Queues */
	down(&ad->RxAppControlQueuelock);

	for (tarang = ad->pTarangs; tarang; tarang = tarang->next) {
		if (ad->device_removed)
			break;

		drop_pkt_flag = TRUE;
		/*
		 * There are cntrl msg from A0 to AC. It has been mapped to 0 to
		 * C bit in the cntrl mask.
		 * Also, by default AD to BF has been masked to the rest of the
		 * bits... which wil be ON by default.
		 * if mask bit is enable to particular pkt status, send it out
		 * to app else stop it.
		 */
		cntrl_msg_mask_bit = (status & 0x1F);
		/*
		 * printk("\ninew  msg  mask bit which is disable in mask:%X",
		 *	cntrl_msg_mask_bit);
		 */
		if (tarang->RxCntrlMsgBitMask & (1 << cntrl_msg_mask_bit))
			drop_pkt_flag = false;

		if ((drop_pkt_flag == TRUE) ||
				(tarang->AppCtrlQueueLen > MAX_APP_QUEUE_LEN)
				|| ((tarang->AppCtrlQueueLen >
					MAX_APP_QUEUE_LEN / 2) &&
				    (high_prio_msg == false))) {
			/*
			 * Assumption:-
			 * 1. every tarang manages it own dropped pkt
			 *    statitistics
			 * 2. Total packet dropped per tarang will be equal to
			 *    the sum of all types of dropped pkt by that
			 *    tarang only.
			 */
			struct bcm_mibs_dropped_cntrl_msg *msg =
				&tarang->stDroppedAppCntrlMsgs;
			switch (*(PUSHORT)skb->data) {
			case CM_RESPONSES:
				msg->cm_responses++;
				break;
			case CM_CONTROL_NEWDSX_MULTICLASSIFIER_RESP:
				msg->cm_control_newdsx_multiclassifier_resp++;
				break;
			case LINK_CONTROL_RESP:
				msg->link_control_resp++;
				break;
			case STATUS_RSP:
				msg->status_rsp++;
				break;
			case STATS_POINTER_RESP:
				msg->stats_pointer_resp++;
				break;
			case IDLE_MODE_STATUS:
				msg->idle_mode_status++;
				break;
			case AUTH_SS_HOST_MSG:
				msg->auth_ss_host_msg++;
				break;
			default:
				msg->low_priority_message++;
				break;
			}

			continue;
		}

		new_packet = skb_clone(skb, GFP_KERNEL);
		if (!new_packet)
			break;
		ENQUEUEPACKET(tarang->RxAppControlHead,
				tarang->RxAppControlTail, new_packet);
		tarang->AppCtrlQueueLen++;
	}
	up(&ad->RxAppControlQueuelock);
	wake_up(&ad->process_read_wait_queue);
	dev_kfree_skb(skb);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CP_CTRL_PKT, DBG_LVL_ALL,
			"After wake_up_interruptible");
}

/**
 * @ingroup ctrl_pkt_functions
 * Thread to handle control pkt reception
 */
int control_packet_handler(struct bcm_mini_adapter *ad /* pointer to adapter object*/)
{
	struct sk_buff *ctrl_packet = NULL;
	unsigned long flags = 0;
	/* struct timeval tv; */
	/* int *puiBuffer = NULL; */
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CP_CTRL_PKT, DBG_LVL_ALL,
		"Entering to make thread wait on control packet event!");
	while (1) {
		wait_event_interruptible(ad->process_rx_cntrlpkt,
			atomic_read(&ad->cntrlpktCnt) ||
			ad->bWakeUpDevice ||
			kthread_should_stop());


		if (kthread_should_stop()) {
			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CP_CTRL_PKT,
				DBG_LVL_ALL, "Exiting\n");
			return 0;
		}
		if (TRUE == ad->bWakeUpDevice) {
			ad->bWakeUpDevice = false;
			if ((false == ad->bTriedToWakeUpFromlowPowerMode)
					&& ((TRUE == ad->IdleMode) ||
					    (TRUE == ad->bShutStatus))) {
				BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS,
					CP_CTRL_PKT, DBG_LVL_ALL,
					"Calling InterfaceAbortIdlemode\n");
				/*
				 * ad->bTriedToWakeUpFromlowPowerMode
				 *					= TRUE;
				 */
				InterfaceIdleModeWakeup(ad);
			}
			continue;
		}

		while (atomic_read(&ad->cntrlpktCnt)) {
			spin_lock_irqsave(&ad->control_queue_lock, flags);
			ctrl_packet = ad->RxControlHead;
			if (ctrl_packet) {
				DEQUEUEPACKET(ad->RxControlHead,
					ad->RxControlTail);
				/* ad->RxControlHead=ctrl_packet->next; */
			}

			spin_unlock_irqrestore(&ad->control_queue_lock,
						flags);
			handle_rx_control_packet(ad, ctrl_packet);
			atomic_dec(&ad->cntrlpktCnt);
		}

		SetUpTargetDsxBuffers(ad);
	}
	return STATUS_SUCCESS;
}

INT flushAllAppQ(void)
{
	struct bcm_mini_adapter *ad = GET_BCM_ADAPTER(gblpnetdev);
	struct bcm_tarang_data *tarang = NULL;
	struct sk_buff *drop_packet = NULL;

	for (tarang = ad->pTarangs; tarang; tarang = tarang->next) {
		while (tarang->RxAppControlHead != NULL) {
			drop_packet = tarang->RxAppControlHead;
			DEQUEUEPACKET(tarang->RxAppControlHead,
					tarang->RxAppControlTail);
			dev_kfree_skb(drop_packet);
		}
		tarang->AppCtrlQueueLen = 0;
		/* dropped contrl packet statistics also should be reset. */
		memset((PVOID)&tarang->stDroppedAppCntrlMsgs, 0,
			sizeof(struct bcm_mibs_dropped_cntrl_msg));

	}
	return STATUS_SUCCESS;
}


