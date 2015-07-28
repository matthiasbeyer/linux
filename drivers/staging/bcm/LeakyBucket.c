/**********************************************************************
* 			LEAKYBUCKET.C
*	This file contains the routines related to Leaky Bucket Algorithm.
***********************************************************************/
#include "headers.h"

/*********************************************************************
* Function    - UpdateTokenCount()
*
* Description - This function calculates the token count for each
*				channel and updates the same in Adapter strucuture.
*
* Parameters  - ad: Pointer to the Adapter structure.
*
* Returns     - None
**********************************************************************/

static VOID UpdateTokenCount(register struct bcm_mini_adapter *ad)
{
	ULONG curr_time;
	INT i = 0;
	ktime_t tv;
	struct bcm_packet_info *curr_pi;

	BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TOKEN_COUNTS, DBG_LVL_ALL, "=====>\n");
	if (NULL == ad) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TOKEN_COUNTS, DBG_LVL_ALL,
				"Adapter found NULL!\n");
		return;
	}

	tv = ktime_get_raw();
	for (i = 0; i < NO_OF_QUEUES; i++) {
		curr_pi = &ad->PackInfo[i];

		if (TRUE == curr_pi->bValid && (1 == curr_pi->ucDirection)) {
			ktime_t curr_ktime = ktime_sub(tv, curr_pi->stLastUpdateTokenAt);
			curr_time = ktime_to_ns(curr_ktime)/1000/1000; // ns to ms
			if (0 != curr_time) {
				curr_pi->uiCurrentTokenCount += (ULONG)
					((curr_pi->uiMaxAllowedRate) *
					((ULONG)((curr_time)))/1000);
				memcpy(&curr_pi->stLastUpdateTokenAt, &tv,
				       sizeof(ktime_t));
				curr_pi->liLastUpdateTokenAt = curr_time;
				if (curr_pi->uiCurrentTokenCount >=
				    curr_pi->uiMaxBucketSize) {
					curr_pi->uiCurrentTokenCount =
						curr_pi->uiMaxBucketSize;
				}
			}
		}
	}
	BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TOKEN_COUNTS, DBG_LVL_ALL, "<=====\n");
}


/*********************************************************************
* Function    - IsPacketAllowedForFlow()
*
* Description - This function checks whether the given packet from the
*				specified queue can be allowed for transmission by
*				checking the token count.
*
* Parameters  - ad	      :	Pointer to the Adpater structure.
* 			  - iQIndex	      :	The queue Identifier.
* 			  - ulPacketLength:	Number of bytes to be transmitted.
*
* Returns     - The number of bytes allowed for transmission.
*
***********************************************************************/
static ULONG GetSFTokenCount(struct bcm_mini_adapter *ad, struct bcm_packet_info *sf)
{
	BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TOKEN_COUNTS, DBG_LVL_ALL,
			"IsPacketAllowedForFlow ===>");

	/* Validate the parameters */
	if (NULL == ad || (sf < ad->PackInfo &&
	    (uintptr_t)sf > (uintptr_t) &ad->PackInfo[HiPriority])) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TOKEN_COUNTS, DBG_LVL_ALL,
				"IPAFF: Got wrong Parameters:Adapter: %p, QIndex: %zd\n",
				ad, (sf-ad->PackInfo));
		return 0;
	}

	if (false != sf->bValid && sf->ucDirection) {
		if (0 != sf->uiCurrentTokenCount) {
			return sf->uiCurrentTokenCount;
		} else {
			BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TOKEN_COUNTS,
					DBG_LVL_ALL,
					"Not enough tokens in queue %zd Available %u\n",
					sf-ad->PackInfo, sf->uiCurrentTokenCount);
			sf->uiPendedLast = 1;
		}
	} else {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TOKEN_COUNTS, DBG_LVL_ALL,
				"IPAFF: Queue %zd not valid\n",
				sf-ad->PackInfo);
	}
	BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TOKEN_COUNTS, DBG_LVL_ALL,
			"IsPacketAllowedForFlow <===");
	return 0;
}

/**
@ingroup tx_functions
This function despatches packet from the specified queue.
@return Zero(success) or Negative value(failure)
*/
static INT SendPacketFromQueue(struct bcm_mini_adapter *ad,/**<Logical Adapter*/
			       struct bcm_packet_info *sf, /**<Queue identifier*/
			       struct sk_buff *packet)	/**<Pointer to the packet to be sent*/
{
	INT status = STATUS_FAILURE;
	UINT i = 0, pkt_len = 0;

	BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, SEND_QUEUE, DBG_LVL_ALL, "=====>");
	if (!ad || !packet || !sf) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, SEND_QUEUE, DBG_LVL_ALL,
				"Got NULL Adapter or Packet");
		return -EINVAL;
	}

	if (sf->liDrainCalculated == 0)
		sf->liDrainCalculated = jiffies;
	/* send the packet to the fifo.. */
	pkt_len = packet->len;
	status = SetupNextSend(ad, packet, sf->usVCID_Value);
	if (status == 0) {
		for (i = 0; i < MIBS_MAX_HIST_ENTRIES; i++) {
			if ((pkt_len <= MIBS_PKTSIZEHIST_RANGE*(i+1)) &&
			    (pkt_len > MIBS_PKTSIZEHIST_RANGE*(i)))
				ad->aTxPktSizeHist[i]++;
		}
	}
	BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, SEND_QUEUE, DBG_LVL_ALL, "<=====");
	return status;
}

static void get_data_packet(struct bcm_mini_adapter *ad,
			    struct bcm_packet_info *ps_sf)
{
	int packet_len;
	struct sk_buff *qpacket;

	if (!ps_sf->ucDirection)
		return;

	BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_PACKETS, DBG_LVL_ALL,
			"UpdateTokenCount ");
	if (ad->IdleMode || ad->bPreparingForLowPowerMode)
		return; /* in idle mode */

	/* Check for Free Descriptors */
	if (atomic_read(&ad->CurrNumFreeTxDesc) <=
	    MINIMUM_PENDING_DESCRIPTORS) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_PACKETS, DBG_LVL_ALL,
				" No Free Tx Descriptor(%d) is available for Data pkt..",
				atomic_read(&ad->CurrNumFreeTxDesc));
		return;
	}

	spin_lock_bh(&ps_sf->SFQueueLock);
	qpacket = ps_sf->FirstTxQueue;

	if (qpacket) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_PACKETS, DBG_LVL_ALL,
				"Dequeuing Data Packet");

		if (ps_sf->bEthCSSupport)
			packet_len = qpacket->len;
		else
			packet_len = qpacket->len - ETH_HLEN;

		packet_len <<= 3;
		if (packet_len <= GetSFTokenCount(ad, ps_sf)) {
			BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_PACKETS,
					DBG_LVL_ALL, "Allowed bytes %d",
					(packet_len >> 3));

			DEQUEUEPACKET(ps_sf->FirstTxQueue, ps_sf->LastTxQueue);
			ps_sf->uiCurrentBytesOnHost -= (qpacket->len);
			ps_sf->uiCurrentPacketsOnHost--;
				atomic_dec(&ad->TotalPacketCount);
			spin_unlock_bh(&ps_sf->SFQueueLock);

			SendPacketFromQueue(ad, ps_sf, qpacket);
			ps_sf->uiPendedLast = false;
		} else {
			BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_PACKETS,
					DBG_LVL_ALL, "For Queue: %zd\n",
					ps_sf - ad->PackInfo);
			BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_PACKETS,
					DBG_LVL_ALL,
					"\nAvailable Tokens = %d required = %d\n",
					ps_sf->uiCurrentTokenCount,
					packet_len);
			/*
			this part indicates that because of
			non-availability of the tokens
			pkt has not been send out hence setting the
			pending flag indicating the host to send it out
			first next iteration.
			*/
			ps_sf->uiPendedLast = TRUE;
			spin_unlock_bh(&ps_sf->SFQueueLock);
		}
	} else {
		spin_unlock_bh(&ps_sf->SFQueueLock);
	}
}

static void send_control_packet(struct bcm_mini_adapter *ad,
				struct bcm_packet_info *ps_sf)
{
	char *ctrl_packet = NULL;
	INT status = 0;

	if ((atomic_read(&ad->CurrNumFreeTxDesc) > 0) &&
	    (atomic_read(&ad->index_rd_txcntrlpkt) !=
	     atomic_read(&ad->index_wr_txcntrlpkt))) {
		ctrl_packet = ad->txctlpacket
		[(atomic_read(&ad->index_rd_txcntrlpkt)%MAX_CNTRL_PKTS)];
		if (ctrl_packet) {
			BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_PACKETS,
					DBG_LVL_ALL,
					"Sending Control packet");
			status = SendControlPacket(ad, ctrl_packet);
			if (STATUS_SUCCESS == status) {
				spin_lock_bh(&ps_sf->SFQueueLock);
				ps_sf->NumOfPacketsSent++;
				ps_sf->uiSentBytes += ((struct bcm_leader *)ctrl_packet)->PLength;
				ps_sf->uiSentPackets++;
				atomic_dec(&ad->TotalPacketCount);
				ps_sf->uiCurrentBytesOnHost -= ((struct bcm_leader *)ctrl_packet)->PLength;
				ps_sf->uiCurrentPacketsOnHost--;
				atomic_inc(&ad->index_rd_txcntrlpkt);
				spin_unlock_bh(&ps_sf->SFQueueLock);
			} else {
				BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_PACKETS,
						DBG_LVL_ALL,
						"SendControlPacket Failed\n");
			}
		} else {
			BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_PACKETS,
					DBG_LVL_ALL,
					" Control Pkt is not available, Indexing is wrong....");
		}
	}
}

/************************************************************************
* Function    - CheckAndSendPacketFromIndex()
*
* Description - This function dequeues the data/control packet from the
*				specified queue for transmission.
*
* Parameters  - ad : Pointer to the driver control structure.
* 			  - iQIndex : The queue Identifier.
*
* Returns     - None.
*
****************************************************************************/
static VOID CheckAndSendPacketFromIndex(struct bcm_mini_adapter *ad,
					struct bcm_packet_info *sf)
{
	BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_PACKETS, DBG_LVL_ALL,
			"%zd ====>", (sf-ad->PackInfo));
	if ((sf != &ad->PackInfo[HiPriority]) &&
	    ad->LinkUpStatus &&
	    atomic_read(&sf->uiPerSFTxResourceCount)) { /* Get data packet */

		get_data_packet(ad, sf);
	} else {
		send_control_packet(ad, sf);
	}
}


/*******************************************************************
* Function    - transmit_packets()
*
* Description - This function transmits the packets from different
*				queues, if free descriptors are available on target.
*
* Parameters  - ad:  Pointer to the Adapter structure.
*
* Returns     - None.
********************************************************************/
VOID transmit_packets(struct bcm_mini_adapter *ad)
{
	UINT prev_total_cnt = 0;
	int i = 0;

	bool exit_flag = TRUE;

	BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_PACKETS, DBG_LVL_ALL, "=====>");

	if (NULL == ad) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_PACKETS, DBG_LVL_ALL,
				"Got NULL Adapter");
		return;
	}
	if (ad->device_removed == TRUE) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_PACKETS, DBG_LVL_ALL,
				"Device removed");
		return;
	}

	BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_PACKETS, DBG_LVL_ALL,
			"\nUpdateTokenCount ====>\n");

	UpdateTokenCount(ad);

	BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_PACKETS, DBG_LVL_ALL,
			"\nPruneQueueAllSF ====>\n");

	PruneQueueAllSF(ad);

	prev_total_cnt = atomic_read(&ad->TotalPacketCount);

	for (i = HiPriority; i >= 0; i--) {
		if (!prev_total_cnt || (TRUE == ad->device_removed))
				break;

		if (ad->PackInfo[i].bValid &&
		    ad->PackInfo[i].uiPendedLast &&
		    ad->PackInfo[i].uiCurrentBytesOnHost) {
			BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_PACKETS,
					DBG_LVL_ALL,
					"Calling CheckAndSendPacketFromIndex..");
			CheckAndSendPacketFromIndex(ad, &ad->PackInfo[i]);
			prev_total_cnt--;
		}
	}

	while (prev_total_cnt > 0 && !ad->device_removed) {
		exit_flag = TRUE;
		/* second iteration to parse non-pending queues */
		for (i = HiPriority; i >= 0; i--) {
			if (!prev_total_cnt || (TRUE == ad->device_removed))
				break;

			if (ad->PackInfo[i].bValid &&
			    ad->PackInfo[i].uiCurrentBytesOnHost &&
			    !ad->PackInfo[i].uiPendedLast) {
				BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_PACKETS,
						DBG_LVL_ALL,
						"Calling CheckAndSendPacketFromIndex..");
				CheckAndSendPacketFromIndex(ad, &ad->PackInfo[i]);
				prev_total_cnt--;
				exit_flag = false;
			}
		}

		if (ad->IdleMode || ad->bPreparingForLowPowerMode) {
			BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_PACKETS,
					DBG_LVL_ALL, "In Idle Mode\n");
			break;
		}
		if (exit_flag == TRUE)
			break;
	} /* end of inner while loop */

	update_per_cid_rx(ad);
	ad->txtransmit_running = 0;
	BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, TX_PACKETS, DBG_LVL_ALL, "<======");
}
