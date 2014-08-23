/*
 * File Name: hostmibs.c
 *
 * Author: Beceem Communications Pvt. Ltd
 *
 * Abstract: This file contains the routines to copy the statistics used by
 * the driver to the Host MIBS structure and giving the same to Application.
 */

#include "headers.h"

INT ProcessGetHostMibs(struct bcm_mini_adapter *ad,
		       struct bcm_host_stats_mibs *host_mibs)
{
	struct bcm_phs_entry *service_flow_entry = NULL;
	struct bcm_phs_rule *phs_rule = NULL;
	struct bcm_phs_classifier_table *pstClassifierTable = NULL;
	struct bcm_phs_classifier_entry *pstClassifierRule = NULL;
	struct bcm_phs_extension *pDeviceExtension = &ad->stBCMPhsContext;
	struct bcm_mibs_host_info *host_info;
	UINT nClassifierIndex = 0;
	UINT nPhsTableIndex = 0;
	UINT nSfIndex = 0;
	UINT uiIndex = 0;

	if (pDeviceExtension == NULL) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, HOST_MIBS,
				DBG_LVL_ALL, "Invalid Device Extension\n");
		return STATUS_FAILURE;
	}

	/* Copy the classifier Table */
	for (nClassifierIndex = 0; nClassifierIndex < MAX_CLASSIFIERS;
							nClassifierIndex++) {
		if (ad->astClassifierTable[nClassifierIndex].bUsed == TRUE)
			memcpy(&host_mibs->astClassifierTable[nClassifierIndex],
			       &ad->astClassifierTable[nClassifierIndex],
			       sizeof(struct bcm_mibs_classifier_rule));
	}

	/* Copy the SF Table */
	for (nSfIndex = 0; nSfIndex < NO_OF_QUEUES; nSfIndex++) {
		if (ad->PackInfo[nSfIndex].bValid) {
			memcpy(&host_mibs->astSFtable[nSfIndex],
			       &ad->PackInfo[nSfIndex],
			       sizeof(struct bcm_mibs_table));
		} else {
			/* If index in not valid,
			 * don't process this for the PHS table.
			 * Go For the next entry.
			 */
			continue;
		}

		/* Retrieve the SFID Entry Index for requested Service Flow */
		if (PHS_INVALID_TABLE_INDEX ==
		    GetServiceFlowEntry(pDeviceExtension->
					pstServiceFlowPhsRulesTable,
					ad->PackInfo[nSfIndex].
					usVCID_Value, &service_flow_entry))

			continue;

		pstClassifierTable = service_flow_entry->pstClassifierTable;

		for (uiIndex = 0; uiIndex < MAX_PHSRULE_PER_SF; uiIndex++) {
			pstClassifierRule = &pstClassifierTable->stActivePhsRulesList[uiIndex];

			if (pstClassifierRule->bUsed) {
				phs_rule = pstClassifierRule->pstPhsRule;

				host_mibs->astPhsRulesTable[nPhsTableIndex].
				    ulSFID = ad->PackInfo[nSfIndex].ulSFID;

				memcpy(&host_mibs->astPhsRulesTable[nPhsTableIndex].u8PHSI,
				       &phs_rule->u8PHSI,
				       sizeof(struct bcm_phs_rule));
				nPhsTableIndex++;

			}

		}

	}

	/* Copy other Host Statistics parameters */
	host_info = &host_mibs->stHostInfo;
	host_info->GoodTransmits    = ad->dev->stats.tx_packets;
	host_info->GoodReceives	    = ad->dev->stats.rx_packets;
	host_info->CurrNumFreeDesc  = atomic_read(&ad->CurrNumFreeTxDesc);
	host_info->BEBucketSize	    = ad->BEBucketSize;
	host_info->rtPSBucketSize   = ad->rtPSBucketSize;
	host_info->TimerActive	    = ad->TimerActive;
	host_info->u32TotalDSD	    = ad->u32TotalDSD;

	memcpy(host_info->aTxPktSizeHist, ad->aTxPktSizeHist,
	       sizeof(UINT32) * MIBS_MAX_HIST_ENTRIES);
	memcpy(host_info->aRxPktSizeHist, ad->aRxPktSizeHist,
	       sizeof(UINT32) * MIBS_MAX_HIST_ENTRIES);

	return STATUS_SUCCESS;
}

VOID GetDroppedAppCntrlPktMibs(struct bcm_host_stats_mibs *host_mibs,
			       struct bcm_tarang_data *pTarang)
{
	memcpy(&(host_mibs->stDroppedAppCntrlMsgs),
	       &(pTarang->stDroppedAppCntrlMsgs),
	       sizeof(struct bcm_mibs_dropped_cntrl_msg));
}

VOID CopyMIBSExtendedSFParameters(struct bcm_mini_adapter *ad,
				  struct bcm_connect_mgr_params *psfLocalSet,
				  UINT uiSearchRuleIndex)
{
	struct bcm_mibs_parameters *t =
		&ad->PackInfo[uiSearchRuleIndex].stMibsExtServiceFlowTable;

	t->wmanIfSfid = psfLocalSet->u32SFID;
	t->wmanIfCmnCpsMaxSustainedRate =
		psfLocalSet->u32MaxSustainedTrafficRate;
	t->wmanIfCmnCpsMaxTrafficBurst = psfLocalSet->u32MaxTrafficBurst;
	t->wmanIfCmnCpsMinReservedRate = psfLocalSet->u32MinReservedTrafficRate;
	t->wmanIfCmnCpsToleratedJitter = psfLocalSet->u32ToleratedJitter;
	t->wmanIfCmnCpsMaxLatency = psfLocalSet->u32MaximumLatency;
	t->wmanIfCmnCpsFixedVsVariableSduInd =
		psfLocalSet->u8FixedLengthVSVariableLengthSDUIndicator;
	t->wmanIfCmnCpsFixedVsVariableSduInd =
		ntohl(t->wmanIfCmnCpsFixedVsVariableSduInd);
	t->wmanIfCmnCpsSduSize = psfLocalSet->u8SDUSize;
	t->wmanIfCmnCpsSduSize = ntohl(t->wmanIfCmnCpsSduSize);
	t->wmanIfCmnCpsSfSchedulingType =
		psfLocalSet->u8ServiceFlowSchedulingType;
	t->wmanIfCmnCpsSfSchedulingType =
		ntohl(t->wmanIfCmnCpsSfSchedulingType);
	t->wmanIfCmnCpsArqEnable = psfLocalSet->u8ARQEnable;
	t->wmanIfCmnCpsArqEnable = ntohl(t->wmanIfCmnCpsArqEnable);
	t->wmanIfCmnCpsArqWindowSize = ntohs(psfLocalSet->u16ARQWindowSize);
	t->wmanIfCmnCpsArqWindowSize = ntohl(t->wmanIfCmnCpsArqWindowSize);
	t->wmanIfCmnCpsArqBlockLifetime =
		ntohs(psfLocalSet->u16ARQBlockLifeTime);
	t->wmanIfCmnCpsArqBlockLifetime =
		ntohl(t->wmanIfCmnCpsArqBlockLifetime);
	t->wmanIfCmnCpsArqSyncLossTimeout =
		ntohs(psfLocalSet->u16ARQSyncLossTimeOut);
	t->wmanIfCmnCpsArqSyncLossTimeout =
		ntohl(t->wmanIfCmnCpsArqSyncLossTimeout);
	t->wmanIfCmnCpsArqDeliverInOrder = psfLocalSet->u8ARQDeliverInOrder;
	t->wmanIfCmnCpsArqDeliverInOrder =
		ntohl(t->wmanIfCmnCpsArqDeliverInOrder);
	t->wmanIfCmnCpsArqRxPurgeTimeout =
		ntohs(psfLocalSet->u16ARQRxPurgeTimeOut);
	t->wmanIfCmnCpsArqRxPurgeTimeout =
		ntohl(t->wmanIfCmnCpsArqRxPurgeTimeout);
	t->wmanIfCmnCpsArqBlockSize = ntohs(psfLocalSet->u16ARQBlockSize);
	t->wmanIfCmnCpsArqBlockSize = ntohl(t->wmanIfCmnCpsArqBlockSize);
	t->wmanIfCmnCpsReqTxPolicy = psfLocalSet->u8RequesttransmissionPolicy;
	t->wmanIfCmnCpsReqTxPolicy = ntohl(t->wmanIfCmnCpsReqTxPolicy);
	t->wmanIfCmnSfCsSpecification = psfLocalSet->u8CSSpecification;
	t->wmanIfCmnSfCsSpecification = ntohl(t->wmanIfCmnSfCsSpecification);
	t->wmanIfCmnCpsTargetSaid = ntohs(psfLocalSet->u16TargetSAID);
	t->wmanIfCmnCpsTargetSaid = ntohl(t->wmanIfCmnCpsTargetSaid);

}
