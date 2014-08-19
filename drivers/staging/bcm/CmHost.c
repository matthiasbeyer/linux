/************************************************************
 * CMHOST.C
 * This file contains the routines for handling Connection
 * Management.
 ************************************************************/

#include "headers.h"

enum E_CLASSIFIER_ACTION {
	eInvalidClassifierAction,
	eAddClassifier,
	eReplaceClassifier,
	eDeleteClassifier
};

static ULONG GetNextTargetBufferLocation(struct bcm_mini_adapter *ad,
		B_UINT16 tid);
static void restore_endianess_of_classifier_entry(
		struct bcm_classifier_rule *classifier_entry,
		enum bcm_ipaddr_context ip_addr_context);

static void apply_phs_rule_to_all_classifiers(
		register struct bcm_mini_adapter *ad,
		register UINT search_rule_idx,
		USHORT vcid,
		struct bcm_phs_rule *phs_rule,
		struct bcm_phs_rules *c_phs_rules,
		struct bcm_add_indication_alt *add_indication);

/************************************************************
 * Function - SearchSfid
 *
 * Description - This routinue would search QOS queues having
 *  specified SFID as input parameter.
 *
 * Parameters -	ad: Pointer to the ad structure
 *  sf_id : Given SFID for matching
 *
 * Returns - Queue index for this SFID(If matched)
 *  Else Invalid Queue Index(If Not matched)
 ************************************************************/
int SearchSfid(struct bcm_mini_adapter *ad, UINT sf_id)
{
	int i;

	for (i = (NO_OF_QUEUES-1); i >= 0; i--)
		if (ad->PackInfo[i].ulSFID == sf_id)
			return i;

	return NO_OF_QUEUES+1;
}

/***************************************************************
 * Function -SearchFreeSfid
 *
 * Description - This routinue would search Free available SFID.
 *
 * Parameter - ad: Pointer to the ad structure
 *
 * Returns - Queue index for the free SFID
 *  Else returns Invalid Index.
 ****************************************************************/
static int SearchFreeSfid(struct bcm_mini_adapter *ad)
{
	int i;

	for (i = 0; i < (NO_OF_QUEUES-1); i++)
		if (ad->PackInfo[i].ulSFID == 0)
			return i;

	return NO_OF_QUEUES+1;
}

/*
 * Function: SearchClsid
 * Description:	This routinue would search Classifier  having specified ClassifierID as input parameter
 * Input parameters: struct bcm_mini_adapter *ad - ad Context
 *  unsigned int uiSfid   - The SF in which the classifier is to searched
 *  B_UINT16  classifier_id - The classifier ID to be searched
 * Return: int :Classifier table index of matching entry
 */
static int SearchClsid(struct bcm_mini_adapter *ad,
		ULONG sf_id,
		B_UINT16 classifier_id)
{
	int i;

	for (i = 0; i < MAX_CLASSIFIERS; i++) {
		if ((ad->astClassifierTable[i].bUsed) &&
			(ad->astClassifierTable[i].uiClassifierRuleIndex
				== classifier_id) &&
			(ad->astClassifierTable[i].ulSFID == sf_id))
			return i;
	}

	return MAX_CLASSIFIERS+1;
}

/*
 * @ingroup ctrl_pkt_functions
 * This routinue would search Free available Classifier entry in classifier table.
 * @return free Classifier Entry index in classifier table for specified SF
 */
static int SearchFreeClsid(struct bcm_mini_adapter *ad /**ad Context*/)
{
	int i;

	for (i = 0; i < MAX_CLASSIFIERS; i++) {
		if (!ad->astClassifierTable[i].bUsed)
			return i;
	}

	return MAX_CLASSIFIERS+1;
}

static VOID deleteSFBySfid(struct bcm_mini_adapter *ad,
		UINT search_rule_idx)
{
	/* deleting all the packet held in the SF */
	flush_queue(ad, search_rule_idx);

	/* Deleting the all classifiers for this SF */
	DeleteAllClassifiersForSF(ad, search_rule_idx);

	/* Resetting only MIBS related entries in the SF */
	memset((PVOID)&ad->PackInfo[search_rule_idx], 0,
			sizeof(struct bcm_mibs_table));
}

static inline VOID
CopyIpAddrToClassifier(struct bcm_classifier_rule *classifier_entry,
		B_UINT8 ip_addr_len, B_UINT8 *ip_addr_mask_src,
		bool ip_v6, enum bcm_ipaddr_context ip_addr_context)
{
	int i = 0;
	UINT ip_addr_byte_len = IP_LENGTH_OF_ADDRESS;
	UCHAR *classifier_ip_addr = NULL;
	UCHAR *classifier_ip_mask = NULL;
	struct bcm_mini_adapter *ad = GET_BCM_ADAPTER(gblpnetdev);

	if (ip_v6)
		ip_addr_byte_len = IPV6_ADDRESS_SIZEINBYTES;

	/* Destination Ip Address */
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
			"Ip Address Range Length:0x%X ", ip_addr_len);
	if ((ip_v6 ? (IPV6_ADDRESS_SIZEINBYTES * MAX_IP_RANGE_LENGTH * 2) :
			(TOTAL_MASKED_ADDRESS_IN_BYTES)) >= ip_addr_len) {

		union u_ip_address *st_dest_ip =
			&classifier_entry->stDestIpAddress;

		union u_ip_address *st_src_ip =
			&classifier_entry->stSrcIpAddress;

		/*
		 * checking both the mask and address togethor in Classification.
		 * So length will be : TotalLengthInBytes/ip_addr_byte_len * 2
		 * (ip_addr_byte_len for address and ip_addr_byte_len for mask)
		 */
		if (ip_addr_context == eDestIpAddress) {
			classifier_entry->ucIPDestinationAddressLength =
				ip_addr_len/(ip_addr_byte_len * 2);
			if (ip_v6) {
				classifier_ip_addr =
					st_dest_ip->ucIpv6Address;
				classifier_ip_mask =
					st_dest_ip->ucIpv6Mask;
			} else {
				classifier_ip_addr =
					st_dest_ip->ucIpv4Address;
				classifier_ip_mask =
					st_dest_ip->ucIpv4Mask;
			}
		} else if (ip_addr_context == eSrcIpAddress) {
			classifier_entry->ucIPSourceAddressLength =
				ip_addr_len/(ip_addr_byte_len * 2);
			if (ip_v6) {
				classifier_ip_addr =
					st_src_ip->ucIpv6Address;
				classifier_ip_mask = st_src_ip->ucIpv6Mask;
			} else {
				classifier_ip_addr =
					st_src_ip->ucIpv4Address;
				classifier_ip_mask = st_src_ip->ucIpv4Mask;
			}
		}
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
				"Address Length:0x%X\n",
				classifier_entry->ucIPDestinationAddressLength);
		while ((ip_addr_len >= ip_addr_byte_len)
				&& (i < MAX_IP_RANGE_LENGTH)) {
			memcpy(classifier_ip_addr +
				(i * ip_addr_byte_len),
				(ip_addr_mask_src
					+ (i * ip_addr_byte_len * 2)),
				ip_addr_byte_len);

			if (!ip_v6) {
				if (ip_addr_context == eSrcIpAddress) {
					st_src_ip->ulIpv4Addr[i] =
						ntohl(st_src_ip->ulIpv4Addr[i]);
					BCM_DEBUG_PRINT(ad,
							DBG_TYPE_OTHERS,
							CONN_MSG,
							DBG_LVL_ALL,
							"Src Ip Address:0x%luX ",
							st_src_ip->ulIpv4Addr[i]);
				} else if (ip_addr_context == eDestIpAddress) {
					st_dest_ip->ulIpv4Addr[i] =
						ntohl(st_dest_ip->ulIpv4Addr[i]);
					BCM_DEBUG_PRINT(ad,
							DBG_TYPE_OTHERS,
							CONN_MSG,
							DBG_LVL_ALL,
							"Dest Ip Address:0x%luX ",
							st_dest_ip->ulIpv4Addr[i]);
				}
			}
			ip_addr_len -= ip_addr_byte_len;
			if (ip_addr_len >= ip_addr_byte_len) {
				memcpy(classifier_ip_mask +
					(i * ip_addr_byte_len),
					(ip_addr_mask_src
						+ ip_addr_byte_len
						+ (i * ip_addr_byte_len * 2)),
					ip_addr_byte_len);

				if (!ip_v6) {
					if (ip_addr_context == eSrcIpAddress) {
						st_src_ip->ulIpv4Mask[i] =
							ntohl(st_src_ip->ulIpv4Mask[i]);
						BCM_DEBUG_PRINT(ad,
								DBG_TYPE_OTHERS,
								CONN_MSG,
								DBG_LVL_ALL,
								"Src Ip Mask Address:0x%luX ",
								st_src_ip->ulIpv4Mask[i]);
					} else if (ip_addr_context == eDestIpAddress) {
						st_dest_ip->ulIpv4Mask[i] =
							ntohl(st_dest_ip->ulIpv4Mask[i]);
						BCM_DEBUG_PRINT(ad,
								DBG_TYPE_OTHERS,
								CONN_MSG,
								DBG_LVL_ALL,
								"Dest Ip Mask Address:0x%luX ",
								st_dest_ip->ulIpv4Mask[i]);
					}
				}
				ip_addr_len -= ip_addr_byte_len;
			}
			if (ip_addr_len == 0)
				classifier_entry->bDestIpValid = TRUE;

			i++;
		}
		if (ip_v6) {
			/* Restore EndianNess of Struct */
			restore_endianess_of_classifier_entry(
					classifier_entry,
					ip_addr_context
					);
		}
	}
}

void ClearTargetDSXBuffer(struct bcm_mini_adapter *ad, B_UINT16 TID, bool free_all)
{
	int i;
	struct bcm_targetdsx_buffer *curr_buf;

	for (i = 0; i < ad->ulTotalTargetBuffersAvailable; i++) {
		curr_buf = &ad->astTargetDsxBuffer[i];

		if (curr_buf->valid)
			continue;

		if ((free_all) || (curr_buf->tid == TID)) {
			BCM_DEBUG_PRINT(ad, DBG_TYPE_PRINTK, 0, 0,
					"ClearTargetDSXBuffer: found tid %d buffer cleared %lx\n",
					TID, curr_buf->ulTargetDsxBuffer);
			curr_buf->valid = 1;
			curr_buf->tid = 0;
			ad->ulFreeTargetBufferCnt++;
		}
	}
}

/*
 * @ingroup ctrl_pkt_functions
 * copy classifier rule into the specified SF index
 */
static inline VOID CopyClassifierRuleToSF(struct bcm_mini_adapter *ad,
		struct bcm_convergence_types *psfCSType,
		UINT search_rule_idx,
		UINT nClassifierIndex)
{
	struct bcm_classifier_rule *classifier_entry = NULL;
	/* VOID *pvPhsContext = NULL; */
	int i;
	/* UCHAR ucProtocolLength=0; */
	/* ULONG ulPhsStatus; */

	struct bcm_packet_class_rules *pack_class_rule =
		&psfCSType->cCPacketClassificationRule;

	if (ad->PackInfo[search_rule_idx].usVCID_Value == 0 ||
		nClassifierIndex > (MAX_CLASSIFIERS-1))
		return;

	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
			"Storing Classifier Rule Index : %X",
			ntohs(pack_class_rule->u16PacketClassificationRuleIndex));

	if (nClassifierIndex > MAX_CLASSIFIERS-1)
		return;

	classifier_entry = &ad->astClassifierTable[nClassifierIndex];
	if (classifier_entry) {
		/* Store if Ipv6 */
		classifier_entry->bIpv6Protocol =
			(ad->PackInfo[search_rule_idx].ucIpVersion == IPV6) ? TRUE : false;

		/* Destinaiton Port */
		classifier_entry->ucDestPortRangeLength =
			pack_class_rule->u8ProtocolDestPortRangeLength / 4;
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
				"Destination Port Range Length:0x%X ",
				classifier_entry->ucDestPortRangeLength);

		if (pack_class_rule->u8ProtocolDestPortRangeLength <= MAX_PORT_RANGE) {
			for (i = 0; i < (classifier_entry->ucDestPortRangeLength); i++) {
				classifier_entry->usDestPortRangeLo[i] =
					*((PUSHORT)(pack_class_rule->u8ProtocolDestPortRange+i));
				classifier_entry->usDestPortRangeHi[i] =
					*((PUSHORT)(pack_class_rule->u8ProtocolDestPortRange+2+i));
				classifier_entry->usDestPortRangeLo[i] =
					ntohs(classifier_entry->usDestPortRangeLo[i]);
				BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS,
						CONN_MSG, DBG_LVL_ALL,
						"Destination Port Range Lo:0x%X ",
						classifier_entry->usDestPortRangeLo[i]);
				classifier_entry->usDestPortRangeHi[i] =
					ntohs(classifier_entry->usDestPortRangeHi[i]);
			}
		} else {
			classifier_entry->ucDestPortRangeLength = 0;
		}

		/* Source Port */
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
				"Source Port Range Length:0x%X ",
				pack_class_rule->u8ProtocolSourcePortRangeLength);
		if (pack_class_rule->u8ProtocolSourcePortRangeLength <= MAX_PORT_RANGE) {
			classifier_entry->ucSrcPortRangeLength =
				pack_class_rule->u8ProtocolSourcePortRangeLength/4;
			for (i = 0; i < (classifier_entry->ucSrcPortRangeLength); i++) {
				classifier_entry->usSrcPortRangeLo[i] =
					*((PUSHORT)(pack_class_rule->
							u8ProtocolSourcePortRange+i));
				classifier_entry->usSrcPortRangeHi[i] =
					*((PUSHORT)(pack_class_rule->
							u8ProtocolSourcePortRange+2+i));
				classifier_entry->usSrcPortRangeLo[i] =
					ntohs(classifier_entry->usSrcPortRangeLo[i]);
				BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS,
						CONN_MSG, DBG_LVL_ALL,
						"Source Port Range Lo:0x%X ",
						classifier_entry->usSrcPortRangeLo[i]);
				classifier_entry->usSrcPortRangeHi[i] =
					ntohs(classifier_entry->usSrcPortRangeHi[i]);
			}
		}
		/* Destination Ip Address and Mask */
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
				"Ip Destination Parameters : ");
		CopyIpAddrToClassifier(classifier_entry,
				pack_class_rule->u8IPDestinationAddressLength,
				pack_class_rule->u8IPDestinationAddress,
				(ad->PackInfo[search_rule_idx].ucIpVersion == IPV6) ?
			TRUE : false, eDestIpAddress);

		/* Source Ip Address and Mask */
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
				"Ip Source Parameters : ");

		CopyIpAddrToClassifier(classifier_entry,
				pack_class_rule->u8IPMaskedSourceAddressLength,
				pack_class_rule->u8IPMaskedSourceAddress,
				(ad->PackInfo[search_rule_idx].ucIpVersion == IPV6) ? TRUE : false,
				eSrcIpAddress);

		/* TOS */
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
				"TOS Length:0x%X ",
				pack_class_rule->u8IPTypeOfServiceLength);
		if (pack_class_rule->u8IPTypeOfServiceLength == 3) {
			classifier_entry->ucIPTypeOfServiceLength =
				pack_class_rule->u8IPTypeOfServiceLength;
			classifier_entry->ucTosLow =
				pack_class_rule->u8IPTypeOfService[0];
			classifier_entry->ucTosHigh =
				pack_class_rule->u8IPTypeOfService[1];
			classifier_entry->ucTosMask =
				pack_class_rule->u8IPTypeOfService[2];
			classifier_entry->bTOSValid = TRUE;
		}
		if (pack_class_rule->u8Protocol == 0) {
			/* we didn't get protocol field filled in by the BS */
			classifier_entry->ucProtocolLength = 0;
		} else {
			classifier_entry->ucProtocolLength = 1; /* 1 valid protocol */
		}

		classifier_entry->ucProtocol[0] = pack_class_rule->u8Protocol;
		classifier_entry->u8ClassifierRulePriority =
			pack_class_rule->u8ClassifierRulePriority;

		/* store the classifier rule ID and set this classifier entry as valid */
		classifier_entry->ucDirection =
			ad->PackInfo[search_rule_idx].ucDirection;
		classifier_entry->uiClassifierRuleIndex =
			ntohs(pack_class_rule->u16PacketClassificationRuleIndex);
		classifier_entry->usVCID_Value =
			ad->PackInfo[search_rule_idx].usVCID_Value;
		classifier_entry->ulSFID =
			ad->PackInfo[search_rule_idx].ulSFID;
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
				"Search Index %d Dir: %d, Index: %d, Vcid: %d\n",
				search_rule_idx,
				classifier_entry->ucDirection,
				classifier_entry->uiClassifierRuleIndex,
				classifier_entry->usVCID_Value);

		if (pack_class_rule->u8AssociatedPHSI)
			classifier_entry->u8AssociatedPHSI =
				pack_class_rule->u8AssociatedPHSI;

		/* Copy ETH CS Parameters */
		classifier_entry->ucEthCSSrcMACLen =
			(pack_class_rule->u8EthernetSourceMACAddressLength);
		memcpy(classifier_entry->au8EThCSSrcMAC,
				pack_class_rule->u8EthernetSourceMACAddress,
				MAC_ADDRESS_SIZE);
		memcpy(classifier_entry->au8EThCSSrcMACMask,
				pack_class_rule->u8EthernetSourceMACAddress
				+ MAC_ADDRESS_SIZE, MAC_ADDRESS_SIZE);
		classifier_entry->ucEthCSDestMACLen =
			(pack_class_rule->u8EthernetDestMacAddressLength);
		memcpy(classifier_entry->au8EThCSDestMAC,
				pack_class_rule->u8EthernetDestMacAddress,
				MAC_ADDRESS_SIZE);
		memcpy(classifier_entry->au8EThCSDestMACMask,
				pack_class_rule->u8EthernetDestMacAddress
				+ MAC_ADDRESS_SIZE, MAC_ADDRESS_SIZE);
		classifier_entry->ucEtherTypeLen =
			(pack_class_rule->u8EthertypeLength);
		memcpy(classifier_entry->au8EthCSEtherType,
				pack_class_rule->u8Ethertype,
				NUM_ETHERTYPE_BYTES);
		memcpy(classifier_entry->usUserPriority,
				&pack_class_rule->u16UserPriority, 2);
		classifier_entry->usVLANID =
			ntohs(pack_class_rule->u16VLANID);
		classifier_entry->usValidityBitMap =
			ntohs(pack_class_rule->u16ValidityBitMap);

		classifier_entry->bUsed = TRUE;
	}
}

/*
 * @ingroup ctrl_pkt_functions
 */
static inline VOID DeleteClassifierRuleFromSF(struct bcm_mini_adapter *ad,
		UINT search_rule_idx, UINT nClassifierIndex)
{
	struct bcm_classifier_rule *classifier_entry = NULL;
	B_UINT16 u16PacketClassificationRuleIndex;
	USHORT usVCID;
	/* VOID *pvPhsContext = NULL; */
	/*ULONG ulPhsStatus; */

	usVCID = ad->PackInfo[search_rule_idx].usVCID_Value;

	if (nClassifierIndex > MAX_CLASSIFIERS-1)
		return;

	if (usVCID == 0)
		return;

	u16PacketClassificationRuleIndex =
		ad->astClassifierTable[nClassifierIndex].uiClassifierRuleIndex;
	classifier_entry = &ad->astClassifierTable[nClassifierIndex];
	if (classifier_entry) {
		classifier_entry->bUsed = false;
		classifier_entry->uiClassifierRuleIndex = 0;
		memset(classifier_entry, 0,
				sizeof(struct bcm_classifier_rule));

		/* Delete the PHS Rule for this classifier */
		PhsDeleteClassifierRule(&ad->stBCMPhsContext, usVCID,
				u16PacketClassificationRuleIndex);
	}
}

/*
 * @ingroup ctrl_pkt_functions
 */
VOID DeleteAllClassifiersForSF(struct bcm_mini_adapter *ad,
		UINT search_rule_idx)
{
	struct bcm_classifier_rule *classifier_entry = NULL;
	int i;
	/* B_UINT16  u16PacketClassificationRuleIndex; */
	USHORT ulVCID;
	/* VOID *pvPhsContext = NULL; */
	/* ULONG ulPhsStatus; */

	ulVCID = ad->PackInfo[search_rule_idx].usVCID_Value;

	if (ulVCID == 0)
		return;

	for (i = 0; i < MAX_CLASSIFIERS; i++) {
		if (ad->astClassifierTable[i].usVCID_Value == ulVCID) {
			classifier_entry = &ad->astClassifierTable[i];

			if (classifier_entry->bUsed)
				DeleteClassifierRuleFromSF(ad,
						search_rule_idx, i);
		}
	}

	/* Delete All Phs Rules Associated with this SF */
	PhsDeleteSFRules(&ad->stBCMPhsContext, ulVCID);
}

/*
 * This routinue  copies the Connection Management
 * related data into the ad structure.
 * @ingroup ctrl_pkt_functions
 */
static VOID CopyToAdapter(register struct bcm_mini_adapter *ad, /* <Pointer to the ad structure */
			register struct bcm_connect_mgr_params *psfLocalSet, /* Pointer to the connection manager parameters structure */
			register UINT search_rule_idx, /* <Index of Queue, to which this data belongs */
			register UCHAR ucDsxType,
			struct bcm_add_indication_alt *add_indication) {

	/* UCHAR ucProtocolLength = 0; */
	ULONG sf_id;
	UINT nClassifierIndex = 0;
	enum E_CLASSIFIER_ACTION eClassifierAction = eInvalidClassifierAction;
	B_UINT16 u16PacketClassificationRuleIndex = 0;
	int i;
	struct bcm_convergence_types *psfCSType = NULL;
	struct bcm_phs_rule phs_rule;
	struct bcm_packet_info *curr_packinfo =
		&ad->PackInfo[search_rule_idx];
	USHORT vcid = curr_packinfo->usVCID_Value;
	UINT UGIValue = 0;

	curr_packinfo->bValid = TRUE;
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
			"Search Rule Index = %d\n", search_rule_idx);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
			"%s: SFID= %x ", __func__, ntohl(psfLocalSet->u32SFID));
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
			"Updating Queue %d", search_rule_idx);

	sf_id = ntohl(psfLocalSet->u32SFID);
	/* Store IP Version used */
	/* Get The Version Of IP used (IPv6 or IPv4) from CSSpecification field of SF */

	curr_packinfo->bIPCSSupport = 0;
	curr_packinfo->bEthCSSupport = 0;

	/* Enable IP/ETh CS Support As Required */
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
			"CopyToad : u8CSSpecification : %X\n",
			psfLocalSet->u8CSSpecification);
	switch (psfLocalSet->u8CSSpecification) {
	case eCSPacketIPV4:
		curr_packinfo->bIPCSSupport = IPV4_CS;
		break;
	case eCSPacketIPV6:
		curr_packinfo->bIPCSSupport = IPV6_CS;
		break;
	case eCS802_3PacketEthernet:
	case eCS802_1QPacketVLAN:
		curr_packinfo->bEthCSSupport = ETH_CS_802_3;
		break;
	case eCSPacketIPV4Over802_1QVLAN:
	case eCSPacketIPV4Over802_3Ethernet:
		curr_packinfo->bIPCSSupport = IPV4_CS;
		curr_packinfo->bEthCSSupport = ETH_CS_802_3;
		break;
	case eCSPacketIPV6Over802_1QVLAN:
	case eCSPacketIPV6Over802_3Ethernet:
		curr_packinfo->bIPCSSupport = IPV6_CS;
		curr_packinfo->bEthCSSupport = ETH_CS_802_3;
		break;
	default:
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
				"Error in value of CS Classification.. setting default to IP CS\n");
		curr_packinfo->bIPCSSupport = IPV4_CS;
		break;
	}

	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
			"CopyToad : Queue No : %X ETH CS Support :  %X  , IP CS Support : %X\n",
			search_rule_idx,
			curr_packinfo->bEthCSSupport,
			curr_packinfo->bIPCSSupport);

	/* Store IP Version used */
	/* Get The Version Of IP used (IPv6 or IPv4) from CSSpecification field of SF */
	if (curr_packinfo->bIPCSSupport == IPV6_CS)
		curr_packinfo->ucIpVersion = IPV6;
	else
		curr_packinfo->ucIpVersion = IPV4;

	/* To ensure that the ETH CS code doesn't gets executed if the BS doesn't supports ETH CS */
	if (!ad->bETHCSEnabled)
		curr_packinfo->bEthCSSupport = 0;

	if (psfLocalSet->u8ServiceClassNameLength > 0 && psfLocalSet->u8ServiceClassNameLength < 32)
		memcpy(curr_packinfo->ucServiceClassName,
				psfLocalSet->u8ServiceClassName,
				psfLocalSet->u8ServiceClassNameLength);

	curr_packinfo->u8QueueType = psfLocalSet->u8ServiceFlowSchedulingType;

	if (curr_packinfo->u8QueueType == BE && curr_packinfo->ucDirection)
		ad->usBestEffortQueueIndex = search_rule_idx;

	curr_packinfo->ulSFID = ntohl(psfLocalSet->u32SFID);

	curr_packinfo->u8TrafficPriority = psfLocalSet->u8TrafficPriority;

	/* copy all the classifier in the Service Flow param  structure */
	for (i = 0; i < psfLocalSet->u8TotalClassifiers; i++) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
				"Classifier index =%d", i);
		psfCSType = &psfLocalSet->cConvergenceSLTypes[i];
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
				"Classifier index =%d", i);

		if (psfCSType->cCPacketClassificationRule.u8ClassifierRulePriority)
			curr_packinfo->bClassifierPriority = TRUE;

		if (psfCSType->cCPacketClassificationRule.u8ClassifierRulePriority)
			curr_packinfo->bClassifierPriority = TRUE;

		if (ucDsxType == DSA_ACK) {
			eClassifierAction = eAddClassifier;
		} else if (ucDsxType == DSC_ACK) {
			switch (psfCSType->u8ClassfierDSCAction) {
			case 0: /* DSC Add Classifier */
				eClassifierAction = eAddClassifier;
				break;
			case 1: /* DSC Replace Classifier */
				eClassifierAction = eReplaceClassifier;
				break;
			case 2: /* DSC Delete Classifier */
				eClassifierAction = eDeleteClassifier;
				break;
			default:
				eClassifierAction = eInvalidClassifierAction;
			}
		}

		u16PacketClassificationRuleIndex = ntohs(psfCSType->cCPacketClassificationRule.u16PacketClassificationRuleIndex);

		switch (eClassifierAction) {
		case eAddClassifier:
			/* Get a Free Classifier Index From Classifier table for this SF to add the Classifier */
			/* Contained in this message */
			nClassifierIndex = SearchClsid(ad,
					sf_id,
					u16PacketClassificationRuleIndex);

			if (nClassifierIndex > MAX_CLASSIFIERS) {
				nClassifierIndex = SearchFreeClsid(ad);
				if (nClassifierIndex > MAX_CLASSIFIERS) {
					/* Failed To get a free Entry */
					BCM_DEBUG_PRINT(ad,
							DBG_TYPE_OTHERS,
							CONN_MSG,
							DBG_LVL_ALL,
							"Error Failed To get a free Classifier Entry");
					break;
				}
				/* Copy the Classifier Rule for this service flow into our Classifier table maintained per SF. */
				CopyClassifierRuleToSF(ad, psfCSType,
						search_rule_idx,
						nClassifierIndex);
			} else {
				/* This Classifier Already Exists and it is invalid to Add Classifier with existing PCRI */
				BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS,
						CONN_MSG,
						DBG_LVL_ALL,
						"CopyToad: Error The Specified Classifier Already Exists and attempted To Add Classifier with Same PCRI : 0x%x\n",
						u16PacketClassificationRuleIndex);
			}
			break;
		case eReplaceClassifier:
			/* Get the Classifier Index From Classifier table for this SF and replace existing  Classifier */
			/* with the new classifier Contained in this message */
			nClassifierIndex = SearchClsid(ad, sf_id,
					u16PacketClassificationRuleIndex);
			if (nClassifierIndex > MAX_CLASSIFIERS) {
				/* Failed To search the classifier */
				BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS,
						CONN_MSG, DBG_LVL_ALL,
						"Error Search for Classifier To be replaced failed");
				break;
			}
			/* Copy the Classifier Rule for this service flow into our Classifier table maintained per SF. */
			CopyClassifierRuleToSF(ad, psfCSType,
					search_rule_idx, nClassifierIndex);
			break;
		case eDeleteClassifier:
			/* Get the Classifier Index From Classifier table for this SF and replace existing  Classifier */
			/* with the new classifier Contained in this message */
			nClassifierIndex = SearchClsid(ad, sf_id,
					u16PacketClassificationRuleIndex);
			if (nClassifierIndex > MAX_CLASSIFIERS)	{
				/* Failed To search the classifier */
				BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS,
						CONN_MSG, DBG_LVL_ALL,
						"Error Search for Classifier To be deleted failed");
				break;
			}

			/* Delete This classifier */
			DeleteClassifierRuleFromSF(ad, search_rule_idx,
					nClassifierIndex);
			break;
		default:
			/* Invalid Action for classifier */
			break;
		}
	}

	/* Repeat parsing Classification Entries to process PHS Rules */
	for (i = 0; i < psfLocalSet->u8TotalClassifiers; i++) {
		psfCSType = &psfLocalSet->cConvergenceSLTypes[i];
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
				"psfCSType->u8PhsDSCAction : 0x%x\n",
				psfCSType->u8PhsDSCAction);

		switch (psfCSType->u8PhsDSCAction) {
		case eDeleteAllPHSRules:
			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG,
					DBG_LVL_ALL,
					"Deleting All PHS Rules For VCID: 0x%X\n",
					vcid);

			/* Delete All the PHS rules for this Service flow */
			PhsDeleteSFRules(&ad->stBCMPhsContext, vcid);
			break;
		case eDeletePHSRule:
			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG,
					DBG_LVL_ALL,
					"PHS DSC Action = Delete PHS Rule\n");

			if (psfCSType->cPhsRule.u8PHSI)
				PhsDeletePHSRule(&ad->stBCMPhsContext,
						vcid,
						psfCSType->cCPacketClassificationRule.u8AssociatedPHSI);

			break;
		default:
			if (ucDsxType == DSC_ACK) {
				/* BCM_DEBUG_PRINT(CONN_MSG,("Invalid PHS DSC Action For DSC\n",psfCSType->cPhsRule.u8PHSI)); */
				break; /* FOr DSC ACK Case PHS DSC Action must be in valid set */
			}
		/* Proceed To Add PHS rule for DSA_ACK case even if PHS DSC action is unspecified */
		/* No Break Here . Intentionally! */

		case eAddPHSRule:
		case eSetPHSRule:
			if (psfCSType->cPhsRule.u8PHSI)	{
				/* Apply This PHS Rule to all classifiers whose Associated PHSI Match */
				apply_phs_rule_to_all_classifiers(ad,
						search_rule_idx,
						vcid,
						&phs_rule,
						&psfCSType->cPhsRule,
						add_indication);
			}
			break;
		}
	}

	if (psfLocalSet->u32MaxSustainedTrafficRate == 0) {
		/* No Rate Limit . Set Max Sustained Traffic Rate to Maximum */
		curr_packinfo->uiMaxAllowedRate = WIMAX_MAX_ALLOWED_RATE;
	} else if (ntohl(psfLocalSet->u32MaxSustainedTrafficRate) > WIMAX_MAX_ALLOWED_RATE) {
		/* Too large Allowed Rate specified. Limiting to Wi Max  Allowed rate */
		curr_packinfo->uiMaxAllowedRate = WIMAX_MAX_ALLOWED_RATE;
	} else {
		curr_packinfo->uiMaxAllowedRate =
			ntohl(psfLocalSet->u32MaxSustainedTrafficRate);
	}

	curr_packinfo->uiMaxLatency = ntohl(psfLocalSet->u32MaximumLatency);
	if (curr_packinfo->uiMaxLatency == 0) /* 0 should be treated as infinite */
		curr_packinfo->uiMaxLatency = MAX_LATENCY_ALLOWED;

	if ((curr_packinfo->u8QueueType == ERTPS ||
			curr_packinfo->u8QueueType == UGS))
		UGIValue = ntohs(psfLocalSet->u16UnsolicitedGrantInterval);

	if (UGIValue == 0)
		UGIValue = DEFAULT_UG_INTERVAL;

	/*
	 * For UGI based connections...
	 * DEFAULT_UGI_FACTOR*UGIInterval worth of data is the max token count at host...
	 * The extra amount of token is to ensure that a large amount of jitter won't have loss in throughput...
	 * In case of non-UGI based connection, 200 frames worth of data is the max token count at host...
	 */
	curr_packinfo->uiMaxBucketSize =
		(DEFAULT_UGI_FACTOR*curr_packinfo->uiMaxAllowedRate*UGIValue)/1000;

	if (curr_packinfo->uiMaxBucketSize < WIMAX_MAX_MTU*8) {
		UINT UGIFactor = 0;
		/* Special Handling to ensure the biggest size of packet can go out from host to FW as follows:
		 * 1. Any packet from Host to FW can go out in different packet size.
		 * 2. So in case the Bucket count is smaller than MTU, the packets of size (Size > TokenCount), will get dropped.
		 * 3. We can allow packets of MaxSize from Host->FW that can go out from FW in multiple SDUs by fragmentation at Wimax Layer
		 */
		UGIFactor = (curr_packinfo->uiMaxLatency/UGIValue + 1);

		if (UGIFactor > DEFAULT_UGI_FACTOR)
			curr_packinfo->uiMaxBucketSize =
				(UGIFactor*curr_packinfo->uiMaxAllowedRate*UGIValue)/1000;

		if (curr_packinfo->uiMaxBucketSize > WIMAX_MAX_MTU*8)
			curr_packinfo->uiMaxBucketSize = WIMAX_MAX_MTU*8;
	}

	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
			"LAT: %d, UGI: %d\n", curr_packinfo->uiMaxLatency,
			UGIValue);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
			"uiMaxAllowedRate: 0x%x, u32MaxSustainedTrafficRate: 0x%x ,uiMaxBucketSize: 0x%x",
			curr_packinfo->uiMaxAllowedRate,
			ntohl(psfLocalSet->u32MaxSustainedTrafficRate),
			curr_packinfo->uiMaxBucketSize);

	/* copy the extended SF Parameters to Support MIBS */
	CopyMIBSExtendedSFParameters(ad, psfLocalSet, search_rule_idx);

	/* store header suppression enabled flag per SF */
	curr_packinfo->bHeaderSuppressionEnabled =
		!(psfLocalSet->u8RequesttransmissionPolicy &
			MASK_DISABLE_HEADER_SUPPRESSION);

	kfree(curr_packinfo->pstSFIndication);
	curr_packinfo->pstSFIndication = add_indication;

	/* Re Sort the SF list in PackInfo according to Traffic Priority */
	SortPackInfo(ad);

	/* Re Sort the Classifier Rules table and re - arrange
	 * according to Classifier Rule Priority
	 */
	SortClassifiers(ad);
	DumpPhsRules(&ad->stBCMPhsContext);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
			"%s <=====", __func__);
}

/***********************************************************************
 * Function - DumpCmControlPacket
 *
 * Description - This routinue Dumps the Contents of the AddIndication
 *  Structure in the Connection Management Control Packet
 *
 * Parameter - pvBuffer: Pointer to the buffer containing the
 *  AddIndication data.
 *
 * Returns - None
 *************************************************************************/
static VOID DumpCmControlPacket(PVOID pvBuffer)
{
	int uiLoopIndex;
	int nIndex;
	struct bcm_add_indication_alt *add_indication;
	UINT nCurClassifierCnt;
	struct bcm_mini_adapter *ad = GET_BCM_ADAPTER(gblpnetdev);

	add_indication = pvBuffer;
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "======>");
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8Type: 0x%X", add_indication->u8Type);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8Direction: 0x%X", add_indication->u8Direction);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u16TID: 0x%X", ntohs(add_indication->u16TID));
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u16CID: 0x%X", ntohs(add_indication->u16CID));
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u16VCID: 0x%X", ntohs(add_indication->u16VCID));
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, " AuthorizedSet--->");
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u32SFID: 0x%X", htonl(add_indication->sfAuthorizedSet.u32SFID));
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u16CID: 0x%X", htons(add_indication->sfAuthorizedSet.u16CID));
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8ServiceClassNameLength: 0x%X",
			add_indication->sfAuthorizedSet.u8ServiceClassNameLength);

	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8ServiceClassName: 0x%X ,0x%X , 0x%X, 0x%X, 0x%X, 0x%X",
			add_indication->sfAuthorizedSet.u8ServiceClassName[0],
			add_indication->sfAuthorizedSet.u8ServiceClassName[1],
			add_indication->sfAuthorizedSet.u8ServiceClassName[2],
			add_indication->sfAuthorizedSet.u8ServiceClassName[3],
			add_indication->sfAuthorizedSet.u8ServiceClassName[4],
			add_indication->sfAuthorizedSet.u8ServiceClassName[5]);

	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8MBSService: 0x%X", add_indication->sfAuthorizedSet.u8MBSService);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8QosParamSet: 0x%X", add_indication->sfAuthorizedSet.u8QosParamSet);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8TrafficPriority: 0x%X, %p",
			add_indication->sfAuthorizedSet.u8TrafficPriority, &add_indication->sfAuthorizedSet.u8TrafficPriority);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u32MaxSustainedTrafficRate: 0x%X 0x%p",
			add_indication->sfAuthorizedSet.u32MaxSustainedTrafficRate,
			&add_indication->sfAuthorizedSet.u32MaxSustainedTrafficRate);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u32MaxTrafficBurst: 0x%X", add_indication->sfAuthorizedSet.u32MaxTrafficBurst);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u32MinReservedTrafficRate	: 0x%X",
			add_indication->sfAuthorizedSet.u32MinReservedTrafficRate);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8VendorSpecificQoSParamLength: 0x%X",
			add_indication->sfAuthorizedSet.u8VendorSpecificQoSParamLength);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8VendorSpecificQoSParam: 0x%X",
			add_indication->sfAuthorizedSet.u8VendorSpecificQoSParam[0]);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8ServiceFlowSchedulingType: 0x%X",
			add_indication->sfAuthorizedSet.u8ServiceFlowSchedulingType);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u32ToleratedJitter: 0x%X", add_indication->sfAuthorizedSet.u32ToleratedJitter);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u32MaximumLatency: 0x%X", add_indication->sfAuthorizedSet.u32MaximumLatency);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8FixedLengthVSVariableLengthSDUIndicator: 0x%X",
			add_indication->sfAuthorizedSet.u8FixedLengthVSVariableLengthSDUIndicator);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8SDUSize: 0x%X",	add_indication->sfAuthorizedSet.u8SDUSize);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u16TargetSAID: 0x%X", add_indication->sfAuthorizedSet.u16TargetSAID);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8ARQEnable: 0x%X", add_indication->sfAuthorizedSet.u8ARQEnable);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u16ARQWindowSize: 0x%X", add_indication->sfAuthorizedSet.u16ARQWindowSize);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u16ARQRetryTxTimeOut: 0x%X", add_indication->sfAuthorizedSet.u16ARQRetryTxTimeOut);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u16ARQRetryRxTimeOut: 0x%X", add_indication->sfAuthorizedSet.u16ARQRetryRxTimeOut);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u16ARQBlockLifeTime: 0x%X", add_indication->sfAuthorizedSet.u16ARQBlockLifeTime);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u16ARQSyncLossTimeOut: 0x%X", add_indication->sfAuthorizedSet.u16ARQSyncLossTimeOut);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8ARQDeliverInOrder: 0x%X", add_indication->sfAuthorizedSet.u8ARQDeliverInOrder);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u16ARQRxPurgeTimeOut: 0x%X", add_indication->sfAuthorizedSet.u16ARQRxPurgeTimeOut);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u16ARQBlockSize: 0x%X", add_indication->sfAuthorizedSet.u16ARQBlockSize);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8CSSpecification: 0x%X",	add_indication->sfAuthorizedSet.u8CSSpecification);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8TypeOfDataDeliveryService: 0x%X",
			add_indication->sfAuthorizedSet.u8TypeOfDataDeliveryService);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u16SDUInterArrivalTime: 0x%X", add_indication->sfAuthorizedSet.u16SDUInterArrivalTime);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u16TimeBase: 0x%X", add_indication->sfAuthorizedSet.u16TimeBase);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8PagingPreference: 0x%X", add_indication->sfAuthorizedSet.u8PagingPreference);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u16UnsolicitedPollingInterval: 0x%X",
			add_indication->sfAuthorizedSet.u16UnsolicitedPollingInterval);

	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "sfAuthorizedSet.u8HARQChannelMapping %x  %x %x ",
			*(unsigned int *)add_indication->sfAuthorizedSet.u8HARQChannelMapping,
			*(unsigned int *)&add_indication->sfAuthorizedSet.u8HARQChannelMapping[4],
			*(USHORT *)&add_indication->sfAuthorizedSet.u8HARQChannelMapping[8]);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8TrafficIndicationPreference: 0x%X",
			add_indication->sfAuthorizedSet.u8TrafficIndicationPreference);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, " Total Classifiers Received: 0x%X", add_indication->sfAuthorizedSet.u8TotalClassifiers);

	nCurClassifierCnt = add_indication->sfAuthorizedSet.u8TotalClassifiers;
	if (nCurClassifierCnt > MAX_CLASSIFIERS_IN_SF)
		nCurClassifierCnt = MAX_CLASSIFIERS_IN_SF;

	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL,  "add_indication->sfAuthorizedSet.bValid %d", add_indication->sfAuthorizedSet.bValid);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL,  "add_indication->sfAuthorizedSet.u16MacOverhead %x", add_indication->sfAuthorizedSet.u16MacOverhead);
	if (!add_indication->sfAuthorizedSet.bValid)
		add_indication->sfAuthorizedSet.bValid = 1;
	for (nIndex = 0; nIndex < nCurClassifierCnt; nIndex++) {
		struct bcm_convergence_types *psfCSType = NULL;

		psfCSType =  &add_indication->sfAuthorizedSet.cConvergenceSLTypes[nIndex];

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "psfCSType = %p", psfCSType);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "CCPacketClassificationRuleSI====>");
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8ClassifierRulePriority: 0x%X ",
				psfCSType->cCPacketClassificationRule.u8ClassifierRulePriority);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL,  "u8IPTypeOfServiceLength: 0x%X ",
				psfCSType->cCPacketClassificationRule.u8IPTypeOfServiceLength);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8IPTypeOfService[3]: 0x%X ,0x%X ,0x%X ",
				psfCSType->cCPacketClassificationRule.u8IPTypeOfService[0],
				psfCSType->cCPacketClassificationRule.u8IPTypeOfService[1],
				psfCSType->cCPacketClassificationRule.u8IPTypeOfService[2]);

		for (uiLoopIndex = 0; uiLoopIndex < 1; uiLoopIndex++)
			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8Protocol: 0x%02X ",
					psfCSType->cCPacketClassificationRule.u8Protocol);

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8IPMaskedSourceAddressLength: 0x%X ",
				psfCSType->cCPacketClassificationRule.u8IPMaskedSourceAddressLength);

		for (uiLoopIndex = 0; uiLoopIndex < 32; uiLoopIndex++)
			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8IPMaskedSourceAddress[32]: 0x%02X ",
					psfCSType->cCPacketClassificationRule.u8IPMaskedSourceAddress[uiLoopIndex]);

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8IPDestinationAddressLength: 0x%X ",
				psfCSType->cCPacketClassificationRule.u8IPDestinationAddressLength);

		for (uiLoopIndex = 0; uiLoopIndex < 32; uiLoopIndex++)
			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8IPDestinationAddress[32]: 0x%02X ",
					psfCSType->cCPacketClassificationRule.u8IPDestinationAddress[uiLoopIndex]);

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8ProtocolSourcePortRangeLength:0x%X ",
				psfCSType->cCPacketClassificationRule.u8ProtocolSourcePortRangeLength);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8ProtocolSourcePortRange[4]: 0x%02X ,0x%02X ,0x%02X ,0x%02X ",
				psfCSType->cCPacketClassificationRule.u8ProtocolSourcePortRange[0],
				psfCSType->cCPacketClassificationRule.u8ProtocolSourcePortRange[1],
				psfCSType->cCPacketClassificationRule.u8ProtocolSourcePortRange[2],
				psfCSType->cCPacketClassificationRule.u8ProtocolSourcePortRange[3]);

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8ProtocolDestPortRangeLength: 0x%02X ",
				psfCSType->cCPacketClassificationRule.u8ProtocolDestPortRangeLength);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8ProtocolDestPortRange[4]: 0x%02X ,0x%02X ,0x%02X ,0x%02X ",
				psfCSType->cCPacketClassificationRule.u8ProtocolDestPortRange[0],
				psfCSType->cCPacketClassificationRule.u8ProtocolDestPortRange[1],
				psfCSType->cCPacketClassificationRule.u8ProtocolDestPortRange[2],
				psfCSType->cCPacketClassificationRule.u8ProtocolDestPortRange[3]);

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8EthernetDestMacAddressLength: 0x%02X ",
				psfCSType->cCPacketClassificationRule.u8EthernetDestMacAddressLength);

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
				DBG_LVL_ALL, "u8EthernetDestMacAddress[6]: %pM",
				psfCSType->cCPacketClassificationRule.
						u8EthernetDestMacAddress);

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8EthernetSourceMACAddressLength: 0x%02X ",
				psfCSType->cCPacketClassificationRule.u8EthernetDestMacAddressLength);

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
				DBG_LVL_ALL, "u8EthernetSourceMACAddress[6]: %pM",
				psfCSType->cCPacketClassificationRule.
						u8EthernetSourceMACAddress);

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8EthertypeLength: 0x%02X ",
				psfCSType->cCPacketClassificationRule.u8EthertypeLength);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8Ethertype[3]: 0x%02X ,0x%02X ,0x%02X ",
				psfCSType->cCPacketClassificationRule.u8Ethertype[0],
				psfCSType->cCPacketClassificationRule.u8Ethertype[1],
				psfCSType->cCPacketClassificationRule.u8Ethertype[2]);

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u16UserPriority: 0x%X ", psfCSType->cCPacketClassificationRule.u16UserPriority);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u16VLANID: 0x%X ", psfCSType->cCPacketClassificationRule.u16VLANID);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8AssociatedPHSI: 0x%02X ", psfCSType->cCPacketClassificationRule.u8AssociatedPHSI);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u16PacketClassificationRuleIndex: 0x%X ",
				psfCSType->cCPacketClassificationRule.u16PacketClassificationRuleIndex);

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8VendorSpecificClassifierParamLength: 0x%X ",
				psfCSType->cCPacketClassificationRule.u8VendorSpecificClassifierParamLength);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8VendorSpecificClassifierParam[1]: 0x%X ",
				psfCSType->cCPacketClassificationRule.u8VendorSpecificClassifierParam[0]);
#ifdef VERSION_D5
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8IPv6FlowLableLength: 0x%X ",
				psfCSType->cCPacketClassificationRule.u8IPv6FlowLableLength);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
				DBG_LVL_ALL, "u8IPv6FlowLable[6]: 0x%*ph ",
				6, psfCSType->cCPacketClassificationRule.
					      u8IPv6FlowLable);
#endif
	}

	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "bValid: 0x%02X", add_indication->sfAuthorizedSet.bValid);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "AdmittedSet--->");
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u32SFID: 0x%X", add_indication->sfAdmittedSet.u32SFID);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u16CID: 0x%X", add_indication->sfAdmittedSet.u16CID);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8ServiceClassNameLength: 0x%X",
			add_indication->sfAdmittedSet.u8ServiceClassNameLength);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL,
			"u8ServiceClassName: 0x%*ph",
			6, add_indication->sfAdmittedSet.u8ServiceClassName);

	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8MBSService: 0x%02X", add_indication->sfAdmittedSet.u8MBSService);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8QosParamSet: 0x%02X", add_indication->sfAdmittedSet.u8QosParamSet);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8TrafficPriority: 0x%02X", add_indication->sfAdmittedSet.u8TrafficPriority);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u32MaxTrafficBurst: 0x%X", add_indication->sfAdmittedSet.u32MaxTrafficBurst);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u32MinReservedTrafficRate: 0x%X",
			add_indication->sfAdmittedSet.u32MinReservedTrafficRate);

	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8VendorSpecificQoSParamLength: 0x%02X",
			add_indication->sfAdmittedSet.u8VendorSpecificQoSParamLength);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8VendorSpecificQoSParam: 0x%02X",
			add_indication->sfAdmittedSet.u8VendorSpecificQoSParam[0]);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8ServiceFlowSchedulingType: 0x%02X",
			add_indication->sfAdmittedSet.u8ServiceFlowSchedulingType);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u32ToleratedJitter: 0x%X", add_indication->sfAdmittedSet.u32ToleratedJitter);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u32MaximumLatency: 0x%X", add_indication->sfAdmittedSet.u32MaximumLatency);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8FixedLengthVSVariableLengthSDUIndicator: 0x%02X",
			add_indication->sfAdmittedSet.u8FixedLengthVSVariableLengthSDUIndicator);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8SDUSize: 0x%02X", add_indication->sfAdmittedSet.u8SDUSize);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u16TargetSAID: 0x%02X", add_indication->sfAdmittedSet.u16TargetSAID);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8ARQEnable: 0x%02X", add_indication->sfAdmittedSet.u8ARQEnable);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u16ARQWindowSize: 0x%X", add_indication->sfAdmittedSet.u16ARQWindowSize);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u16ARQRetryTxTimeOut: 0x%X", add_indication->sfAdmittedSet.u16ARQRetryTxTimeOut);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u16ARQRetryRxTimeOut: 0x%X", add_indication->sfAdmittedSet.u16ARQRetryRxTimeOut);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u16ARQBlockLifeTime: 0x%X", add_indication->sfAdmittedSet.u16ARQBlockLifeTime);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u16ARQSyncLossTimeOut: 0x%X", add_indication->sfAdmittedSet.u16ARQSyncLossTimeOut);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8ARQDeliverInOrder: 0x%02X", add_indication->sfAdmittedSet.u8ARQDeliverInOrder);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u16ARQRxPurgeTimeOut: 0x%X", add_indication->sfAdmittedSet.u16ARQRxPurgeTimeOut);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u16ARQBlockSize: 0x%X", add_indication->sfAdmittedSet.u16ARQBlockSize);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8CSSpecification: 0x%02X", add_indication->sfAdmittedSet.u8CSSpecification);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8TypeOfDataDeliveryService: 0x%02X",
			add_indication->sfAdmittedSet.u8TypeOfDataDeliveryService);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u16SDUInterArrivalTime: 0x%X", add_indication->sfAdmittedSet.u16SDUInterArrivalTime);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u16TimeBase: 0x%X", add_indication->sfAdmittedSet.u16TimeBase);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8PagingPreference: 0x%X", add_indication->sfAdmittedSet.u8PagingPreference);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8TrafficIndicationPreference: 0x%02X",
			add_indication->sfAdmittedSet.u8TrafficIndicationPreference);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, " Total Classifiers Received: 0x%X", add_indication->sfAdmittedSet.u8TotalClassifiers);

	nCurClassifierCnt = add_indication->sfAdmittedSet.u8TotalClassifiers;
	if (nCurClassifierCnt > MAX_CLASSIFIERS_IN_SF)
		nCurClassifierCnt = MAX_CLASSIFIERS_IN_SF;

	for (nIndex = 0; nIndex < nCurClassifierCnt; nIndex++) {
		struct bcm_convergence_types *psfCSType = NULL;

		psfCSType =  &add_indication->sfAdmittedSet.cConvergenceSLTypes[nIndex];
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, " CCPacketClassificationRuleSI====>");
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8ClassifierRulePriority: 0x%02X ",
				psfCSType->cCPacketClassificationRule.u8ClassifierRulePriority);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8IPTypeOfServiceLength: 0x%02X",
				psfCSType->cCPacketClassificationRule.u8IPTypeOfServiceLength);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
				DBG_LVL_ALL, "u8IPTypeOfService[3]: 0x%*ph",
				3, psfCSType->cCPacketClassificationRule.
					      u8IPTypeOfService);
		for (uiLoopIndex = 0; uiLoopIndex < 1; uiLoopIndex++)
			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8Protocol: 0x%02X ", psfCSType->cCPacketClassificationRule.u8Protocol);

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8IPMaskedSourceAddressLength: 0x%02X ",
				psfCSType->cCPacketClassificationRule.u8IPMaskedSourceAddressLength);

		for (uiLoopIndex = 0; uiLoopIndex < 32; uiLoopIndex++)
			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8IPMaskedSourceAddress[32]: 0x%02X ",
					psfCSType->cCPacketClassificationRule.u8IPMaskedSourceAddress[uiLoopIndex]);

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8IPDestinationAddressLength: 0x%02X ",
				psfCSType->cCPacketClassificationRule.u8IPDestinationAddressLength);

		for (uiLoopIndex = 0; uiLoopIndex < 32; uiLoopIndex++)
			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8IPDestinationAddress[32]: 0x%02X ",
					psfCSType->cCPacketClassificationRule.u8IPDestinationAddress[uiLoopIndex]);

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8ProtocolSourcePortRangeLength: 0x%02X ",
				psfCSType->cCPacketClassificationRule.u8ProtocolSourcePortRangeLength);

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
				DBG_LVL_ALL, "u8ProtocolSourcePortRange[4]: 0x%*ph ",
				4, psfCSType->cCPacketClassificationRule.
						u8ProtocolSourcePortRange);

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8ProtocolDestPortRangeLength: 0x%02X ",
				psfCSType->cCPacketClassificationRule.u8ProtocolDestPortRangeLength);

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
				DBG_LVL_ALL, "u8ProtocolDestPortRange[4]: 0x%*ph ",
				4, psfCSType->cCPacketClassificationRule.
						u8ProtocolDestPortRange);

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8EthernetDestMacAddressLength: 0x%02X ",
				psfCSType->cCPacketClassificationRule.u8EthernetDestMacAddressLength);

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
				DBG_LVL_ALL, "u8EthernetDestMacAddress[6]: %pM",
				psfCSType->cCPacketClassificationRule.
						u8EthernetDestMacAddress);

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8EthernetSourceMACAddressLength: 0x%02X ",
				psfCSType->cCPacketClassificationRule.u8EthernetDestMacAddressLength);

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
				DBG_LVL_ALL, "u8EthernetSourceMACAddress[6]: %pM",
				psfCSType->cCPacketClassificationRule.
						u8EthernetSourceMACAddress);

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8EthertypeLength: 0x%02X ", psfCSType->cCPacketClassificationRule.u8EthertypeLength);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
				DBG_LVL_ALL, "u8Ethertype[3]: 0x%*ph",
				3, psfCSType->cCPacketClassificationRule.
					      u8Ethertype);

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u16UserPriority: 0x%X ", psfCSType->cCPacketClassificationRule.u16UserPriority);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u16VLANID: 0x%X ", psfCSType->cCPacketClassificationRule.u16VLANID);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8AssociatedPHSI: 0x%02X ", psfCSType->cCPacketClassificationRule.u8AssociatedPHSI);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u16PacketClassificationRuleIndex: 0x%X ",
				psfCSType->cCPacketClassificationRule.u16PacketClassificationRuleIndex);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8VendorSpecificClassifierParamLength: 0x%02X",
				psfCSType->cCPacketClassificationRule.u8VendorSpecificClassifierParamLength);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8VendorSpecificClassifierParam[1]: 0x%02X ",
				psfCSType->cCPacketClassificationRule.u8VendorSpecificClassifierParam[0]);
#ifdef VERSION_D5
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8IPv6FlowLableLength: 0x%X ",
				psfCSType->cCPacketClassificationRule.u8IPv6FlowLableLength);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
				DBG_LVL_ALL, "u8IPv6FlowLable[6]: 0x%*ph ",
				6, psfCSType->cCPacketClassificationRule.
					      u8IPv6FlowLable);
#endif
	}

	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "bValid: 0x%X", add_indication->sfAdmittedSet.bValid);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, " ActiveSet--->");
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u32SFID: 0x%X", add_indication->sfActiveSet.u32SFID);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u16CID: 0x%X", add_indication->sfActiveSet.u16CID);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8ServiceClassNameLength: 0x%X", add_indication->sfActiveSet.u8ServiceClassNameLength);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL,
			"u8ServiceClassName: 0x%*ph",
			6, add_indication->sfActiveSet.u8ServiceClassName);

	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8MBSService: 0x%02X", add_indication->sfActiveSet.u8MBSService);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8QosParamSet: 0x%02X", add_indication->sfActiveSet.u8QosParamSet);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8TrafficPriority: 0x%02X", add_indication->sfActiveSet.u8TrafficPriority);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u32MaxTrafficBurst: 0x%X", add_indication->sfActiveSet.u32MaxTrafficBurst);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u32MinReservedTrafficRate: 0x%X",
			add_indication->sfActiveSet.u32MinReservedTrafficRate);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8VendorSpecificQoSParamLength: 0x%02X",
			add_indication->sfActiveSet.u8VendorSpecificQoSParamLength);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8VendorSpecificQoSParam: 0x%02X",
			add_indication->sfActiveSet.u8VendorSpecificQoSParam[0]);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8ServiceFlowSchedulingType: 0x%02X",
			add_indication->sfActiveSet.u8ServiceFlowSchedulingType);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u32ToleratedJitter: 0x%X", add_indication->sfActiveSet.u32ToleratedJitter);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u32MaximumLatency: 0x%X",	add_indication->sfActiveSet.u32MaximumLatency);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8FixedLengthVSVariableLengthSDUIndicator: 0x%02X",
			add_indication->sfActiveSet.u8FixedLengthVSVariableLengthSDUIndicator);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, "u8SDUSize: 0x%X",	add_indication->sfActiveSet.u8SDUSize);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, " u16TargetSAID: 0x%X", add_indication->sfActiveSet.u16TargetSAID);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, " u8ARQEnable: 0x%X", add_indication->sfActiveSet.u8ARQEnable);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, " u16ARQWindowSize: 0x%X", add_indication->sfActiveSet.u16ARQWindowSize);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, " u16ARQRetryTxTimeOut: 0x%X", add_indication->sfActiveSet.u16ARQRetryTxTimeOut);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, " u16ARQRetryRxTimeOut: 0x%X", add_indication->sfActiveSet.u16ARQRetryRxTimeOut);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, " u16ARQBlockLifeTime: 0x%X", add_indication->sfActiveSet.u16ARQBlockLifeTime);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, " u16ARQSyncLossTimeOut: 0x%X", add_indication->sfActiveSet.u16ARQSyncLossTimeOut);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, " u8ARQDeliverInOrder: 0x%X", add_indication->sfActiveSet.u8ARQDeliverInOrder);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, " u16ARQRxPurgeTimeOut: 0x%X", add_indication->sfActiveSet.u16ARQRxPurgeTimeOut);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, " u16ARQBlockSize: 0x%X", add_indication->sfActiveSet.u16ARQBlockSize);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, " u8CSSpecification: 0x%X", add_indication->sfActiveSet.u8CSSpecification);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, " u8TypeOfDataDeliveryService: 0x%X",
			add_indication->sfActiveSet.u8TypeOfDataDeliveryService);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, " u16SDUInterArrivalTime: 0x%X", add_indication->sfActiveSet.u16SDUInterArrivalTime);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, " u16TimeBase: 0x%X", add_indication->sfActiveSet.u16TimeBase);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, " u8PagingPreference: 0x%X", add_indication->sfActiveSet.u8PagingPreference);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, " u8TrafficIndicationPreference: 0x%X",
			add_indication->sfActiveSet.u8TrafficIndicationPreference);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL, " Total Classifiers Received: 0x%X", add_indication->sfActiveSet.u8TotalClassifiers);

	nCurClassifierCnt = add_indication->sfActiveSet.u8TotalClassifiers;
	if (nCurClassifierCnt > MAX_CLASSIFIERS_IN_SF)
		nCurClassifierCnt = MAX_CLASSIFIERS_IN_SF;

	for (nIndex = 0; nIndex < nCurClassifierCnt; nIndex++)	{
		struct bcm_convergence_types *psfCSType = NULL;
		struct bcm_packet_class_rules *clsRule = NULL;

		psfCSType = &add_indication->sfActiveSet.cConvergenceSLTypes[nIndex];
		clsRule	= &psfCSType->cCPacketClassificationRule;

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
				DBG_LVL_ALL, " CCPacketClassificationRuleSI====>");
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
				DBG_LVL_ALL, " u8ClassifierRulePriority: 0x%X ",
				clsRule->u8ClassifierRulePriority);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
				DBG_LVL_ALL, " u8IPTypeOfServiceLength: 0x%X ",
				clsRule->u8IPTypeOfServiceLength);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
				DBG_LVL_ALL,
				" u8IPTypeOfService[3]: 0x%X ,0x%X ,0x%X ",
				clsRule->u8IPTypeOfService[0],
				clsRule->u8IPTypeOfService[1],
				clsRule->u8IPTypeOfService[2]);

		for (uiLoopIndex = 0; uiLoopIndex < 1; uiLoopIndex++)
			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
					DBG_LVL_ALL,
					" u8Protocol: 0x%X ",
					clsRule->u8Protocol);

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
				DBG_LVL_ALL,
				"u8IPMaskedSourceAddressLength: 0x%X ",
				clsRule->u8IPMaskedSourceAddressLength);

		for (uiLoopIndex = 0; uiLoopIndex < 32; uiLoopIndex++)
			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
					DBG_LVL_ALL,
					"u8IPMaskedSourceAddress[32]: 0x%X ",
					clsRule->u8IPMaskedSourceAddress[uiLoopIndex]);

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
				DBG_LVL_ALL,
				"u8IPDestinationAddressLength: 0x%02X ",
				clsRule->u8IPDestinationAddressLength);

		for (uiLoopIndex = 0; uiLoopIndex < 32; uiLoopIndex++)
			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
					DBG_LVL_ALL,
					" u8IPDestinationAddress[32]:0x%X ",
					clsRule->u8IPDestinationAddress[uiLoopIndex]);

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
				DBG_LVL_ALL,
				" u8ProtocolSourcePortRangeLength: 0x%X ",
				clsRule->u8ProtocolSourcePortRangeLength);

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
				DBG_LVL_ALL,
				" u8ProtocolSourcePortRange[4]: 0x%X ,0x%X ,0x%X ,0x%X ",
				clsRule->u8ProtocolSourcePortRange[0],
				clsRule->u8ProtocolSourcePortRange[1],
				clsRule->u8ProtocolSourcePortRange[2],
				clsRule->u8ProtocolSourcePortRange[3]);

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
				DBG_LVL_ALL,
				" u8ProtocolDestPortRangeLength: 0x%X ",
				clsRule->u8ProtocolDestPortRangeLength);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
				DBG_LVL_ALL,
				" u8ProtocolDestPortRange[4]: 0x%X ,0x%X ,0x%X ,0x%X ",
				clsRule->u8ProtocolDestPortRange[0],
				clsRule->u8ProtocolDestPortRange[1],
				clsRule->u8ProtocolDestPortRange[2],
				clsRule->u8ProtocolDestPortRange[3]);

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
				DBG_LVL_ALL,
				" u8EthernetDestMacAddressLength: 0x%X ",
				clsRule->u8EthernetDestMacAddressLength);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
				DBG_LVL_ALL,
				" u8EthernetDestMacAddress[6]: 0x%X ,0x%X ,0x%X ,0x%X ,0x%X ,0x%X",
				clsRule->u8EthernetDestMacAddress[0],
				clsRule->u8EthernetDestMacAddress[1],
				clsRule->u8EthernetDestMacAddress[2],
				clsRule->u8EthernetDestMacAddress[3],
				clsRule->u8EthernetDestMacAddress[4],
				clsRule->u8EthernetDestMacAddress[5]);

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
				DBG_LVL_ALL,
				" u8EthernetSourceMACAddressLength: 0x%X ",
				clsRule->u8EthernetDestMacAddressLength);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
				DBG_LVL_ALL,
				"u8EthernetSourceMACAddress[6]: 0x%X ,0x%X ,0x%X ,0x%X ,0x%X ,0x%X",
				clsRule->u8EthernetSourceMACAddress[0],
				clsRule->u8EthernetSourceMACAddress[1],
				clsRule->u8EthernetSourceMACAddress[2],
				clsRule->u8EthernetSourceMACAddress[3],
				clsRule->u8EthernetSourceMACAddress[4],
				clsRule->u8EthernetSourceMACAddress[5]);

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
				DBG_LVL_ALL, " u8EthertypeLength: 0x%X ",
				clsRule->u8EthertypeLength);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
				DBG_LVL_ALL,
				" u8Ethertype[3]: 0x%X ,0x%X ,0x%X ",
				clsRule->u8Ethertype[0],
				clsRule->u8Ethertype[1],
				clsRule->u8Ethertype[2]);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
				DBG_LVL_ALL, " u16UserPriority: 0x%X ",
				clsRule->u16UserPriority);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
				DBG_LVL_ALL, " u16VLANID: 0x%X ",
				clsRule->u16VLANID);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
				DBG_LVL_ALL, " u8AssociatedPHSI: 0x%X ",
				clsRule->u8AssociatedPHSI);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
				DBG_LVL_ALL,
				" u16PacketClassificationRuleIndex:0x%X ",
				clsRule->u16PacketClassificationRuleIndex);

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
				DBG_LVL_ALL,
				" u8VendorSpecificClassifierParamLength:0x%X ",
				clsRule->u8VendorSpecificClassifierParamLength);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
				DBG_LVL_ALL,
				" u8VendorSpecificClassifierParam[1]:0x%X ",
				clsRule->u8VendorSpecificClassifierParam[0]);
#ifdef VERSION_D5
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
				DBG_LVL_ALL, " u8IPv6FlowLableLength: 0x%X ",
				clsRule->u8IPv6FlowLableLength);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL,
				DBG_LVL_ALL,
				" u8IPv6FlowLable[6]: 0x%X ,0x%X ,0x%X ,0x%X ,0x%X ,0x%X ",
				clsRule->u8IPv6FlowLable[0],
				clsRule->u8IPv6FlowLable[1],
				clsRule->u8IPv6FlowLable[2],
				clsRule->u8IPv6FlowLable[3],
				clsRule->u8IPv6FlowLable[4],
				clsRule->u8IPv6FlowLable[5]);
#endif
	}

	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, DUMP_CONTROL, DBG_LVL_ALL,
			" bValid: 0x%X", add_indication->sfActiveSet.bValid);
}

static inline ULONG RestoreSFParam(struct bcm_mini_adapter *ad,
		ULONG ulAddrSFParamSet, PUCHAR pucDestBuffer)
{
	UINT  nBytesToRead = sizeof(struct bcm_connect_mgr_params);

	if (ulAddrSFParamSet == 0 || NULL == pucDestBuffer) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
				"Got Param address as 0!!");
		return 0;
	}
	ulAddrSFParamSet = ntohl(ulAddrSFParamSet);

	/* Read out the SF Param Set At the indicated Location */
	if (rdm(ad, ulAddrSFParamSet, (PUCHAR)pucDestBuffer, nBytesToRead) < 0)
		return STATUS_FAILURE;

	return 1;
}

static ULONG StoreSFParam(struct bcm_mini_adapter *ad, PUCHAR pucSrcBuffer,
		ULONG ulAddrSFParamSet)
{
	UINT nBytesToWrite = sizeof(struct bcm_connect_mgr_params);
	int ret = 0;

	if (ulAddrSFParamSet == 0 || NULL == pucSrcBuffer)
		return 0;

	ret = wrm(ad, ulAddrSFParamSet, (u8 *)pucSrcBuffer, nBytesToWrite);
	if (ret < 0) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
				"%s:%d WRM failed", __func__, __LINE__);
		return ret;
	}
	return 1;
}

ULONG StoreCmControlResponseMessage(struct bcm_mini_adapter *ad,
		PVOID pvBuffer, UINT *puBufferLength)
{
	struct bcm_add_indication_alt *pstAddIndicationAlt = NULL;
	struct bcm_add_indication *add_indication = NULL;
	struct bcm_del_request *pstDeletionRequest;
	UINT search_rule_idx;
	ULONG sf_id;

	pstAddIndicationAlt = pvBuffer;

	/*
	 * In case of DSD Req By MS, we should immediately delete this SF so that
	 * we can stop the further classifying the pkt for this SF.
	 */
	if (pstAddIndicationAlt->u8Type == DSD_REQ) {
		pstDeletionRequest = pvBuffer;

		sf_id = ntohl(pstDeletionRequest->u32SFID);
		search_rule_idx = SearchSfid(ad, sf_id);

		if (search_rule_idx < NO_OF_QUEUES) {
			deleteSFBySfid(ad, search_rule_idx);
			ad->u32TotalDSD++;
		}
		return 1;
	}

	if ((pstAddIndicationAlt->u8Type == DSD_RSP) ||
		(pstAddIndicationAlt->u8Type == DSD_ACK)) {
		/* No Special handling send the message as it is */
		return 1;
	}
	/* For DSA_REQ, only up to "psfAuthorizedSet" parameter should be accessed by driver! */

	add_indication = kmalloc(sizeof(struct bcm_add_indication),
			GFP_KERNEL);
	if (add_indication == NULL)
		return 0;

	/* AUTHORIZED SET */
	add_indication->psfAuthorizedSet = (struct bcm_connect_mgr_params *)
			GetNextTargetBufferLocation(ad,
					pstAddIndicationAlt->u16TID);
	if (!add_indication->psfAuthorizedSet) {
		kfree(add_indication);
		return 0;
	}

	if (StoreSFParam(ad, (PUCHAR)&pstAddIndicationAlt->sfAuthorizedSet,
				(ULONG)add_indication->psfAuthorizedSet) != 1) {
		kfree(add_indication);
		return 0;
	}

	/* this can't possibly be right */
	add_indication->psfAuthorizedSet =
		(struct bcm_connect_mgr_params *) ntohl(
				(ULONG)add_indication->psfAuthorizedSet);

	if (pstAddIndicationAlt->u8Type == DSA_REQ) {
		struct bcm_add_request AddRequest;

		AddRequest.u8Type = pstAddIndicationAlt->u8Type;
		AddRequest.eConnectionDir = pstAddIndicationAlt->u8Direction;
		AddRequest.u16TID = pstAddIndicationAlt->u16TID;
		AddRequest.u16CID = pstAddIndicationAlt->u16CID;
		AddRequest.u16VCID = pstAddIndicationAlt->u16VCID;
		AddRequest.psfParameterSet = add_indication->psfAuthorizedSet;
		(*puBufferLength) = sizeof(struct bcm_add_request);
		memcpy(pvBuffer, &AddRequest, sizeof(struct bcm_add_request));
		kfree(add_indication);
		return 1;
	}

	/* Since it's not DSA_REQ, we can access all field in pstAddIndicationAlt */
	/* We need to extract the structure from the buffer and pack it differently */

	add_indication->u8Type = pstAddIndicationAlt->u8Type;
	add_indication->eConnectionDir = pstAddIndicationAlt->u8Direction;
	add_indication->u16TID = pstAddIndicationAlt->u16TID;
	add_indication->u16CID = pstAddIndicationAlt->u16CID;
	add_indication->u16VCID = pstAddIndicationAlt->u16VCID;
	add_indication->u8CC = pstAddIndicationAlt->u8CC;

	/* ADMITTED SET */
	add_indication->psfAdmittedSet = (struct bcm_connect_mgr_params *)
		GetNextTargetBufferLocation(ad,
				pstAddIndicationAlt->u16TID);
	if (!add_indication->psfAdmittedSet) {
		kfree(add_indication);
		return 0;
	}
	if (StoreSFParam(ad, (PUCHAR)&pstAddIndicationAlt->sfAdmittedSet,
				(ULONG)add_indication->psfAdmittedSet) != 1) {
		kfree(add_indication);
		return 0;
	}

	add_indication->psfAdmittedSet =
		(struct bcm_connect_mgr_params *) ntohl(
				(ULONG) add_indication->psfAdmittedSet);

	/* ACTIVE SET */
	add_indication->psfActiveSet = (struct bcm_connect_mgr_params *)
		GetNextTargetBufferLocation(ad,
				pstAddIndicationAlt->u16TID);
	if (!add_indication->psfActiveSet) {
		kfree(add_indication);
		return 0;
	}
	if (StoreSFParam(ad, (PUCHAR)&pstAddIndicationAlt->sfActiveSet,
				(ULONG)add_indication->psfActiveSet) != 1) {
		kfree(add_indication);
		return 0;
	}

	add_indication->psfActiveSet =
		(struct bcm_connect_mgr_params *) ntohl(
				(ULONG)add_indication->psfActiveSet);

	(*puBufferLength) = sizeof(struct bcm_add_indication);
	*(struct bcm_add_indication *)pvBuffer = *add_indication;
	kfree(add_indication);
	return 1;
}

static inline struct bcm_add_indication_alt
*RestoreCmControlResponseMessage(register struct bcm_mini_adapter *ad,
		register PVOID pvBuffer)
{
	ULONG ulStatus = 0;
	struct bcm_add_indication *add_indication = NULL;
	struct bcm_add_indication_alt *pstAddIndicationDest = NULL;

	add_indication = pvBuffer;
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
			"=====>");
	if ((add_indication->u8Type == DSD_REQ) ||
		(add_indication->u8Type == DSD_RSP) ||
		(add_indication->u8Type == DSD_ACK))
		return pvBuffer;

	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
			"Inside RestoreCmControlResponseMessage ");
	/*
	 * Need to Allocate memory to contain the SUPER Large structures
	 * Our driver can't create these structures on Stack :(
	 */
	pstAddIndicationDest = kmalloc(sizeof(struct bcm_add_indication_alt),
			GFP_KERNEL);

	if (pstAddIndicationDest) {
		memset(pstAddIndicationDest, 0,
				sizeof(struct bcm_add_indication_alt));
	} else {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG,
				DBG_LVL_ALL,
				"Failed to allocate memory for SF Add Indication Structure ");
		return NULL;
	}
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
			"AddIndication-u8Type : 0x%X",
			add_indication->u8Type);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
			"AddIndication-u8Direction : 0x%X",
			add_indication->eConnectionDir);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
			"AddIndication-u8TID : 0x%X",
			ntohs(add_indication->u16TID));
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
			"AddIndication-u8CID : 0x%X",
			ntohs(add_indication->u16CID));
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
			"AddIndication-u16VCID : 0x%X",
			ntohs(add_indication->u16VCID));
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
			"AddIndication-autorized set loc : %p",
			add_indication->psfAuthorizedSet);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
			"AddIndication-admitted set loc : %p",
			add_indication->psfAdmittedSet);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
			"AddIndication-Active set loc : %p",
			add_indication->psfActiveSet);

	pstAddIndicationDest->u8Type = add_indication->u8Type;
	pstAddIndicationDest->u8Direction = add_indication->eConnectionDir;
	pstAddIndicationDest->u16TID = add_indication->u16TID;
	pstAddIndicationDest->u16CID = add_indication->u16CID;
	pstAddIndicationDest->u16VCID = add_indication->u16VCID;
	pstAddIndicationDest->u8CC = add_indication->u8CC;

	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
			"Restoring Active Set ");
	ulStatus = RestoreSFParam(ad,
			(ULONG)add_indication->psfActiveSet,
			(PUCHAR)&pstAddIndicationDest->sfActiveSet);
	if (ulStatus != 1)
		goto failed_restore_sf_param;

	if (pstAddIndicationDest->sfActiveSet.u8TotalClassifiers > MAX_CLASSIFIERS_IN_SF)
		pstAddIndicationDest->sfActiveSet.u8TotalClassifiers =
			MAX_CLASSIFIERS_IN_SF;

	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
			"Restoring Admitted Set ");
	ulStatus = RestoreSFParam(ad,
			(ULONG)add_indication->psfAdmittedSet,
			(PUCHAR)&pstAddIndicationDest->sfAdmittedSet);
	if (ulStatus != 1)
		goto failed_restore_sf_param;

	if (pstAddIndicationDest->sfAdmittedSet.u8TotalClassifiers > MAX_CLASSIFIERS_IN_SF)
		pstAddIndicationDest->sfAdmittedSet.u8TotalClassifiers =
			MAX_CLASSIFIERS_IN_SF;

	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
			"Restoring Authorized Set ");
	ulStatus = RestoreSFParam(ad,
			(ULONG)add_indication->psfAuthorizedSet,
			(PUCHAR)&pstAddIndicationDest->sfAuthorizedSet);
	if (ulStatus != 1)
		goto failed_restore_sf_param;

	if (pstAddIndicationDest->sfAuthorizedSet.u8TotalClassifiers > MAX_CLASSIFIERS_IN_SF)
		pstAddIndicationDest->sfAuthorizedSet.u8TotalClassifiers =
			MAX_CLASSIFIERS_IN_SF;

	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
			"Dumping the whole raw packet");
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
		"============================================================");
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
			" pstAddIndicationDest->sfActiveSet size  %zx %p",
			sizeof(*pstAddIndicationDest), pstAddIndicationDest);
	/* BCM_DEBUG_PRINT_BUFFER(ad,DBG_TYPE_OTHERS, CONN_MSG,
	 *		DBG_LVL_ALL, (unsigned char *)pstAddIndicationDest,
	 *		sizeof(*pstAddIndicationDest));
	 */
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
			"============================================================");
	return pstAddIndicationDest;
failed_restore_sf_param:
	kfree(pstAddIndicationDest);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
			"<=====");
	return NULL;
}

ULONG SetUpTargetDsxBuffers(struct bcm_mini_adapter *ad)
{
	ULONG ulTargetDsxBuffersBase = 0;
	ULONG ulCntTargetBuffers;
	ULONG i;
	int Status;

	if (!ad) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
				"ad was NULL!!!");
		return 0;
	}

	if (ad->astTargetDsxBuffer[0].ulTargetDsxBuffer)
		return 1;

	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
			"Size of Each DSX Buffer(Also size of connection manager parameters): %zx ",
			sizeof(struct bcm_connect_mgr_params));
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
			"Reading DSX buffer From Target location %x ",
			DSX_MESSAGE_EXCHANGE_BUFFER);

	Status = rdmalt(ad, DSX_MESSAGE_EXCHANGE_BUFFER,
			(PUINT)&ulTargetDsxBuffersBase, sizeof(UINT));
	if (Status < 0) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
				"RDM failed!!");
		return 0;
	}

	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
			"Base Address Of DSX  Target Buffer : 0x%lx",
			ulTargetDsxBuffersBase);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
			"Tgt Buffer is Now %lx :", ulTargetDsxBuffersBase);
	ulCntTargetBuffers = DSX_MESSAGE_EXCHANGE_BUFFER_SIZE /
		sizeof(struct bcm_connect_mgr_params);

	ad->ulTotalTargetBuffersAvailable =
		ulCntTargetBuffers > MAX_TARGET_DSX_BUFFERS ?
		MAX_TARGET_DSX_BUFFERS : ulCntTargetBuffers;

	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
			" Total Target DSX Buffer setup %lx ",
			ad->ulTotalTargetBuffersAvailable);

	for (i = 0; i < ad->ulTotalTargetBuffersAvailable; i++) {
		ad->astTargetDsxBuffer[i].ulTargetDsxBuffer = ulTargetDsxBuffersBase;
		ad->astTargetDsxBuffer[i].valid = 1;
		ad->astTargetDsxBuffer[i].tid = 0;
		ulTargetDsxBuffersBase += sizeof(struct bcm_connect_mgr_params);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL, "  Target DSX Buffer %lx setup at 0x%lx",
				i, ad->astTargetDsxBuffer[i].ulTargetDsxBuffer);
	}
	ad->ulCurrentTargetBuffer = 0;
	ad->ulFreeTargetBufferCnt = ad->ulTotalTargetBuffersAvailable;
	return 1;
}

static ULONG GetNextTargetBufferLocation(struct bcm_mini_adapter *ad,
		B_UINT16 tid)
{
	ULONG dsx_buf;
	ULONG idx, max_try;

	if ((ad->ulTotalTargetBuffersAvailable == 0)
			|| (ad->ulFreeTargetBufferCnt == 0)) {
		ClearTargetDSXBuffer(ad, tid, false);
		return 0;
	}

	idx = ad->ulCurrentTargetBuffer;
	max_try = ad->ulTotalTargetBuffersAvailable;
	while ((max_try) && (ad->astTargetDsxBuffer[idx].valid != 1)) {
		idx = (idx+1) % ad->ulTotalTargetBuffersAvailable;
		max_try--;
	}

	if (max_try == 0) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_PRINTK, 0, 0,
				"\n GetNextTargetBufferLocation : Error No Free Target DSX Buffers FreeCnt : %lx ",
				ad->ulFreeTargetBufferCnt);
		ClearTargetDSXBuffer(ad, tid, false);
		return 0;
	}

	dsx_buf = ad->astTargetDsxBuffer[idx].ulTargetDsxBuffer;
	ad->astTargetDsxBuffer[idx].valid = 0;
	ad->astTargetDsxBuffer[idx].tid = tid;
	ad->ulFreeTargetBufferCnt--;
	idx = (idx+1)%ad->ulTotalTargetBuffersAvailable;
	ad->ulCurrentTargetBuffer = idx;
	BCM_DEBUG_PRINT(ad, DBG_TYPE_PRINTK, 0, 0,
			"GetNextTargetBufferLocation :Returning address %lx tid %d\n",
			dsx_buf, tid);

	return dsx_buf;
}

int AllocAdapterDsxBuffer(struct bcm_mini_adapter *ad)
{
	/*
	 * Need to Allocate memory to contain the SUPER Large structures
	 * Our driver can't create these structures on Stack
	 */
	ad->caDsxReqResp = kmalloc(sizeof(struct bcm_add_indication_alt)
			+ LEADER_SIZE, GFP_KERNEL);
	if (!ad->caDsxReqResp)
		return -ENOMEM;

	return 0;
}

int FreeadDsxBuffer(struct bcm_mini_adapter *ad)
{
	kfree(ad->caDsxReqResp);
	return 0;
}

/*
 * @ingroup ctrl_pkt_functions
 * This routinue would process the Control responses
 * for the Connection Management.
 * @return - Queue index for the free SFID else returns Invalid Index.
 */
bool CmControlResponseMessage(struct bcm_mini_adapter *ad,  /* <Pointer to the ad structure */
				PVOID pvBuffer /* Starting Address of the Buffer, that contains the AddIndication Data */)
{
	struct bcm_connect_mgr_params *psfLocalSet = NULL;
	struct bcm_add_indication_alt *add_indication = NULL;
	struct bcm_change_indication *pstChangeIndication = NULL;
	struct bcm_leader *pLeader = NULL;
	INT search_rule_idx = 0;
	ULONG sf_id;

	/*
	 * Otherwise the message contains a target address from where we need to
	 * read out the rest of the service flow param structure
	 */
	add_indication = RestoreCmControlResponseMessage(ad, pvBuffer);
	if (add_indication == NULL) {
		ClearTargetDSXBuffer(ad, ((struct bcm_add_indication *)pvBuffer)->u16TID, false);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_PRINTK, 0, 0, "Error in restoring Service Flow param structure from DSx message");
		return false;
	}

	DumpCmControlPacket(add_indication);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL, "====>");
	pLeader = (struct bcm_leader *)ad->caDsxReqResp;

	pLeader->Status = CM_CONTROL_NEWDSX_MULTICLASSIFIER_REQ;
	pLeader->Vcid = 0;

	ClearTargetDSXBuffer(ad, add_indication->u16TID, false);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_PRINTK, 0, 0, "### TID RECEIVED %d\n", add_indication->u16TID);
	switch (add_indication->u8Type) {
	case DSA_REQ:
		pLeader->PLength = sizeof(struct bcm_add_indication_alt);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL, "Sending DSA Response....\n");
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL, "SENDING DSA RESPONSE TO MAC %d", pLeader->PLength);
		*((struct bcm_add_indication_alt *)&(ad->caDsxReqResp[LEADER_SIZE]))
			= *add_indication;
		((struct bcm_add_indication_alt *)&(ad->caDsxReqResp[LEADER_SIZE]))->u8Type = DSA_RSP;

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL, " VCID = %x", ntohs(add_indication->u16VCID));
		CopyBufferToControlPacket(ad, (PVOID)ad->caDsxReqResp);
		kfree(add_indication);
		break;
	case DSA_RSP:
		pLeader->PLength = sizeof(struct bcm_add_indication_alt);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL, "SENDING DSA ACK TO MAC %d",
				pLeader->PLength);
		*((struct bcm_add_indication_alt *)&(ad->caDsxReqResp[LEADER_SIZE]))
			= *add_indication;
		((struct bcm_add_indication_alt *)&(ad->caDsxReqResp[LEADER_SIZE]))->u8Type = DSA_ACK;
		/* FALLTHROUGH */
	case DSA_ACK:
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL, "VCID:0x%X",
				ntohs(add_indication->u16VCID));
		search_rule_idx = SearchFreeSfid(ad);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL, "search_rule_idx:0x%X ",
				search_rule_idx);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL, "Direction:0x%X ",
				add_indication->u8Direction);
		if (search_rule_idx < NO_OF_QUEUES) {
			ad->PackInfo[search_rule_idx].ucDirection =
				add_indication->u8Direction;
			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL, "bValid:0x%X ",
					add_indication->sfActiveSet.bValid);
			if (add_indication->sfActiveSet.bValid == TRUE)
				ad->PackInfo[search_rule_idx].bActiveSet = TRUE;

			if (add_indication->sfAuthorizedSet.bValid == TRUE)
				ad->PackInfo[search_rule_idx].bAuthorizedSet = TRUE;

			if (add_indication->sfAdmittedSet.bValid == TRUE)
				ad->PackInfo[search_rule_idx].bAdmittedSet = TRUE;

			if (add_indication->sfActiveSet.bValid == false) {
				ad->PackInfo[search_rule_idx].bActive = false;
				ad->PackInfo[search_rule_idx].bActivateRequestSent = false;
				if (add_indication->sfAdmittedSet.bValid)
					psfLocalSet = &add_indication->sfAdmittedSet;
				else if (add_indication->sfAuthorizedSet.bValid)
					psfLocalSet = &add_indication->sfAuthorizedSet;
			} else {
				psfLocalSet = &add_indication->sfActiveSet;
				ad->PackInfo[search_rule_idx].bActive = TRUE;
			}

			if (!psfLocalSet) {
				BCM_DEBUG_PRINT(ad, DBG_TYPE_PRINTK, 0, 0, "No set is valid\n");
				ad->PackInfo[search_rule_idx].bActive = false;
				ad->PackInfo[search_rule_idx].bValid = false;
				ad->PackInfo[search_rule_idx].usVCID_Value = 0;
				kfree(add_indication);
			} else if (psfLocalSet->bValid && (add_indication->u8CC == 0)) {
				BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL, "DSA ACK");
				ad->PackInfo[search_rule_idx].usVCID_Value = ntohs(add_indication->u16VCID);
				ad->PackInfo[search_rule_idx].usCID = ntohs(add_indication->u16CID);

				if (UPLINK_DIR == add_indication->u8Direction)
					atomic_set(&ad->PackInfo[search_rule_idx].uiPerSFTxResourceCount, DEFAULT_PERSFCOUNT);

				CopyToAdapter(ad, psfLocalSet, search_rule_idx, DSA_ACK, add_indication);
				/* don't free add_indication */

				/* Inside CopyToAdapter, Sorting of all the SFs take place.
				 * Hence any access to the newly added SF through search_rule_idx is invalid.
				 * SHOULD BE STRICTLY AVOIDED.
				 */
				/* *(PULONG)(((PUCHAR)pvBuffer)+1)=psfLocalSet->u32SFID; */
				memcpy((((PUCHAR)pvBuffer)+1), &psfLocalSet->u32SFID, 4);

				if (add_indication->sfActiveSet.bValid == TRUE) {
					if (UPLINK_DIR == add_indication->u8Direction) {
						if (!ad->LinkUpStatus) {
							netif_carrier_on(ad->dev);
							netif_start_queue(ad->dev);
							ad->LinkUpStatus = 1;
							if (netif_msg_link(ad))
								pr_info(PFX "%s: link up\n", ad->dev->name);
							atomic_set(&ad->TxPktAvail, 1);
							wake_up(&ad->tx_packet_wait_queue);
							ad->liTimeSinceLastNetEntry = get_seconds();
						}
					}
				}
			} else {
				ad->PackInfo[search_rule_idx].bActive = false;
				ad->PackInfo[search_rule_idx].bValid = false;
				ad->PackInfo[search_rule_idx].usVCID_Value = 0;
				kfree(add_indication);
			}
		} else {
			BCM_DEBUG_PRINT(ad, DBG_TYPE_PRINTK, 0, 0, "DSA ACK did not get valid SFID");
			kfree(add_indication);
			return false;
		}
		break;
	case DSC_REQ:
		pLeader->PLength = sizeof(struct bcm_change_indication);
		pstChangeIndication = (struct bcm_change_indication *)add_indication;
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL, "SENDING DSC RESPONSE TO MAC %d", pLeader->PLength);

		*((struct bcm_change_indication *)&(ad->caDsxReqResp[LEADER_SIZE])) = *pstChangeIndication;
		((struct bcm_change_indication *)&(ad->caDsxReqResp[LEADER_SIZE]))->u8Type = DSC_RSP;

		CopyBufferToControlPacket(ad, (PVOID)ad->caDsxReqResp);
		kfree(add_indication);
		break;
	case DSC_RSP:
		pLeader->PLength = sizeof(struct bcm_change_indication);
		pstChangeIndication = (struct bcm_change_indication *)add_indication;
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL, "SENDING DSC ACK TO MAC %d", pLeader->PLength);
		*((struct bcm_change_indication *)&(ad->caDsxReqResp[LEADER_SIZE])) = *pstChangeIndication;
		((struct bcm_change_indication *)&(ad->caDsxReqResp[LEADER_SIZE]))->u8Type = DSC_ACK;
		/* FALLTHROUGH */
	case DSC_ACK:
		pstChangeIndication = (struct bcm_change_indication *)add_indication;
		search_rule_idx = SearchSfid(ad, ntohl(pstChangeIndication->sfActiveSet.u32SFID));
		if (search_rule_idx > NO_OF_QUEUES-1)
			BCM_DEBUG_PRINT(ad, DBG_TYPE_PRINTK, 0, 0, "SF doesn't exist for which DSC_ACK is received");

		if (search_rule_idx < NO_OF_QUEUES) {
			ad->PackInfo[search_rule_idx].ucDirection = pstChangeIndication->u8Direction;
			if (pstChangeIndication->sfActiveSet.bValid == TRUE)
				ad->PackInfo[search_rule_idx].bActiveSet = TRUE;

			if (pstChangeIndication->sfAuthorizedSet.bValid == TRUE)
				ad->PackInfo[search_rule_idx].bAuthorizedSet = TRUE;

			if (pstChangeIndication->sfAdmittedSet.bValid == TRUE)
				ad->PackInfo[search_rule_idx].bAdmittedSet = TRUE;

			if (pstChangeIndication->sfActiveSet.bValid == false) {
				ad->PackInfo[search_rule_idx].bActive = false;
				ad->PackInfo[search_rule_idx].bActivateRequestSent = false;

				if (pstChangeIndication->sfAdmittedSet.bValid)
					psfLocalSet = &pstChangeIndication->sfAdmittedSet;
				else if (pstChangeIndication->sfAuthorizedSet.bValid)
					psfLocalSet = &pstChangeIndication->sfAuthorizedSet;
			} else {
				psfLocalSet = &pstChangeIndication->sfActiveSet;
				ad->PackInfo[search_rule_idx].bActive = TRUE;
			}

			if (!psfLocalSet) {
				BCM_DEBUG_PRINT(ad, DBG_TYPE_PRINTK, 0, 0, "No set is valid\n");
				ad->PackInfo[search_rule_idx].bActive = false;
				ad->PackInfo[search_rule_idx].bValid = false;
				ad->PackInfo[search_rule_idx].usVCID_Value = 0;
				kfree(add_indication);
			} else if (psfLocalSet->bValid && (pstChangeIndication->u8CC == 0)) {
				ad->PackInfo[search_rule_idx].usVCID_Value = ntohs(pstChangeIndication->u16VCID);
				BCM_DEBUG_PRINT(ad, DBG_TYPE_PRINTK, 0, 0, "CC field is %d bvalid = %d\n",
						pstChangeIndication->u8CC, psfLocalSet->bValid);
				BCM_DEBUG_PRINT(ad, DBG_TYPE_PRINTK, 0, 0, "VCID= %d\n", ntohs(pstChangeIndication->u16VCID));
				ad->PackInfo[search_rule_idx].usCID = ntohs(pstChangeIndication->u16CID);
				CopyToAdapter(ad, psfLocalSet, search_rule_idx, DSC_ACK, add_indication);

				*(PULONG)(((PUCHAR)pvBuffer)+1) = psfLocalSet->u32SFID;
			} else if (pstChangeIndication->u8CC == 6) {
				deleteSFBySfid(ad, search_rule_idx);
				kfree(add_indication);
			}
		} else {
			BCM_DEBUG_PRINT(ad, DBG_TYPE_PRINTK, 0, 0, "DSC ACK did not get valid SFID");
			kfree(add_indication);
			return false;
		}
		break;
	case DSD_REQ:
		pLeader->PLength = sizeof(struct bcm_del_indication);
		*((struct bcm_del_indication *)&(ad->caDsxReqResp[LEADER_SIZE])) = *((struct bcm_del_indication *)add_indication);

		sf_id = ntohl(((struct bcm_del_indication *)add_indication)->u32SFID);
		search_rule_idx = SearchSfid(ad, sf_id);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL, "DSD - Removing connection %x", search_rule_idx);

		if (search_rule_idx < NO_OF_QUEUES) {
			/* Delete All Classifiers Associated with this SFID */
			deleteSFBySfid(ad, search_rule_idx);
			ad->u32TotalDSD++;
		}

		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL, "SENDING DSD RESPONSE TO MAC");
		((struct bcm_del_indication *)&(ad->caDsxReqResp[LEADER_SIZE]))->u8Type = DSD_RSP;
		CopyBufferToControlPacket(ad, (PVOID)ad->caDsxReqResp);
		/* FALLTHROUGH */
	case DSD_RSP:
		/* Do nothing as SF has already got Deleted */
		break;
	case DSD_ACK:
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL, "DSD ACK Rcd, let App handle it\n");
		break;
	default:
		kfree(add_indication);
		return false;
	}
	return TRUE;
}

int get_dsx_sf_data_to_application(struct bcm_mini_adapter *ad,
		UINT uiSFId, void __user *user_buffer)
{
	int status = 0;
	struct bcm_packet_info *psSfInfo = NULL;

	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
			"status =%d", status);
	status = SearchSfid(ad, uiSFId);
	if (status >= NO_OF_QUEUES) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
				"SFID %d not present in queue !!!", uiSFId);
		return -EINVAL;
	}
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
			"status =%d", status);
	psSfInfo = &ad->PackInfo[status];
	if (psSfInfo->pstSFIndication
			&& copy_to_user(user_buffer, psSfInfo->pstSFIndication,
				sizeof(struct bcm_add_indication_alt))) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_PRINTK, 0, 0,
				"copy to user failed SFID %d, present in queue !!!",
				uiSFId);
		status = -EFAULT;
		return status;
	}
	return STATUS_SUCCESS;
}

VOID OverrideServiceFlowParams(struct bcm_mini_adapter *ad,
		PUINT puiBuffer)
{
	B_UINT32 u32NumofSFsinMsg = ntohl(*(puiBuffer + 1));
	struct bcm_stim_sfhostnotify *pHostInfo = NULL;
	UINT search_rule_idx = 0;
	ULONG sf_id = 0;

	puiBuffer += 2;
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
			"u32NumofSFsinMsg: 0x%x\n", u32NumofSFsinMsg);

	while (u32NumofSFsinMsg != 0 && u32NumofSFsinMsg < NO_OF_QUEUES) {
		u32NumofSFsinMsg--;
		pHostInfo = (struct bcm_stim_sfhostnotify *)puiBuffer;
		puiBuffer = (PUINT)(pHostInfo + 1);

		sf_id = ntohl(pHostInfo->SFID);
		search_rule_idx = SearchSfid(ad, sf_id);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
				"SFID: 0x%lx\n", sf_id);

		if (search_rule_idx >= NO_OF_QUEUES
				|| search_rule_idx == HiPriority) {
			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG,
					DBG_LVL_ALL,
					"The SFID <%lx> doesn't exist in host entry or is Invalid\n",
					sf_id);
			continue;
		}

		if (pHostInfo->RetainSF == false) {
			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG,
					DBG_LVL_ALL, "Going to Delete SF");
			deleteSFBySfid(ad, search_rule_idx);
		} else {
			struct bcm_packet_info *packinfo =
				&ad->PackInfo[search_rule_idx];

			packinfo->usVCID_Value = ntohs(pHostInfo->VCID);
			packinfo->usCID = ntohs(pHostInfo->newCID);
			packinfo->bActive = false;

			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG,
					DBG_LVL_ALL,
					"pHostInfo->QoSParamSet: 0x%x\n",
					pHostInfo->QoSParamSet);

			if (pHostInfo->QoSParamSet & 0x1)
				packinfo->bAuthorizedSet = TRUE;
			if (pHostInfo->QoSParamSet & 0x2)
				packinfo->bAdmittedSet = TRUE;
			if (pHostInfo->QoSParamSet & 0x4) {
				packinfo->bActiveSet = TRUE;
				packinfo->bActive = TRUE;
			}
		}
	}
}

static void restore_endianess_of_classifier_entry(
		struct bcm_classifier_rule *classifier_entry,
		enum bcm_ipaddr_context ip_addr_context)
{
	int i;
	union u_ip_address *stSrc  = &classifier_entry->stSrcIpAddress;
	union u_ip_address *stDest = &classifier_entry->stDestIpAddress;

	for (i = 0; i < MAX_IP_RANGE_LENGTH * 4; i++) {
		if (ip_addr_context == eSrcIpAddress) {
			stSrc->ulIpv6Addr[i] = ntohl(stSrc->ulIpv6Addr[i]);
			stSrc->ulIpv6Mask[i] = ntohl(stSrc->ulIpv6Mask[i]);
		} else if (ip_addr_context == eDestIpAddress) {
			stDest->ulIpv6Addr[i] = ntohl(stDest->ulIpv6Addr[i]);
			stDest->ulIpv6Mask[i] = ntohl(stDest->ulIpv6Mask[i]);
		}
	}
}

static void apply_phs_rule_to_all_classifiers(
		register struct bcm_mini_adapter *ad,		/* <Pointer to the ad structure */
		register UINT search_rule_idx,			/* <Index of Queue, to which this data belongs */
		USHORT vcid,
		struct bcm_phs_rule *phs_rule,
		struct bcm_phs_rules *c_phs_rules,
		struct bcm_add_indication_alt *add_indication)
{
	unsigned int uiClassifierIndex = 0;
	struct bcm_classifier_rule *curr_classifier = NULL;

	if (add_indication->u8Direction == UPLINK_DIR) {
		for (uiClassifierIndex = 0; uiClassifierIndex < MAX_CLASSIFIERS; uiClassifierIndex++) {
			curr_classifier =
				&ad->astClassifierTable[uiClassifierIndex];
			if ((curr_classifier->bUsed) &&
				(curr_classifier->ulSFID == ad->PackInfo[search_rule_idx].ulSFID) &&
				(curr_classifier->u8AssociatedPHSI == c_phs_rules->u8PHSI)) {
				BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, CONN_MSG, DBG_LVL_ALL,
						"Adding PHS Rule For Classifier: 0x%x c_phs_rules.u8PHSI: 0x%x\n",
						curr_classifier->uiClassifierRuleIndex,
						c_phs_rules->u8PHSI);
				/* Update The PHS Rule for this classifier as Associated PHSI id defined */

				/* Copy the PHS Rule */
				phs_rule->u8PHSI = c_phs_rules->u8PHSI;
				phs_rule->u8PHSFLength = c_phs_rules->u8PHSFLength;
				phs_rule->u8PHSMLength = c_phs_rules->u8PHSMLength;
				phs_rule->u8PHSS = c_phs_rules->u8PHSS;
				phs_rule->u8PHSV = c_phs_rules->u8PHSV;
				memcpy(phs_rule->u8PHSF, c_phs_rules->u8PHSF, MAX_PHS_LENGTHS);
				memcpy(phs_rule->u8PHSM, c_phs_rules->u8PHSM, MAX_PHS_LENGTHS);
				phs_rule->u8RefCnt = 0;
				phs_rule->bUnclassifiedPHSRule = false;
				phs_rule->PHSModifiedBytes = 0;
				phs_rule->PHSModifiedNumPackets = 0;
				phs_rule->PHSErrorNumPackets = 0;

				/* bPHSRuleAssociated = TRUE; */
				/* Store The PHS Rule for this classifier */

				PhsUpdateClassifierRule(
					&ad->stBCMPhsContext,
					vcid,
					curr_classifier->uiClassifierRuleIndex,
					phs_rule,
					curr_classifier->u8AssociatedPHSI);

				/* Update PHS Rule For the Classifier */
				if (phs_rule->u8PHSI) {
					curr_classifier->u32PHSRuleID = phs_rule->u8PHSI;
					memcpy(&curr_classifier->sPhsRule, phs_rule, sizeof(struct bcm_phs_rule));
				}
			}
		}
	} else {
		/* Error PHS Rule specified in signaling could not be applied to any classifier */

		/* Copy the PHS Rule */
		phs_rule->u8PHSI = c_phs_rules->u8PHSI;
		phs_rule->u8PHSFLength = c_phs_rules->u8PHSFLength;
		phs_rule->u8PHSMLength = c_phs_rules->u8PHSMLength;
		phs_rule->u8PHSS = c_phs_rules->u8PHSS;
		phs_rule->u8PHSV = c_phs_rules->u8PHSV;
		memcpy(phs_rule->u8PHSF, c_phs_rules->u8PHSF, MAX_PHS_LENGTHS);
		memcpy(phs_rule->u8PHSM, c_phs_rules->u8PHSM, MAX_PHS_LENGTHS);
		phs_rule->u8RefCnt = 0;
		phs_rule->bUnclassifiedPHSRule = TRUE;
		phs_rule->PHSModifiedBytes = 0;
		phs_rule->PHSModifiedNumPackets = 0;
		phs_rule->PHSErrorNumPackets = 0;
		/* Store The PHS Rule for this classifier */

		/*
		 * Passing the argument u8PHSI instead of clsid. Because for DL with no classifier rule,
		 * clsid will be zero hence we can't have multiple PHS rules for the same SF.
		 * To support multiple PHS rule, passing u8PHSI.
		 */
		PhsUpdateClassifierRule(
			&ad->stBCMPhsContext,
			vcid,
			phs_rule->u8PHSI,
			phs_rule,
			phs_rule->u8PHSI);
	}
}
