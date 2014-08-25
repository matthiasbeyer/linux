#include "headers.h"

static bool MatchSrcIpv6Address(struct bcm_classifier_rule *classifier_rule,
	struct bcm_ipv6_hdr *ipv6_hdr);
static bool MatchDestIpv6Address(struct bcm_classifier_rule *classifier_rule,
	struct bcm_ipv6_hdr *ipv6_hdr);
static VOID DumpIpv6Header(struct bcm_ipv6_hdr *ipv6_hdr);

static UCHAR *GetNextIPV6ChainedHeader(UCHAR **payload,
	UCHAR *nxt_hdr, bool *parse_done, USHORT *payload_len)
{
	UCHAR *ret_hdr = NULL;
	UCHAR *pld = NULL;
	USHORT  nxt_hdr_offset = 0;
	struct bcm_mini_adapter *ad = GET_BCM_ADAPTER(gblpnetdev);

	if ((payload == NULL) || (*payload_len == 0) ||
		(*parse_done)) {
		*parse_done = TRUE;
		return NULL;
	}

	ret_hdr = *payload;
	pld = *payload;

	if (!ret_hdr || !pld) {
		*parse_done = TRUE;
		return NULL;
	}

	/* Get the Nextt Header Type */
	*parse_done = false;


	switch (*nxt_hdr) {
	case IPV6HDR_TYPE_HOPBYHOP:
		BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, IPV6_DBG,
				DBG_LVL_ALL, "\nIPv6 HopByHop Header");
		nxt_hdr_offset += sizeof(struct bcm_ipv6_options_hdr);
		break;

	case IPV6HDR_TYPE_ROUTING:
		{
			struct bcm_ipv6_routing_hdr *pstIpv6RoutingHeader;

			BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, IPV6_DBG,
					DBG_LVL_ALL, "\nIPv6 Routing Header");
			pstIpv6RoutingHeader =
				(struct bcm_ipv6_routing_hdr *)pld;
			nxt_hdr_offset += sizeof(struct bcm_ipv6_routing_hdr);
			nxt_hdr_offset += pstIpv6RoutingHeader->ucNumAddresses *
					      IPV6_ADDRESS_SIZEINBYTES;
		}
		break;

	case IPV6HDR_TYPE_FRAGMENTATION:
		BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, IPV6_DBG,
				DBG_LVL_ALL,
				"\nIPv6 Fragmentation Header");
		nxt_hdr_offset += sizeof(struct bcm_ipv6_fragment_hdr);
		break;

	case IPV6HDR_TYPE_DESTOPTS:
		{
			struct bcm_ipv6_dest_options_hdr *pstIpv6DestOptsHdr =
				(struct bcm_ipv6_dest_options_hdr *)pld;
			int nTotalOptions = pstIpv6DestOptsHdr->ucHdrExtLen;

			BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, IPV6_DBG,
					DBG_LVL_ALL,
					"\nIPv6 DestOpts Header Header");
			nxt_hdr_offset += sizeof(struct bcm_ipv6_dest_options_hdr);
			nxt_hdr_offset += nTotalOptions *
					      IPV6_DESTOPTS_HDR_OPTIONSIZE;
		}
		break;


	case IPV6HDR_TYPE_AUTHENTICATION:
		{
			struct bcm_ipv6_authentication_hdr *pstIpv6AuthHdr =
				(struct bcm_ipv6_authentication_hdr *)pld;
			int nHdrLen = pstIpv6AuthHdr->ucLength;

			BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, IPV6_DBG,
					DBG_LVL_ALL,
					"\nIPv6 Authentication Header");
			nxt_hdr_offset += nHdrLen * 4;
		}
		break;

	case IPV6HDR_TYPE_ENCRYPTEDSECURITYPAYLOAD:
		BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, IPV6_DBG,
				DBG_LVL_ALL,
				"\nIPv6 Encrypted Security Payload Header");
		*parse_done = TRUE;
		break;

	case IPV6_ICMP_HDR_TYPE:
		BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, IPV6_DBG,
				DBG_LVL_ALL, "\nICMP Header");
		*parse_done = TRUE;
		break;

	case TCP_HEADER_TYPE:
		BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, IPV6_DBG,
				DBG_LVL_ALL, "\nTCP Header");
		*parse_done = TRUE;
		break;

	case UDP_HEADER_TYPE:
		BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, IPV6_DBG,
				DBG_LVL_ALL, "\nUDP Header");
		*parse_done = TRUE;
		break;

	default:
		*parse_done = TRUE;
		break;
	}

	if (*parse_done == false) {
		if (*payload_len <= nxt_hdr_offset) {
			*parse_done = TRUE;
		} else {
			*nxt_hdr = *pld;
			pld += nxt_hdr_offset;
			(*payload_len) -= nxt_hdr_offset;
		}

	}

	*payload = pld;
	return ret_hdr;
}


static UCHAR GetIpv6ProtocolPorts(UCHAR *payload, USHORT *src_port,
	USHORT *dest_port, USHORT payload_len, UCHAR next_header)
{
	UCHAR *ipv6_hdr_scan_context = payload;
	bool done = false;
	UCHAR hdr_type = 0;
	UCHAR *nxt_hdr = NULL;
	struct bcm_mini_adapter *ad = GET_BCM_ADAPTER(gblpnetdev);

	if (!payload || (payload_len == 0))
		return 0;

	*src_port = *dest_port = 0;
	hdr_type = next_header;
	while (!done) {
		nxt_hdr = GetNextIPV6ChainedHeader(&ipv6_hdr_scan_context,
							 &hdr_type,
							 &done,
							 &payload_len);
		if (done) {
			if ((hdr_type == TCP_HEADER_TYPE) ||
				(hdr_type == UDP_HEADER_TYPE)) {
				*src_port = *((PUSHORT)(nxt_hdr));
				*dest_port = *((PUSHORT)(nxt_hdr+2));
				BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, IPV6_DBG,
						DBG_LVL_ALL,
						"\nProtocol Ports - Src Port :0x%x Dest Port : 0x%x",
						ntohs(*src_port),
						ntohs(*dest_port));
			}
			break;

		}
	}
	return hdr_type;
}


/*
 * Arg 1 struct bcm_mini_adapter *ad is a pointer ot the driver control
 * structure
 * Arg 2 PVOID ip_hdr is a pointer to the IP header of the packet
 */
USHORT	IpVersion6(struct bcm_mini_adapter *ad, PVOID ip_hdr,
		   struct bcm_classifier_rule *classifier_rule)
{
	USHORT	dest_port = 0;
	USHORT	src_port = 0;
	UCHAR   nxt_prot_above_ip = 0;
	struct bcm_ipv6_hdr *ipv6_hdr = NULL;
	bool classif_succeed = false;

	BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, IPV6_DBG,
			DBG_LVL_ALL, "IpVersion6 ==========>\n");

	ipv6_hdr = ip_hdr;

	DumpIpv6Header(ipv6_hdr);

	/*
	 * Try to get the next higher layer protocol
	 * and the Ports Nos if TCP or UDP
	 */
	nxt_prot_above_ip = GetIpv6ProtocolPorts((UCHAR *)(ip_hdr +
						     sizeof(struct bcm_ipv6_hdr)),
						     &src_port,
						     &dest_port,
						     ipv6_hdr->usPayloadLength,
						     ipv6_hdr->ucNextHeader);

	do {
		if (classifier_rule->ucDirection == 0) {
			/*
			 * cannot be processed for classification.
			 * it is a down link connection
			 */
			break;
		}

		if (!classifier_rule->bIpv6Protocol) {
			/*
			 * We are looking for Ipv6 Classifiers
			 * Lets ignore this classifier and try the next one
			 */
			break;
		}

		classif_succeed = MatchSrcIpv6Address(classifier_rule,
							     ipv6_hdr);
		if (!classif_succeed)
			break;

		classif_succeed = MatchDestIpv6Address(classifier_rule,
							      ipv6_hdr);
		if (!classif_succeed)
			break;

		/*
		 * Match the protocol type.
		 * For IPv6 the next protocol at end of
		 * Chain of IPv6 prot headers
		 */
		classif_succeed = MatchProtocol(classifier_rule,
						       nxt_prot_above_ip);
		if (!classif_succeed)
			break;

		BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, IPV6_DBG,
				DBG_LVL_ALL, "\nIPv6 Protocol Matched");

		if ((nxt_prot_above_ip == TCP_HEADER_TYPE) ||
			(nxt_prot_above_ip == UDP_HEADER_TYPE)) {
			/* Match Src Port */
			BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, IPV6_DBG,
					DBG_LVL_ALL, "\nIPv6 Source Port:%x\n",
					ntohs(src_port));
			classif_succeed = MatchSrcPort(classifier_rule,
							      ntohs(src_port));
			if (!classif_succeed)
				break;

			BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, IPV6_DBG,
					DBG_LVL_ALL, "\nIPv6 Src Port Matched");

			/* Match Dest Port */
			BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, IPV6_DBG,
					DBG_LVL_ALL,
					"\nIPv6 Destination Port:%x\n",
					ntohs(dest_port));
			classif_succeed = MatchDestPort(classifier_rule,
							       ntohs(dest_port));
			if (!classif_succeed)
				break;
			BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, IPV6_DBG,
					DBG_LVL_ALL,
					"\nIPv6 Dest Port Matched");
		}
	} while (0);

	if (classif_succeed == TRUE) {
		INT iMatchedSFQueueIndex = 0;

		iMatchedSFQueueIndex = SearchSfid(ad,
						  classifier_rule->ulSFID);
		if ((iMatchedSFQueueIndex >= NO_OF_QUEUES) ||
		    (ad->PackInfo[iMatchedSFQueueIndex].bActive == false))
			classif_succeed = false;
	}

	return classif_succeed;
}


static bool MatchSrcIpv6Address(struct bcm_classifier_rule *classifier_rule,
				struct bcm_ipv6_hdr *ipv6_hdr)
{
	UINT i = 0;
	UINT ipv6_add_idx = 0;
	UINT ipv6_addr_no_long_words = 4;
	ULONG aulSrcIP[4];
	struct bcm_mini_adapter *ad = GET_BCM_ADAPTER(gblpnetdev);
	union u_ip_address *src_addr = &classifier_rule->stSrcIpAddress;

	/*
	 * This is the no. of Src Addresses ie Range of IP Addresses contained
	 * in the classifier rule for which we need to match
	 */
	UINT  uiCountIPSrcAddresses =
		(UINT)classifier_rule->ucIPSourceAddressLength;


	if (uiCountIPSrcAddresses == 0)
		return TRUE;


	/* First Convert the Ip Address in the packet to Host Endian order */
	for (ipv6_add_idx = 0;
	     ipv6_add_idx < ipv6_addr_no_long_words;
	     ipv6_add_idx++)
		aulSrcIP[ipv6_add_idx] =
			ntohl(ipv6_hdr->ulSrcIpAddress[ipv6_add_idx]);

	for (i = 0;
	     i < uiCountIPSrcAddresses;
	     i += ipv6_addr_no_long_words) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, IPV6_DBG, DBG_LVL_ALL,
				"\n Src Ipv6 Address In Received Packet :\n ");
		DumpIpv6Address(aulSrcIP);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, IPV6_DBG, DBG_LVL_ALL,
				"\n Src Ipv6 Mask In Classifier Rule:\n");
		DumpIpv6Address(&src_addr->ulIpv6Mask[i]);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, IPV6_DBG, DBG_LVL_ALL,
				"\n Src Ipv6 Address In Classifier Rule :\n");
		DumpIpv6Address(&src_addr->ulIpv6Addr[i]);

		for (ipv6_add_idx = 0;
		     ipv6_add_idx < ipv6_addr_no_long_words;
		     ipv6_add_idx++) {
			if ((src_addr->ulIpv6Mask[i+ipv6_add_idx] &
				aulSrcIP[ipv6_add_idx]) !=
			    src_addr->ulIpv6Addr[i+ipv6_add_idx]) {
				/*
				 * Match failed for current Ipv6 Address
				 * Try next Ipv6 Address
				 */
				break;
			}

			if (ipv6_add_idx ==  ipv6_addr_no_long_words-1) {
				/* Match Found */
				BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, IPV6_DBG,
						DBG_LVL_ALL,
						"Ipv6 Src Ip Address Matched\n");
				return TRUE;
			}
		}
	}
	return false;
}

static bool MatchDestIpv6Address(struct bcm_classifier_rule *classifier_rule,
				 struct bcm_ipv6_hdr *ipv6_hdr)
{
	UINT i = 0;
	UINT ipv6_add_idx = 0;
	UINT ipv6_addr_no_long_words = 4;
	ULONG aulDestIP[4];
	struct bcm_mini_adapter *ad = GET_BCM_ADAPTER(gblpnetdev);
	union u_ip_address *dest_addr = &classifier_rule->stDestIpAddress;

	/*
	 * This is the no. of Destination Addresses
	 * ie Range of IP Addresses contained in the classifier rule
	 * for which we need to match
	 */
	UINT uiCountIPDestinationAddresses =
		(UINT)classifier_rule->ucIPDestinationAddressLength;

	if (uiCountIPDestinationAddresses == 0)
		return TRUE;


	/* First Convert the Ip Address in the packet to Host Endian order */
	for (ipv6_add_idx = 0;
	     ipv6_add_idx < ipv6_addr_no_long_words;
	     ipv6_add_idx++)
		aulDestIP[ipv6_add_idx] =
			ntohl(ipv6_hdr->ulDestIpAddress[ipv6_add_idx]);

	for (i = 0;
	     i < uiCountIPDestinationAddresses;
	     i += ipv6_addr_no_long_words) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, IPV6_DBG, DBG_LVL_ALL,
				"\n Destination Ipv6 Address In Received Packet :\n ");
		DumpIpv6Address(aulDestIP);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, IPV6_DBG, DBG_LVL_ALL,
				"\n Destination Ipv6 Mask In Classifier Rule :\n");
		DumpIpv6Address(&dest_addr->ulIpv6Mask[i]);
		BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, IPV6_DBG, DBG_LVL_ALL,
				"\n Destination Ipv6 Address In Classifier Rule :\n");
		DumpIpv6Address(&dest_addr->ulIpv6Addr[i]);

		for (ipv6_add_idx = 0;
		     ipv6_add_idx < ipv6_addr_no_long_words;
		     ipv6_add_idx++) {
			if ((dest_addr->ulIpv6Mask[i+ipv6_add_idx] &
				aulDestIP[ipv6_add_idx]) !=
			    dest_addr->ulIpv6Addr[i+ipv6_add_idx]) {
				/*
				 * Match failed for current Ipv6 Address.
				 * Try next Ipv6 Address
				 */
				break;
			}

			if (ipv6_add_idx ==  ipv6_addr_no_long_words-1) {
				/* Match Found */
				BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, IPV6_DBG,
						DBG_LVL_ALL,
						"Ipv6 Destination Ip Address Matched\n");
				return TRUE;
			}
		}
	}
	return false;

}

VOID DumpIpv6Address(ULONG *puIpv6Address)
{
	UINT ipv6_addr_no_long_words = 4;
	UINT ipv6_add_idx = 0;
	struct bcm_mini_adapter *ad = GET_BCM_ADAPTER(gblpnetdev);

	for (ipv6_add_idx = 0;
	     ipv6_add_idx < ipv6_addr_no_long_words;
	     ipv6_add_idx++) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, IPV6_DBG, DBG_LVL_ALL,
				":%lx", puIpv6Address[ipv6_add_idx]);
	}

}

static VOID DumpIpv6Header(struct bcm_ipv6_hdr *ipv6_hdr)
{
	UCHAR ucVersion;
	UCHAR ucPrio;
	struct bcm_mini_adapter *ad = GET_BCM_ADAPTER(gblpnetdev);

	BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, IPV6_DBG, DBG_LVL_ALL,
			"----Ipv6 Header---");
	ucVersion = ipv6_hdr->ucVersionPrio & 0xf0;
	BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, IPV6_DBG, DBG_LVL_ALL,
			"Version : %x\n", ucVersion);
	ucPrio = ipv6_hdr->ucVersionPrio & 0x0f;
	BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, IPV6_DBG, DBG_LVL_ALL,
			"Priority : %x\n", ucPrio);
	/*
	 * BCM_DEBUG_PRINT( ad,DBG_TYPE_TX, IPV6_DBG, DBG_LVL_ALL,
	 * "Flow Label : %x\n",(ipv6_hdr->ucVersionPrio &0xf0);
	 */
	BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, IPV6_DBG, DBG_LVL_ALL,
			"Payload Length : %x\n",
			ntohs(ipv6_hdr->usPayloadLength));
	BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, IPV6_DBG, DBG_LVL_ALL,
			"Next Header : %x\n", ipv6_hdr->ucNextHeader);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, IPV6_DBG, DBG_LVL_ALL,
			"Hop Limit : %x\n", ipv6_hdr->ucHopLimit);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, IPV6_DBG, DBG_LVL_ALL,
			"Src Address :\n");
	DumpIpv6Address(ipv6_hdr->ulSrcIpAddress);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, IPV6_DBG, DBG_LVL_ALL,
			"Dest Address :\n");
	DumpIpv6Address(ipv6_hdr->ulDestIpAddress);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_TX, IPV6_DBG, DBG_LVL_ALL,
			"----Ipv6 Header End---");


}
