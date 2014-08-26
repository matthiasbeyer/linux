#include "headers.h"

#define STATUS_IMAGE_CHECKSUM_MISMATCH -199
#define EVENT_SIGNALED 1

static B_UINT16 CFG_CalculateChecksum(B_UINT8 *buff, B_UINT32 size)
{
	B_UINT16 u16CheckSum = 0;

	while (size--) {
		u16CheckSum += (B_UINT8)~(*buff);
		buff++;
	}
	return u16CheckSum;
}

bool IsReqGpioIsLedInNVM(struct bcm_mini_adapter *ad, UINT gpios)
{
	INT status;

	status = (ad->gpioBitMap & gpios) ^ gpios;
	if (status)
		return false;
	else
		return TRUE;
}

static INT LED_Blink(struct bcm_mini_adapter *ad,
		     UINT gpio_num,
		     UCHAR led_idx,
		     ULONG timeout,
		     INT num_of_time,
		     enum bcm_led_events currdriverstate)
{
	int status = STATUS_SUCCESS;
	bool infinite = false;

	/* Check if num_of_time is -ve. If yes, blink led in infinite loop */
	if (num_of_time < 0) {
		infinite = TRUE;
		num_of_time = 1;
	}
	while (num_of_time) {
		if (currdriverstate == ad->DriverState)
			TURN_ON_LED(ad, gpio_num, led_idx);

		/* Wait for timeout after setting on the LED */
		status = wait_event_interruptible_timeout(
				ad->LEDInfo.notify_led_event,
				currdriverstate != ad->DriverState ||
					kthread_should_stop(),
				msecs_to_jiffies(timeout));

		if (kthread_should_stop()) {
			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, LED_DUMP_INFO,
				DBG_LVL_ALL,
				"Led thread got signal to exit..hence exiting");
			ad->LEDInfo.led_thread_running =
					BCM_LED_THREAD_DISABLED;
			TURN_OFF_LED(ad, gpio_num, led_idx);
			status = EVENT_SIGNALED;
			break;
		}
		if (status) {
			TURN_OFF_LED(ad, gpio_num, led_idx);
			status = EVENT_SIGNALED;
			break;
		}

		TURN_OFF_LED(ad, gpio_num, led_idx);
		status = wait_event_interruptible_timeout(
				ad->LEDInfo.notify_led_event,
				currdriverstate != ad->DriverState ||
					kthread_should_stop(),
				msecs_to_jiffies(timeout));
		if (infinite == false)
			num_of_time--;
	}
	return status;
}

static INT ScaleRateofTransfer(ULONG rate)
{
	if (rate <= 3)
		return rate;
	else if ((rate > 3) && (rate <= 100))
		return 5;
	else if ((rate > 100) && (rate <= 200))
		return 6;
	else if ((rate > 200) && (rate <= 300))
		return 7;
	else if ((rate > 300) && (rate <= 400))
		return 8;
	else if ((rate > 400) && (rate <= 500))
		return 9;
	else if ((rate > 500) && (rate <= 600))
		return 10;
	else
		return MAX_NUM_OF_BLINKS;
}

static INT blink_in_normal_bandwidth(struct bcm_mini_adapter *ad,
				     INT *time,
				     INT *time_tx,
				     INT *time_rx,
				     UCHAR gpio_num_tx,
				     UCHAR tx_led_idx,
				     UCHAR gpio_num_rx,
				     UCHAR rx_led_idx,
				     enum bcm_led_events currdriverstate,
				     ulong *timeout)
{
	/*
	 * Assign minimum number of blinks of
	 * either Tx or Rx.
	 */
	*time = (*time_tx > *time_rx ? *time_rx : *time_tx);

	if (*time > 0) {
		/* Blink both Tx and Rx LEDs */
		if ((LED_Blink(ad, 1 << gpio_num_tx, tx_led_idx, *timeout,
			      *time, currdriverstate) == EVENT_SIGNALED) ||
		    (LED_Blink(ad, 1 << gpio_num_rx, rx_led_idx, *timeout,
			      *time, currdriverstate) == EVENT_SIGNALED))
			return EVENT_SIGNALED;
	}

	if (*time == *time_tx) {
		/* Blink pending rate of Rx */
		if (LED_Blink(ad, (1 << gpio_num_rx), rx_led_idx, *timeout,
			      *time_rx - *time,
			      currdriverstate) == EVENT_SIGNALED)
			return EVENT_SIGNALED;

		*time = *time_rx;
	} else {
		/* Blink pending rate of Tx */
		if (LED_Blink(ad, 1 << gpio_num_tx, tx_led_idx, *timeout,
			      *time_tx - *time,
			      currdriverstate) == EVENT_SIGNALED)
			return EVENT_SIGNALED;

		*time = *time_tx;
	}

	return 0;
}

static INT LED_Proportional_Blink(struct bcm_mini_adapter *ad,
				  UCHAR gpio_num_tx,
				  UCHAR tx_led_idx,
				  UCHAR gpio_num_rx,
				  UCHAR rx_led_idx,
				  enum bcm_led_events currdriverstate)
{
	/* Initial values of TX and RX packets */
	ULONG64 init_num_tx_pkts = 0, init_num_rx_pkts = 0;
	/* values of TX and RX packets after 1 sec */
	ULONG64 final_num_tx_pkts = 0, final_num_rx_pkts = 0;
	/* Rate of transfer of Tx and Rx in 1 sec */
	ULONG64 rate_of_transfer_tx = 0, rate_of_transfer_rx = 0;
	int status = STATUS_SUCCESS;
	INT num_of_time = 0, num_of_time_tx = 0, num_of_time_rx = 0;
	UINT rem_delay = 0;
	/* UINT GPIO_num = DISABLE_GPIO_NUM; */
	ulong timeout = 0;

	/* Read initial value of packets sent/received */
	init_num_tx_pkts = ad->dev->stats.tx_packets;
	init_num_rx_pkts = ad->dev->stats.rx_packets;

	/* Scale the rate of transfer to no of blinks. */
	num_of_time_tx = ScaleRateofTransfer((ULONG)rate_of_transfer_tx);
	num_of_time_rx = ScaleRateofTransfer((ULONG)rate_of_transfer_rx);

	while ((ad->device_removed == false)) {
		timeout = 50;

		if (EVENT_SIGNALED == blink_in_normal_bandwidth(ad,
								&num_of_time,
								&num_of_time_tx,
								&num_of_time_rx,
								gpio_num_tx,
								tx_led_idx,
								gpio_num_rx,
								rx_led_idx,
								currdriverstate,
								&timeout))
			return EVENT_SIGNALED;


		/*
		 * If Tx/Rx rate is less than maximum blinks per second,
		 * wait till delay completes to 1 second
		 */
		rem_delay = MAX_NUM_OF_BLINKS - num_of_time;
		if (rem_delay > 0) {
			timeout = 100 * rem_delay;
			status = wait_event_interruptible_timeout(
					ad->LEDInfo.notify_led_event,
					currdriverstate != ad->DriverState
						|| kthread_should_stop(),
					msecs_to_jiffies(timeout));

			if (kthread_should_stop()) {
				BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS,
					LED_DUMP_INFO, DBG_LVL_ALL,
					"Led thread got signal to exit..hence exiting");
				ad->LEDInfo.led_thread_running =
						BCM_LED_THREAD_DISABLED;
				return EVENT_SIGNALED;
			}
			if (status)
				return EVENT_SIGNALED;
		}

		/* Turn off both Tx and Rx LEDs before next second */
		TURN_OFF_LED(ad, 1 << gpio_num_tx, tx_led_idx);
		TURN_OFF_LED(ad, 1 << gpio_num_rx, tx_led_idx);

		/*
		 * Read the Tx & Rx packets transmission after 1 second and
		 * calculate rate of transfer
		 */
		final_num_tx_pkts = ad->dev->stats.tx_packets;
		final_num_rx_pkts = ad->dev->stats.rx_packets;

		rate_of_transfer_tx = final_num_tx_pkts -
						init_num_tx_pkts;
		rate_of_transfer_rx = final_num_rx_pkts -
						init_num_rx_pkts;

		/* Read initial value of packets sent/received */
		init_num_tx_pkts = final_num_tx_pkts;
		init_num_rx_pkts = final_num_rx_pkts;

		/* Scale the rate of transfer to no of blinks. */
		num_of_time_tx =
			ScaleRateofTransfer((ULONG)rate_of_transfer_tx);
		num_of_time_rx =
			ScaleRateofTransfer((ULONG)rate_of_transfer_rx);

	}
	return status;
}

/*
 * -----------------------------------------------------------------------------
 * Procedure:   ValidateDSDParamsChecksum
 *
 * Description: Reads DSD Params and validates checkusm.
 *
 * Arguments:
 *      ad - Pointer to Adapter structure.
 *      param_offset - Start offset of the DSD parameter to be read and
 *			validated.
 *      param_len - Length of the DSD Parameter.
 *
 * Returns:
 *  <OSAL_STATUS_CODE>
 * -----------------------------------------------------------------------------
 */
static INT ValidateDSDParamsChecksum(struct bcm_mini_adapter *ad,
				     ULONG param_offset,
				     USHORT param_len)
{
	INT status = STATUS_SUCCESS;
	PUCHAR buff = NULL;
	USHORT chksm_org = 0;
	USHORT checksum_calculated = 0;

	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, LED_DUMP_INFO, DBG_LVL_ALL,
			"LED Thread:ValidateDSDParamsChecksum: 0x%lx 0x%X",
			param_offset, param_len);

	buff = kmalloc(param_len, GFP_KERNEL);
	if (!buff) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, LED_DUMP_INFO,
				DBG_LVL_ALL,
				"LED Thread: ValidateDSDParamsChecksum Allocation failed");
		return -ENOMEM;

	}

	/* Read the DSD data from the parameter offset. */
	if (STATUS_SUCCESS != BeceemNVMRead(ad, (PUINT)buff,
					    param_offset, param_len)) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, LED_DUMP_INFO,
				DBG_LVL_ALL,
				"LED Thread: ValidateDSDParamsChecksum BeceemNVMRead failed");
		status = STATUS_IMAGE_CHECKSUM_MISMATCH;
		goto exit;
	}

	/* Calculate the checksum of the data read from the DSD parameter. */
	checksum_calculated = CFG_CalculateChecksum(buff, param_len);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, LED_DUMP_INFO, DBG_LVL_ALL,
			"LED Thread: usCheckSumCalculated = 0x%x\n",
			checksum_calculated);

	/*
	 * End of the DSD parameter will have a TWO bytes checksum stored in it.
	 * Read it and compare with the calculated Checksum.
	 */
	if (STATUS_SUCCESS != BeceemNVMRead(ad, (PUINT)&chksm_org,
					    param_offset+param_len, 2)) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, LED_DUMP_INFO,
				DBG_LVL_ALL,
				"LED Thread: ValidateDSDParamsChecksum BeceemNVMRead failed");
		status = STATUS_IMAGE_CHECKSUM_MISMATCH;
		goto exit;
	}
	chksm_org = ntohs(chksm_org);
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, LED_DUMP_INFO, DBG_LVL_ALL,
			"LED Thread: chksm_org = 0x%x", chksm_org);

	/*
	 * Compare the checksum calculated with the checksum read
	 * from DSD section
	 */
	if (checksum_calculated ^ chksm_org) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, LED_DUMP_INFO,
				DBG_LVL_ALL,
				"LED Thread: ValidateDSDParamsChecksum: Checksums don't match");
		status = STATUS_IMAGE_CHECKSUM_MISMATCH;
		goto exit;
	}

exit:
	kfree(buff);
	return status;
}


/*
 * -----------------------------------------------------------------------------
 * Procedure:   ValidateHWParmStructure
 *
 * Description: Validates HW Parameters.
 *
 * Arguments:
 *      ad - Pointer to Adapter structure.
 *      hw_param_offset - Start offset of the HW parameter Section to be read
 *				and validated.
 *
 * Returns:
 *  <OSAL_STATUS_CODE>
 * -----------------------------------------------------------------------------
 */
static INT ValidateHWParmStructure(struct bcm_mini_adapter *ad,
				   ULONG hw_param_offset)
{

	INT status = STATUS_SUCCESS;
	USHORT hw_param_len = 0;
	/*
	 * Add DSD start offset to the hwParamOffset to get
	 * the actual address.
	 */
	hw_param_offset += DSD_START_OFFSET;

	/* Read the Length of HW_PARAM structure */
	BeceemNVMRead(ad, (PUINT)&hw_param_len, hw_param_offset, 2);
	hw_param_len = ntohs(hw_param_len);
	if (0 == hw_param_len || hw_param_len > ad->uiNVMDSDSize)
		return STATUS_IMAGE_CHECKSUM_MISMATCH;

	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, LED_DUMP_INFO, DBG_LVL_ALL,
			"LED Thread:hw_param_len = 0x%x", hw_param_len);
	status = ValidateDSDParamsChecksum(ad, hw_param_offset,
					   hw_param_len);
	return status;
} /* ValidateHWParmStructure() */

static int ReadLEDInformationFromEEPROM(struct bcm_mini_adapter *ad,
					UCHAR gpio_ary[])
{
	int status = STATUS_SUCCESS;

	ULONG  read_val	= 0;
	USHORT hw_param_data	= 0;
	USHORT eeprom_version	= 0;
	UCHAR  i		= 0;
	UCHAR  gpio_info[32]	= {0};

	BeceemNVMRead(ad, (PUINT)&eeprom_version,
		      EEPROM_VERSION_OFFSET, 2);

	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, LED_DUMP_INFO, DBG_LVL_ALL,
			"eeprom_version: Minor:0x%X Major:0x%x",
			eeprom_version & 0xFF,
			((eeprom_version >> 8) & 0xFF));


	if (((eeprom_version>>8)&0xFF) < EEPROM_MAP5_MAJORVERSION) {
		BeceemNVMRead(ad, (PUINT)&hw_param_data,
			      EEPROM_HW_PARAM_POINTER_ADDRESS, 2);
		hw_param_data = ntohs(hw_param_data);
		read_val = hw_param_data;
	} else {
		/*
		 * Validate Compatibility section and then read HW param
		 * if compatibility section is valid.
		 */
		status = ValidateDSDParamsChecksum(ad,
						   DSD_START_OFFSET,
						   COMPATIBILITY_SECTION_LENGTH_MAP5);

		if (status != STATUS_SUCCESS)
			return status;

		BeceemNVMRead(ad, (PUINT)&read_val,
			      EEPROM_HW_PARAM_POINTER_ADDRRES_MAP5, 4);
		read_val = ntohl(read_val);
	}


	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, LED_DUMP_INFO, DBG_LVL_ALL,
			"LED Thread: Start address of HW_PARAM structure = 0x%lx",
			read_val);

	/*
	 * Validate if the address read out is within the DSD.
	 * ad->uiNVMDSDSize gives whole DSD size inclusive of Autoinit.
	 * lower limit should be above DSD_START_OFFSET and
	 * upper limit should be below (ad->uiNVMDSDSize-DSD_START_OFFSET)
	 */
	if (read_val < DSD_START_OFFSET ||
			read_val > (ad->uiNVMDSDSize-DSD_START_OFFSET))
		return STATUS_IMAGE_CHECKSUM_MISMATCH;

	status = ValidateHWParmStructure(ad, read_val);
	if (status)
		return status;

	/*
	 * Add DSD_START_OFFSET to the offset read from the EEPROM.
	 * This will give the actual start HW Parameters start address.
	 * To read GPIO section, add GPIO offset further.
	 */

	read_val += DSD_START_OFFSET;
			/* = start address of hw param section. */
	read_val += GPIO_SECTION_START_OFFSET;
			/* = GPIO start offset within HW Param section. */

	/*
	 * Read the GPIO values for 32 GPIOs from EEPROM and map the function
	 * number to GPIO pin number to gpio_ary
	 */
	BeceemNVMRead(ad, (UINT *)gpio_info, read_val, 32);
	for (i = 0; i < 32; i++) {

		switch (gpio_info[i]) {
		case RED_LED:
			gpio_ary[RED_LED] = i;
			ad->gpioBitMap |= (1 << i);
			break;
		case BLUE_LED:
			gpio_ary[BLUE_LED] = i;
			ad->gpioBitMap |= (1 << i);
			break;
		case YELLOW_LED:
			gpio_ary[YELLOW_LED] = i;
			ad->gpioBitMap |= (1 << i);
			break;
		case GREEN_LED:
			gpio_ary[GREEN_LED] = i;
			ad->gpioBitMap |= (1 << i);
			break;
		default:
			break;
		}

	}
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, LED_DUMP_INFO, DBG_LVL_ALL,
			"GPIO's bit map correspond to LED :0x%X",
			ad->gpioBitMap);
	return status;
}


static int ReadConfigFileStructure(struct bcm_mini_adapter *ad,
				   bool *enable_thread)
{
	int status = STATUS_SUCCESS;
	/* Array to store GPIO numbers from EEPROM */
	UCHAR gpio_ary[NUM_OF_LEDS+1];
	UINT i = 0;
	UINT num_of_led_type = 0;
	PUCHAR puCFGData	= NULL;
	UCHAR bData = 0;
	struct bcm_led_state_info *curr_led_state;

	memset(gpio_ary, DISABLE_GPIO_NUM, NUM_OF_LEDS+1);

	if (!ad->pstargetparams || IS_ERR(ad->pstargetparams)) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, LED_DUMP_INFO,
				DBG_LVL_ALL, "Target Params not Avail.\n");
		return -ENOENT;
	}

	/* Populate gpio_ary with GPIO numbers for LED functions */
	/* Read the GPIO numbers from EEPROM */
	status = ReadLEDInformationFromEEPROM(ad, gpio_ary);
	if (status == STATUS_IMAGE_CHECKSUM_MISMATCH) {
		*enable_thread = false;
		return STATUS_SUCCESS;
	} else if (status) {
		*enable_thread = false;
		return status;
	}

	/*
	 * CONFIG file read successfully. Deallocate the memory of
	 * uiFileNameBufferSize
	 */
	BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, LED_DUMP_INFO, DBG_LVL_ALL,
			"LED Thread: Config file read successfully\n");
	puCFGData = (PUCHAR) &ad->pstargetparams->HostDrvrConfig1;

	/*
	 * Offset for HostDrvConfig1, HostDrvConfig2, HostDrvConfig3 which
	 * will have the information of LED type, LED on state for different
	 * driver state and LED blink state.
	 */

	for (i = 0; i < NUM_OF_LEDS; i++) {
		bData = *puCFGData;
		curr_led_state = &ad->LEDInfo.LEDState[i];

		/*
		 * Check Bit 8 for polarity. If it is set,
		 * polarity is reverse polarity
		 */
		if (bData & 0x80) {
			curr_led_state->BitPolarity = 0;
			/* unset the bit 8 */
			bData = bData & 0x7f;
		}

		curr_led_state->LED_Type = bData;
		if (bData <= NUM_OF_LEDS)
			curr_led_state->GPIO_Num = gpio_ary[bData];
		else
			curr_led_state->GPIO_Num = DISABLE_GPIO_NUM;

		puCFGData++;
		bData = *puCFGData;
		curr_led_state->LED_On_State = bData;
		puCFGData++;
		bData = *puCFGData;
		curr_led_state->LED_Blink_State = bData;
		puCFGData++;
	}

	/*
	 * Check if all the LED settings are disabled. If it is disabled,
	 * dont launch the LED control thread.
	 */
	for (i = 0; i < NUM_OF_LEDS; i++) {
		curr_led_state = &ad->LEDInfo.LEDState[i];

		if ((curr_led_state->LED_Type == DISABLE_GPIO_NUM) ||
			(curr_led_state->LED_Type == 0x7f) ||
			(curr_led_state->LED_Type == 0))
			num_of_led_type++;
	}
	if (num_of_led_type >= NUM_OF_LEDS)
		*enable_thread = false;

	return status;
}

/*
 * -----------------------------------------------------------------------------
 * Procedure:   LedGpioInit
 *
 * Description: Initializes LED GPIOs. Makes the LED GPIOs to OUTPUT mode
 *			  and make the initial state to be OFF.
 *
 * Arguments:
 *      ad - Pointer to MINI_ADAPTER structure.
 *
 * Returns: VOID
 *
 * -----------------------------------------------------------------------------
 */
static VOID LedGpioInit(struct bcm_mini_adapter *ad)
{
	UINT uiResetValue = 0;
	UINT i      = 0;
	struct bcm_led_state_info *curr_led_state;

	/* Set all LED GPIO Mode to output mode */
	if (rdmalt(ad, GPIO_MODE_REGISTER, &uiResetValue,
		   sizeof(uiResetValue)) < 0)
		BCM_DEBUG_PRINT (ad, DBG_TYPE_OTHERS, LED_DUMP_INFO,
			DBG_LVL_ALL, "LED Thread: RDM Failed\n");
	for (i = 0; i < NUM_OF_LEDS; i++) {
		curr_led_state = &ad->LEDInfo.LEDState[i];

		if (curr_led_state->GPIO_Num != DISABLE_GPIO_NUM)
			uiResetValue |= (1 << curr_led_state->GPIO_Num);

		TURN_OFF_LED(ad, 1 << curr_led_state->GPIO_Num, i);

	}
	if (wrmalt(ad, GPIO_MODE_REGISTER, &uiResetValue,
		   sizeof(uiResetValue)) < 0)
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, LED_DUMP_INFO,
				DBG_LVL_ALL, "LED Thread: WRM Failed\n");

	ad->LEDInfo.bIdle_led_off = false;
}

static INT BcmGetGPIOPinInfo(struct bcm_mini_adapter *ad,
			     UCHAR *GPIO_num_tx,
			     UCHAR *GPIO_num_rx,
			     UCHAR *uiLedTxIndex,
			     UCHAR *uiLedRxIndex,
			     enum bcm_led_events currdriverstate)
{
	UINT i = 0;
	struct bcm_led_state_info *led_state_info;

	*GPIO_num_tx = DISABLE_GPIO_NUM;
	*GPIO_num_rx = DISABLE_GPIO_NUM;

	for (i = 0; i < NUM_OF_LEDS; i++) {
		led_state_info = &ad->LEDInfo.LEDState[i];

		if (((currdriverstate == NORMAL_OPERATION) ||
			(currdriverstate == IDLEMODE_EXIT) ||
			(currdriverstate == FW_DOWNLOAD)) &&
		    (led_state_info->LED_Blink_State & currdriverstate) &&
		    (led_state_info->GPIO_Num != DISABLE_GPIO_NUM)) {
			if (*GPIO_num_tx == DISABLE_GPIO_NUM) {
				*GPIO_num_tx = led_state_info->GPIO_Num;
				*uiLedTxIndex = i;
			} else {
				*GPIO_num_rx = led_state_info->GPIO_Num;
				*uiLedRxIndex = i;
			}
		} else {
			if ((led_state_info->LED_On_State & currdriverstate) &&
			    (led_state_info->GPIO_Num != DISABLE_GPIO_NUM)) {
				*GPIO_num_tx = led_state_info->GPIO_Num;
				*uiLedTxIndex = i;
			}
		}
	}
	return STATUS_SUCCESS;
}

static void handle_adapter_driver_state(struct bcm_mini_adapter *ad,
					enum bcm_led_events currdriverstate,
					UCHAR GPIO_num,
					UCHAR dummyGPIONum,
					UCHAR led_idx,
					UCHAR dummyIndex,
					ulong timeout,
					UINT uiResetValue,
					UINT i)
{
	switch (ad->DriverState) {
	case DRIVER_INIT:
		currdriverstate = DRIVER_INIT;
				/* ad->DriverState; */
		BcmGetGPIOPinInfo(ad, &GPIO_num, &dummyGPIONum,
				  &led_idx, &dummyIndex,
				  currdriverstate);

		if (GPIO_num != DISABLE_GPIO_NUM)
			TURN_ON_LED(ad, 1 << GPIO_num, led_idx);

		break;
	case FW_DOWNLOAD:
		/*
		 * BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS,
		 *	LED_DUMP_INFO, DBG_LVL_ALL,
		 *	"LED Thread: FW_DN_DONE called\n");
		 */
		currdriverstate = FW_DOWNLOAD;
		BcmGetGPIOPinInfo(ad, &GPIO_num, &dummyGPIONum,
				  &led_idx, &dummyIndex,
				  currdriverstate);

		if (GPIO_num != DISABLE_GPIO_NUM) {
			timeout = 50;
			LED_Blink(ad, 1 << GPIO_num, led_idx, timeout,
				  -1, currdriverstate);
		}
		break;
	case FW_DOWNLOAD_DONE:
		currdriverstate = FW_DOWNLOAD_DONE;
		BcmGetGPIOPinInfo(ad, &GPIO_num, &dummyGPIONum,
				  &led_idx, &dummyIndex, currdriverstate);
		if (GPIO_num != DISABLE_GPIO_NUM)
			TURN_ON_LED(ad, 1 << GPIO_num, led_idx);
		break;

	case SHUTDOWN_EXIT:
		/*
		 * no break, continue to NO_NETWORK_ENTRY
		 * state as well.
		 */
	case NO_NETWORK_ENTRY:
		currdriverstate = NO_NETWORK_ENTRY;
		BcmGetGPIOPinInfo(ad, &GPIO_num, &dummyGPIONum,
				  &led_idx, &dummyGPIONum, currdriverstate);
		if (GPIO_num != DISABLE_GPIO_NUM)
			TURN_ON_LED(ad, 1 << GPIO_num, led_idx);
		break;
	case NORMAL_OPERATION:
		{
			UCHAR GPIO_num_tx = DISABLE_GPIO_NUM;
			UCHAR GPIO_num_rx = DISABLE_GPIO_NUM;
			UCHAR uiLEDTx = 0;
			UCHAR uiLEDRx = 0;

			currdriverstate = NORMAL_OPERATION;
			ad->LEDInfo.bIdle_led_off = false;

			BcmGetGPIOPinInfo(ad, &GPIO_num_tx, &GPIO_num_rx,
					  &uiLEDTx, &uiLEDRx, currdriverstate);
			if ((GPIO_num_tx == DISABLE_GPIO_NUM) &&
					(GPIO_num_rx == DISABLE_GPIO_NUM)) {
				GPIO_num = DISABLE_GPIO_NUM;
			} else {
				/*
				 * If single LED is selected, use same
				 * for both Tx and Rx
				 */
				if (GPIO_num_tx == DISABLE_GPIO_NUM) {
					GPIO_num_tx = GPIO_num_rx;
					uiLEDTx = uiLEDRx;
				} else if (GPIO_num_rx == DISABLE_GPIO_NUM) {
					GPIO_num_rx = GPIO_num_tx;
					uiLEDRx = uiLEDTx;
				}
				/*
				 * Blink the LED in proportionate
				 * to Tx and Rx transmissions.
				 */
				LED_Proportional_Blink(ad,
						       GPIO_num_tx, uiLEDTx,
						       GPIO_num_rx, uiLEDRx,
						       currdriverstate);
			}
		}
		break;
	case LOWPOWER_MODE_ENTER:
		currdriverstate = LOWPOWER_MODE_ENTER;
		if (DEVICE_POWERSAVE_MODE_AS_MANUAL_CLOCK_GATING ==
				ad->ulPowerSaveMode) {
			/* Turn OFF all the LED */
			uiResetValue = 0;
			for (i = 0; i < NUM_OF_LEDS; i++) {
				if (ad->LEDInfo.LEDState[i].GPIO_Num != DISABLE_GPIO_NUM)
					TURN_OFF_LED(ad,
						     (1 << ad->LEDInfo.LEDState[i].GPIO_Num),
						     i);
			}

		}
		/* Turn off LED And WAKE-UP for Sendinf IDLE mode ACK */
		ad->LEDInfo.bLedInitDone = false;
		ad->LEDInfo.bIdle_led_off = TRUE;
		wake_up(&ad->LEDInfo.idleModeSyncEvent);
		GPIO_num = DISABLE_GPIO_NUM;
		break;
	case IDLEMODE_CONTINUE:
		currdriverstate = IDLEMODE_CONTINUE;
		GPIO_num = DISABLE_GPIO_NUM;
		break;
	case IDLEMODE_EXIT:
		break;
	case DRIVER_HALT:
		currdriverstate = DRIVER_HALT;
		GPIO_num = DISABLE_GPIO_NUM;
		for (i = 0; i < NUM_OF_LEDS; i++) {
			if (ad->LEDInfo.LEDState[i].GPIO_Num !=
					DISABLE_GPIO_NUM)
				TURN_OFF_LED(ad,
					     (1 << ad->LEDInfo.LEDState[i].GPIO_Num),
					     i);
		}
		/* ad->DriverState = DRIVER_INIT; */
		break;
	case LED_THREAD_INACTIVE:
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, LED_DUMP_INFO,
				DBG_LVL_ALL, "InActivating LED thread...");
		currdriverstate = LED_THREAD_INACTIVE;
		ad->LEDInfo.led_thread_running =
				BCM_LED_THREAD_RUNNING_INACTIVELY;
		ad->LEDInfo.bLedInitDone = false;
		/* disable ALL LED */
		for (i = 0; i < NUM_OF_LEDS; i++) {
			if (ad->LEDInfo.LEDState[i].GPIO_Num !=
					DISABLE_GPIO_NUM)
				TURN_OFF_LED(ad,
					     (1 << ad->LEDInfo.LEDState[i].GPIO_Num),
					     i);
		}
		break;
	case LED_THREAD_ACTIVE:
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, LED_DUMP_INFO,
				DBG_LVL_ALL, "Activating LED thread again...");
		if (ad->LinkUpStatus == false)
			ad->DriverState = NO_NETWORK_ENTRY;
		else
			ad->DriverState = NORMAL_OPERATION;

		ad->LEDInfo.led_thread_running =
				BCM_LED_THREAD_RUNNING_ACTIVELY;
		break;
		/* return; */
	default:
		break;
	}
}

static VOID LEDControlThread(struct bcm_mini_adapter *ad)
{
	UINT i = 0;
	UCHAR GPIO_num = 0;
	UCHAR led_idx = 0;
	UINT uiResetValue = 0;
	enum bcm_led_events currdriverstate = 0;
	ulong timeout = 0;

	INT status = 0;

	UCHAR dummyGPIONum = 0;
	UCHAR dummyIndex = 0;

	/* currdriverstate = ad->DriverState; */
	ad->LEDInfo.bIdleMode_tx_from_host = false;

	/*
	 * Wait till event is triggered
	 *
	 * wait_event(ad->LEDInfo.notify_led_event,
	 *	currdriverstate!= ad->DriverState);
	 */

	GPIO_num = DISABLE_GPIO_NUM;

	while (TRUE) {
		/* Wait till event is triggered */
		if ((GPIO_num == DISABLE_GPIO_NUM)
						||
				((currdriverstate != FW_DOWNLOAD) &&
				 (currdriverstate != NORMAL_OPERATION) &&
				 (currdriverstate != LOWPOWER_MODE_ENTER))
						||
				(currdriverstate == LED_THREAD_INACTIVE))
			status = wait_event_interruptible(
					ad->LEDInfo.notify_led_event,
					currdriverstate != ad->DriverState
						|| kthread_should_stop());

		if (kthread_should_stop() || ad->device_removed) {
			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, LED_DUMP_INFO,
				DBG_LVL_ALL,
				"Led thread got signal to exit..hence exiting");
			ad->LEDInfo.led_thread_running =
						BCM_LED_THREAD_DISABLED;
			TURN_OFF_LED(ad, 1 << GPIO_num, led_idx);
			return; /* STATUS_FAILURE; */
		}

		if (GPIO_num != DISABLE_GPIO_NUM)
			TURN_OFF_LED(ad, 1 << GPIO_num, led_idx);

		if (ad->LEDInfo.bLedInitDone == false) {
			LedGpioInit(ad);
			ad->LEDInfo.bLedInitDone = TRUE;
		}

		handle_adapter_driver_state(ad,
					    currdriverstate,
					    GPIO_num,
					    dummyGPIONum,
					    led_idx,
					    dummyIndex,
					    timeout,
					    uiResetValue,
					    i
					    );
	}
	ad->LEDInfo.led_thread_running = BCM_LED_THREAD_DISABLED;
}

int InitLedSettings(struct bcm_mini_adapter *ad)
{
	int status = STATUS_SUCCESS;
	bool enable_thread = TRUE;
	UCHAR i = 0;

	/*
	 * Initially set BitPolarity to normal polarity. The bit 8 of LED type
	 * is used to change the polarity of the LED.
	 */

	for (i = 0; i < NUM_OF_LEDS; i++)
		ad->LEDInfo.LEDState[i].BitPolarity = 1;

	/*
	 * Read the LED settings of CONFIG file and map it
	 * to GPIO numbers in EEPROM
	 */
	status = ReadConfigFileStructure(ad, &enable_thread);
	if (STATUS_SUCCESS != status) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, LED_DUMP_INFO,
				DBG_LVL_ALL,
				"LED Thread: FAILED in ReadConfigFileStructure\n");
		return status;
	}

	if (ad->LEDInfo.led_thread_running) {
		if (enable_thread) {
			;
		} else {
			ad->DriverState = DRIVER_HALT;
			wake_up(&ad->LEDInfo.notify_led_event);
			ad->LEDInfo.led_thread_running =
						BCM_LED_THREAD_DISABLED;
		}

	} else if (enable_thread) {
		/* Create secondary thread to handle the LEDs */
		init_waitqueue_head(&ad->LEDInfo.notify_led_event);
		init_waitqueue_head(&ad->LEDInfo.idleModeSyncEvent);
		ad->LEDInfo.led_thread_running =
					BCM_LED_THREAD_RUNNING_ACTIVELY;
		ad->LEDInfo.bIdle_led_off = false;
		ad->LEDInfo.led_cntrl_threadid =
			kthread_run((int (*)(void *)) LEDControlThread,
				    ad, "led_control_thread");
		if (IS_ERR(ad->LEDInfo.led_cntrl_threadid)) {
			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS, LED_DUMP_INFO,
					DBG_LVL_ALL,
					"Not able to spawn Kernel Thread\n");
			ad->LEDInfo.led_thread_running =
				BCM_LED_THREAD_DISABLED;
			return PTR_ERR(ad->LEDInfo.led_cntrl_threadid);
		}
	}
	return status;
}
