#include "headers.h"

/*
Function:	InterfaceIdleModeWakeup

Description:	This is the hardware specific Function for
		waking up HW device from Idle mode.
		A software abort pattern is written to the
		device to wake it and necessary power state
		transitions from host are performed here.

Input parameters: IN struct bcm_mini_adapter *ad
		  - Miniport Adapter Context

Return:		BCM_STATUS_SUCCESS - If Wakeup of the HW Interface
				     was successful.
		Other              - If an error occurred.
*/

/*
Function:	InterfaceIdleModeRespond

Description:	This is the hardware specific Function for
		responding to Idle mode request from target.
		Necessary power state transitions from host for
		idle mode or other device specific initializations
		are performed here.

Input parameters: IN struct bcm_mini_adapter * ad
		  - Miniport Adapter Context

Return:		BCM_STATUS_SUCCESS - If Idle mode response related
				     HW configuration was successful.
		Other              - If an error occurred.
*/

/*
"dmem bfc02f00  100" tells how many time device went in Idle mode.
this value will be at address bfc02fa4.just before value d0ea1dle.

Set time value by writing at bfc02f98 7d0

checking the Ack timer expire on kannon by running command
d qcslog .. if it shows e means host has not send response
to f/w with in 200 ms. Response should be
send to f/w with in 200 ms after the Idle/Shutdown req issued

*/


int InterfaceIdleModeRespond(struct bcm_mini_adapter *ad,
			unsigned int *buff)
{
	int	status = STATUS_SUCCESS;
	unsigned int	reg_read = 0;
	int bytes;

	if (ntohl(*buff) == GO_TO_IDLE_MODE_PAYLOAD) {
		if (ntohl(*(buff+1)) == 0) {

			status = wrmalt(ad, SW_ABORT_IDLEMODE_LOC,
					&reg_read, sizeof(reg_read));
			if (status)
				return status;

			if (ad->ulPowerSaveMode ==
				DEVICE_POWERSAVE_MODE_AS_MANUAL_CLOCK_GATING) {
				reg_read = 0x00000000;
				status = wrmalt(ad,
					DEBUG_INTERRUPT_GENERATOR_REGISTOR,
					&reg_read, sizeof(reg_read));
				if (status)
					return status;
			}
			/* Below Register should not br read in case of
			 * Manual and Protocol Idle mode */
			else if (ad->ulPowerSaveMode !=
				DEVICE_POWERSAVE_MODE_AS_PROTOCOL_IDLE_MODE) {
				/* clear on read Register */
				bytes = rdmalt(ad, DEVICE_INT_OUT_EP_REG0,
					&reg_read, sizeof(reg_read));
				if (bytes < 0) {
					status = bytes;
					return status;
				}
				/* clear on read Register */
				bytes = rdmalt(ad, DEVICE_INT_OUT_EP_REG1,
					&reg_read, sizeof(reg_read));
				if (bytes < 0) {
					status = bytes;
					return status;
				}
			}

			/* Set Idle Mode Flag to False and
			 * Clear IdleMode reg. */
			ad->IdleMode = false;
			ad->bTriedToWakeUpFromlowPowerMode = false;

			wake_up(&ad->lowpower_mode_wait_queue);

		} else {
			if (TRUE == ad->IdleMode)
				return status;

			reg_read = 0;

			if (ad->chip_id == BCS220_2 ||
				ad->chip_id == BCS220_2BC ||
					ad->chip_id == BCS250_BC ||
					ad->chip_id == BCS220_3) {

				bytes = rdmalt(ad, HPM_CONFIG_MSW,
					&reg_read, sizeof(reg_read));
				if (bytes < 0) {
					status = bytes;
					return status;
				}


				reg_read |= (1<<17);

				status = wrmalt(ad, HPM_CONFIG_MSW,
					&reg_read, sizeof(reg_read));
				if (status)
					return status;
			}
			SendIdleModeResponse(ad);
		}
	} else if (ntohl(*buff) == IDLE_MODE_SF_UPDATE_MSG) {
		OverrideServiceFlowParams(ad, buff);
	}
	return status;
}

static int InterfaceAbortIdlemode(struct bcm_mini_adapter *ad,
				unsigned int pattern)
{
	int status = STATUS_SUCCESS;
	unsigned int value;
	unsigned int chip_id;
	unsigned long timeout = 0, itr = 0;

	int lenwritten = 0;
	unsigned char aucAbortPattern[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
						0xFF, 0xFF, 0xFF};
	struct bcm_interface_adapter *interf_ad =
				ad->pvInterfaceAdapter;

	/* Abort Bus suspend if its already suspended */
	if ((TRUE == interf_ad->bSuspended) &&
			(TRUE == ad->bDoSuspend))
		status = usb_autopm_get_interface(
				interf_ad->interface);

	if ((ad->ulPowerSaveMode ==
			DEVICE_POWERSAVE_MODE_AS_MANUAL_CLOCK_GATING) ||
	   (ad->ulPowerSaveMode ==
			DEVICE_POWERSAVE_MODE_AS_PROTOCOL_IDLE_MODE)) {
		/* write the SW abort pattern. */
		status = wrmalt(ad, SW_ABORT_IDLEMODE_LOC,
				&pattern, sizeof(pattern));
		if (status)
			return status;
	}

	if (ad->ulPowerSaveMode ==
		DEVICE_POWERSAVE_MODE_AS_MANUAL_CLOCK_GATING) {
		value = 0x80000000;
		status = wrmalt(ad,
				DEBUG_INTERRUPT_GENERATOR_REGISTOR,
				&value, sizeof(value));
		if (status)
			return status;
	} else if (ad->ulPowerSaveMode !=
			DEVICE_POWERSAVE_MODE_AS_PROTOCOL_IDLE_MODE) {
		/*
		 * Get a Interrupt Out URB and send 8 Bytes Down
		 * To be Done in Thread Context.
		 * Not using Asynchronous Mechanism.
		 */
		status = usb_interrupt_msg(interf_ad->udev,
			usb_sndintpipe(interf_ad->udev,
			interf_ad->sIntrOut.int_out_endpointAddr),
			aucAbortPattern,
			8,
			&lenwritten,
			5000);
		if (status)
			return status;
		else
			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS,
				IDLE_MODE, DBG_LVL_ALL,
				"NOB Sent down :%d", lenwritten);

		/* mdelay(25); */

		timeout = jiffies +  msecs_to_jiffies(50);
		while (time_after(timeout, jiffies)) {
			itr++;
			rdmalt(ad, CHIP_ID_REG, &chip_id, sizeof(UINT));
			if (0xbece3200 == (chip_id&~(0xF0)))
				chip_id = chip_id&~(0xF0);
			if (chip_id == ad->chip_id)
				break;
		}
		if (time_before(timeout, jiffies))
			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS,
				IDLE_MODE, DBG_LVL_ALL,
				"Not able to read chip-id even after 25 msec");
		else
			BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS,
				IDLE_MODE, DBG_LVL_ALL,
				"Number of completed iteration to"
				"read chip-id :%lu", itr);

		status = wrmalt(ad, SW_ABORT_IDLEMODE_LOC,
				&pattern, sizeof(status));
		if (status)
			return status;
	}
	return status;
}
int InterfaceIdleModeWakeup(struct bcm_mini_adapter *ad)
{
	if (ad->bTriedToWakeUpFromlowPowerMode) {
		BCM_DEBUG_PRINT(ad, DBG_TYPE_OTHERS,
		IDLE_MODE, DBG_LVL_ALL,
		"Wake up already attempted.. ignoring\n");
	} else {
		ad->bTriedToWakeUpFromlowPowerMode = TRUE;
		InterfaceAbortIdlemode(ad, ad->usIdleModePattern);

	}
	return 0;
}

void InterfaceHandleShutdownModeWakeup(struct bcm_mini_adapter *ad)
{
	unsigned int uiRegVal = 0;
	INT Status = 0;
	int bytes;

	if (ad->ulPowerSaveMode ==
		DEVICE_POWERSAVE_MODE_AS_MANUAL_CLOCK_GATING) {
		/* clear idlemode interrupt. */
		uiRegVal = 0;
		Status = wrmalt(ad,
			DEBUG_INTERRUPT_GENERATOR_REGISTOR,
			&uiRegVal, sizeof(uiRegVal));
		if (Status)
			return;
	}

	else {

/* clear Interrupt EP registers. */
		bytes = rdmalt(ad,
			DEVICE_INT_OUT_EP_REG0,
			&uiRegVal, sizeof(uiRegVal));
		if (bytes < 0) {
			Status = bytes;
			return;
		}

		bytes = rdmalt(ad,
			DEVICE_INT_OUT_EP_REG1,
			&uiRegVal, sizeof(uiRegVal));
		if (bytes < 0) {
			Status = bytes;
			return;
		}
	}
}

