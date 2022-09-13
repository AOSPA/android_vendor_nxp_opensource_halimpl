/*
* Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted (subject to the limitations in the
* disclaimer below) provided that the following conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*
*     * Redistributions in binary form must reproduce the above
*       copyright notice, this list of conditions and the following
*       disclaimer in the documentation and/or other materials provided
*       with the distribution.
*
*     * Neither the name of Qualcomm Innovation Center, Inc. nor the names of its
*       contributors may be used to endorse or promote products derived
*       from this software without specific prior written permission.
*
* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
* GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
* HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
* GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
* IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
* OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "phNfcDynamicProtection.h"
#include <phNxpNciHal_Adaptation.h>
#include <phNxpNciHal.h>
#include <phNxpNciHal_utils.h>
#include "CPeripheralAccessControl.h"
#include "peripheralStateUtils.h"

typedef int32_t (*getNfcPhStatusFnPtr)(void *context);
typedef void* (*registerNfcPhCBFnPtr)(uint32_t peripheral, PeripheralStateCB NotifyEvent);
typedef int32_t (*deregisterNfcPhCBFnPtr)(void *context);

void* mSecureLibInstance = NULL;
getNfcPhStatusFnPtr mGetPhState = NULL;
registerNfcPhCBFnPtr mRegisterPhCb = NULL;
deregisterNfcPhCBFnPtr mDeregisterPhCb = NULL;
void* mSecureModeContext = NULL;

uint32_t pType = CPeripheralAccessControl_NFC_UID;
static int phSecureState = 1;
static int sync_enable;

extern phNxpNciHal_Control_t nxpncihal_ctrl;
sem_t secure_call_flow_sync_sem;

/*******************************************************************************
*
* Function         notifyNfcDriver
*
* Description      Notify the nfc driver for the exit and entry of secure zone
*
* Parameters       opt - control  codefro  the secure/non secure zone
*
* Returns          0 if ioctl command completed successfully else -1 on failure.
*******************************************************************************/
int32_t notifyNfcDriver(int32_t opt)
{
  int32_t NfcFid = -1, result = 0;
  if ((NfcFid = open(NFC_DEV_NODE_NAME, O_RDWR)) < 0) {
    ALOGE("secure call open failed errno = %d\n", errno);
    return -1;
  }

  if(opt) {
    if(-1 == (result = ioctl(NfcFid, NFC_SECURE_ZONE, 1))) {
      goto err;
    }
  }
  else {
    if(-1 == (result = ioctl(NfcFid, NFC_SECURE_ZONE, 0))) {
      goto err;
    }
  }

err:
  close(NfcFid);

  return result;
}

/*******************************************************************************
*
* Function         checkNfcSecureStatus
*
* Description      Check secure status by NFC HAL to allow user control NFC on/off
*
* Parameters       void
*
* Returns          1 if periheral in secure state else 0.
*******************************************************************************/
uint8_t checkNfcSecureStatus(void)
{
  uint8_t NfcSecureState;

  if(sync_enable) {
    /* call flow  sync is required only if  device boots in secure state */
    sem_wait(&secure_call_flow_sync_sem);
    NfcSecureState = phSecureState;
    sem_post(&secure_call_flow_sync_sem);
    sync_enable = 0;
  } else {
    NfcSecureState = phSecureState;
  }

  return NfcSecureState;
}

/*******************************************************************************
*
* Function         notifyNfcPeripheralEvent
*
* Description      Event Notification Call back function received from TZ for the exit and entry of secure zone
*
* Parameters       Nfcperi - nfc peripheral UID,
*                  NfcSecureState - nfc periheral secure zone status
*
* Returns          0 if setting device into secure/non-secure zone is successful else -1 on failure.
*******************************************************************************/
int32_t notifyNfcPeripheralEvent(const uint32_t Nfcperi, const uint8_t NfcSecureState) {

  /**
  * peripherals can handle the securemode notification
  * based on the event received from TZ
  */
  int32_t  result = 0;
  uint8_t curState = NfcSecureState;
  ALOGD("%s: Received Notification from TZ...\n", __func__);
  ALOGD("%s: HAL peripheral %d Entered into NfcSecureState %d\n", __func__, Nfcperi, NfcSecureState);

  if (NfcSecureState == STATE_RESET_CONNECTION) {
    /**
    * Handling the state where connection got broken to get
    * state change notification
    */
    ALOGD("%s: Possible ssgtzd link got broken..\n", __func__);
    curState = register_routine(pType);
    ALOGD("Func %s: Peripheral[0x%x], Current State is [%d]\n", __func__,pType, NfcSecureState);
  }

  switch(curState) {
    case STATE_SECURE:
      /**
      * Peripheral Entering Secure mode
      */
      if(phSecureState == 0) {
        phSecureState = 1;
        if (nxpncihal_ctrl.halStatus != HAL_STATUS_CLOSE) {
          /*Ideal conditions this should never be called, called only when TZ notifies before disabling the  NFC to avoid NFC crash */
          ALOGD("Received Secure Zone entry notifications from TZ during NFC active state; disable NFC\n");
          (*nxpncihal_ctrl.p_nfc_stack_cback)(HAL_TZ_SECURE_ZONE_DISABLE_NFC_EVT, HAL_NFC_STATUS_OK);
          /*wait untill NFC is closed*/
          while(nxpncihal_ctrl.halStatus == HAL_STATUS_CLOSE) break;
        }
        result = notifyNfcDriver(phSecureState);
        if(result == -1) {
          ALOGE("driver notify call failed during secure entry\n");
          return result;
        }
      }
      ALOGD("Entry Secure zone successful\n");
      break;
    case STATE_NONSECURE:
      /**
      * Peripheral Exiting Secure mode
      */
      if(phSecureState == 1) {
        phSecureState = 0;
        if(-1 == (result = notifyNfcDriver(phSecureState))) {
          ALOGE("driver notify call failed during secure exit\n");
          return  result;
        }
        sem_post(&secure_call_flow_sync_sem);
      }
      ALOGD("Exit Secure zone successful\n");
      break;
    default:
      ALOGD("%s: Wrong State notified\n", __func__);
  }

  return result;
}

int register_routine(uint32_t pUID) {
  int ret = PRPHRL_SUCCESS;
  int dyn_reg_cnt = 0;
  /** This is a blocking call until ssgtzd gets restored */
  do {
    mSecureModeContext = mRegisterPhCb(pType, notifyNfcPeripheralEvent);
    if(mSecureModeContext != NULL) {
      ALOGD("%s: Call back registered for Peripheral[0x%x] \n", __func__, pUID);
      break;
    }
    /** Ideally registeration should happen; In any case if secure libraries/TZ fails, we need to retry until registration is ssuccessful */
    dyn_reg_cnt++;
    if((dyn_reg_cnt>0) && (dyn_reg_cnt%10==0))
      ALOGD("Register NFC peripheral with secureLib trial %d\n", dyn_reg_cnt);
    /** Wait for some time before retry */
    sleep(1);
  } while (true);

  /* Getting current peripheral state after re-connection
   * If get peripheral fails, derigister and retry the sequence
   */
  ret = mGetPhState(mSecureModeContext);
  if (ret == PRPHRL_ERROR) {
    ALOGD("%s: Failed to get Peripheral state from TZ\n", __func__);
  }

  return ret;
}

/*******************************************************************************
*
* Function         registerNfcDynamicProtection
*
* Description      Registers NFC peripheral for controlling dynamic protection with secure libraries
*
* Parameters       None
*
* Returns          0 if resgistering is successful else -1 on failure.
*******************************************************************************/
int8_t registerNfcDynamicProtection(void)
{

  /*Register the  peripheral with PCS  rountine and check for the peripheral  status*/
  int8_t status = 0;
  uint8_t pState =  STATE_NONSECURE;

  /*Register Peripheral*/
  /*call flow sync during secure  boot*/
  sem_init(&secure_call_flow_sync_sem,0,0);

  mSecureLibInstance = (void *)dlopen("libPeripheralStateUtils.so", RTLD_LAZY);

  if(mSecureLibInstance == NULL) {
    ALOGE("%s: failed loading securelib instance\n", __func__);
    return -1;
  }

  mRegisterPhCb = (registerNfcPhCBFnPtr)dlsym(mSecureLibInstance, "registerPeripheralCB");

  if(mRegisterPhCb == NULL) {
    ALOGE("Error linking registerPeripheralCB\n");
    goto on_error;
  }

  mGetPhState = (getNfcPhStatusFnPtr)dlsym(mSecureLibInstance, "getPeripheralState");

  if(mGetPhState == NULL) {
    ALOGE("Error linking getPeripheralState\n");
    goto on_error;
  }

  /*Register Peripheral*/
  if((pState = register_routine(pType)) == PRPHRL_ERROR)
    goto on_error;
  ALOGD("%s: Callback registered to TZ and waiting for notification\n", __func__);

  if(pState ==  STATE_NONSECURE) {
    sync_enable = 0;
    phSecureState = 0;

    if(-1 == (notifyNfcDriver(phSecureState))) {
      ALOGE("driver notify call failed during secure entry\n");
    } else {
      sem_post(&secure_call_flow_sync_sem);
      ALOGD("Exit secure mode successful\n");
    }
  } else {
    sync_enable = 1;
    phSecureState = 1;
    ALOGD("Entry secure mode successful\n");
  }

  ALOGD("%s success", __func__);
  return status;

on_error:
  /*
  * Calling this function is must to clean up the
  * resources properly in lib
  */
  mDeregisterPhCb = (deregisterNfcPhCBFnPtr)dlsym(mSecureLibInstance, "deregisterPeripheralCB");

  if(mDeregisterPhCb != NULL) {
    mDeregisterPhCb(mSecureModeContext);
  }

  mGetPhState = NULL;
  mRegisterPhCb = NULL;
  mDeregisterPhCb = NULL;
  dlclose(mSecureLibInstance);
  mSecureLibInstance = NULL;
  mSecureModeContext = NULL;
  status = -1;
  return status;
}
