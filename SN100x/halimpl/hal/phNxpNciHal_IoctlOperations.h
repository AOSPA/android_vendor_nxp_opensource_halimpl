/*
 * Copyright (C) 2019 NXP Semiconductors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "phNfcStatus.h"
#include "phNxpConfig.h"
#include "phNxpLog.h"
#include <hardware/nfc.h>
#include <vendor/nxp/hardware/nfc/2.0/types.h>
namespace vendor {
namespace nxp {
namespace hardware {
namespace nfc {
namespace V2_0 {
struct INqNfc;
} } } } }

using vendor::nxp::hardware::nfc::V2_0::Constants;
using vendor::nxp::hardware::nfc::V2_0::nfc_nci_IoctlInOutData_t;
using vendor::nxp::hardware::nfc::V2_0::NfcEvent1;
using vendor::nxp::hardware::nfc::V2_0::NfcEvent2;
using vendor::nxp::hardware::nfc::V2_0::NfcEvent3;
using vendor::nxp::hardware::nfc::V2_0::NfcFwUpdateStatus;
using vendor::nxp::hardware::nfc::V2_0::NxpNfcHalStatus;

/******************************************************************************
 ** Function         phNxpNciHal_ioctlIf
 **
 ** Description      This function shall be called from HAL when libnfc-nci
 **                  calls phNxpNciHal_ioctl() to perform any IOCTL operation
 **
 ** Returns          return 0 on success and -1 on fail,
 ******************************************************************************/
int phNxpNciHal_ioctlIf(long arg, void *p_data);

/*******************************************************************************
 **
 ** Function         phNxpNciHal_savePersistLog
 **
 ** Description      Save persist log with “reason” at available index.
 **
 ** Parameters       uint8_t reason
 **
 ** Returns          returns the  index of saved reason/Log.
 *******************************************************************************/
uint8_t phNxpNciHal_savePersistLog(uint8_t reason);

/*******************************************************************************
 **
 ** Function         phNxpNciHal_loadPersistLog
 **
 ** Description      If given index is valid, return a log at the given index.
 **
 ** Parameters       uint8_t index
 **
 ** Returns          If index found, return a log as string else
 **                  return a "" string
 *******************************************************************************/
string phNxpNciHal_loadPersistLog(uint8_t index);

/*******************************************************************************
**
** Function         phNxpNciHal_getSystemProperty
**
** Description      It shall be used to get property value of the given Key
**
** Parameters       string key
**
** Returns          It returns the property value of the key
*******************************************************************************/
string phNxpNciHal_getSystemProperty(string key);

/*******************************************************************************
 **
 ** Function         phNxpNciHal_setSystemProperty
 **
 ** Description      It shall be used to save/chage value to system property
 **                  based on provided key.
 **
 ** Parameters       string key, string value
 **
 ** Returns          true if success, false if fail
 *******************************************************************************/
bool phNxpNciHal_setSystemProperty(string key, string value);

/*******************************************************************************
**
** Function         phNxpNciHal_getNxpConfig
**
** Description      It shall be used to read config values from the
*libnfc-nxp.conf
**
** Parameters       nxpConfigs config
**
** Returns          void
*******************************************************************************/
string phNxpNciHal_getNxpConfigIf();

/*******************************************************************************
**
** Function         phNxpNciHal_resetEse
**
** Description      It shall be used to to reset eSE by proprietary command.
**
** Parameters       None
**
** Returns          status of eSE reset response
*******************************************************************************/
NFCSTATUS phNxpNciHal_resetEse();

/******************************************************************************
** Function         phNxpNciHal_setNxpTransitConfig
**
** Description      This function overwrite libnfc-nxpTransit.conf file
**                  with transitConfValue.
**
** Returns          void.
**
*******************************************************************************/
void phNxpNciHal_setNxpTransitConfig(char *transitConfValue);

/*******************************************************************************
 **
 ** Function:        phNxpNciHal_CheckFwRegFlashRequired()
 **
 ** Description:     Updates FW and Reg configurations if required
 **
 ** Returns:         status
 **
 ********************************************************************************/
int phNxpNciHal_CheckFwRegFlashRequired(uint8_t *fw_update_req,
                                        uint8_t *rf_update_req,
                                        uint8_t skipEEPROMRead);

/*******************************************************************************
 **
 ** Function:        property_get_intf()
 **
 ** Description:     Gets property value for the input property name
 **
 ** Parameters       propName:   Name of the property whichs value need to get
 **                  valueStr:   output value of the property.
 **                  defaultStr: default value of the property if value is not
 **                              there this will be set to output value.
 **
 ** Returns:         actual length of the property value
 **
 ********************************************************************************/
int property_get_intf(const char *propName, char *valueStr,
                      const char *defaultStr);

/*******************************************************************************
 **
 ** Function:        property_set_intf()
 **
 ** Description:     Sets property value for the input property name
 **
 ** Parameters       propName:   Name of the property whichs value need to set
 **                  valueStr:   value of the property.
 **
 ** Returns:        returns 0 on success, < 0 on failure
 **
 ********************************************************************************/
int property_set_intf(const char *propName, const char *valueStr);

#undef PROPERTY_VALUE_MAX
#define PROPERTY_VALUE_MAX 92
#define property_get(a, b, c) property_get_intf(a, b, c)
#define property_set(a, b) property_set_intf(a, b)
