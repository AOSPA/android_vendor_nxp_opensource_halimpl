/******************************************************************************
 *
 *  Copyright 2019-2021 NXP
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

#define LOG_TAG "android.hardware.nfc@1.2-impl"
#include "Nfc.h"
#include <log/log.h>
#include "halimpl/inc/phNxpNciHal_Adaptation.h"
#include "phNfcStatus.h"

#define CHK_STATUS(x) \
  ((x) == NFCSTATUS_SUCCESS) ? (V1_0::NfcStatus::OK) : (V1_0::NfcStatus::FAILED)

#define NXP_EN_SN110U 1
#define NXP_EN_SN100U 1
#define NXP_EN_SN220U 1
#define NXP_EN_PN557 1
#define NFC_NXP_MW_ANDROID_VER (13U)  /* Android version used by NFC MW */
#define NFC_NXP_MW_VERSION_MAJ (0x03) /* MW Major Version */
#define NFC_NXP_MW_VERSION_MIN (0x00) /* MW Minor Version */
#define NFC_NXP_MW_CUSTOMER_ID (0x00) /* MW Customer Id */
#define NFC_NXP_MW_RC_VERSION (0x00)  /* MW RC Version */

extern bool nfc_debug_enabled;

namespace android {
namespace hardware {
namespace nfc {
namespace V1_2 {
namespace implementation {

sp<V1_1::INfcClientCallback> Nfc::mCallbackV1_1 = nullptr;
sp<V1_0::INfcClientCallback> Nfc::mCallbackV1_0 = nullptr;

static void printNfcMwVersion() {
  uint32_t validation = (NXP_EN_SN100U << 13);
  validation |= (NXP_EN_SN110U << 14);
  validation |= (NXP_EN_SN220U << 15);
  validation |= (NXP_EN_PN557 << 11);

  ALOGE("MW-HAL Version: NFC_AR_%02X_%04X_%02d.%02x.%02x",
        NFC_NXP_MW_CUSTOMER_ID, validation, NFC_NXP_MW_ANDROID_VER,
        NFC_NXP_MW_VERSION_MAJ, NFC_NXP_MW_VERSION_MIN);
}

Return<V1_0::NfcStatus> Nfc::open_1_1(
    const sp<V1_1::INfcClientCallback>& clientCallback) {
  if (clientCallback == nullptr) {
    ALOGD_IF(nfc_debug_enabled, "Nfc::open null callback");
    return V1_0::NfcStatus::FAILED;
  } else {
    mCallbackV1_1 = clientCallback;
    mCallbackV1_1->linkToDeath(this, 0 /*cookie*/);
  }
  return open(clientCallback);
}

// Methods from ::android::hardware::nfc::V1_0::INfc follow.
Return<V1_0::NfcStatus> Nfc::open(
    const sp<V1_0::INfcClientCallback>& clientCallback) {
  ALOGD_IF(nfc_debug_enabled, "Nfc::open Enter");
  if (clientCallback == nullptr) {
    ALOGD_IF(nfc_debug_enabled, "Nfc::open null callback");
    return V1_0::NfcStatus::FAILED;
  } else {
    mCallbackV1_0 = clientCallback;
    mCallbackV1_0->linkToDeath(this, 0 /*cookie*/);
  }
  printNfcMwVersion();
  NFCSTATUS status = phNxpNciHal_open(eventCallback, dataCallback);
  ALOGD_IF(nfc_debug_enabled, "Nfc::open Exit");
  return CHK_STATUS(status);
}

Return<uint32_t> Nfc::write(const hidl_vec<uint8_t>& data) {
  hidl_vec<uint8_t> copy = data;
  return phNxpNciHal_write(copy.size(), &copy[0]);
}

Return<V1_0::NfcStatus> Nfc::coreInitialized(const hidl_vec<uint8_t>& data) {
  hidl_vec<uint8_t> copy = data;
  NFCSTATUS status = phNxpNciHal_core_initialized(copy.size(), &copy[0]);
  return CHK_STATUS(status);
}

Return<V1_0::NfcStatus> Nfc::prediscover() {
  NFCSTATUS status = phNxpNciHal_pre_discover();
  return CHK_STATUS(status);
}

Return<V1_0::NfcStatus> Nfc::close() {
  if (mCallbackV1_1 == nullptr && mCallbackV1_0 == nullptr) {
    return V1_0::NfcStatus::FAILED;
  }
  NFCSTATUS status = phNxpNciHal_close(false);

  if (mCallbackV1_1 != nullptr) {
    mCallbackV1_1->unlinkToDeath(this);
    mCallbackV1_1 = nullptr;
  }
  if (mCallbackV1_0 != nullptr) {
    mCallbackV1_0->unlinkToDeath(this);
    mCallbackV1_0 = nullptr;
  }
  return CHK_STATUS(status);
}

Return<V1_0::NfcStatus> Nfc::controlGranted() {
  NFCSTATUS status = phNxpNciHal_control_granted();
  return CHK_STATUS(status);
}

Return<V1_0::NfcStatus> Nfc::powerCycle() {
  NFCSTATUS status = phNxpNciHal_power_cycle();
  return CHK_STATUS(status);
}

// Methods from ::android::hardware::nfc::V1_1::INfc follow.
Return<void> Nfc::factoryReset() {
  phNxpNciHal_do_factory_reset();
  return Void();
}

Return<V1_0::NfcStatus> Nfc::closeForPowerOffCase() {
  if (mCallbackV1_1 == nullptr && mCallbackV1_0 == nullptr) {
    return V1_0::NfcStatus::FAILED;
  }
  NFCSTATUS status = phNxpNciHal_configDiscShutdown();

  if (mCallbackV1_1 != nullptr) {
    mCallbackV1_1->unlinkToDeath(this);
    mCallbackV1_1 = nullptr;
  }
  if (mCallbackV1_0 != nullptr) {
    mCallbackV1_0->unlinkToDeath(this);
    mCallbackV1_0 = nullptr;
  }
  return CHK_STATUS(status);
}

Return<void> Nfc::getConfig(getConfig_cb hidl_cb) {
  android::hardware::nfc::V1_1::NfcConfig nfcVendorConfig;
  phNxpNciHal_getVendorConfig(nfcVendorConfig);
  hidl_cb(nfcVendorConfig);
  return Void();
}

Return<void> Nfc::getConfig_1_2(getConfig_1_2_cb hidl_cb) {
  NfcConfig nfcVendorConfig;
  phNxpNciHal_getVendorConfig_1_2(nfcVendorConfig);
  hidl_cb(nfcVendorConfig);
  return Void();
}

void Nfc::serviceDied(uint64_t /*cookie*/, const wp<IBase>& /*who*/) {
  if (mCallbackV1_1 == nullptr && mCallbackV1_0 == nullptr) {
    return;
  }
  phNxpNciHal_close(true);

  if (mCallbackV1_1 != nullptr) {
    mCallbackV1_1->unlinkToDeath(this);
    mCallbackV1_1 = nullptr;
  }
  if (mCallbackV1_0 != nullptr) {
    mCallbackV1_0->unlinkToDeath(this);
    mCallbackV1_0 = nullptr;
  }
}

}  // namespace implementation
}  // namespace V1_2
}  // namespace nfc
}  // namespace hardware
}  // namespace android
