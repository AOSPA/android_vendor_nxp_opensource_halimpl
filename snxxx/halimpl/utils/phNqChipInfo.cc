/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "phNqChipInfo.h"

/**
 * @brief This function retrieves chip id and fw version
 *
 * This function reads and stores the NFC HW chip id and
 * fw version information by sending ioctl call to NFC
 * driver which will be later used to select and load
 * the desired config file.
 *
 * @param address of a structure variable to store chip id & fw version
 * @return it returns 0 on success, -1 on failure
**/

int get_chip_info(chip_info_t *nq_chip_info)
{
    int ret = 0;
    int fp = 0;
    unsigned int chip_version = 0x00;
    unsigned int major_version = 0x00;
    unsigned int minor_version = 0x00;
    unsigned int rom_version = 0x00;
    union nqx_uinfo nqx_info;

    if ((ret = (fp = open(NFC_DEV_NODE_NAME, O_RDWR))) < 0)
    {
        ALOGE("NQ open error retcode = %d, errno = %d\n", ret, errno);
        return -1;
    }

    if((nqx_info.i = ioctl(fp, NFCC_GET_INFO, 0)) < 0)
    {
        ALOGE("Ioctl call fail with errno = %d\n", errno);
        close(fp);
        return -1;
    }

    chip_version = nqx_info.info.chip_type;

    snprintf(nq_chip_info->nq_chipid, CHIP_MAX_LEN, "0x%02x", chip_version);
    ALOGD("Chip id from NFC HW = %s\n", nq_chip_info->nq_chipid);

    rom_version = nqx_info.info.rom_version;
    major_version = nqx_info.info.fw_major;
    minor_version = nqx_info.info.fw_minor;

    snprintf(nq_chip_info->nq_fw_ver, FW_MAX_LEN, "%02x.%02x.%02x", rom_version,
              major_version, minor_version);
    ALOGD("Firmware version from NFC HW = %s\n", nq_chip_info->nq_fw_ver);
    close(fp);

    return 0;
}
