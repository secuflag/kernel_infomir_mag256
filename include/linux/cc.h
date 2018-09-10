/*
 * This file is part STMicroelectronics CryptoCore driver
 *
 * Copyright 2015, STMicroelectronics - All Rights Reserved
 * Author(s): Pierre Peiffer <pierre.peiffer@st.com> for STMicroelectronics.
 *
 * License type: GPLv2
 *
 * STMicroelectronics CryptoCore driver is free software; you can redistribute
 * it and/or modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * STMicroelectronics CryptoCore driver  is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * the CryptoCore driver. If not, see <http://www.gnu.org/licenses/>.
 *
 * STMicroelectronics CryptoCore driver may alternatively be licensed by
 * STMicroelectronics under BSD license.
 */

#ifndef __CC_H
#define __CC_H

#include <linux/types.h>
#include <linux/stcc.h>

enum srd_fw_type {
	SRD_FW_SECLX,
	SRD_FW_TZ,
	SRD_FW_SDP,
	SRD_FW_MISC,
};

int stcc_sigdma_load(u32 type, struct section *sect0, struct section *sect1,
		     u32 *err);
int stcc_srd_load(enum srd_fw_type type, struct section *srd,
		  int (*cb)(const void *p), const void *param, u32 *err);

#endif /* __CC_H */
