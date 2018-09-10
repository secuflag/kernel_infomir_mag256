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

#ifndef __STCC_H
#define __STCC_H

#include <stddef.h>
#include <linux/ioctl.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CC_DEVNAME "stcc"
#define N_SIGDMA_SECTION 2

struct section {
	void *ptr;
	size_t size;
};

struct stcc_sigdma {
	unsigned int type;
	unsigned int err_code;
	struct section sect[N_SIGDMA_SECTION];
};

enum stcc_prov_op {
	STCC_PROV_INSERT,
	STCC_PROV_CHECK,
	STCC_PROV_CPK,
	STCC_PROV_DIV,
};

struct stcc_prov {
	enum stcc_prov_op op;
	unsigned int err_code;
	void *msg;
	size_t size;
	unsigned int flag;
};

#define STCC_IOC_MAGIC 'C'
#define STCC_SIGDMA_LOAD_IOC _IOW(STCC_IOC_MAGIC, 161, struct stcc_sigdma)
#define STCC_PROV_IOC        _IOW(STCC_IOC_MAGIC, 162, struct stcc_prov)

#ifdef __cplusplus
}
#endif

#endif /* __STCC_H */
