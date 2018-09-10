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

#ifndef __CC_INTERNALS_H
#define __CC_INTERNALS_H

#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/fs.h>

/**
 * struct cc_device - internal driver structure
 *
 * @miscdev:      user device
 * @lock:         serialize communication with CC device
 * @waitq:        wait for CC device ack
 * @dev:          current device
 * @io_base:      CC Device memory map ptr
 * @mbx_p:        CC mailbox
 * @ctrl_words_p: CC interface
 * @sigchk_it:    CC interrupt, retrieved from DT
 * @sigdma_it:    CC interrupt, retrieved from DT
 * @mbx_status:   Memory copy of mailbox status
 * @version:      CC-IRAM version
 * @debugfs_dir:  debug file system directory
 */
struct cc_device {
	struct miscdevice miscdev;
	struct mutex lock;
	wait_queue_head_t waitq;
	struct device *dev;
	void __iomem *io_base;
	struct mbx __iomem *mbx_p;
	struct control_words __iomem *ctrl_words_p;
	int sigchk_it;
	int sigdma_it;
	u32 mbx_status;
	u32 version;
	struct dentry *debugfs_dir;
};

int cc_debug_init(struct cc_device *core);
void cc_debug_exit(struct cc_device *core);

#endif /* __CC_INTERNALS_H */
