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

#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/platform_device.h>

#include "cc_internals.h"

static int cc_debugfs_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

#define IROM_MINOR_VERSION(v)  ((v) & 0xFF)
#define IROM_MAJOR_VERSION(v)  (((v) >> 8) & 0xFF)
#define IRAM_MINOR_VERSION(v)  (((v) >> 16) & 0xFF)
#define IRAM_MAJOR_VERSION(v)  (((v) >> 24) & 0xF)
#define IRAM_PATCH_VERSION(v)  (((v) >> 28) & 0xF)

static int cc_debugfs_version_read(struct file *file, char __user *user_buf,
				   size_t size, loff_t *ppos)
{
	struct cc_device *cc_dev = (struct cc_device *)file->private_data;
	char buf[64];
	u32 ver = cc_dev->version;
	int i;

	i = scnprintf(buf, 64, "Version: 0x%08x\nIROM v%u.%u\nIRAM v%u.%u.%u\n",
		      ver,
		      IROM_MAJOR_VERSION(ver), IROM_MINOR_VERSION(ver),
		      IRAM_MAJOR_VERSION(ver), IRAM_MINOR_VERSION(ver),
		      IRAM_PATCH_VERSION(ver)
		);

	return simple_read_from_buffer(user_buf, size, ppos, buf, i);
}

static const struct file_operations version_fops = {
	.open = cc_debugfs_open,
	.read = cc_debugfs_version_read,
	.owner = THIS_MODULE,
};

int cc_debug_init(struct cc_device *cc_dev)
{
	struct dentry *file_entry;

	cc_dev->debugfs_dir = 0;

	cc_dev->debugfs_dir = debugfs_create_dir("stcc", NULL);
	if (!cc_dev->debugfs_dir) {
		dev_warn(cc_dev->dev, "creating debugfs stcc/ dir failed\n");
		return -ENODEV;
	}

	file_entry = debugfs_create_file("version", 0444,
					  cc_dev->debugfs_dir, cc_dev,
					  &version_fops);
	if (!file_entry) {
		dev_warn(cc_dev->dev, "creating debugfs stcc/version file failed\n");
		return -ENODEV;
	}
	return 0;
}

void cc_debug_exit(struct cc_device *cc_dev)
{
	debugfs_remove_recursive(cc_dev->debugfs_dir);
	cc_dev->debugfs_dir = NULL;
}
