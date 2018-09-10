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

#include <linux/atomic.h>
#include <linux/bug.h>
#include <linux/capability.h>
#include <linux/cc.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "cc_internals.h"

/* CC commands */
#define MBX_CMD_SR         BIT(0)
#define MBX_CMD_PROV_CPK   BIT(4)
#define MBX_CMD_PROV_INS   BIT(5)
#define MBX_CMD_PROV_DIV   BIT(6)
#define MBX_CMD_PROV_CHECK BIT(7)
#define MBX_CMD_SIG_DMA    BIT(8)

#define MBX_MASK (				\
	MBX_CMD_SR       |			\
	MBX_CMD_SIG_DMA  |			\
	MBX_CMD_PROV_CPK |			\
	MBX_CMD_PROV_INS |			\
	MBX_CMD_PROV_DIV |			\
	MBX_CMD_PROV_CHECK)
#define MBX_REQ(x) (x << 16)

/* SRD sub-commands */
#define CMD_SRD_ADD   0x01
#define CMD_SRD_GRANT 0x20

struct mbx {
	u32 status;
	u32 set;
	u32 clear;
	u32 mask;
};

struct control_words {
	u32 version;
	u32 sigchk_command;
	u32 sigchk_srd_addr;
	u32 reserved0[3];
	u32 sigdma_struct_addr;
	u32 reserved1[2];
	u32 prov_custid_addr;
	u32 prov_msg_addr;
	u32 prov_div_addr;
	u32 prov_test_addr;
	u32 prov_setperm_addr;
	u32 reserved2[2];
	u32 sigchk_error_code;
	u32 sigdma_error_code;
	u32 reserved3[1];
	u32 prov_error_code;
};

static atomic_t cc_dev_probed = ATOMIC_INIT(0);

static struct cc_device *cc_dev_p;

/**
 * CC private methods
 */
/**
 * cc_sigdma_strerr - Return a verbose error string for sigdma load
 * errors
 *
 * @err: error code
 */
static const char *cc_sigdma_strerr(int err)
{
	switch (err) {
	case BIT(0):
		return "Invalid sigDMA header format";
	case BIT(1):
		return "SigDMA data authentication failed";
	case BIT(2):
		return "Sales type checking failed";
	case BIT(3):
		return "Version control checking failed";
	case BIT(4):
	case BIT(5):
		return "Invalid key";
	case BIT(6):
		return "Invalid sigDMA format";
	default:
		return "";
	}
}

/**
 * cc_srd_strerr - Return a verbose error string for srd load errors
 *
 * @err: error code
 */
static const char *cc_srd_strerr(int err)
{
	switch (err) {
	case BIT(0):
		return "Invalid SRD authentication";
	case BIT(1):
		return "Invalid SRD region authentication";
	case BIT(2):
		return "Background hash checking failed";
	case BIT(3):
		return "Sales type checking failed";
	case BIT(4):
		return "Version control checking failed";
	case BIT(5):
		return "Invalid SRD operation";
	case BIT(6):
	case BIT(7):
	case BIT(10):
		return "Invalid SRD format";
	case BIT(8):
	case BIT(9):
		return "Invalid key";
	case BIT(20):
		return "Invalid encryption mode";
	default:
		return "";
	}
}

/**
 * cc_prov_strerr - Return a verbose error string for provisioning errors
 *
 * @op:  type of provisioning operation (check, insert, cpk, diversity)
 * @err: error code
 */
static const char *cc_prov_strerr(enum stcc_prov_op op, int err)
{
	switch (op) {
	case STCC_PROV_CPK:
		switch (err) {
		case BIT(0):
			return "Invalid RPK";
		case BIT(1):
			return "CPK already computed";
		case BIT(3):
			return "Parameter location in memory is invalid";
		default:
			return "";
		}
	case STCC_PROV_INSERT:
	case STCC_PROV_DIV:
	case STCC_PROV_CHECK:
		switch (err) {
		case BIT(3):
			return "Parameter location in memory is invalid";
		default:
			return "";
		}
	default:
		return "";
	}
}

static bool cc_valid(void)
{
	return atomic_read(&cc_dev_probed) == 1 && cc_dev_p;
}

/**
 * cc_send_request - Send a request to the CC device
 * WARNING: cc_dev_p->lock must be held
 *
 * @cmd:   command to send
 * @err_p: pointer to mapped memory where status can be retrieved from
 * @err:   pointer to error value retrieved from CC device
 */
static int cc_send_request(u32 cmd, u32 __iomem *err_p, u32 *err)
{
	int ret;

	if (!err_p)
		return -EINVAL;

	iowrite32(MBX_REQ(cmd), &cc_dev_p->mbx_p->set);

	ret = wait_event_interruptible_timeout(cc_dev_p->waitq,
					       (cc_dev_p->mbx_status & cmd),
					       msecs_to_jiffies(10000));

	if (ret < 0)
		return ret;

	if (ret == 0)
		return -ETIMEDOUT;

	cc_dev_p->mbx_status &= ~cmd;
	*err = ioread32(err_p);
	if (*err) {
		dev_warn(cc_dev_p->dev, "request failed: %x\n",
			 *err);
		return -EIO;
	}

	return 0;
}

/**
 * cc_sigdma_load - Request a sigdma load of data
 *
 * @type:  type of data to load (unused for now)
 * @sect0: pointer to sect0 to load (must point to contiguous memory)
 * @sect1: pointer to sect1 to load (must point to contiguous memory)
 * @err:   request error value when method returns -EIO error
 */
static int cc_sigdma_load(u32 type,
			  struct section *sect0, struct section *sect1,
			  u32 *err)
{
	struct local {
		u32 ptr1;
		u32 ptr2;
		u32 ptr3;
	} *hdr_p;
	dma_addr_t dma_sect0, dma_sect1;
	int ret = 0;
	(void) type;

	if (!cc_valid())
		return -ENODEV;

	if (sect0->size < sizeof(*hdr_p))
		return -EINVAL;

	hdr_p = sect0->ptr;

	/* Prepare to flush memory for CC device */

	/* need to get phys addr and update content before flushing content */
	dma_sect0 = dma_map_single(cc_dev_p->dev, sect0->ptr, sect0->size,
				   DMA_TO_DEVICE);
	if (dma_mapping_error(cc_dev_p->dev, dma_sect0))
		return -ENOMEM;

	dma_sect1 = dma_map_single(cc_dev_p->dev, sect1->ptr, sect1->size,
				   DMA_TO_DEVICE);

	if (dma_mapping_error(cc_dev_p->dev, dma_sect1)) {
		dma_unmap_single(cc_dev_p->dev,
				 dma_sect0, sect0->size, DMA_TO_DEVICE);
		return -ENOMEM;
	}

	/* convert relative addresses to absolute addresses */
	hdr_p->ptr1 += dma_sect0;
	hdr_p->ptr2 += dma_sect0;
	hdr_p->ptr3 = dma_sect1;
	dma_sync_single_for_device(cc_dev_p->dev,
				   dma_sect0, sect0->size, DMA_TO_DEVICE);

	/* Now do the com. with the device */
	/* Mutex used to serialize all com */
	ret = mutex_lock_interruptible(&cc_dev_p->lock);
	if (ret)
		goto out_unmap;

	writel_relaxed(dma_sect0, &cc_dev_p->ctrl_words_p->sigdma_struct_addr);

	*err = 0;
	ret = cc_send_request(MBX_CMD_SIG_DMA,
			      &cc_dev_p->ctrl_words_p->sigdma_error_code,
			      err);
	mutex_unlock(&cc_dev_p->lock);
	if (ret)
		dev_err(cc_dev_p->dev, "SigDma error - %s (%x)\n",
			cc_sigdma_strerr(*err), *err);
out_unmap:
	dma_unmap_single(cc_dev_p->dev, dma_sect1, sect1->size, DMA_TO_DEVICE);
	dma_unmap_single(cc_dev_p->dev, dma_sect0, sect0->size, DMA_TO_DEVICE);

	return ret;
}

/**
 * stcc_srd_load - Request a srd load of data
 *
 * @type:  type of SRD firmware to load
 * @srd:   pointer to srd section to load (must point to contiguous memory)
 * @cb:    the loading firmware callback
 * @param: pointer to parameter of the callback
 * @err:   CC error code, if any.
 */
static int cc_srd_load(enum srd_fw_type type, struct section *srd,
		       int (*cb)(const void *p),  const void *param, u32 *err)
{
	struct local {
		u32 res[44];
		u32 ptr1;
		u32 ptr2;
	} *hdr_p;
	dma_addr_t dma_addr;
	int ret, ret2;
	u32 err2;
	(void) type;

	if (!cc_valid())
		return -ENODEV;

	if (srd->size < sizeof(*hdr_p))
		return -EINVAL;
	hdr_p = srd->ptr;

	/* Flush memory and get physical address of SRD */
	dma_addr = dma_map_single(cc_dev_p->dev, srd->ptr, srd->size,
				  DMA_TO_DEVICE);
	if (dma_mapping_error(cc_dev_p->dev, dma_addr))
		return -ENOMEM;

	/* convert relative addresses to absolute addresses */
	hdr_p->ptr1 += dma_addr;
	hdr_p->ptr2 += dma_addr;
	dma_sync_single_for_device(cc_dev_p->dev,
				   dma_addr, srd->size, DMA_TO_DEVICE);

	/* Now do the com. with the device */
	/* Mutex used to serialize the whole com */
	ret = mutex_lock_interruptible(&cc_dev_p->lock);
	if (ret)
		goto out_unmap;

	writel_relaxed(dma_addr, &cc_dev_p->ctrl_words_p->sigchk_srd_addr);
	writel_relaxed(CMD_SRD_GRANT, &cc_dev_p->ctrl_words_p->sigchk_command);

	err2 = *err = 0;
	ret = cc_send_request(MBX_CMD_SR,
			      &cc_dev_p->ctrl_words_p->sigchk_error_code,
			      err);
	if (ret)
		dev_err(cc_dev_p->dev, "SigChk grant error - %s (%x)\n",
			cc_srd_strerr(*err), *err);
	if ((ret == 0) && (cb))
		ret = (*cb)(param);

	/*
	 * The second command must be sent to CC, whatever the result of the
	 * first command, to unblock CC
	 */
	writel_relaxed(dma_addr, &cc_dev_p->ctrl_words_p->sigchk_srd_addr);
	writel_relaxed(CMD_SRD_ADD, &cc_dev_p->ctrl_words_p->sigchk_command);
	ret2 = cc_send_request(MBX_CMD_SR,
			       &cc_dev_p->ctrl_words_p->sigchk_error_code,
			       &err2);
	mutex_unlock(&cc_dev_p->lock);
	if (ret2)
		dev_err(cc_dev_p->dev, "SigChk add error - %s (%x)\n",
			cc_srd_strerr(err2), err2);

	/* Return the first error */
	if  (!ret && ret2)
		ret = ret2;
	if (!*err && err2)
		*err = err2;
out_unmap:
	dma_unmap_single(cc_dev_p->dev, dma_addr, srd->size, DMA_TO_DEVICE);

	return ret;
}

/**
 * cc_provisioning - Request a provisioning operation
 *
 * @op:   type of operation (check, insert, cpk, diversity)
 * @msg:  provisioning message (must point to contiguous memory)
 * @size: size of the provisioning message
 * @flag: optional flag value used by insert or check operation
 * @err:  request error value when method returns -EIO error
 */
static int cc_provisioning(enum stcc_prov_op op, void *msg, size_t size,
			   u32 flag, u32 *err)
{
	int ret = 0;
	dma_addr_t dma_msg;
	u32 cmd;

	if (!cc_valid())
		return -ENODEV;

	if ((op != STCC_PROV_INSERT) && (op != STCC_PROV_CHECK) &&
	    (op != STCC_PROV_CPK) && (op != STCC_PROV_DIV))
		return -EINVAL;
	if ((msg == NULL) || (size == 0))
		return -EINVAL;

	dma_msg = dma_map_single(cc_dev_p->dev, msg, size, DMA_TO_DEVICE);
	if (dma_mapping_error(cc_dev_p->dev, dma_msg))
		return -ENOMEM;

	/* Now do the com. with the device */
	/* Mutex used to serialize all com */
	ret = mutex_lock_interruptible(&cc_dev_p->lock);
	if (ret)
		goto out_unmap;

	switch (op) {
	case STCC_PROV_CPK:
		cmd = MBX_CMD_PROV_CPK;
		writel_relaxed(dma_msg,
			       &cc_dev_p->ctrl_words_p->prov_custid_addr);
		break;
	case STCC_PROV_INSERT:
		cmd = MBX_CMD_PROV_INS;
		writel_relaxed(dma_msg, &cc_dev_p->ctrl_words_p->prov_msg_addr);
		writel_relaxed(flag, &cc_dev_p->ctrl_words_p->prov_test_addr);
		break;
	case STCC_PROV_DIV:
		cmd = MBX_CMD_PROV_DIV;
		writel_relaxed(dma_msg, &cc_dev_p->ctrl_words_p->prov_div_addr);
		break;
	case STCC_PROV_CHECK:
		cmd = MBX_CMD_PROV_CHECK;
		writel_relaxed(dma_msg, &cc_dev_p->ctrl_words_p->prov_msg_addr);
		writel_relaxed(flag,
			       &cc_dev_p->ctrl_words_p->prov_setperm_addr);
		break;
	default:
		ret = -EINVAL;
		goto out_unlock;
	}

	*err = 0;
	ret = cc_send_request(cmd, &cc_dev_p->ctrl_words_p->prov_error_code,
			      err);
	if (ret)
		dev_err(cc_dev_p->dev, "Provisioning error - %s (%x)\n",
			cc_prov_strerr(op, *err), *err);

out_unlock:
	mutex_unlock(&cc_dev_p->lock);
out_unmap:
	dma_unmap_single(cc_dev_p->dev, dma_msg, size, DMA_TO_DEVICE);

	return ret;
}

/**
 * CC public methods
 */
/**
 * stcc_sigdma_load - Request a sigdma load of data
 *
 * @type:  type of data to load (unused for now)
 * @sect0: pointer to sect0 to load
 * @sect1: pointer to sect1 to load
 * @err:   pointer to returned request error value
 */
int stcc_sigdma_load(u32 type, struct section *sect0, struct section *sect1,
		     u32 *err)
{
	struct section sect[N_SIGDMA_SECTION];
	int ret = 0;
	(void)type;

	if ((sect0 == NULL) || (sect1 == NULL) ||
	    (sect0->ptr == NULL) || (sect1->ptr == NULL) ||
	    (err == NULL))
		return -EINVAL;

	sect[0].size = sect0->size;
	sect[0].ptr = kmalloc(sect[0].size, GFP_KERNEL);
	if (sect[0].ptr == NULL)
		return -ENOMEM;
	sect[1].size = sect1->size;
	sect[1].ptr = kmalloc(sect[1].size, GFP_KERNEL);
	if (sect[1].ptr == NULL) {
		kfree(sect[0].ptr);
		return -ENOMEM;
	}
	memcpy(sect[0].ptr, sect0->ptr, sect[0].size);
	memcpy(sect[1].ptr, sect1->ptr, sect[1].size);

	ret = cc_sigdma_load(0, &sect[0], &sect[1], err);

	kfree(sect[0].ptr);
	kfree(sect[1].ptr);

	return ret;
}
EXPORT_SYMBOL(stcc_sigdma_load);

/**
 * stcc_srd_load - Request a srd load of data
 *
 * @type:  type of SRD firmware to load
 * @srd:   pointer to srd section to load
 * @cb:    the loading firmware callback
 * @param: pointer to parameter of the callback
 * @err:   CC error code, if any.
 */
int stcc_srd_load(enum srd_fw_type type, struct section *srd,
		  int (*cb)(const void *p),  const void *param, u32 *err)
{
	struct section sect;
	int ret = 0;

	if ((srd == NULL) || (srd->ptr == NULL) || (err == NULL))
		return -EINVAL;

	sect.size = srd->size;
	sect.ptr = kmalloc(sect.size, GFP_KERNEL);
	if (sect.ptr == NULL)
		return -ENOMEM;
	memcpy(sect.ptr, srd->ptr, sect.size);

	ret = cc_srd_load(type, &sect, cb, param, err);

	kfree(sect.ptr);

	return ret;
}
EXPORT_SYMBOL(stcc_srd_load);

/**
 * File operations
 */
static int cc_open(struct inode *inode, struct file *filp)
{
	/* Require some capabilities */
	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	/* Enforce root only access */
	if (current_uid() != 0)
		return -EPERM;

	if (!cc_valid())
		return -ENODEV;

	return 0;
}

static long cc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	u32 err = 0;

	switch (cmd) {
	case STCC_SIGDMA_LOAD_IOC:
	{
		struct stcc_sigdma sigdma;
		struct section sect[N_SIGDMA_SECTION];
		int i;

		for (i = 0; i < N_SIGDMA_SECTION; i++)
			sect[i].ptr = NULL;

		if (copy_from_user(&sigdma, (void __user *)arg, sizeof(sigdma)))
			return -EFAULT;

		for (i = 0; i < N_SIGDMA_SECTION; i++)
			if ((sigdma.sect[i].ptr == NULL) ||
			    (sigdma.sect[i].size == 0))
				return -EINVAL;

		for (i = 0; (i < N_SIGDMA_SECTION) && (ret == 0); i++) {
			sect[i].size = sigdma.sect[i].size;
			sect[i].ptr = kmalloc(sect[i].size, GFP_KERNEL);
			if (sect[i].ptr == NULL) {
				ret = -ENOMEM;
				break;
			}
			if (copy_from_user(sect[i].ptr,
					   sigdma.sect[i].ptr,
					   sect[i].size)) {
				ret = -EFAULT;
				break;
			}
		}

		if (ret == 0)
			ret = cc_sigdma_load(0, &sect[0], &sect[1], &err);

		for (i = 0; i < N_SIGDMA_SECTION; i++)
			kfree(sect[i].ptr);

		if (ret == -EIO) {
			sigdma.err_code = err;
			if (copy_to_user((void __user *)arg, &sigdma,
					 sizeof(sigdma)))
				return -EFAULT;
		}
		return ret;
	}
	case STCC_PROV_IOC:
	{
		struct stcc_prov prov;
		struct section msg;

		if (copy_from_user(&prov, (void __user *)arg, sizeof(prov)))
			return -EFAULT;

		if ((prov.msg == NULL) || (prov.size == 0))
			return -EINVAL;

		msg.size = prov.size;
		msg.ptr = kmalloc(msg.size, GFP_KERNEL);
		if (msg.ptr == NULL)
			return -ENOMEM;
		if (copy_from_user(msg.ptr, prov.msg, msg.size)) {
			kfree(msg.ptr);
			return -EFAULT;
		}

		ret = cc_provisioning(prov.op, msg.ptr, msg.size,
				      prov.flag, &err);

		kfree(msg.ptr);

		if (ret == -EIO) {
			prov.err_code = err;
			if (copy_to_user((void __user *)arg, &prov,
					 sizeof(prov)))
				return -EFAULT;
		}
		return ret;
	}
	default:
		return -EINVAL;
	}
}

static const struct file_operations cc_fops = {
	.owner = THIS_MODULE,
	.open = cc_open,
	.unlocked_ioctl = cc_ioctl,
};

/**
 * Common IRQ handler for both CC interrupts
 */
static irqreturn_t cc_interrupt_handler(int irq, void *data)
{
	u32 status;

	status = ioread32(&cc_dev_p->mbx_p->status);
	cc_dev_p->mbx_status |= (status & MBX_MASK);
	iowrite32((status & MBX_MASK), (&cc_dev_p->mbx_p->clear));
	wake_up_all(&cc_dev_p->waitq);

	return IRQ_HANDLED;
}

/**
 * Module operations: init and exit and LPM support
 */

/**
 * cc_resume() -  Module resume function for the cc driver.
 */
static int cc_resume(struct platform_device *pdev)
{
	iowrite32(MBX_MASK, &cc_dev_p->mbx_p->mask);
	return 0;
}

/**
 * cc_probe() -  Module init function for the cc driver.
 * This routine loads the cc core driver
 *
 * @pdev: platform device.
 */
static int cc_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct cc_device *cc_p;
	struct device_node *np;
	struct resource *res;
	u32 mbx_offset, ctrlw_offset;

	if (pdev == NULL)
		return -EINVAL;

	if (WARN(atomic_inc_return(&cc_dev_probed) != 1 || cc_dev_p,
		 "stcc device already probed !"))
		return -EINVAL;

	np = pdev->dev.of_node;
	if (np == NULL)
		return -EINVAL;

	dev_info(&pdev->dev, "Probing device\n");

	cc_p = devm_kzalloc(&pdev->dev, sizeof(*cc_p), GFP_KERNEL);
	if (!cc_p) {
		dev_err(&pdev->dev, "CC device alloc failed\n");
		return -ENOMEM;
	}

	/* First, retrieve required info from DT */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (IS_ERR(res)) {
		dev_err(&pdev->dev, "invalid io memory resource\n");
		return PTR_ERR(res);
	}

	/* check validity and coherency of resource retrieved from DT */
	if ((res->end < res->start) ||
	    (resource_size(res) <= 0) ||
	    (resource_size(res) & (PAGE_SIZE - 1)) ||
	    (res->start & (PAGE_SIZE - 1)))
		return -EINVAL;

	/* MMIO region must be big enough to fit all registers */
	if (resource_size(res) < (sizeof(struct control_words) +
				  sizeof(struct mbx)))
		return -EINVAL;

	cc_p->sigchk_it = platform_get_irq(pdev, 0);
	if (cc_p->sigchk_it < 0) {
		dev_err(&pdev->dev, "First IT not found in DT\n");
		return cc_p->sigchk_it;
	}

	cc_p->sigdma_it = platform_get_irq(pdev, 1);
	if (cc_p->sigdma_it < 0) {
		dev_err(&pdev->dev, "Second IT not found in DT\n");
		return cc_p->sigdma_it;
	}

	if (of_property_read_u32(np, "st-control-words-offset",
				 &ctrlw_offset)) {
		dev_err(&pdev->dev, "st-cw-offset not found in DT\n");
		return -EINVAL;
	}

	if ((ctrlw_offset >= (ctrlw_offset +
			      sizeof(struct control_words))) ||
	    (resource_size(res) < (ctrlw_offset +
				   sizeof(struct control_words))))
		return -EINVAL;

	if (of_property_read_u32(np, "st-mbx-offset", &mbx_offset)) {
		dev_err(&pdev->dev, "st-mbx-offset not found in DT\n");
		return -EINVAL;
	}

	if ((mbx_offset >= (mbx_offset +
			    sizeof(struct mbx))) ||
	    (resource_size(res) < (mbx_offset +
				   sizeof(struct mbx))))
		return -EINVAL;

	/* Overlap ? */
	if (ctrlw_offset < (mbx_offset +
			    sizeof(struct mbx)) &&
	    mbx_offset < (ctrlw_offset +
			  sizeof(struct control_words)))
		return -EINVAL;

	/* create user device */
	cc_p->miscdev.parent = &pdev->dev;
	cc_p->miscdev.minor = MISC_DYNAMIC_MINOR;
	cc_p->miscdev.name = CC_DEVNAME;
	cc_p->miscdev.fops = &cc_fops;
	cc_p->miscdev.mode = S_IRUSR|S_IXUSR;

	ret = misc_register(&cc_p->miscdev);
	if (ret) {
		dev_err(&pdev->dev, "%s() device create failed\n", __func__);
		return ret;
	}

	/* map required memory */
	cc_p->io_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(cc_p->io_base)) {
		dev_err(&pdev->dev, "can't request and remap IO MEM\n");
		ret = PTR_ERR(cc_p->io_base);
		goto error;
	}
	if (!cc_p->io_base) {
		dev_err(&pdev->dev, "can't request and remap IO MEM\n");
		ret = -ENODEV;
		goto error;
	}

	/* init required ressources for CC interface: IRQ and MBX */
	cc_p->mbx_p = cc_p->io_base + mbx_offset;
	cc_p->ctrl_words_p = cc_p->io_base + ctrlw_offset;

	ret = devm_request_irq(&pdev->dev, cc_p->sigchk_it,
			       cc_interrupt_handler,
			       0, "CC_SIGCHK_MBX", NULL);
	if (ret) {
		dev_err(&pdev->dev, "can't install first interrupt handler\n");
		goto error;
	}

	ret = devm_request_irq(&pdev->dev, cc_p->sigdma_it,
			       cc_interrupt_handler,
			       0, "CC_SIGDMA_MBX", NULL);
	if (ret) {
		dev_err(&pdev->dev, "can't install second interrupt handler\n");
		goto error;
	}

	/* debugfs init */
	ret = cc_debug_init(cc_p);
	if (ret < 0) {
		dev_err(&pdev->dev, "can't create debugfs\n");
		goto error;
	}

	mutex_init(&cc_p->lock);
	init_waitqueue_head(&cc_p->waitq);

	iowrite32(MBX_MASK, &cc_p->mbx_p->mask);
	cc_p->version = ioread32(&cc_p->ctrl_words_p->version);
	platform_set_drvdata(pdev, cc_p);

	cc_p->dev = &pdev->dev;
	cc_dev_p = cc_p;
	dev_info(&pdev->dev, "device probed successfully\n");

	return 0;
error:
	misc_deregister(&cc_p->miscdev);
	return ret;
}

/**
 * cc_remove() - Module exit function for the cc driver
 */
static int cc_remove(struct platform_device *pdev)
{
	struct cc_device *cc_p;

	cc_p = platform_get_drvdata(pdev);
	cc_dev_p = NULL;

	platform_set_drvdata(pdev, NULL);

	writel_relaxed(0, &cc_p->mbx_p->mask);

	cc_debug_exit(cc_p);

	mutex_destroy(&cc_p->lock);
	misc_deregister(&cc_p->miscdev);

	return 0;
}

static const struct of_device_id cc_ids[] = {
	{
		.compatible = "st,cc",
	},
	{},
};

static struct platform_driver platform_cc_driver = {
	.resume = cc_resume,
	.remove = cc_remove,
	.probe = cc_probe,
	.driver = {
		.name = "st-cc",
		.owner = THIS_MODULE,
		.of_match_table = cc_ids,
	},
};

module_platform_driver(platform_cc_driver);

MODULE_DEVICE_TABLE(of, cc_ids);
MODULE_AUTHOR("Pierre Peiffer");
MODULE_DESCRIPTION("STMicroelectronics CryptoCore driver");
MODULE_SUPPORTED_DEVICE("");
MODULE_LICENSE("Dual BSD/GPL v2");
MODULE_VERSION("1.0");
