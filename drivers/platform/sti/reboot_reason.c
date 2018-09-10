/*
 * -------------------------------------------------------------------------
 * Copyright (C) 2015  STMicroelectronics
 * Author: Sudeep Biswas <sudeep.biswas@st.com>
 *
 * May be copied or modified under the terms of the GNU General Public
 * License V.2 ONLY.  See linux/COPYING for more information.
 * -------------------------------------------------------------------------
 */

#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/power/st_lpm.h>
#include <linux/debugfs.h>

#define OBV_REG_SYSCFG_OFFSET 0x7d0
#define OBV_REG_POR_VAL 0x1
#define OBV_REG_RESET_SOURCE_COUNTER_MASK GENMASK(1, 0)
#define OBV_REG_FP_RESET_SHIFT 6
#define OBV_REG_WATCHDOG_SHIFT 8
#define OBV_REG_SBC_WATCHDOG_SHIFT 18

#define SBC_ALL_WAKEUP_TRIGGER_MASK GENMASK(9, 0)

#define MAX_DESC_LEN 32

enum detector_type {SYSCFG, SBC};

/**
 * struct reboot_cause - Describes a reboot cause
 * @desc	Description of the cause
 * @detector	Describes which component capture this reboot cause.
 *		It could be either SYSCFG register or SBC.
 * @count	For SBC captured causes, it stores the wake-up trigger
 *		value. Hence it shows the trigger that caused the cause last
 *		time when it occurred.
 *		For SYSCFG capture causes, it stores the number of occurance
 *		of the cause till now (since the last cold boot).
 *		Every occurance of the cause (and hence reboot due to it)
 *		increments this value. Note that maximum of 3 such occurance
 *		can be recorded after which the count remains at 3 irrespective
 *		of further occurance of the cause. This is reset to 0 only if
 *		a cold boot happens.
 *@mask	mask that should be applied on the SYSCFG reg/SBC trigger value
 *	to determine if the cause happened and getting the current count
 *	value if it is a SYSCFG captured cause.
 *@shift	mask to be shifted before applying
 */
struct reboot_cause {
	char desc[MAX_DESC_LEN];
	enum detector_type detector;
	int count;
	unsigned long mask;
	unsigned long shift;
};

/* SBC captured reboot causes followed by SYSCFG captured causes */
static struct reboot_cause reasons[] = {
	{
		.mask = ST_LPM_WAKEUP_MANUAL_RST,
		.shift = 0,
		.desc = "Linux reboot cmd",
		.detector = SBC,
	},
	{
		.mask = SBC_ALL_WAKEUP_TRIGGER_MASK,
		.shift = 0,
		.desc = "DCPS Wake-up",
		.detector = SBC,
	},
	{
		.mask = OBV_REG_RESET_SOURCE_COUNTER_MASK,
		.shift = OBV_REG_FP_RESET_SHIFT,
		.desc = "Front panel reset",
		.detector = SYSCFG,
	},
	{
		.mask = OBV_REG_RESET_SOURCE_COUNTER_MASK,
		.shift = OBV_REG_WATCHDOG_SHIFT,
		.desc = "Host Watchdog reset",
		.detector = SYSCFG,
	},
	{
		.mask = OBV_REG_RESET_SOURCE_COUNTER_MASK,
		.shift = OBV_REG_SBC_WATCHDOG_SHIFT,
		.desc = "SBC Watchdog reset",
		.detector = SYSCFG,
	},
	{},
};

#ifdef CONFIG_DEBUG_FS
static int reboot_sources_stats_show(struct seq_file *m, void *unused)
{
	struct reboot_cause *p;

	bool coldboot = *(bool *)m->private;

	seq_puts(m, "name\t\t\t\tcount\t\twakeup_event_mask\n");

	if (coldboot) {
		seq_printf(m, "%s\n", "Cold-Boot");
		return 0;
	}

	for (p = reasons; *p->desc; p++) {
		if (!p->count)
			continue;
		if (p->detector == SBC)
			seq_printf(m, "%s\t\t\t\t\t0x%x\n", p->desc, p->count);
		else
			seq_printf(m, "%s\t\t%d\n", p->desc, p->count);
	}

	return 0;
}

static int reboot_sources_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, reboot_sources_stats_show, inode->i_private);
}

static const struct file_operations reboot_sources_stats_fops = {
	.owner = THIS_MODULE,
	.open = reboot_sources_stats_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
#endif

static int __init sti_wakeup_status(void)
{
	int ret;
	unsigned int obv_reg, temp;
	struct regmap *reg;
	enum st_lpm_wakeup_devices lpm_wakeup_source;
	struct reboot_cause *p;

	static bool coldboot;
	bool sbc_reset = false;

	/* First read the SYSCFG observation register */
	reg = syscon_regmap_lookup_by_compatible("st,stih407-sbc-reg-syscfg");
	if (IS_ERR(reg)) {
		pr_err("Reboot cause: SBC Syscfg base not found in DT\n");
		return PTR_ERR(reg);
	}

	ret = regmap_read(reg, OBV_REG_SYSCFG_OFFSET, &obv_reg);
	if (ret) {
		pr_err("Reboot cause: Obv register read failed\n");
		return ret;
	}

	/* If cold boot, return */
	if (obv_reg == OBV_REG_POR_VAL) {
		pr_info("Reboot cause: Cold-Boot\n");
		coldboot = true;
		goto end;
	}

	/* Next read the state of SBC wake-up triggers that causes reboot */
	ret = st_lpm_get_wakeup_device(&lpm_wakeup_source);
	if (ret)
		sbc_reset = true;

	for (p = reasons; *p->desc; p++) {
		switch (p->detector) {
		case SBC:
			if (sbc_reset)
				continue;
			temp = (unsigned int)lpm_wakeup_source;
			break;
		case SYSCFG:
			temp = obv_reg;
			break;
		default:
			pr_err("Reboot cause: Invalid cause detector\n");
			return -EINVAL;
		}

		temp &= (p->mask << p->shift);
		if (!temp)
			continue;

		p->count = temp >> p->shift;

		if (p->detector == SBC)
			pr_info("SBC Reboot cause: %s\n", p->desc);
		else
			pr_info("SYSCFG Reboot count for cause: %s = %d (max 3)\n",
				p->desc, p->count);
	}

end:
#ifdef CONFIG_DEBUG_FS
	debugfs_create_file("reboot_sources",
			    S_IRUGO,
			    NULL,
			    &coldboot,
			    &reboot_sources_stats_fops);
#endif

	return 0;
}

late_initcall(sti_wakeup_status);
