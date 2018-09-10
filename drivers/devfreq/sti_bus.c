/* drivers/devfreq/sti_bus.c
 *
 * Copyright (C) ST-Microelectronics SA 2015
 * Author:  Vincent Guittot <vincent.guittot@st.com> for ST-Microelectronics.
 * License terms:  GNU General Public License (GPL), version 2
 *
 * This file is largely based on drivers/devfreq/exynos4_bus.c
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/suspend.h>
#include <linux/opp.h>
#include <linux/devfreq.h>
#include <linux/clk.h>

/*
 * Set this flag in data if the frequency changes must occur out of the freeze
 * sequence; ie before starting the suspend and after resuming. If the bit is
 * clear, we change the frequency of the clocks at the late step of the suspend
 * and restore them at the early step of the resume
 */
#define STI_EARLY_DECREASE 0x1

struct busfreq_clk_info {
	struct list_head clk_node;
	struct clk *clk;
	unsigned long low_rate, backup_rate;
};

struct busfreq_data {
	struct list_head	clks;
	struct device *dev;
	struct devfreq *devfreq;
	struct notifier_block pm_notifier;
	struct mutex lock;
	bool disable;
	int type;
};

static int sti_bus_target(struct device *dev, unsigned long *_freq,
			      u32 flags)
{
	int err = 0;
	struct busfreq_data *data = dev_get_drvdata(dev);
	struct busfreq_clk_info *clk_info;
	struct opp *opp;
	unsigned long freq, volt;

	if (data->disable)
		return 0;

	/* Choose closest OPP to requested freq */
	rcu_read_lock();
	opp = devfreq_recommended_opp(dev, _freq, flags);
	if (IS_ERR(opp)) {
		rcu_read_unlock();
		return PTR_ERR(opp);
	}
	freq = opp_get_freq(opp);
	volt = opp_get_voltage(opp);
	rcu_read_unlock();

	dev_dbg(dev, "targeting %lukHz %luuV\n", freq, volt);

	mutex_lock(&data->lock);

	/* Set clock of the first element for now */
	clk_info = list_first_entry_or_null(&data->clks,
			struct busfreq_clk_info, clk_node);

	err = clk_set_rate(clk_info->clk, freq);

	mutex_unlock(&data->lock);
	return err;
}

static int sti_bus_get_dev_status(struct device *dev,
				      struct devfreq_dev_status *stat)
{
	struct busfreq_data *data = dev_get_drvdata(dev);

	stat->current_frequency = 0;

	/* Return that we are fully busy for now */
	stat->busy_time = 100;
	stat->total_time = 100;

	return 0;
}

static void sti_bus_exit(struct device *dev)
{
	struct busfreq_data *data = dev_get_drvdata(dev);

	/* Nothing to do for now */
}

static struct devfreq_dev_profile sti_devfreq_profile = {
	.initial_freq	= 100000,
	.polling_ms	= 0,
	.target		= sti_bus_target,
	.get_dev_status	= sti_bus_get_dev_status,
	.exit		= sti_bus_exit,
};

/* Decrease freq of registered clocks to recorded low rate */
static int sti_bus_decrease_freq(struct busfreq_data *data)
{
	struct busfreq_clk_info *clk_info;
	int ret;

	list_for_each_entry(clk_info, &data->clks, clk_node) {
		clk_info->backup_rate = clk_get_rate(clk_info->clk);
		dev_info(data->dev, "Set from %lu to %lu\n",
				clk_info->backup_rate,
				clk_info->low_rate);
		ret = clk_set_rate(clk_info->clk, clk_info->low_rate);
		if (ret)
			return ret;
	}

	return 0;
}

/* Restore freq fo registered clocks */
static int sti_bus_restore_freq(struct busfreq_data *data)
{
	struct busfreq_clk_info *clk_info;
	int ret;

	list_for_each_entry(clk_info, &data->clks, clk_node) {
		dev_info(data->dev, "Set back to %lu\n", clk_info->backup_rate);
		ret = clk_set_rate(clk_info->clk, clk_info->backup_rate);
		if (ret)
			return ret;
	}

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int sti_bus_suspend_late(struct device *dev)
{
	struct busfreq_data *data = dev_get_drvdata(dev);

	if (data->type & STI_EARLY_DECREASE)
		return 0;

	return sti_bus_decrease_freq(data);

}

static int sti_bus_resume_early(struct device *dev)
{
	struct busfreq_data *data = dev_get_drvdata(dev);

	if (data->type & STI_EARLY_DECREASE)
		return 0;

	return sti_bus_restore_freq(data);
}

static const struct dev_pm_ops sti_bus_pm = {
	.suspend_late = sti_bus_suspend_late,
	.resume_early = sti_bus_resume_early,
};

#define ST_BUS_PM (&sti_bus_pm)
#else
#define ST_BUS_PM NULL
#endif

static int sti_bus_pm_notifier_event(struct notifier_block *this,
		unsigned long event, void *ptr)
{
	struct busfreq_data *data = container_of(this, struct busfreq_data,
						 pm_notifier);

	if (!(data->type & STI_EARLY_DECREASE))
		return NOTIFY_DONE;

	switch (event) {
	case PM_SUSPEND_PREPARE:
		sti_bus_decrease_freq(data);
		return NOTIFY_OK;
	case PM_POST_RESTORE:
	case PM_POST_SUSPEND:
		sti_bus_restore_freq(data);
		return NOTIFY_OK;
	}

	return NOTIFY_DONE;
}

static int default_conf;
static struct of_device_id st_busfreq_match[] = {
	{ .compatible = "st,busfreq", .data = &default_conf },
	{},
};
MODULE_DEVICE_TABLE(of, st_busfreq_match);

static int sti_busfreq_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;
	struct busfreq_data *data;
	struct clk *clk;
	struct busfreq_clk_info *clk_info;
	int i, err;

	struct property	*prop;
	const __be32 *cur;
	u32 rate;

	match = of_match_device((st_busfreq_match), &pdev->dev);
	if (!match || !match->data) {
		dev_err(&pdev->dev, "No device match found\n");
		return -ENODEV;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(struct busfreq_data),
			GFP_KERNEL);
	if (data == NULL) {
		dev_err(dev, "Cannot allocate memory for busfreq_data.\n");
		return -ENOMEM;
	}

	data->type = *(int *)(match->data);
	data->dev = dev;
	data->pm_notifier.notifier_call = sti_bus_pm_notifier_event;
	mutex_init(&data->lock);
	INIT_LIST_HEAD(&data->clks);

	/* populate opp table */
	if (of_init_opp_table(dev))
		data->disable = true;
	else
		data->disable = false;

	/* Get bus clocks */
	i = 0;
	of_property_for_each_u32(dev->of_node, "clock-frequency", prop,
			cur, rate) {

		clk = of_clk_get(dev->of_node, i);
		if (IS_ERR(clk)) {
			err = PTR_ERR(clk);
			goto out_error;
		}

		clk_info = devm_kzalloc(&pdev->dev,
				sizeof(struct busfreq_clk_info), GFP_KERNEL);
		if (clk_info == NULL) {
			dev_err(dev, "Cannot allocate memory for clk info.\n");

			err = -ENOMEM;
			goto out_error;
		}

		clk_info->clk = clk;
		clk_info->low_rate = rate;
		list_add_tail(&clk_info->clk_node, &data->clks);

		dev_info(dev, "Adding clock with reduced freq %lu.\n", rate);

		i++;

	}

	/* Set driver data */
	platform_set_drvdata(pdev, data);

	data->devfreq = devfreq_add_device(dev, &sti_devfreq_profile,
					   "performance", NULL);
	if (IS_ERR(data->devfreq)) {
		err = PTR_ERR(data->devfreq);
		goto out_error;
	}

	devfreq_register_opp_notifier(dev, data->devfreq);

	err = register_pm_notifier(&data->pm_notifier);
	if (err) {
		dev_err(dev, "Failed to setup pm notifier\n");
		devfreq_remove_device(data->devfreq);
		goto out_error;
	}

	return 0;

out_error:
	list_for_each_entry(clk_info, &data->clks, clk_node) {
		clk_put(clk_info->clk);
	}

	return err;

}

static int sti_busfreq_remove(struct platform_device *pdev)
{
	struct busfreq_data *data = platform_get_drvdata(pdev);
	struct busfreq_clk_info *clk_info;

	unregister_pm_notifier(&data->pm_notifier);

	list_for_each_entry(clk_info, &data->clks, clk_node) {
		clk_put(clk_info->clk);
	}

	devfreq_remove_device(data->devfreq);

	return 0;
}

static struct platform_driver sti_busfreq_driver = {
	.driver = {
		.name	= "st-busfreq",
		.owner	= THIS_MODULE,
		.of_match_table = st_busfreq_match,
		.pm	= ST_BUS_PM,
	},
	.probe	= sti_busfreq_probe,
	.remove	= sti_busfreq_remove,
};

static int __init sti_busfreq_init(void)
{
	return platform_driver_register(&sti_busfreq_driver);
}
late_initcall(sti_busfreq_init);

static void __exit sti_busfreq_exit(void)
{
	platform_driver_unregister(&sti_busfreq_driver);
}
module_exit(sti_busfreq_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("STI busfreq driver with devfreq framework");
MODULE_AUTHOR("Vincent Guittot <vincent.guittot@st.com");
