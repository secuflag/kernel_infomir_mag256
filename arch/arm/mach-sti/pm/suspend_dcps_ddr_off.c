/*
 * -------------------------------------------------------------------------
 * Copyright (C) 2015  STMicroelectronics
 * Author: Sudeep Biswas <sudeep.biswas@st.com>
 *
 * May be copied or modified under the terms of the GNU General Public
 * License V.2 ONLY.  See linux/COPYING for more information.
 *
 * ------------------------------------------------------------------------- */

#include <asm/cacheflush.h>
#include <linux/io.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#include <linux/power/st_lpm_def.h>
#include <linux/power/st_lpm.h>

#include "suspend.h"
#include "suspend_internal.h"
#include "poke_table.h"

#define STI_PIO_PER_BLOCK 8

static struct poke_operation __dcps_lmi_off[] = {
	POKE32(0x0, 0x0),
};

static struct poke_operation __dcps_enter_passive[] = {
	/*
	 * Send message 'ENTER_PASSIVE' (0x5)
	 */
	POKE32(0x0, LPM_MSG_ENTER_PASSIVE),
};

static struct sti_suspend_table sti_dcps_tables[MAX_SUSPEND_TABLE_SIZE];

int sti_dcps_ddr_off_prepare(struct sti_hw_state_desc *state)
{
	void *__iomem __va_buf = state->buffer_data.sti_buffer_code;
	unsigned int sz;

	memcpy(state->buffer_data.sti_buffer_code, sti_cps_on_buffer,
	       sti_cps_on_buffer_sz);

	__va_buf += sti_cps_on_buffer_sz;
	__va_buf = (void *)L2CC_CACHE_ALIGN((u32)__va_buf);

	state->buffer_data.sti_locking_code = __va_buf;

	memcpy(__va_buf, sti_cps_lock_code, sti_cps_lock_code_sz);

	sz = sti_cps_get_lockable_size(state);
	state->buffer_data.sz = L2CC_CACHE_ALIGN(sz);

	return 0;
}

int sti_dcps_ddr_off_enter(struct sti_hw_state_desc *state)
{
	/* Flush pending data in dcache */
	flush_cache_all();
	pr_info("sti pm dcps: System entering DCPS....\n");

	local_irq_disable();

	/* Call to enter DCPS */
	sti_cps_exec((long)(&state->buffer_data));
	/* Control should _NOT_ reach here */
	BUG();
}

int sti_dcps_ddr_off_setup(struct sti_hw_state_desc *state,
			   unsigned long *ddr_pctl_addr,
			   unsigned int no_pctl,
			   struct sti_ddr3_low_power_info *ddr_sys_conf)
{
	struct device_node *np;
	int gpio_pin, ret, poweroff_gpio_offset;
	unsigned int gpioa, port, port_size;
	int index = 0, i;
	void __iomem *gpio_mem;
	void __iomem *lpm_mem;
	unsigned int *tlb_va = state->buffer_data.tlb_lock_addr;
	unsigned int *nrtlb = &(state->buffer_data.nr_tlb_lock);
	unsigned int lock_code_sz, sz;

	*nrtlb = 0;

	if (state->ddr_state == DDR_ON || state->ddr_state == DDR_SR)
		return -EINVAL;

	np = of_find_node_by_name(NULL, "ddr-pctl-controller");
	if (IS_ERR_OR_NULL(np))
		return -ENODEV;

	gpio_pin = of_get_named_gpio(np, "st,ddr-poweroff-gpio", 0);
	if (!gpio_is_valid(gpio_pin)) {
		of_node_put(np);
		return -EINVAL;
	}

	ret = gpio_request(gpio_pin, "ddr-poweroff");
	if (ret != 0) {
		of_node_put(np);
		return ret;
	}

	gpio_direction_output(gpio_pin, 1);

	of_node_put(np);

	port = gpio_pin / STI_PIO_PER_BLOCK;

	np = of_find_node_by_name(NULL,
				  "pin-controller-sbc");
	if (IS_ERR_OR_NULL(np) ||
	    of_property_read_u32_index(np,
				       "ranges",
				       1,
				       &gpioa) ||
	    of_property_read_u32_index(np,
				       "ranges",
				       2,
				       &port_size)) {
		if (!IS_ERR_OR_NULL(np))
			of_node_put(np);

		gpio_free(gpio_pin);
		return -ENODEV;
	}

	of_node_put(np);

	poweroff_gpio_offset = port * STI_GPIO_REGS_MAP_SZ;
	if ((poweroff_gpio_offset + STI_GPIO_REGS_MAP_SZ) > port_size) {
		gpio_free(gpio_pin);
		return -EINVAL;
	}

	gpioa += poweroff_gpio_offset;

	gpio_mem = ioremap_nocache(gpioa, STI_GPIO_REGS_MAP_SZ);

	if (!gpio_mem) {
		gpio_free(gpio_pin);
		return -ENOMEM;
	}

	*(((long *)__dcps_lmi_off) + 1) = (unsigned int)
					(STI_GPIO_REG_CLR_POUT + gpio_mem);

	/* Need to place the GPIO VA->PA mapping to the TLB */
	tlb_va[(*nrtlb)++] = (unsigned int)(STI_GPIO_REG_CLR_POUT + gpio_mem);

	*(((long *)__dcps_lmi_off) + 2) = 1 << (gpio_pin % STI_PIO_PER_BLOCK);

	np = of_find_compatible_node(NULL, NULL, "st,lpm");
	if (IS_ERR_OR_NULL(np)) {
		gpio_free(gpio_pin);
		iounmap(gpio_mem);
		return -ENODEV;
	}

	lpm_mem = of_iomap(np, 1);
	if (!lpm_mem) {
		of_node_put(np);
		gpio_free(gpio_pin);
		iounmap(gpio_mem);
		return -ENOMEM;
	}

	*(((long *)__dcps_enter_passive) + 1) = (unsigned int) (lpm_mem + 0x4);

	/* Need to place the lpm VA->PA mapping to the TLB */
	tlb_va[(*nrtlb)++] = (unsigned int) (lpm_mem + 0x4);

	of_node_put(np);

	/* Below poke will switch off the LMI power */
	populate_suspend_table_entry(&sti_dcps_tables[index++],
				     (long *)__dcps_lmi_off,
				     NULL,
				     sizeof(__dcps_lmi_off),
				     0,
				     0);

	/* Below poke will ask LPM F/w to switch off the host power */
	populate_suspend_table_entry(&sti_dcps_tables[index++],
				     (long *)__dcps_enter_passive,
				     NULL,
				     sizeof(__dcps_enter_passive),
				     0,
				     0);

	INIT_LIST_HEAD(&state->state_tables);
	for (i = 0; i < index; ++i)
		list_add_tail(&sti_dcps_tables[i].node,
			      &state->state_tables);

	sz = sti_cps_get_lockable_size(state);

	lock_code_sz = L2CC_CACHE_ALIGN(sz) + sti_cps_lock_code_sz;
	if (lock_code_sz > PAGE_SIZE ||
	    lock_code_sz > sti_get_l2cc_way_size()) {
		pr_err("sti dcps: lockablecode + locking code > pagesize/way");
		goto err;
	}

	state->cache_buffer = (unsigned char *)__get_free_page(GFP_KERNEL);

	if (!state->cache_buffer) {
		pr_err("sti pm dcps: cache buffer alloc failed in CPS setup\n");
		goto err;
	}

	return 0;
err:
	gpio_free(gpio_pin);
	iounmap(gpio_mem);
	iounmap(lpm_mem);

	return -ENODEV;
}
