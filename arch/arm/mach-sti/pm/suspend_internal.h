/*
 * -------------------------------------------------------------------------
 * Copyright (C) 2014  STMicroelectronics
 * Author:	Sudeep Biswas		<sudeep.biswas@st.com>
 *		Francesco M. Virlinzi	<francesco.virlinzi@st.com>
 * May be copied or modified under the terms of the GNU General Public
 * License V.2 ONLY.  See linux/COPYING for more information.
 * ------------------------------------------------------------------------- */

#ifndef __SUSPEND_INTERNAL__
#define __SUSPEND_INTERNAL__

static inline
void populate_suspend_table_entry(struct sti_suspend_table *entry,
				  long *entry_tbl, long *exit_tbl,
				  int size_entry, int size_exit,
				  unsigned long base_address)
{
	entry->enter = entry_tbl;
	entry->enter_size = size_entry;
	entry->exit = exit_tbl;
	entry->exit_size = size_exit;
	entry->base_address = base_address;
}

/* Maximum number of suspend table entries with entry and exit procedures */
#define MAX_SUSPEND_TABLE_SIZE 6

/* HPS specific call declarations */
void sti_hps_exec(struct sti_suspend_buffer_data *buffer_data,
		  unsigned long va_2_pa);

int sti_hps_on_eram(void);
extern unsigned long sti_hps_on_eram_sz;

/* CPS specific call declarations */
int sti_cps_exec(unsigned long);

int sti_cps_on_buffer(void);
extern unsigned long sti_cps_on_buffer_sz;

int sti_cps_lock_code(unsigned long);
extern unsigned long sti_cps_lock_code_sz;

/* Function that is executed after a CPS wakeup on primary core */
void sti_defrost_kernel(void);

/* L2CC cache line size is fixed and always 32 bytes */
#define L2CC_CACHE_LINE_SIZE	32
#define L2CC_CACHE_ALIGN(a) \
(((a) + (L2CC_CACHE_LINE_SIZE - 1)) & (~(L2CC_CACHE_LINE_SIZE - 1)))

/* functions used by dcps state implementation from the cps implementation */
int sti_cps_get_lockable_size(struct sti_hw_state_desc *state);

/* ST GPIO specific constants */
#define STI_GPIO_REG_CLR_POUT   0x08
#define STI_GPIO_REGS_MAP_SZ    0x1000

#endif /* __SUSPEND_INTERNAL__ */

