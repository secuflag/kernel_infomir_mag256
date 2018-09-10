#ifndef __ASM_ARM_TZ_H
#define __ASM_ARM_TZ_H

/* Support of ARMv7 TrustZone generic features */
#if (__LINUX_ARM_ARCH__ < 7) || !defined(CONFIG_SMP)

/* no ATM TZ or no SMP => no need for shared spinlocks */
static inline void tz_spin_lock(unsigned long *lock) { }
static inline int tz_spin_trylock(unsigned long *lock) { return 1; }
static inline void tz_spin_unlock(unsigned long *lock) { }

#else

#include <asm/processor.h>

/*
 * Shared spinning mutex support
 *
 * Shared mutex between linux and TrustZone worlds require use of very basic
 * ARMv6+ DDR mutex cells:
 * - lock is defined by the value stored in a 32bit DDR cell (4 byte aligned).
 * - value is 0: mutex is not locked, value is 1: mutex is locked.
 * - use of ldrex/strex instructions and a memory barrier when required.
 * - basic power saving: WFE while lock is locked, SEV on lock release.
 * - no extra complexity.
 *
 * Actually, this is the pre kernel 3.6 ARM arch_spinlock support.
 */
static inline void tz_spin_lock(unsigned long *lock)
{
	unsigned long tmp;

	__asm__ __volatile__(
"1:	ldrex	%0, [%1]\n"
"	teq	%0, #0\n"
	WFE("ne")
"	strexeq	%0, %2, [%1]\n"
"	teqeq	%0, #0\n"
"	bne	1b"
	: "=&r" (tmp)
	: "r" (lock), "r" (1)
	: "cc");
/*
 * ARMv6+ ticket-based spin-locking.
 *
 * A memory barrier is required after we get a lock, and before we
 * release it, because V6+ CPUs are assumed to have weakly ordered
 * memory.
 */
	smp_mb();
}

static inline int tz_spin_trylock(unsigned long *lock)
{
	unsigned long tmp;

	__asm__ __volatile__(
"	ldrex	%0, [%1]\n"
"	teq	%0, #0\n"
"	strexeq	%0, %2, [%1]"
	: "=&r" (tmp)
	: "r" (lock), "r" (1)
	: "cc");

	if (tmp)
		return 0;
/*
 * ARMv6+ ticket-based spin-locking.
 *
 * A memory barrier is required after we get a lock, and before we
 * release it, because V6+ CPUs are assumed to have weakly ordered
 * memory.
 */
	smp_mb();
	return 1;
}

static inline void tz_spin_unlock(unsigned long *lock)
{
/*
 * ARMv6+ ticket-based spin-locking.
 *
 * A memory barrier is required after we get a lock, and before we
 * release it, because V6+ CPUs are assumed to have weakly ordered
 * memory.
 */
	smp_mb();
	*lock = 0;

	dsb_sev();
}
#endif

#endif /* __ASM_ARM_TZ_H */
