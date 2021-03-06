/*
 * arch/arm/include/debug/sti.S
 *
 * Debugging macro include header
 * Copyright (C) 2013 STMicroelectronics (R&D) Limited.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define STIH41X_COMMS_BASE              0xfed00000
#define STIH41X_ASC2_BASE               (STIH41X_COMMS_BASE+0x32000)

#define STIH41X_SBC_LPM_BASE            0xfe400000
#define STIH41X_SBC_COMMS_BASE          (STIH41X_SBC_LPM_BASE + 0x100000)
#define STIH41X_SBC_ASC1_BASE           (STIH41X_SBC_COMMS_BASE + 0x31000)

#define STIH407_SBC_COMMS_BASE          0x09500000
#define STIH407_SBC_ASC0_BASE           (STIH407_SBC_COMMS_BASE + 0x30000)

/* macro to get at IO space when running virtually */
#ifdef CONFIG_MMU
/*
 * Statically mapped addresses:
 *
 * fc000000 to ffffffff (physical) -> fb000000 to feffffff (virtual)
 */
#define VIRT_ADDRESS(x)		(((x) & 0x03ffffff) + 0xfb000000)
#else
#define VIRT_ADDRESS(x)		(x)
#endif

#if IS_ENABLED(CONFIG_STIH41X_DEBUG_ASC2)
#define DEBUG_LL_UART_BASE	STIH41X_ASC2_BASE
#endif

#if IS_ENABLED(CONFIG_STIH41X_DEBUG_SBC_ASC1)
#define DEBUG_LL_UART_BASE	STIH41X_SBC_ASC1_BASE
#endif

#if IS_ENABLED(CONFIG_STIH407_DEBUG_SBC_ASC0)
#define DEBUG_LL_UART_BASE	STIH407_SBC_ASC0_BASE
#endif

#ifndef DEBUG_LL_UART_BASE
#error "DEBUG UART is not Configured"
#endif

#define ASC_TX_BUF_OFF  0x04
#define ASC_CTRL_OFF    0x0c
#define ASC_STA_OFF     0x14

#define ASC_STA_TX_FULL         (1<<9)
#define ASC_STA_TX_EMPTY        (1<<1)


		.macro	addruart, rp, rv, tmp
		ldr	\rp,      =DEBUG_LL_UART_BASE	@ physical base
		ldr	\rv,      =VIRT_ADDRESS(DEBUG_LL_UART_BASE) @ virt base
		.endm

                .macro  senduart,rd,rx
                strb    \rd, [\rx, #ASC_TX_BUF_OFF]
                .endm

                .macro  waituart,rd,rx
1001:           ldr     \rd, [\rx, #ASC_STA_OFF]
                tst     \rd, #ASC_STA_TX_FULL
                bne     1001b
                .endm

                .macro  busyuart,rd,rx
1001:           ldr     \rd, [\rx, #ASC_STA_OFF]
                tst     \rd, #ASC_STA_TX_EMPTY
                beq     1001b
                .endm
