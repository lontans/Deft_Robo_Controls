#include "host/soft_dfu.h"
#include "main.h"

#define SOFT_DFU_MAGIC ((uint32_t)0x5A5AC0DEu)
#define SOFT_DFU_GUARD ((uint32_t)(~SOFT_DFU_MAGIC))

/* AN2606 Rev 70 Table for STM32G47xxx/48xxx: system memory starts at
 * 0x1FFF0000 (bootloader ID 0xD5 @ 0x1FFF6FFE). USB DFU uses PA11/PA12. */
#define SOFT_DFU_SYSMEM_BASE ((uint32_t)0x1FFF0000u)

typedef struct {
	volatile uint32_t magic;
	volatile uint32_t guard;
} soft_dfu_sig_t;

typedef void (*soft_dfu_reset_fn_t)(void);

typedef struct {
	uint32_t stack_top;
	soft_dfu_reset_fn_t reset;
} soft_dfu_leave_vt_t;

extern uint32_t _estack;

static void soft_dfu_leave_reset(void)
{
	/* AN3156 Leave jumps into app code without a chip reset; USB FS is left
	 * dirty and CDC often never re-enumerates. Force a real system reset. */
	NVIC_SystemReset();
	for (;;) {
	}
}

__attribute__((section(".soft_dfu_leave_vt"), used))
static const soft_dfu_leave_vt_t soft_dfu_leave_vt = {
	.stack_top = (uint32_t)&_estack,
	.reset = soft_dfu_leave_reset,
};

/* Placed in the dedicated .dfu_sig section (see STM32G474RETX_FLASH.ld,
 * right after .bss) so it survives a warm reset: that section sits outside
 * the _sbss.._ebss range Reset_Handler's LoopFillZerobss zeroes, and
 * outside .data's flash-copy range too. */
__attribute__((section(".dfu_sig"))) static soft_dfu_sig_t s_dfu_sig;

bool soft_dfu_is_command(const host_command_image_t *cmd)
{
	if (cmd == NULL)
		return false;

	return cmd->pdu.data[0] == (uint8_t)SOFT_DFU_TAG0 &&
	       cmd->pdu.data[1] == (uint8_t)SOFT_DFU_TAG1 &&
	       cmd->pdu.data[2] == (uint8_t)SOFT_DFU_TAG2 &&
	       cmd->pdu.data[3] == (uint8_t)SOFT_DFU_TAG3;
}

void soft_dfu_on_command(const host_command_image_t *cmd)
{
	(void)cmd;

	s_dfu_sig.magic = SOFT_DFU_MAGIC;
	s_dfu_sig.guard = SOFT_DFU_GUARD;

	/* Ensure signature is visible before the reset clears pipelines. */
	__DSB();
	__ISB();

	NVIC_SystemReset(); /* never returns */

	for (;;) {
	}
}

static void soft_dfu_nvic_reset(void)
{
	uint32_t i;

	for (i = 0u; i < (uint32_t)(sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0])); i++) {
		NVIC->ICER[i] = 0xFFFFFFFFu;
		NVIC->ICPR[i] = 0xFFFFFFFFu;
	}
}

void soft_dfu_check_and_jump(void)
{
	uint32_t sysmem_msp;
	uint32_t sysmem_reset;
	void (*sysmem_boot)(void);

	if (s_dfu_sig.magic != SOFT_DFU_MAGIC || s_dfu_sig.guard != SOFT_DFU_GUARD)
		return;

	/* Consume the signature before attempting the jump: if the jump below
	 * lands somewhere invalid and hard-faults, the NEXT reset must boot
	 * the app instead of retrying forever. */
	s_dfu_sig.magic = 0u;
	s_dfu_sig.guard = 0u;
	__DSB();

	/*
	 * Do NOT call HAL_RCC_DeInit() here. This runs before HAL_Init(), so
	 * SysTick/uwTick are not running; HAL timeout loops hang forever and
	 * the board leaves CDC without ever enumerating DFU. After
	 * NVIC_SystemReset the RCC is already near the bootloader's expected
	 * HSI default.
	 */
	SysTick->CTRL = 0u;
	SysTick->LOAD = 0u;
	SysTick->VAL  = 0u;

	soft_dfu_nvic_reset();

	/* Remap system memory to 0x00000000 (SYSCFG MEM_MODE = 001b). */
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);
	(void)RCC->APB2ENR;
	MODIFY_REG(SYSCFG->MEMRMP, SYSCFG_MEMRMP_MEM_MODE,
	           SYSCFG_MEMRMP_MEM_MODE_0);

	sysmem_msp   = *(volatile uint32_t *)(SOFT_DFU_SYSMEM_BASE + 0x00u);
	sysmem_reset = *(volatile uint32_t *)(SOFT_DFU_SYSMEM_BASE + 0x04u);

	/* Sanity: MSP in SRAM, Reset_Handler Thumb bit set. */
	if ((sysmem_msp & 0xFFF00000u) != 0x20000000u)
		return;
	if ((sysmem_reset & 1u) == 0u)
		return;

	sysmem_boot = (void (*)(void))sysmem_reset;

	__DSB();
	__ISB();
	__set_MSP(sysmem_msp);

	/* ROM USB DFU needs interrupts; leaving PRIMASK set prevents enum. */
	__enable_irq();

	sysmem_boot();

	for (;;) {
	} /* unreachable */
}
