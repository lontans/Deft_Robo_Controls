#include "host/soft_dfu.h"
#include "main.h"

#define SOFT_DFU_MAGIC ((uint32_t)0x5A5AC0DEu)
#define SOFT_DFU_GUARD ((uint32_t)(~SOFT_DFU_MAGIC))

/* AN2606, STM32G4 category 2/3 devices: system memory (ROM bootloader) base
 * address. TODO(hardware verify): confirm against the actual AN2606 STM32G4
 * table before first use -- written from memory, not cross-checked against
 * silicon or the PDF. Other Cortex-M4 STM32 lines sharing this base (F2/F4/
 * L4/G0) make it a reasonable bet, not a confirmed fact. */
#define SOFT_DFU_SYSMEM_BASE ((uint32_t)0x1FFF0000u)

typedef struct {
	volatile uint32_t magic;
	volatile uint32_t guard;
} soft_dfu_sig_t;

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

	NVIC_SystemReset(); /* never returns */

	for (;;) {
	}
}

void soft_dfu_check_and_jump(void)
{
	if (s_dfu_sig.magic != SOFT_DFU_MAGIC || s_dfu_sig.guard != SOFT_DFU_GUARD)
		return;

	/* Consume the signature before attempting the jump: if the jump below
	 * lands somewhere invalid (wrong SOFT_DFU_SYSMEM_BASE) and hard-faults,
	 * the NEXT reset -- triggered by pressing reset or a power cycle, since
	 * a hard fault with no debugger and no watchdog just hangs -- must boot
	 * normally instead of retrying the same bad jump forever. */
	s_dfu_sig.magic = 0u;
	s_dfu_sig.guard = 0u;

	/* Undo whatever clock tree main() hasn't even configured yet -- this
	 * runs before SystemClock_Config(), so RCC is close to its post-reset
	 * default already; DeInit is cheap insurance, not a real correction.
	 * Must run BEFORE disabling SysTick/IRQs below: HAL_RCC_DeInit() times
	 * out internal wait-for-flag loops via HAL_GetTick(), which needs a
	 * live SysTick + interrupts to ever advance. Doing this after would
	 * turn a bounded timeout into an infinite spin on any branch that
	 * actually waits. */
	HAL_RCC_DeInit();

	__disable_irq();

	SysTick->CTRL = 0u;
	SysTick->LOAD = 0u;
	SysTick->VAL  = 0u;

	uint32_t sysmem_msp   = *(volatile uint32_t *)(SOFT_DFU_SYSMEM_BASE + 0x00u);
	uint32_t sysmem_reset = *(volatile uint32_t *)(SOFT_DFU_SYSMEM_BASE + 0x04u);

	__set_MSP(sysmem_msp);
	((void (*)(void))sysmem_reset)();

	for (;;) {
	} /* unreachable */
}
