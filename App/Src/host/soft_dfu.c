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

static void soft_dfu_flash_wait_bsy(void)
{
	while ((FLASH->SR & FLASH_SR_BSY) != 0u) {
	}
}

/*
 * Program nBOOT0 via option bytes and launch (resets the MCU).
 *
 * This board has nSWBOOT0=0 (BOOT0 taken from the nBOOT0 option bit). Soft
 * MEMRMP jumps into system memory leave CDC dead without enumerating
 * 0483:DF11 on this hardware; forcing system-memory boot through nBOOT0=0
 * does enumerate DFU (verified on the bench). Leave restores nBOOT0=1.
 *
 * Register-level so the Leave trampoline can call this before HAL exists.
 * Does not return on success (OBL_LAUNCH resets).
 */
static void soft_dfu_ob_launch_nboot0(uint32_t nboot0_set)
{
	uint32_t optr;

	soft_dfu_flash_wait_bsy();

	if ((FLASH->CR & FLASH_CR_LOCK) != 0u) {
		FLASH->KEYR = FLASH_KEY1;
		FLASH->KEYR = FLASH_KEY2;
	}
	if ((FLASH->CR & FLASH_CR_OPTLOCK) != 0u) {
		FLASH->OPTKEYR = FLASH_OPTKEY1;
		FLASH->OPTKEYR = FLASH_OPTKEY2;
	}

	optr = FLASH->OPTR;
	/* Keep boot selection on the option bit (not the BOOT0 pin / PB8). */
	optr &= ~FLASH_OPTR_nSWBOOT0;
	if (nboot0_set != 0u) {
		optr |= FLASH_OPTR_nBOOT0;
	} else {
		optr &= ~FLASH_OPTR_nBOOT0;
	}

	FLASH->OPTR = optr;
	FLASH->CR |= FLASH_CR_OPTSTRT;
	soft_dfu_flash_wait_bsy();

	/* Clears OPTSTRT and forces an option-byte reload + system reset. */
	FLASH->CR |= FLASH_CR_OBL_LAUNCH;

	for (;;) {
	}
}

static void soft_dfu_leave_reset(void)
{
	/* AN3156 Leave jumps here without a chip reset. Restore flash boot
	 * (nBOOT0=1) then OBL_LAUNCH resets into the app with clean USB. */
	soft_dfu_ob_launch_nboot0(1u);
}

__attribute__((section(".soft_dfu_leave_vt"), used))
static const soft_dfu_leave_vt_t soft_dfu_leave_vt = {
	.stack_top = (uint32_t)&_estack,
	.reset = soft_dfu_leave_reset,
};

/* Placed in the dedicated .dfu_sig section (see STM32G474RETX_FLASH.ld,
 * right after .bss) so it survives a warm reset: that section sits outside
 * the _sbss.._ebss range Reset_Handler's LoopFillZerobss zeroes, and
 * outside .data's flash-copy range too. Legacy soft-jump fallback only. */
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

static void soft_dfu_usb_force_disconnect(void)
{
	uint32_t i;

	/* Drop D+ pull-up so the host sees a clean CDC detach before reset /
	 * ROM DFU re-enum. Register-level only — this runs with or without HAL. */
	SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_USBEN);
	(void)RCC->APB1ENR1;
	CLEAR_BIT(USB->BCDR, USB_BCDR_DPPU);
	USB->CNTR = (uint16_t)(USB_CNTR_FRES | USB_CNTR_PDWN);
	for (i = 0u; i < 200000u; i++) {
		__NOP();
	}
}

static void soft_dfu_usb_hw_reset(void)
{
	SET_BIT(RCC->APB1ENR1, RCC_APB1ENR1_USBEN);
	(void)RCC->APB1ENR1;
	SET_BIT(RCC->APB1RSTR1, RCC_APB1RSTR1_USBRST);
	CLEAR_BIT(RCC->APB1RSTR1, RCC_APB1RSTR1_USBRST);
	CLEAR_BIT(USB->BCDR, USB_BCDR_DPPU);
	USB->CNTR = (uint16_t)(USB_CNTR_FRES | USB_CNTR_PDWN);
	CLEAR_BIT(RCC->APB1ENR1, RCC_APB1ENR1_USBEN);
}

void soft_dfu_on_command(const host_command_image_t *cmd)
{
	(void)cmd;

	soft_dfu_usb_force_disconnect();

	/* Primary path: option-byte boot into system memory (USB DFU). */
	soft_dfu_ob_launch_nboot0(0u);

	/* Unreachable unless OB programming refused to launch — legacy soft jump. */
	s_dfu_sig.magic = SOFT_DFU_MAGIC;
	s_dfu_sig.guard = SOFT_DFU_GUARD;
	__DSB();
	__ISB();
	NVIC_SystemReset();

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
	soft_dfu_usb_hw_reset();

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
