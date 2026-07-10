#include "plant/plant_config_nvm.h"
#include "plant/actuator.h"
#include "plant/plugins/damiao.h"

#include "stm32g4xx_hal.h"

#include <stddef.h>
#include <string.h>

#define PLANT_CFG_NVM_MAGIC   0x50434647u /* 'PCFG' */
#define PLANT_CFG_NVM_VERSION 1u

#define PLANT_CFG_RAMFUNC __attribute__((section(".RamFunc")))

/* Last 2 KiB page of 512 KiB flash (STM32G474, page size 0x800). */
#define PLANT_CFG_FLASH_PAGE_SIZE  0x800u
#define PLANT_CFG_FLASH_BASE       0x08000000u
#define PLANT_CFG_FLASH_SIZE       0x00080000u
#define PLANT_CFG_NVM_ADDR \
	(PLANT_CFG_FLASH_BASE + PLANT_CFG_FLASH_SIZE - PLANT_CFG_FLASH_PAGE_SIZE)

typedef struct __attribute__((packed)) {
	uint8_t schematic_bus;
	uint8_t protocol;
	uint8_t motor_id;
	uint8_t flags; /* bit0 = enabled */
} plant_cfg_nvm_slot_t;

typedef struct __attribute__((packed)) {
	uint32_t magic;
	uint16_t version;
	uint16_t length;
	uint32_t crc32;
	uint8_t  slot_count;
	uint8_t  reserved[3];
	plant_cfg_nvm_slot_t slots[ACTUATOR_COUNT];
} plant_cfg_nvm_image_t;

static plant_cfg_nvm_slot_t s_resp_slots[ACTUATOR_COUNT];
static uint8_t              s_last_status = PLANT_CFG_STATUS_OK;
static uint8_t              s_last_op;
static bool                 s_resp_pending;

static bool nvm_image_valid(const plant_cfg_nvm_image_t *img);

#define PLANT_CFG_NVM_PAGE \
	((uint32_t)((PLANT_CFG_NVM_ADDR - PLANT_CFG_FLASH_BASE) / PLANT_CFG_FLASH_PAGE_SIZE))

static PLANT_CFG_RAMFUNC void ramfunc_flash_cache_invalidate(void)
{
	if ((FLASH->ACR & FLASH_ACR_ICEN) != 0u) {
		CLEAR_BIT(FLASH->ACR, FLASH_ACR_ICEN);
		SET_BIT(FLASH->ACR, FLASH_ACR_ICRST);
		CLEAR_BIT(FLASH->ACR, FLASH_ACR_ICRST);
		SET_BIT(FLASH->ACR, FLASH_ACR_ICEN);
	}
}

static PLANT_CFG_RAMFUNC bool ramfunc_flash_wait_ready(void)
{
	uint32_t spin = 8000000u;

	while ((FLASH->SR & FLASH_SR_BSY) != 0u) {
		if (--spin == 0u)
			return false;
	}

	if ((FLASH->SR & FLASH_FLAG_SR_ERRORS) != 0u) {
		FLASH->SR = FLASH_FLAG_SR_ERRORS;
		return false;
	}

	if ((FLASH->SR & FLASH_SR_EOP) != 0u)
		FLASH->SR = FLASH_SR_EOP;

	return true;
}

static PLANT_CFG_RAMFUNC void ramfunc_flash_unlock(void)
{
	if ((FLASH->CR & FLASH_CR_LOCK) != 0u) {
		FLASH->KEYR = 0x45670123u;
		FLASH->KEYR = 0xCDEF89ABu;
	}
}

static PLANT_CFG_RAMFUNC void ramfunc_flash_lock(void)
{
	SET_BIT(FLASH->CR, FLASH_CR_LOCK);
}

static PLANT_CFG_RAMFUNC bool ramfunc_flash_erase_page(uint32_t page)
{
	if (!ramfunc_flash_wait_ready())
		return false;

	ramfunc_flash_cache_invalidate();
	MODIFY_REG(FLASH->CR, FLASH_CR_PNB, (page & 0xFFu) << FLASH_CR_PNB_Pos);
	FLASH->CR |= FLASH_CR_PER;
	FLASH->CR |= FLASH_CR_STRT;

	if (!ramfunc_flash_wait_ready())
		return false;

	FLASH->CR &= ~(FLASH_CR_PER | FLASH_CR_PNB);
	return true;
}

static PLANT_CFG_RAMFUNC bool ramfunc_flash_program_dword(uint32_t addr, uint64_t data)
{
	if (!ramfunc_flash_wait_ready())
		return false;

	FLASH->CR |= FLASH_CR_PG;
	*(volatile uint32_t *)addr = (uint32_t)data;
	__ISB();
	*(volatile uint32_t *)(addr + 4u) = (uint32_t)(data >> 32);

	if (!ramfunc_flash_wait_ready())
		return false;

	FLASH->CR &= ~FLASH_CR_PG;
	return true;
}

static PLANT_CFG_RAMFUNC uint64_t ramfunc_load_u64(const uint8_t *p, uint32_t n)
{
	uint64_t dw = 0xFFFFFFFFFFFFFFFFULL;
	uint32_t i;

	if (n > 8u)
		n = 8u;
	for (i = 0; i < n; i++)
		dw = (dw & ~((uint64_t)0xFFu << (8u * i))) | ((uint64_t)p[i] << (8u * i));
	return dw;
}

static PLANT_CFG_RAMFUNC bool flash_image_matches(const plant_cfg_nvm_image_t *expected)
{
	const uint8_t *flash = (const uint8_t *)PLANT_CFG_NVM_ADDR;
	const uint8_t *src = (const uint8_t *)expected;
	uint32_t i;

	for (i = 0; i < sizeof(plant_cfg_nvm_image_t); i++) {
		if (flash[i] != src[i])
			return false;
	}
	return true;
}

static PLANT_CFG_RAMFUNC bool plant_config_nvm_save_image(const plant_cfg_nvm_image_t *img)
{
	uint32_t primask;
	uint32_t addr;
	uint32_t remaining;
	const uint8_t *bytes;

	if (img == NULL)
		return false;

	primask = __get_PRIMASK();
	__disable_irq();
	ramfunc_flash_unlock();
	FLASH->SR = FLASH_FLAG_ALL_ERRORS;

	if (!ramfunc_flash_erase_page(PLANT_CFG_NVM_PAGE))
		goto fail;

	bytes = (const uint8_t *)img;
	addr = PLANT_CFG_NVM_ADDR;
	remaining = (uint32_t)sizeof(*img);

	while (remaining >= 8u) {
		uint64_t dw = ramfunc_load_u64(bytes, 8u);

		if (!ramfunc_flash_program_dword(addr, dw))
			goto fail;
		bytes += 8;
		addr += 8;
		remaining -= 8;
	}
	if (remaining > 0u) {
		uint64_t dw = ramfunc_load_u64(bytes, remaining);

		if (!ramfunc_flash_program_dword(addr, dw))
			goto fail;
	}

	ramfunc_flash_cache_invalidate();
	ramfunc_flash_lock();
	__set_PRIMASK(primask);

	__DSB();
	__ISB();
	return flash_image_matches(img);

fail:
	ramfunc_flash_lock();
	__set_PRIMASK(primask);
	return false;
}

static uint32_t crc32_ieee(const uint8_t *data, uint32_t len)
{
	uint32_t crc = 0xFFFFFFFFu;

	for (uint32_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (uint8_t b = 0; b < 8u; b++) {
			if (crc & 1u)
				crc = (crc >> 1) ^ 0xEDB88320u;
			else
				crc >>= 1;
		}
	}
	return crc ^ 0xFFFFFFFFu;
}

static can_bus_id_t schematic_to_can_bus(uint8_t schematic_bus)
{
	if (schematic_bus < 1u || schematic_bus > 6u)
		return CAN_BUS_CH1;
	return (can_bus_id_t)(schematic_bus - 1u);
}

static uint8_t can_bus_to_schematic(can_bus_id_t bus)
{
	if (bus >= CAN_BUS_COUNT)
		return 1u;
	return (uint8_t)((uint8_t)bus + 1u);
}

static void table_to_nvm_slots(plant_cfg_nvm_slot_t *dst)
{
	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
		dst[i].schematic_bus = can_bus_to_schematic(actuator_table[i].bus);
		dst[i].protocol = (uint8_t)actuator_table[i].protocol;
		dst[i].motor_id = (uint8_t)(actuator_table[i].motor_id & 0xFFu);
		dst[i].flags = actuator_table[i].enabled ? 1u : 0u;
	}
}

static void nvm_slots_to_table(const plant_cfg_nvm_slot_t *src)
{
	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
		actuator_table[i].bus = schematic_to_can_bus(src[i].schematic_bus);
		actuator_table[i].protocol = (protocol_t)src[i].protocol;
		if (actuator_table[i].protocol >= PROTO_COUNT)
			actuator_table[i].protocol = PROTO_NONE;
		actuator_table[i].motor_id = src[i].motor_id;
		actuator_table[i].master_id = (src[i].protocol == (uint8_t)PROTO_DAMIAO) ?
		                              DM_MASTER_ID_AUTO : 0u;
		actuator_table[i].enabled = (src[i].flags & 1u) != 0u;
	}
}

static bool nvm_image_valid(const plant_cfg_nvm_image_t *img)
{
	if (img == NULL)
		return false;
	if (img->magic != PLANT_CFG_NVM_MAGIC)
		return false;
	if (img->version != PLANT_CFG_NVM_VERSION)
		return false;
	if (img->length != sizeof(plant_cfg_nvm_image_t))
		return false;
	if (img->slot_count != ACTUATOR_COUNT)
		return false;

	uint32_t expect = crc32_ieee((const uint8_t *)img + offsetof(plant_cfg_nvm_image_t, slot_count),
	                             sizeof(plant_cfg_nvm_image_t) -
	                             offsetof(plant_cfg_nvm_image_t, slot_count));
	return img->crc32 == expect;
}

static void build_nvm_image(plant_cfg_nvm_image_t *img)
{
	memset(img, 0, sizeof(*img));
	img->magic = PLANT_CFG_NVM_MAGIC;
	img->version = PLANT_CFG_NVM_VERSION;
	img->length = (uint16_t)sizeof(plant_cfg_nvm_image_t);
	img->slot_count = ACTUATOR_COUNT;
	table_to_nvm_slots(img->slots);
	img->crc32 = crc32_ieee((const uint8_t *)img + offsetof(plant_cfg_nvm_image_t, slot_count),
	                        sizeof(plant_cfg_nvm_image_t) -
	                        offsetof(plant_cfg_nvm_image_t, slot_count));
}

void plant_config_load_factory_defaults(void)
{
	/* YAM / Damiao CH1 daisy defaults — override via CFG SET after discover. */
	static const uint8_t k_ids[ACTUATOR_COUNT] = {
		0x01u, 0x02u, 0x03u, 0x04u, 0x05u, 0x06u, 0x07u,
	};

	for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
		actuator_table[i] = (actuator_config_t){
			.bus = CAN_BUS_CH1,
			.protocol = PROTO_DAMIAO,
			.motor_id = k_ids[i],
			.master_id = DM_MASTER_ID_AUTO,
			.enabled = true,
		};
	}
}

bool plant_config_nvm_load(void)
{
	const plant_cfg_nvm_image_t *flash_img =
		(const plant_cfg_nvm_image_t *)PLANT_CFG_NVM_ADDR;

	if (!nvm_image_valid(flash_img))
		return false;

	nvm_slots_to_table(flash_img->slots);
	return true;
}

bool plant_config_nvm_save(void)
{
	plant_cfg_nvm_image_t img;

	build_nvm_image(&img);
	return plant_config_nvm_save_image(&img);
}

static void stage_response(uint8_t op, uint8_t status)
{
	table_to_nvm_slots(s_resp_slots);
	s_last_op = op;
	s_last_status = status;
	s_resp_pending = true;
}

bool plant_config_is_command(const host_command_image_t *cmd)
{
	if (cmd == NULL)
		return false;
	return cmd->pdu.data[0] == (uint8_t)PLANT_CFG_PDU_TAG0 &&
	       cmd->pdu.data[1] == (uint8_t)PLANT_CFG_PDU_TAG1 &&
	       cmd->pdu.data[2] == (uint8_t)PLANT_CFG_PDU_TAG2;
}

void plant_config_on_command(const host_command_image_t *cmd)
{
	const uint8_t *p;
	uint8_t op;
	uint8_t slot;

	if (!plant_config_is_command(cmd))
		return;

	p = cmd->pdu.data;
	op = p[3];
	slot = p[4];

	switch (op) {
	case PLANT_CFG_OP_GET:
		stage_response(op, PLANT_CFG_STATUS_OK);
		return;

	case PLANT_CFG_OP_SET:
		if (slot >= ACTUATOR_COUNT) {
			stage_response(op, PLANT_CFG_STATUS_BAD_ARG);
			return;
		}
		actuator_table[slot].bus = schematic_to_can_bus(p[8]);
		actuator_table[slot].protocol = (protocol_t)p[9];
		if (actuator_table[slot].protocol >= PROTO_COUNT)
			actuator_table[slot].protocol = PROTO_NONE;
		actuator_table[slot].motor_id = p[10];
		if (p[9] == (uint8_t)PROTO_DAMIAO) {
			actuator_table[slot].master_id =
				(p[11] == 0xFFu) ? DM_MASTER_ID_AUTO : (uint32_t)p[11];
		} else {
			actuator_table[slot].master_id = 0u;
		}
		actuator_table[slot].enabled = (p[12] & 1u) != 0u;
		stage_response(op, PLANT_CFG_STATUS_OK);
		return;

	case PLANT_CFG_OP_SAVE:
		stage_response(op, plant_config_nvm_save() ? PLANT_CFG_STATUS_OK :
		                                           PLANT_CFG_STATUS_FLASH_ERR);
		return;

	case PLANT_CFG_OP_LOAD:
		stage_response(op, plant_config_nvm_load() ? PLANT_CFG_STATUS_OK :
		                                           PLANT_CFG_STATUS_BAD_CRC);
		return;

	case PLANT_CFG_OP_DEFAULTS:
		plant_config_load_factory_defaults();
		stage_response(op, PLANT_CFG_STATUS_OK);
		return;

	default:
		stage_response(op, PLANT_CFG_STATUS_BAD_ARG);
		return;
	}
}

void plant_config_feedback_fill(host_pdu_feedback_t *pdu)
{
	uint8_t i;

	if (pdu == NULL || !s_resp_pending)
		return;

	memset(pdu->data, 0, sizeof(pdu->data));
	pdu->data[0] = (uint8_t)PLANT_CFG_PDU_RESP_TAG0;
	pdu->data[1] = (uint8_t)PLANT_CFG_PDU_RESP_TAG1;
	pdu->data[2] = (uint8_t)PLANT_CFG_PDU_RESP_TAG2;
	pdu->data[3] = (uint8_t)(s_last_op | 0x80u);
	pdu->data[4] = s_last_status;
	/* Compact 3 B/slot so ACTUATOR_COUNT=7 fits in 32 B PDU:
	 * [bus][protocol|(enabled<<7)][motor_id]  (header uses bytes 0..5). */
	pdu->data[5] = ACTUATOR_COUNT;

	for (i = 0; i < ACTUATOR_COUNT; i++) {
		uint8_t off = (uint8_t)(6u + i * 3u);

		if ((off + 3u) > HOST_PDU_PAYLOAD_BYTES)
			break;
		pdu->data[off + 0] = s_resp_slots[i].schematic_bus;
		pdu->data[off + 1] = (uint8_t)((s_resp_slots[i].protocol & 0x7Fu) |
		                               ((s_resp_slots[i].flags & 1u) ? 0x80u : 0u));
		pdu->data[off + 2] = s_resp_slots[i].motor_id;
	}

	s_resp_pending = false;
}

/* CFG GET packing: 6 B header + 3 B/slot must fit HOST_PDU_PAYLOAD_BYTES. */
_Static_assert(6u + ACTUATOR_COUNT * 3u <= HOST_PDU_PAYLOAD_BYTES,
               "CFG GET slot packing exceeds PDU payload");

