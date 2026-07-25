#include "plant/led.h"
#include "plant/plugins/sk9822.h"
#include "plant/plant_config.h"
#include "plant/spi3_role.h"
#include "host/pdb_link.h"
#include "host/uart4_mode.h"
#include "main.h"
#include <string.h>

/* mode 0 must mean OFF — host images zero-fill leds[], so TEST cannot be 0.
 * 5-bit enum only (host_led_command_t); see docs/rfc-led-factory-patterns.md. */
#define LED_MODE_OFF                 0u
#define LED_MODE_TEST                1u  /* single-pixel chase (snake) */
#define LED_MODE_FLASH               2u  /* full-strip ~2 Hz red blink */
#define LED_MODE_SOLID_GREEN         3u
#define LED_MODE_SOLID_YELLOW        4u
#define LED_MODE_SOLID_RED           5u
#define LED_MODE_BLINK_YELLOW_SLOW   6u  /* caution: 1 Hz 50% */
#define LED_MODE_BLINK_RED_FAST      7u  /* estop/fault: 5 Hz 50% */
/* Idle: cornflower blue #6495ED, 1 Hz 50% (500 on / 500 off). */
#define LED_MODE_IDLE_CORNFLOWER     8u

#define LED_IDLE_RGB_R               100u
#define LED_IDLE_RGB_G               149u
#define LED_IDLE_RGB_B               237u
#define LED_IDLE_HALF_MS             500u

static host_led_command_t g_cmd_live;
static host_led_command_t g_cmd_stage;
static volatile bool      g_cmd_pending;

static sk9822_pixel_t g_pixels[LED_STRIP_MAX];

static uint16_t g_active_count;
static uint32_t g_last_ms;
static uint8_t  g_last_flash_phase = 0xFFu; /* force first phase-edge TX */
static uint8_t  g_last_solid_key = 0xFFu;   /* mode|bri key for solid edge TX */
static bool     g_strip_known_off;
static uint8_t  g_mode_effective; /* last mode actually animated (PDB override aware) */

/* PDB / PDU traffic-light → LED (supplant USB LedDesire lap test when UART4=PDB).
 * Host desire stays staged; override is local to led_service.
 *   NORMAL + fresh     → IDLE_CORNFLOWER (500/500 blink)
 *   SOFT_KILL_REQ      → BLINK_YELLOW_SLOW (parking / caution)
 *   SOFT_KILL_READY    → SOLID_RED (contactors open)
 *   HARD_ESTOP / stale → BLINK_RED_FAST
 *   PDBF estop_sense=0 → BLINK_RED_FAST (PDU-reported wire; not local PB7 —
 *   this bench's PB7 net reads stuck-low so GPIO would mask all colors) */
#if UART4_MODE == UART4_MODE_PDB
static uint8_t led_mode_from_pdb(void)
{
	uint8_t kill = pdb_link_kill_state();
	uint8_t peer_estop = pdb_link_peer_estop_sense();

	if (kill == (uint8_t)PDB_KILL_HARD_ESTOP || peer_estop == 0u)
		return LED_MODE_BLINK_RED_FAST;
	if (kill == (uint8_t)PDB_KILL_SOFT_READY)
		return LED_MODE_SOLID_RED;
	if (kill == (uint8_t)PDB_KILL_SOFT_REQ)
		return LED_MODE_BLINK_YELLOW_SLOW;
	if (kill == (uint8_t)PDB_KILL_NORMAL && pdb_link_is_fresh())
		return LED_MODE_IDLE_CORNFLOWER;
	return LED_MODE_BLINK_RED_FAST;
}
#endif

/* 1 Hz 50%: phase 0 = on [0,500), phase 1 = off [500,1000). */
static uint8_t led_idle_phase(uint32_t now_ms)
{
	return (uint8_t)((now_ms / LED_IDLE_HALF_MS) & 1u);
}

static bool led_idle_on(uint8_t phase)
{
	return phase == 0u;
}

#define LED_PERIOD_MS 50u /* TEST chase refresh */

/* Factory / traffic-light animator table (RGB + blink half-period). */
typedef struct {
	uint8_t  r;
	uint8_t  g;
	uint8_t  b;
	uint16_t half_period_ms; /* 0 ⇒ solid (100% duty) */
} led_pattern_t;

static const led_pattern_t g_factory_patterns[] = {
	/* [3] SOLID_GREEN */       { 0,   255, 0,   0u },
	/* [4] SOLID_YELLOW */      { 255, 180, 0,   0u },
	/* [5] SOLID_RED */         { 255, 0,   0,   0u },
	/* [6] BLINK_YELLOW_SLOW */ { 255, 180, 0, 500u },
	/* [7] BLINK_RED_FAST */    { 255, 0,   0, 100u },
};

static const led_pattern_t *led_factory_pattern(uint8_t mode)
{
	if (mode < LED_MODE_SOLID_GREEN || mode > LED_MODE_BLINK_RED_FAST)
		return NULL;
	return &g_factory_patterns[mode - LED_MODE_SOLID_GREEN];
}


void led_init(void)
{
	memset(&g_cmd_live, 0, sizeof(g_cmd_live));
	memset(&g_cmd_stage, 0, sizeof(g_cmd_stage));
	g_cmd_pending = false;
	g_active_count = LED_STRIP_MAX;
	g_last_ms = 0;
	g_last_flash_phase = 0xFFu;
	g_last_solid_key = 0xFFu;
	g_strip_known_off = false;
	g_mode_effective = LED_MODE_OFF;
	memset(g_pixels, 0, sizeof(g_pixels));
}

void led_command_mount(const host_command_image_t *cmd)
{
	if (cmd == NULL)
		return;
	__disable_irq();
	g_cmd_stage = cmd->leds[0];
	g_cmd_pending = true;
	__enable_irq();
}

void led_sim_kick_test(uint8_t brightness_0_31)
{
	host_led_command_t c;

	memset(&c, 0, sizeof(c));
	c.mode = LED_MODE_TEST;
	c.master_brightness = (uint16_t)(brightness_0_31 & 0x1Fu);
	c.led_count = 0u;

	__disable_irq();
	g_cmd_stage = c;
	g_cmd_pending = true;
	__enable_irq();
}

void led_force_all_off(void)
{
	uint16_t i;
	spi3_role_t saved = spi3_role_get();

	/* Always clock the full configured chain — a short n leaves the tail latched. */
	__disable_irq();
	memset(&g_cmd_live, 0, sizeof(g_cmd_live));
	memset(&g_cmd_stage, 0, sizeof(g_cmd_stage));
	g_cmd_live.mode = LED_MODE_OFF;
	g_cmd_stage.mode = LED_MODE_OFF;
	g_cmd_pending = false;
	g_active_count = LED_STRIP_MAX;
	g_last_flash_phase = 0xFFu;
	g_last_solid_key = 0xFFu;
	g_strip_known_off = true;
	g_mode_effective = LED_MODE_OFF;
	__enable_irq();

	for (i = 0; i < LED_STRIP_MAX; i++) {
		g_pixels[i].r = 0;
		g_pixels[i].g = 0;
		g_pixels[i].b = 0;
	}

	spi3_role_set(SPI3_ROLE_LED);
	(void)sk9822_transmit_blocking(g_pixels, LED_STRIP_MAX, 0u);
	(void)sk9822_transmit_blocking(g_pixels, LED_STRIP_MAX, 0u);
	spi3_role_set(saved);
}

void led_blank_transmit(void)
{
	led_force_all_off();
}

static uint16_t led_resolve_count(const host_led_command_t *c)
{
	uint16_t n;

	if (c->led_count != 0u)
		n = (uint16_t)c->led_count;
	else
		n = (uint16_t)led_table[0].default_count;

	if (n == 0u || n > LED_STRIP_MAX)
		n = LED_STRIP_MAX;

	return n;
}

static void led_fill_rgb(uint16_t n, uint8_t r, uint8_t g, uint8_t b)
{
	uint16_t i;

	for (i = 0; i < n; i++) {
		g_pixels[i].r = r;
		g_pixels[i].g = g;
		g_pixels[i].b = b;
	}
}

static void led_apply_mode(uint8_t mode, uint8_t brightness, uint16_t n)
{
	uint16_t i;
	const led_pattern_t *pat;

	(void)brightness;

	if (mode == LED_MODE_OFF) {
		led_fill_rgb(n, 0, 0, 0);
		return;
	}

	if (mode == LED_MODE_TEST) {
		uint16_t lit = (uint16_t)((HAL_GetTick() / 50u) % n);

		for (i = 0; i < n; i++) {
			if (i == lit) {
				g_pixels[i].r = 255;
				g_pixels[i].g = 0;
				g_pixels[i].b = 0;
			} else {
				g_pixels[i].r = 0;
				g_pixels[i].g = 0;
				g_pixels[i].b = 0;
			}
		}
		return;
	}

	if (mode == LED_MODE_FLASH) {
		/* ~2 Hz full-strip red flash (on 250 ms / off 250 ms). */
		uint8_t on = ((HAL_GetTick() / 250u) & 1u) != 0u;
		uint8_t v = on ? 255u : 0u;

		led_fill_rgb(n, v, 0, 0);
		return;
	}

	if (mode == LED_MODE_IDLE_CORNFLOWER) {
		if (led_idle_on(led_idle_phase(HAL_GetTick())))
			led_fill_rgb(n, LED_IDLE_RGB_R, LED_IDLE_RGB_G, LED_IDLE_RGB_B);
		else
			led_fill_rgb(n, 0, 0, 0);
		return;
	}

	pat = led_factory_pattern(mode);
	if (pat != NULL) {
		uint8_t on = 1u;

		if (pat->half_period_ms != 0u)
			on = ((HAL_GetTick() / (uint32_t)pat->half_period_ms) & 1u) != 0u;
		if (on)
			led_fill_rgb(n, pat->r, pat->g, pat->b);
		else
			led_fill_rgb(n, 0, 0, 0);
		return;
	}

	led_fill_rgb(n, 0, 0, 0);
}

void led_service(void)
{
	uint32_t now = HAL_GetTick();
	uint8_t mode;
	uint8_t brightness;
	uint16_t n;
	uint8_t flash_phase;
	uint8_t solid_key;
	const led_pattern_t *pat;

	if (spi3_role_get() != SPI3_ROLE_LED)
		return;

	/* Prior IT TX still shifting — defer; try again next lap. */
	if (sk9822_tx_busy())
		return;

	__disable_irq();
	if (g_cmd_pending) {
		g_cmd_live = g_cmd_stage;
		g_cmd_pending = false;
	}
	mode = (uint8_t)(g_cmd_live.mode & 0x1Fu);
	brightness = (uint8_t)(g_cmd_live.master_brightness & 0x1Fu);
	__enable_irq();

#if UART4_MODE == UART4_MODE_PDB
	/* Host non-OFF LedDesire wins (bench override). PDB traffic-light only
	 * when host leaves mode OFF — otherwise stale/HARD UART paints permanent red. */
	if (mode == LED_MODE_OFF)
		mode = led_mode_from_pdb();
	if (brightness == 0u)
		brightness = 12u;
#endif
	g_mode_effective = mode;

	n = led_resolve_count(&g_cmd_live);
	g_active_count = n;

	/* OFF: one full-chain blank then idle (no 20 Hz SPI). */
	if (mode == LED_MODE_OFF) {
		if (g_strip_known_off)
			return;
		led_apply_mode(mode, brightness, n);
		if (!sk9822_transmit(g_pixels, n, brightness))
			return;
		g_strip_known_off = true;
		g_last_flash_phase = 0xFFu;
		g_last_solid_key = 0xFFu;
		g_last_ms = now;
		return;
	}

	g_strip_known_off = false;

	/* FLASH + factory blinks + idle 500/500: TX only on phase edge.
	 * Leaving this family invalidates the solid-family cache below --
	 * otherwise returning to a previously-shown solid color after a blink
	 * excursion would wrongly "skip transmit" and leave stale blink pixel
	 * data latched on the strip. */
	if (mode == LED_MODE_FLASH) {
		flash_phase = (uint8_t)((now / 250u) & 1u);
		if (flash_phase == g_last_flash_phase)
			return;
		led_apply_mode(mode, brightness, n);
		if (!sk9822_transmit(g_pixels, n, brightness))
			return;
		g_last_flash_phase = flash_phase;
		g_last_solid_key = 0xFFu;
		g_last_ms = now;
		return;
	}

	if (mode == LED_MODE_IDLE_CORNFLOWER) {
		flash_phase = led_idle_phase(now);
		if (flash_phase == g_last_flash_phase)
			return;
		led_apply_mode(mode, brightness, n);
		if (!sk9822_transmit(g_pixels, n, brightness))
			return;
		g_last_flash_phase = flash_phase;
		g_last_solid_key = 0xFFu;
		g_last_ms = now;
		return;
	}

	pat = led_factory_pattern(mode);
	if (pat != NULL && pat->half_period_ms != 0u) {
		flash_phase = (uint8_t)((now / (uint32_t)pat->half_period_ms) & 1u);
		if (flash_phase == g_last_flash_phase)
			return;
		led_apply_mode(mode, brightness, n);
		if (!sk9822_transmit(g_pixels, n, brightness))
			return;
		g_last_flash_phase = flash_phase;
		g_last_solid_key = 0xFFu;
		g_last_ms = now;
		return;
	}

	/* Solids: TX once per (mode, brightness) edge — no periodic SPI. Full
	 * 5-bit brightness packed into the key (not truncated to 3 bits) so e.g.
	 * brightness 8 -> 24 (same low 3 bits) still retransmits; independent of
	 * g_last_flash_phase (was previously dual-purposed as a "solids latched"
	 * sentinel, which corrupted the blink-family's own edge detection and
	 * vice versa across a mode-family switch -- see FLASH/blink comment
	 * above for the failure this caused in the other direction). */
	if (pat != NULL && pat->half_period_ms == 0u) {
		solid_key = (uint8_t)((mode & 0x07u) | ((brightness & 0x1Fu) << 3));
		if (solid_key == g_last_solid_key)
			return;
		led_apply_mode(mode, brightness, n);
		if (!sk9822_transmit(g_pixels, n, brightness))
			return;
		g_last_solid_key = solid_key;
		g_last_flash_phase = 0xFFu;
		g_last_ms = now;
		return;
	}

	/* TEST chase (and unknown modes blanked inside apply): periodic refresh.
	 * Also invalidates both cached-edge families above, so switching back to
	 * a blink or solid mode after TEST always retransmits at least once
	 * instead of trusting stale pre-TEST state. */
	if ((now - g_last_ms) < LED_PERIOD_MS)
		return;
	led_apply_mode(mode, brightness, n);
	if (!sk9822_transmit(g_pixels, n, brightness))
		return;
	g_last_ms = now;
	g_last_flash_phase = 0xFFu;
	g_last_solid_key = 0xFFu;
}

void led_feedback_snapshot(host_led_feedback_t *dst)
{
	if (dst == NULL)
		return;
	__disable_irq();
	/* Effective/animated mode (PDB override when UART4=PDB), not staged desire. */
	dst->mode_readback = g_mode_effective;
	dst->brightness_readback = g_cmd_live.master_brightness;
	dst->driver_status = 0u;
	__enable_irq();
}
