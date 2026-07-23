#include "plant/led.h"
#include "plant/plugins/sk9822.h"
#include "plant/plant_config.h"
#include "plant/spi3_role.h"
#include "main.h"
#include <string.h>

/* mode 0 must mean OFF — host images zero-fill leds[], so TEST cannot be 0. */
#define LED_MODE_OFF   0u
#define LED_MODE_TEST  1u  /* single-pixel chase (snake) */
#define LED_MODE_FLASH 2u  /* full-strip blink */

static host_led_command_t g_cmd_live;
static host_led_command_t g_cmd_stage;
static volatile bool      g_cmd_pending;

static sk9822_pixel_t g_pixels[LED_STRIP_MAX];

static uint16_t g_active_count;
static uint32_t g_last_ms;
static uint8_t  g_last_flash_phase = 0xFFu; /* force first FLASH TX */
static bool     g_strip_known_off;

#define LED_PERIOD_MS 50u /* TEST chase refresh; FLASH only TX on edge */


void led_init(void)
{
	memset(&g_cmd_live, 0, sizeof(g_cmd_live));
	memset(&g_cmd_stage, 0, sizeof(g_cmd_stage));
	g_cmd_pending = false;
	g_active_count = LED_STRIP_MAX;
	g_last_ms = 0;
	g_last_flash_phase = 0xFFu;
	g_strip_known_off = false;
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
	g_strip_known_off = true;
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

static void led_apply_mode(uint8_t mode, uint8_t brightness, uint16_t n)
{
	uint16_t i;

	(void)brightness;

	if (mode == LED_MODE_OFF) {
		for (i = 0; i < n; i++) {
			g_pixels[i].r = 0;
			g_pixels[i].g = 0;
			g_pixels[i].b = 0;
		}
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

		for (i = 0; i < n; i++) {
			g_pixels[i].r = v;
			g_pixels[i].g = 0;
			g_pixels[i].b = 0;
		}
		return;
	}

	for (i = 0; i < n; i++) {
		g_pixels[i].r = 0;
		g_pixels[i].g = 0;
		g_pixels[i].b = 0;
	}
}

void led_service(void)
{
	uint32_t now = HAL_GetTick();
	uint8_t mode;
	uint8_t brightness;
	uint16_t n;
	uint8_t flash_phase;

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
		g_last_ms = now;
		return;
	}

	g_strip_known_off = false;

	if (mode == LED_MODE_FLASH) {
		flash_phase = (uint8_t)((now / 250u) & 1u);
		if (flash_phase == g_last_flash_phase)
			return;
		led_apply_mode(mode, brightness, n);
		if (!sk9822_transmit(g_pixels, n, brightness))
			return;
		g_last_flash_phase = flash_phase;
		g_last_ms = now;
		return;
	}

	/* TEST chase (and other modes): periodic refresh. */
	if ((now - g_last_ms) < LED_PERIOD_MS)
		return;
	led_apply_mode(mode, brightness, n);
	if (!sk9822_transmit(g_pixels, n, brightness))
		return;
	g_last_ms = now;
}

void led_feedback_snapshot(host_led_feedback_t *dst)
{
	if (dst == NULL)
		return;
	__disable_irq();
	dst->mode_readback = g_cmd_live.mode;
	dst->brightness_readback = g_cmd_live.master_brightness;
	dst->driver_status = 0u;
	__enable_irq();
}
