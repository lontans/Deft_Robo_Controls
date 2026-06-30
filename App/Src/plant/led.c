#include "plant/led.h"
#include "plant/plugins/sk9822.h"
#include "plant/plant_config.h"
#include "main.h"
#include <string.h>

#define LED_MODE_TEST 0u
#define LED_MODE_OFF  1u

static host_led_command_t g_cmd_live;
static host_led_command_t g_cmd_stage;
static volatile bool      g_cmd_pending;

// Pixel array for all LEDs
static sk9822_pixel_t g_pixels[LED_STRIP_MAX];

static uint16_t g_active_count;
static uint32_t g_last_ms;

#define LED_PERIOD_MS 33u // 30Hz SPI refresh

void led_init(void)
{
	// Initialise empty mem array for live command, staged commands
	memset(&g_cmd_live, 0, sizeof(g_cmd_live));
	memset(&g_cmd_stage, 0, sizeof(g_cmd_stage));
	g_cmd_pending = false;
	g_active_count = LED_STRIP_MAX;
	g_last_ms = 0;
	memset(g_pixels, 0, sizeof(g_pixels));
}

void led_command_mount(const host_command_image_t *cmd)
{
	if (cmd == NULL)
		return;
	__disable_irq();
	g_cmd_stage = cmd -> leds[0];
	g_cmd_pending = true;
	__enable_irq();
}

// Robust function to capture number of leds
static uint16_t led_resolve_count(const host_led_command_t *c)
{
	uint16_t n;

	if (c->led_count != 0u)
		n = (uint16_t)c->led_count;
	else
		n = (uint16_t)led_table[0].default_count;

	if (n==0u || n > LED_STRIP_MAX)
		n = LED_STRIP_MAX;

	return n;
}

static void led_apply_mode (uint8_t mode, uint8_t brightness, uint16_t n)
{
	uint16_t i;

	if (mode == LED_MODE_OFF){
		for (i = 0; i < n; i++) {
			g_pixels[i].r = 0;
			g_pixels[i].g = 0;
			g_pixels[i].b = 0;
		}
		return;
	}

	if (mode == LED_MODE_TEST){
		uint16_t lit = (uint16_t)((HAL_GetTick() / 50u) % n);
		for (i = 0; i < n; i++) {
			if (i == lit) {
				g_pixels[i].r = 255;
				g_pixels[i].g = 0;
				g_pixels[i].b = 0;
			}
			else {
				g_pixels[i].r = 0;
				g_pixels[i].g = 0;
				g_pixels[i].b = 0;
			}
		}
		return;
	}
	// Unknown mode defaults to off
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

	if ((now - g_last_ms) < LED_PERIOD_MS)
		return;
	g_last_ms = now;

	__disable_irq();

	if (g_cmd_pending) {
		g_cmd_live = g_cmd_stage; // copy staged command into live
		g_cmd_pending = false;
	}

	mode       = (uint8_t)(g_cmd_live.mode & 0x1Fu);
	brightness = (uint8_t)(g_cmd_live.master_brightness & 0x1Fu);
	__enable_irq();

	n = led_resolve_count(&g_cmd_live);
	g_active_count = n;

	led_apply_mode(mode, brightness, n);

	(void)sk9822_transmit(g_pixels, n, brightness);
}

void led_feedback_snapshot(host_led_feedback_t *dst)
{
	if (dst == NULL)
		return;
	__disable_irq();
	dst -> mode_readback       = g_cmd_live.mode;
	dst -> brightness_readback = g_cmd_live.master_brightness;
	dst -> driver_status       = 0u;
	__enable_irq();
}
