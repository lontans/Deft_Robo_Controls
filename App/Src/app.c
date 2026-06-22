#include "app.h"
#include "plant/plant_config.h"
#include "plant/actuator.h"
#include "plant/control_loop.h"
#include "host/host_link.h"
#include "plant/can/can_router.h"
#include "plant/can/can_frame.h"
#include "plant/plugin_schema/plugin_table.h"

#define APP_CAN_HW_LOOPBACK_TEST 0

volatile uint8_t g_can_loopback_ok = 0;
volatile uint8_t robstride_pack_ok = 0;
volatile uint8_t robstride_parse_ok = 0;

static void can_loopback_self_test(void)
{
	#if APP_CAN_HW_LOOPBACK_TEST
		can_frame_t tx = {0};
		can_frame_t rx = {0};

		tx.id = 0x7F;
		tx.id_type = CAN_ID_EXT;
		tx.dlc = 2;
		tx.data[0] = 0xDE;
		tx.data[1] = 0xAD;

		if (can_tx_enqueue(CAN_BUS_CH1, &tx) != CAN_OK)
			return;

		can_router_poll();

		if (can_rx_pop(CAN_BUS_CH1, &rx) != CAN_OK)
			return;

		if (rx.id == tx.id && rx.data[0] == 0xDE && rx.data[1] == 0xAD)
			g_can_loopback_ok = 1;
	#else
		g_can_loopback_ok = 2;
	#endif
}

static void robstride_pack_test(void)
{
	can_frame_t f = {0};
	if (plugin_pack_tx(&actuator_table[0], &actuator_desire_live[0], &f) != PLUGIN_OK)
		return;

	if (f.id == 0x017FFF7F && f.id_type == CAN_ID_EXT && f.dlc == 8 && f.data[6] == 0x33)
		robstride_pack_ok = 1;
}

static void robstride_parse_test(void)
{
	can_frame_t f = {0};

	f.id_type = CAN_ID_EXT;
	f.id      = 0x02007FFD;
	f.dlc     = 8;
	f.data[0] = 0x7F;
	f.data[1] = 0xFF;
	f.data[2] = 0x7F;
	f.data[3] = 0xFF;
	f.data[4] = 0x7F;
	f.data[5] = 0xFF;
	f.data[6] = 0x00;
	f.data[7] = 0xFA;

	if (plugin_parse_rx(&actuator_table[0], &f, &actuator_state_live[0]) != PLUGIN_OK)
		return;

	if (actuator_state_live[0].temperature > 24.9f && actuator_state_live[0].temperature < 25.1f)
		robstride_parse_ok = 1;
}

void app_init(void)
{
	can_router_init();
	plugin_table_init();
	actuator_init();
	plant_config_init();

	host_link_init();

	can_loopback_self_test();
	robstride_pack_test();
	robstride_parse_test();

	control_loop_init();
}

void app_run(void)
{
	for (;;) {
		host_link_poll_rx();
		host_link_poll_tx();
	}
}
