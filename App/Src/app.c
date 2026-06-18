#include "app.h"
#include "config_loader.h"
#include "can_router.h"#include "can_frame.h"
#include "plugin_table.h"
#include "actuator.h"
#include "usb_protocol.h"
#include "control_loop.h"
volatile uint8_t g_can_loopback_ok = 0; // Debug variablevolatile uint8_t robstride_pack_ok = 0; // Debug variable for Robstride tx pack pluginstatic void can_loopback_self_test(void){	can_frame_t tx = {0};	can_frame_t rx = {0};	// create a sample tx frame to enqueue	tx.id = 0x7F;	tx.id_type = CAN_ID_EXT;	tx.dlc = 2;	tx.data[0] = 0xDE;	tx.data[1] = 0xAD;	if (can_tx_enqueue(CAN_BUS_CH1, &tx) != CAN_OK) {		return;	}	// Poll the router, receive and flush frames	can_router_poll();	// write received frame into sample rx frame	if (can_rx_pop(CAN_BUS_CH1, &rx) != CAN_OK){		return;	}	// check if full transaction was successful, update the debug variable	if (rx.id == tx.id && rx.data[0] == 0xDE && rx.data[1] == 0xAD) {		g_can_loopback_ok = 1;	}}static void robstride_pack_test(void) {	can_frame_t f = {0};	if (plugin_pack_tx(&actuator_table[0], &desire[0], &f) != PLUGIN_OK){		return;	}	if (f.id == 0x7F && f.id_type == CAN_ID_EXT) {  // Temporary expectations, initialised by can_loopback_self_test		robstride_pack_ok = 1;	}}
void app_init(void) {
	can_router_init();   	  // Initialises CAN router
	plugin_table_init();
	actuator_init();          // Initialises skeleton for actuators (table, desires, and state empty)	config_loader_init();     // Fills actuator table after skeleton initialized by actuator_init()	can_loopback_self_test(); // currently running the test script, check if g_can_loopback_ok is correctly 1	robstride_pack_test();    // currently running test script for robstride tx packing
	usb_protocol_init();
	control_loop_init();
}
void app_run(void) {
	usb_protocol_poll();
	can_router_poll();
}

