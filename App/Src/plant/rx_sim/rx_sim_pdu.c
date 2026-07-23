#include "plant/rx_sim/rx_sim_pdu.h"
#include "plant/rx_sim/rx_sim.h"

void rx_sim_pdu_tick(void)
{
	if (!rx_sim_pdu_enabled())
		return;
	/* Stub — real PDB UART telem later. */
}
