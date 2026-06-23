#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifndef HOST_TRANSPORT_UART
#define HOST_TRANSPORT_UART 0   // 1 = UART, 0 = USB
#endif

typedef struct {
	void (*init)(void);
	size_t (*read)(uint8_t *dst, size_t max_len);
	bool   (*write)(const uint8_t *src, size_t len);
	bool   (*tx_ready)(void);
} host_transport_ops_t;

const host_transport_ops_t *host_transport_get(void);
