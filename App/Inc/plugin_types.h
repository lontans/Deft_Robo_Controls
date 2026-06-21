// plugin_types.h — plugin status enum (no actuator dependency)
#pragma once

typedef enum {
	PLUGIN_OK = 0,
	PLUGIN_ERR_PARAM,
	PLUGIN_ERR_UNSUPPORTED,
} plugin_status_t;
