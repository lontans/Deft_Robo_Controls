#include "plant/spi3_role.h"

static spi3_role_t s_role = (spi3_role_t)SPI3_ROLE_DEFAULT;

spi3_role_t spi3_role_get(void)
{
	return s_role;
}

void spi3_role_set(spi3_role_t role)
{
	if (role > SPI3_ROLE_NONE)
		role = SPI3_ROLE_NONE;
	s_role = role;
}
