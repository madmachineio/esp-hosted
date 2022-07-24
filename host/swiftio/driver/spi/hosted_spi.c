#include <swift_spi.h>
#include "hosted_spi.h"
#include <assert.h>

static void *spi_handle;
static void *spi_cs;

int hosted_spi_init(void *spi, void *cs)
{
	spi_handle = spi;
	spi_cs = cs;

	if(spi_cs){
		swifthal_gpio_set(spi_cs, 1);
	}
	return 0;
}

int hosted_spi_transceive(void* tx_data, void* rx_data, int max_buffer_len, int timeout)
{
	int ret = 0;
	
	if(spi_cs){
		swifthal_gpio_set(spi_cs, 0);
	}
	
    ret = swifthal_spi_transceive(spi_handle,
									tx_data, max_buffer_len,
									rx_data, max_buffer_len);

	if(spi_cs){
		swifthal_gpio_set(spi_cs, 1);
	}

	return ret;
}
