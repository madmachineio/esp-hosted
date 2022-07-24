#ifndef _HOSTED_SPI_H_
#define _HOSTED_SPI_H_

int hosted_spi_init(void *spi);
int hosted_spi_transceive(void* tx_data, void* rx_data, uint32_t max_buffer_len, k_timeout_t timeout);

#endif /* _HOSTED_SPI_H_ */
