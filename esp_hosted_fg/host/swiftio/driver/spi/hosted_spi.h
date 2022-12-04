#ifndef _HOSTED_SPI_H_
#define _HOSTED_SPI_H_

int hosted_spi_init(void *spi, void *cs);
int hosted_spi_transceive(void *tx_data, void *rx_data, int max_buffer_len, int timeout);

#endif /* _HOSTED_SPI_H_ */
