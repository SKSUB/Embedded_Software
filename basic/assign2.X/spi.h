// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef SPI_H
#define SPI_H 

#include <xc.h> // include processor files - each processor file is guarded.  
void spi_config();
void spi_put_char(char c);
void spi_put_string(char* str);
void spi_move_cursor(int column);
void spi_move_cursor_second_row(int column);
void spi_clear_first_row();
void spi_clear_second_row();
#endif	/* XC_HEADER_TEMPLATE_H */