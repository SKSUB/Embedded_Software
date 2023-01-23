// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef SPI_H
#define SPI_H 

#include <xc.h> // include processor files - each processor file is guarded.  
void spi_config();
void spi_put_char(char c); //function for putting character in LCD
void spi_position(int pos); //function for the position 
void spi_put_string(char *str); //function for putting string in the LCD
void spi_move_cursor_first_row(int column); //function for moving the cursor in LCD screen first row 
void spi_move_cursor_second_row(int column); //function for moving the cursor in LCD screen second row 
void spi_clear_first_row(); //function for clearing the cursor in LCD screen first row
void spi_clear_second_row(); //function for clearing the cursor in LCD screen second row
void spi_clean_LCD(); //for cleaning the full LCD
void print1(); //printing the first sate in LCD
void print2(); //printing the second state in LCD when the s6 button is toggled 
#endif	/* XC_HEADER_TEMPLATE_H */