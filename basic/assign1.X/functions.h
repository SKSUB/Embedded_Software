// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#define TIMER1 1
#define TIMER2 2  

#include <xc.h> // include processor files - each processor file is guarded.  
int choose_prescalar(int ms, int* tckps, int* pr);
int tmr_wait_ms(int timer, int ms);
void tmr_setup_period(int timer,int ms);
void tmr_wait_period(int timer);
void spi_put_char(char c);
void spi_position(int pos);
void spi_put_string(char* str);
void spi_move_cursor_first_row(int column);
void spi_move_cursor_second_row(int column);
void spi_clear_first_row();
void spi_clear_char_recv();

#endif	/* XC_HEADER_TEMPLATE_H */

