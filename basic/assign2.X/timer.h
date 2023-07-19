// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef TIMER_H
#define TIMER_H

#define TIMER1 1
#define TIMER2 2  

#include <xc.h> // include processor files - each processor file is guarded.  
int choose_prescalar(int ms, int* tckps, int* pr);
int tmr_wait_ms(int timer, int ms);
void tmr_setup_period(int timer,int ms);
void tmr_wait_period(int timer);
#endif	/* XC_HEADER_TEMPLATE_H */

