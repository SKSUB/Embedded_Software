/*
 * File:   function.c
 * Author: satei
 *
 * Created on 01 November 2022, 11:53
 */


#include "xc.h"
#include "functions.h"

int choose_prescalar(int ms, int* tckps, int* pr) {
    long ticks = 1843.2*ms; // Fcy = 1843200 Hz ?> 1843,2 clock ticks in 1 ms
    if ( ticks <= 65535) { // 16 bits check 
     *tckps = 0;
     *pr = ticks ;
    return 0;
    }
    ticks = ticks / 8; // prescalar 1:8;
    if ( ticks <= 65535) {
     *tckps = 1;
     *pr = ticks ;
     return 0;
    } 
    ticks = ticks / 8; // prescalar 1:64;
    if ( ticks <= 65535) {
     *tckps = 2;
     *pr = ticks ;
     return 0;
    }
    ticks = ticks / 4; // prescalar 1:256;
    if ( ticks <= 65535) {
    *tckps = 3;
    *pr = ticks ;
    return 0;
    }
    return 1;
}
   
int tmr_wait_ms(int timer, int ms){
    int pr,tckps;
    choose_prescalar(ms, &tckps, &pr);
    switch (timer){
        case TIMER1:{
        T1CONbits.TCKPS = tckps;
        PR1=pr;
        T1CONbits.TCS = 0;
        T1CONbits.TGATE = 0;

        T1CONbits.TON = 0;
        IFS0bits.T1IF = 0;
        TMR1 = 0;
        T1CONbits.TON = 1;
        while (IFS0bits.T1IF == 0);
            IFS0bits.T1IF = 0;
            T1CONbits.TON = 0;
        break;
        }
    case TIMER2:{
        T2CONbits.TCKPS = tckps;
        PR2=pr;
        T2CONbits.TCS = 0;
        T2CONbits.TGATE = 0;

        T2CONbits.TON = 0;
        IFS0bits.T2IF = 0;
        TMR2 = 0;
        T2CONbits.TON = 1;
        while (IFS0bits.T2IF == 0);
        IFS0bits.T2IF = 0;
        T2CONbits.TON = 0;
        break;
    }
    }
    return 0;
}


void tmr_setup_period(int timer,int ms){
    int pr,tckps;
    choose_prescalar(ms, &tckps, &pr);
    
    switch(timer){
    case TIMER1:
    {
        TMR1=0;
        T1CONbits.TCKPS=tckps; 
        PR1=pr;
        T1CONbits.TCS=0; 
        T1CONbits.TGATE=0;         
        T1CONbits.TON=1; 
        break;
    }
    case TIMER2:
    {
        TMR2=0;
        T2CONbits.TCKPS=tckps; 
        PR2=pr;
        T2CONbits.TCS=0; 
        T2CONbits.TGATE=0;
        T2CONbits.TON=1; 
        break;
    }
    }
}


void tmr_wait_period(int timer){
    switch(timer){
    case TIMER1:
    {
        while(IFS0bits.T1IF==0);
        IFS0bits.T1IF=0;
        break;
    }
    case TIMER2:
    {
        while(IFS0bits.T2IF==0); 
        IFS0bits.T2IF=0;
        break;
    }
    }
}


void spi_put_char(char c){
    while (SPI1STATbits.SPITBF == 1);
    SPI1BUF = c;
}

void spi_position(int pos){
    while (SPI1STATbits.SPITBF == 1);
    SPI1BUF = pos;
}

void spi_put_string(char* str){
    
    for (int i = 0; str[i] != '\0'; i++) {
        while (SPI1STATbits.SPITBF == 1);
        SPI1BUF = str[i];
    }
}


void spi_move_cursor_first_row(int column){
            spi_position(0x80 + column);
            return;
        }


void spi_move_cursor_second_row(int column){
            spi_position(0xC0 + column);
            return;
        }

void spi_clear_first_row(){
    spi_move_cursor_first_row(0);
    int i=0;
    for(i=0;i<16;i++){
        spi_put_char(' ');
    }
}

void spi_clear_char_recv(){
    spi_move_cursor_second_row(10);
    int i;
    for(i=0;i<6;i++){
        spi_put_char(' ');
    }
}


