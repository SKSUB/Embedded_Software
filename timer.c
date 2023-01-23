#include "xc.h"
#include "timer.h"

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
    case TIMER3:
    {
        TMR3 = 0;
        T3CONbits.TCKPS = tckps;
        PR3 = pr;
        T3CONbits.TCS=0; 
        T3CONbits.TGATE=0;
        T3CONbits.TON = 1;
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
    case TIMER3:
    {
        while(IFS0bits.T3IF==0); 
        IFS0bits.T3IF=0;
        break;
    }    
    }
}




