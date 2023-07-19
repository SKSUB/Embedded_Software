/*
 * File:   main.c
 * Author: satei
 *
 * Created on 30 November 2022, 04:44
 */


#include "xc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "p30F4011.h"
#include "timer.h"
#include "parser.h"

#define MAX_TASK 5
#define MAX_DC_Vol 5



//defining hear beat structure 
typedef struct {
    void * (*task) (void*); 
    void* params;
    int n;
    int N;
} heart_beat;

//defining sensor data structure  
typedef struct {
    float current;
    float temperature;
} sensors_data;

// defining message from UART
typedef struct {
    parser_state* parsState;
    int rpm;
} parsed_data;


heart_beat schedInfo[MAX_TASK];

//scheduler 
void scheduler (void* sched){
    heart_beat* schedInfo = (heart_beat*) sched;
    
    int i;
    for (i = 0; i < MAX_TASK; i++) {
        schedInfo[i].n++;
        if (schedInfo[i].n >= schedInfo[i].N) {
            schedInfo[i].task(schedInfo[i].params);
            
            schedInfo[i].n = 0;
        }
    }
}

//task functions
//task for velocity receiving and parsing them 
void* vel_receiver(void* params){
    parsed_data* parsData = (parsed_data*) params;
    
    if (U2STAbits.OERR == 1) {
        U2STAbits.OERR = 0;
    } //overflow checking and clearing
    while (U2STAbits.URXDA == 1) { //check for UART notification 
        int status = parse_byte(parsData->parsState, U2RXREG);//parsing the data
        LATBbits.LATB1 = !LATBbits.LATB1;

        if (status == NEW_MESSAGE) {// new message check
            
            
            if (strcmp(parsData->parsState->msg_type, "MCREF") == 0 && strlen(parsData->parsState->msg_payload) > 0) { //checking the message type and content
                parsData->rpm = atoi(parsData->parsState->msg_payload); // string to integer conversion 
            }
        }
    }
    
    return NULL;
    
}

//task for PWM
void* voltage_pwm(void* params){
    parsed_data* parsData = (parsed_data*) params; 
    
    double dutyCycle = (parsData->rpm / MAX_DC_Vol);
    PDC2 = 2 * PTPER*dutyCycle;
    return NULL;     
}

//task for reading sensors data 
void* read_sensor(void* params){
    sensors_data* sensData = (sensors_data*) params;
    
    while (ADCON1bits.DONE == 0);
    ADCON1bits.DONE = 0;
    //current
    int Bits = ADCBUF0;
    float Volts = 3 + 2 * (Bits / 1024.0);
    sensData->current = (Volts - 3)* 10; //current conversion 
    if (sensData->current > 15) { // check if the current is greater than 15 A 
        LATBbits.LATB1 = 1; // turn the led on
    } else {
        LATBbits.LATB1 = 0;
    }
    // Temperature 
    int tempBits = ADCBUF1;
    float tempVolts = tempBits * 5.0 / 1024.0; // conversion from bits to voltage 
    sensData->temperature = (tempVolts - 0.75) * 100.0 + 25; // conversion from volt to temperature 
    return NULL;
    
}

//task for sending the sensor data 
void* send_sensor(void* params){
    sensors_data* sensData = (sensors_data*) params;
    char msg[15];
    sprintf(msg, "$MCFBK,%0.2f,%0.2f*", sensData->current, sensData->temperature);
     //sending the message to UART
    for (int i = 0; i < strlen(msg); i++) {
        //while (U2STAbits.UTXBF == 0) {
            U2TXREG = msg[i];
        //}
        
        while (U2STAbits.UTXBF == 1) {
        //waiting for the buffer to complete 
        }
    }
    
    return NULL;        
}

//task for blinking LED
void* led_blink(void* params){
    LATBbits.LATB0 = !LATBbits.LATB0; //LED turning on
    return NULL;
}



//ADC configuration 
void adc_configuration(){
    ADCON3bits.ADCS = 16; //TAD value suggested to the board
    ADCON1bits.ASAM = 1; // automatic sampling 
    ADCON1bits.SSRC = 7; //automatic conversion 
    ADCON3bits.SAMC = 1; //auto
    ADCON2bits.CHPS = 1; //CH0 & CH1
    ADCHSbits.CH0SA = 2; //  AN2 connected to CH0
    ADCHSbits.CH123SA = 1; // AN3 connected to CH1
    ADPCFGbits.PCFG2 = 0; // AN2 as analogue
    ADPCFGbits.PCFG3 = 0; // AN3 as analogue
    ADCHSbits.CH0NA = 0;  //negative input in VREF
    ADCHSbits.CH123NA = 0; //connect to ground 
    ADPCFG = 0xFFFF; 
    ADCON1bits.ADON = 1; // ADC enabling
}



int main(void) {
   
    
    //UART2 enabling 
    U2BRG = 11; // baud rate ((7372800 / 4) / (16 * 9600)-1)
    U2MODEbits.STSEL = 0; // 1 stop bit                                       
    U2MODEbits.PDSEL = 0; // 8 bit no parity  
    U2MODEbits.UARTEN = 1; // enable UART
    U2STAbits.UTXEN = 1; // enable U1TX
    
    //ADC enabling 
    adc_configuration();
    
    //PWM enabling
    PTCONbits.PTMOD = 1;
    PTCONbits.PTCKPS = 1; //1:4 pre scaler
    PWMCON1bits.PEN2H = 1;  //PWM2H PORT
    PTPER = 9215; //(1843200/(50*4)) - 1
    PTCONbits.PTEN = 1;
    
    TRISBbits.TRISB0 = 0; // D3 LED
    TRISBbits.TRISB1 = 0; // D4 LED
    
    //parser initialisation 
    parser_state pstate;
    pstate.state = STATE_DOLLAR;
    pstate.index_type = 0;
    pstate.index_payload = 0;
    
    //sensor data initialisation 
    sensors_data sensData;
    sensData.current = 0;
    sensData.temperature = 0;
    
    //message data initialisation 
    parsed_data parsData;
    parsData.rpm = 0;
    parsData.parsState = &pstate;
    
    
    
    schedInfo[0].task = &vel_receiver;      
    schedInfo[1].task = &voltage_pwm;
    schedInfo[2].task = &read_sensor;
    schedInfo[3].task = &send_sensor;
    schedInfo[4].task = &led_blink; 
    schedInfo[0].params = &parsData;        
    schedInfo[1].params = &parsData;
    schedInfo[2].params = &sensData;   
    schedInfo[3].params = &sensData;  
    schedInfo[4].params = NULL; //nothing
    schedInfo[0].n = 0;
    schedInfo[1].n = 0;
    schedInfo[2].n = 0;      
    schedInfo[3].n = 0;     
    schedInfo[4].n = 0;       
    schedInfo[0].N = 2; //UART is 9600 bps; so 1.3 ms to send the 11 bits.
    schedInfo[1].N = 5;  // need to faster than the sending data to UART
    schedInfo[2].N = 10; //read sensor should be less than the sending data to UART    
    schedInfo[3].N = 200; //1Hz  sending data to UART 
    schedInfo[4].N = 100; //1Hz (500 ms on/off)      
            
    tmr_setup_period(TIMER1, 5); //200 Hz (hear beat time)
    
    while(1){        
        scheduler(&schedInfo);
        tmr_wait_period(TIMER1);
    }
    
    return 0;
}

