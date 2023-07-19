/*
 * File:   main.c
 * Author: satei
 *
 * Created on 01 November 2022, 11:53
 */


#include "xc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "p30F4011.h"
#include "functions.h"

#define BUFFER_SIZE 60 //buffer size for UART

int char_count=0; //character counter
char string[16];  //string declaration
 


//UART circular buffer 
typedef struct {
char buffer[BUFFER_SIZE];
int readIndex;
int writeIndex;
} CircularBuffer;

volatile CircularBuffer cb;

void write_buffer( volatile CircularBuffer* cb, char value){
    cb->buffer[cb->writeIndex] = value;
    cb->writeIndex++;
    if (cb->writeIndex == BUFFER_SIZE)
    cb->writeIndex = 0;
}

int read_buffer( volatile CircularBuffer* cb, char* value){
    if (cb->readIndex == cb->writeIndex)
    return 0;
    *value = cb->buffer[cb->readIndex];
    cb->readIndex++;
    if (cb->readIndex == BUFFER_SIZE)
    cb->readIndex = 0;
    return 1;
}




//interrupt for the UART
void __attribute__((__interrupt__,__auto_psv__)) _U2RXInterrupt(){
    IFS1bits.U2RXIF = 0; //turning off the flag
    char val = U2RXREG;
    write_buffer(&cb, val); //putting into buffer
}
    
//interrupt for the s5 button    
void __attribute__((__interrupt__,__auto_psv__)) _INT0Interrupt(){
    IFS0bits.INT0IF = 0; //turning off the flag
    U2TXREG = char_count;//sending the number of characters to UART2
}

//interrupt for the s6 button 
void __attribute__((__interrupt__,__auto_psv__)) _INT1Interrupt(){
    IFS1bits.INT1IF = 0; //turn off the flag
    spi_clear_first_row(); //clearing the first row
    char_count = 0;        //resetting the character count 
    spi_clear_char_recv();//clearing the second row character count
}



int main(void) {
    
    TRISBbits.TRISB0 = 0;
    
    cb.writeIndex = 0;
    cb.readIndex = 0; 
    
    TRISEbits.TRISE8 = 1; //s5 button enabling
    TRISDbits.TRISD0 = 1; //s6 button enabling 
    
    //SPI enabling 
    SPI1CONbits.MSTEN = 1; // master mode
    SPI1CONbits.MODE16 = 0; // 8bit mode
    SPI1CONbits.PPRE = 3; //1:1 scaler
    SPI1CONbits.SPRE = 6; //2:1 scaler
    SPI1STATbits.SPIEN = 1; //enable SPI 
    
    tmr_wait_ms(TIMER1, 1000); //wait 1000 milliseconds for SPI initialisation
    
    //UART2 enabling 
    U2BRG = 11; // baud rate ((7372800 / 4) / (16 * 9600)-1)
    U2MODEbits.STSEL = 0; // 1 stop bit                                       
    U2MODEbits.PDSEL = 0; // 8 bit no parity   
    U2MODEbits.UARTEN = 1; // enable UART
    U2STAbits.UTXEN = 1; // enable U1TX

    //Interrupts enabling             
    IEC0bits.INT0IE = 1;   //enabling for s5    
    IEC1bits.INT1IE = 1;   //enabling for s6
    IEC1bits.U2RXIE = 1;   //enabling for UART2 receiver 
     
    //Print the character received on the second row.
    spi_move_cursor_second_row(0);
    spi_put_string("Char Recv:");
    
    tmr_setup_period(TIMER1,10); //100 Hz
    
    while(1){
        
        tmr_wait_ms(TIMER2,7); //7ms
        
        IEC1bits.U2RXIE = 0; //disabling the UART2 interrupt
        
        char c;
        int read = read_buffer(&cb, &c);
            if (read == 1) {
                spi_move_cursor_first_row(char_count);
                if ((c == '\r') || (c == '\n') || (char_count == 16)) {//checking for special characters and the number characters 
                    spi_clear_first_row();
                    char_count = 0; //resetting the character count 
                    spi_clear_char_recv(); //cleaning the LCD                  
                }else if (c != '\0') {
                    spi_put_char(c); //write the character on SPI
                    char_count++;  //counting the character 
                }   
            }
        sprintf(string,"%d",char_count);//printing the number of characters received 
        spi_move_cursor_second_row(10);
        spi_put_string(string);
        
        IEC1bits.U2RXIE = 1;// re enabling the UART2 interrupt
        tmr_wait_period(TIMER1);     
    } 
    return 0;
}
