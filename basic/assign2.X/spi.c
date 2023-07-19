/*
 * File:   spi.c
 * Author: satei
 *
 * Created on 08 January 2023, 01:03
 */


#include "xc.h"
#include "spi.h"


//SPI enabling 
void spi_config(){
    SPI1CONbits.MSTEN = 1; // master mode
    SPI1CONbits.MODE16 = 0; // 8bit mode
    SPI1CONbits.PPRE = 3; //1:1 scaler
    SPI1CONbits.SPRE = 6; //2:1 scaler
    SPI1STATbits.SPIEN = 1; //enable SPI
}
 
    
 void spi_put_char(char c){
    while (SPI1STATbits.SPITBF == 1);
    SPI1BUF = c;
}


void spi_put_string(char* str){
    for(int i=0 ; str [i]; i++){
        spi_put_char(str[i]);
    }
}


void spi_move_cursor(int column){
            spi_put_char(0x80 + column);
            return;
        }


void spi_move_cursor_second_row(int column){
            spi_put_char(0xC0 + column);
            return;
        }


void spi_clear_first_row(){
    spi_move_cursor(0);
    int i=0;
    for(i=0;i<16;i++){
        spi_put_char(' ');
    }
}

void spi_clear_second_row(){
    spi_move_cursor_second_row(0);
    int i=0;
    for(i=0;i<16;i++){
        spi_put_char(' ');
    }
}


