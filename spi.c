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

void spi_clear_second_row(){
    spi_move_cursor_second_row(0);
    int i=0;
    for(i=0;i<16;i++){
        spi_put_char(' ');
    }
}


void spi_clean_LCD() {
    spi_clear_first_row();
    spi_clear_second_row();
}

//s6 toggle to print on the LCD

void print1(){
    spi_move_cursor_first_row(0);
    spi_put_string("ST:");
    spi_move_cursor_first_row(7);
    spi_put_string("T:");
    spi_move_cursor_second_row(0);
    spi_put_string("R:");
    }

void print2(){
    spi_move_cursor_first_row(0);
    spi_put_string("SA:");
    spi_move_cursor_second_row(0);
    spi_put_string("R:");
}