// DSPIC30F4011 Configuration Bit Settings

// 'C' source line config statements

// FOSC
#pragma config FPR = XT                 // Primary Oscillator Mode (XT)
#pragma config FOS = PRI                // Oscillator Source (Primary Oscillator)
#pragma config FCKSMEN = CSW_FSCM_OFF   // Clock Switching and Monitor (Sw Disabled, Mon Disabled)

// FWDT
#pragma config FWPSB = WDTPSB_16        // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512       // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF            // Watchdog Timer (Disabled)

// FBORPOR
#pragma config FPWRT = PWRT_64          // POR Timer Value (64ms)
#pragma config BODENV = BORV20          // Brown Out Voltage (Reserved)
#pragma config BOREN = PBOR_ON          // PBOR Enable (Enabled)
#pragma config LPOL = PWMxL_ACT_HI      // Low-side PWM Output Polarity (Active High)
#pragma config HPOL = PWMxH_ACT_HI      // High-side PWM Output Polarity (Active High)
#pragma config PWMPIN = RST_IOPIN       // PWM Output Pin Reset (Control with PORT/TRIS regs)
#pragma config MCLRE = MCLR_EN          // Master Clear Enable (Enabled)

// FGS
#pragma config GWRP = GWRP_OFF          // General Code Segment Write Protect (Disabled)
#pragma config GCP = CODE_PROT_OFF      // General Segment Code Protection (Disabled)

// FICD
#pragma config ICS = ICS_PGD            // Comm Channel Select (Use PGC/EMUC and PGD/EMUD)


// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

//SATHIHSH KUMAR SUBRAMANI (S4847560) PROJECT 2021-2022 CURRICULUM 

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "p30F4011.h"
#include "timer.h"
#include "parser.h"
#include "spi.h"


#define MAX_MOTOR_RPM 12000
#define MIN_MOTOR_RPM -12000
#define MAX_PROPELLER 9000
#define MIN_PROPELLER -9000    
#define MAX_TASK 7 //maximum number of task 
#define BUFFER_SIZE 120 //buffer size for UART
#define INDEX_SIZE 10  //buffer size for temperature
#define CONTROLLED_MODE 0 //for defining the states of the board
#define TIMEOUT_MODE 1
#define SAFE_MODE 2
#define s6case_1 0 //for defining the LCD toggling 
#define s6case_2 1

void send_string_UART(char* msg); //function to send the message through UART
void msg_checker(char* msg_type, char* msg_payload); //message for checking the current new message and act accordingly based on the state of the board as well as the specification
void rpm(int rpm1, int rpm2); //updating the PDC and duty cycle 
int state = CONTROLLED_MODE; //microcontroller state defining 
int s6status; //status for LCD screen




//defining hear beat structure 
typedef struct {
    void (*task)(void*); 
    void* params;
    int n;
    int N;
} heart_beat;

heart_beat schedInfo[MAX_TASK];

//scheduler
void scheduler(void* sched){
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







//defining the UART data buffer structure 
typedef struct{
    int buffer[BUFFER_SIZE];
    int readIndex;
    int writeIndex;
}circularBuffer;

//In order to receive all the messages from UART, circular buffer is introduced
volatile circularBuffer cb;

void write_buffer( volatile circularBuffer* cb, char value){
    cb->buffer[cb->writeIndex] = value;
    cb->writeIndex++;
    if (cb->writeIndex == BUFFER_SIZE){
        cb->writeIndex = 0;
    }
}

int read_buffer( volatile circularBuffer* cb, char *value){
    IEC1bits.U2RXIE = 0; //disabling the interrupt of UART
    if (cb->readIndex == cb->writeIndex){
        IEC1bits.U2RXIE = 1; // re enabling the interrupt of UART 
        return 0;
    }
    
    *value = cb->buffer[cb->readIndex];
    cb->readIndex++;
    if (cb->readIndex == BUFFER_SIZE){
    cb->readIndex = 0;
    }
    IEC1bits.U2RXIE = 1; // re enabling the interrupt 
    return 1;
}

//This function is to read the available bytes in the buffer
int read_data( volatile circularBuffer* cb ){
    if (cb->writeIndex >= cb->readIndex){
        return (cb->writeIndex - cb->readIndex);
    } else {
        return (BUFFER_SIZE - (cb->readIndex - cb->writeIndex));
    }
    return 1; //return error 
}





//structure for temperature data to acquire the temperature data and put them in buffer 
typedef struct{
    float index[INDEX_SIZE];
    int write_temp;
    double average_temp;
}temperatureBuffer;

temperatureBuffer tempbuf;

void temp_write_buffer( temperatureBuffer* tempbuf, float data){
    tempbuf->index[tempbuf->write_temp] = data;
    tempbuf->write_temp++;
    if (tempbuf->write_temp == INDEX_SIZE){
        tempbuf->write_temp = 0;
    }
}






//defining the structure for motors data 
typedef struct{
    int maxRPM;
    int minRPM;
    int leftRPM;
    int rightRPM;
    double dutycycle1;
    double dutycycle2;
}motorsData;

motorsData motor_data;







//task functions
//task to parse the UART incoming data 
void UARTparse(void* params){
    
    parser_state* pstate = (parser_state*) params;

    while (read_data(&cb) > 0){
        char c;
        int flag;        
        IEC1bits.U2RXIE = 0;
        read_buffer(&cb,&c); // read the values in the buffer 
        IEC1bits.U2RXIE = 1;
        
        flag = parse_byte(pstate,c); // parse the messages 
        
        if (flag == NEW_MESSAGE){//check for whether the current message is a new message 
            LATBbits.LATB1 = !LATBbits.LATB1;
            msg_checker(pstate->msg_type, pstate->msg_payload); //message for checking the current new message and act accordingly based on the state of the board as well as the specification 
        }
    }        
}




//task for PWM refresh
void PWM(){
    
    rpm(motor_data.leftRPM, motor_data.rightRPM); //function to update the duty cycles and PDC2 and PDC3
}


//task to send the feedback message at 5 Hz
void feedback(){
    char msg_fb[50];
    sprintf(msg_fb, "$MCFBK,%d,%d,%d*", motor_data.leftRPM, motor_data.rightRPM, state); //printing the message to be send
    //sending the message to UART
    send_string_UART(msg_fb);
}


//task for acquisition of temperature and putting into temperature buffer
void temperature_acq(void* params){
    temperatureBuffer* tempBuf = (temperatureBuffer*) params;
    
    while (ADCON1bits.DONE == 0);
    ADCON1bits.DONE = 0;
    int tempBits = ADCBUF0; // read from channel 0
    float tempVolts = tempBits * 5.0 / 1024.0; // bits to Voltage conversion 
    float temperature = (tempVolts - 0.75) * 100.0 + 25; // voltage to temperature coversion
    temp_write_buffer(tempBuf,temperature); //Write values on temperature buffer
}


//task for sending the average temperature message to UART 
void temperature_send(void* params){
    temperatureBuffer* tempBuf = (temperatureBuffer*) params;
    double averageTemp = 0.0;
    //calculating the average temperature 
    for (int i = 0; i <= INDEX_SIZE; i++) {
        averageTemp = averageTemp + tempBuf->index[i];
    }
    tempBuf->average_temp = averageTemp / INDEX_SIZE; //INDEX_SIZE is 10 so the average of last 10 values 
    char msg_avg_temp[50]; //declaring the message 
    sprintf(msg_avg_temp, "$MCTEM,%.2f*", tempBuf->average_temp); //printing the message to send
    //sending the message to UART in 1 Hz
    send_string_UART(msg_avg_temp);
}


//task to handle the LED D3 and D4
void LED(){
    LATBbits.LATB0 = !LATBbits.LATB0; // LED D3 blinking
    /*if (state == TIMEOUT_MODE){
        LATBbits.LATB1 = !LATBbits.LATB1; //LED D4 blinking for timeout mode
    } else 
        LATBbits.LATB1 = 0; // Switch off led D4 for other modes*/
}



//task to print in LCD
void LCD(void* params){
    temperatureBuffer* tempBuf = (temperatureBuffer*) params;
    
    char tempStr[5];
    char rpmStr[14];
    char dutyStr [14];
    
    switch (s6status) {//switching based on state of the LCD 
        case s6case_1:{
            spi_clean_LCD();
            print1(); //this function prints the s6 case 1 in the LCD 
            switch (state) {
                case TIMEOUT_MODE:
                    spi_move_cursor_first_row(4);
                    spi_put_string("T"); 
                    break;
                    
                case SAFE_MODE:
                    spi_move_cursor_first_row(4);
                    spi_put_string("H");
                    break;
                case CONTROLLED_MODE:
                    spi_move_cursor_first_row(4);
                    spi_put_string("C");
                    break;
            }
            sprintf(tempStr, "%.2f", tempBuf->average_temp);
            spi_move_cursor_first_row(10);
            spi_put_string(tempStr);
            sprintf(rpmStr,"%d;%d", motor_data.leftRPM, motor_data.rightRPM);
            spi_move_cursor_second_row(3);
            spi_put_string(rpmStr);
            break;
        }
        
        case s6case_2: {
            spi_clean_LCD();
            print2(); //this function prints the s6 case 2 in the LCD
            sprintf(rpmStr,"%d,%d", motor_data.minRPM, motor_data.maxRPM);
            spi_move_cursor_first_row(4);
            spi_put_string(rpmStr);
            sprintf(dutyStr,"%0.2f,%0.2f", motor_data.dutycycle1, motor_data.dutycycle2);
            spi_move_cursor_second_row(3);
            spi_put_string(dutyStr);
            break;
        }
    }
}






//other functions
//function for sending the string to UART
void send_string_UART(char* msg){
    
    for (int i = 0; i < strlen(msg); i++) {
        while (U2STAbits.UTXBF == 1); //wait for the buffer to complete 
        U2TXREG = msg[i]; //sending the message to UART
    }
}

//message checking function for new message check 
void msg_checker(char* msg_type, char* msg_payload) {
    
    check:{
        switch (state) {
            case TIMEOUT_MODE:{
                
                motor_data.leftRPM = 0;
                motor_data.rightRPM = 0;
                if (strcmp(msg_type, "HLREF") == 0) {
                state = CONTROLLED_MODE;
                goto check;// in order to not miss the message and move to the controlled mode and the message is processed accordingly 
                }
                if (strcmp(msg_type, "HLSAT") == 0) {
                state = CONTROLLED_MODE; 
                goto check; // in order to not miss the message and move to the controlled mode and the message is processed accordingly 
                }
                break;
            }
                
            case SAFE_MODE:{
                if (strcmp(msg_type, "HLENA") == 0) {
                    motor_data.leftRPM = 0;
                    motor_data.rightRPM = 0;
                    state = CONTROLLED_MODE; //changing the state to controlled mode 
                    char msg_2[50];
                    sprintf(msg_2,"$MCACK,ENA,1*");////sending the ack message
                    send_string_UART(msg_2);
                    //resetting the s5 button and timeout timer mode
                    IFS0bits.INT0IF = 0; // Reset interrupt flag for S5 button (precaution)
                    IEC0bits.INT0IE = 1; //enabling the interrupt again
                    T2CONbits.TON = 0; //stop the timer (precaution)
                    IFS0bits.T2IF = 0; // resetting the flag (precaution)
                    IEC0bits.T2IE = 1; //enabling the timer interrupt 
                    T2CONbits.TON = 1;  //start the timer 
                    break;
                } else {
                    char msg_1[50];
                    sprintf(msg_1,"$MCACK,ENA,0*"); //sending the ack message
                    send_string_UART(msg_1);
                    break;
                }
            }
            
            case CONTROLLED_MODE:{
                
                int tempRPM1 = 0;
                int tempRPM2 = 0;
                int tempMAXrpm = 0;
                int tempMINrpm = 0;
                if (strcmp(msg_type, "HLREF") == 0) {
                    sscanf(msg_payload, "%d,%d", &tempRPM1, &tempRPM2);
                    //check for rpm is under the maximum and minimum allowable values 
                    if (tempRPM1 < motor_data.maxRPM){
                        if (tempRPM1 > motor_data.minRPM ){
                            motor_data.leftRPM = tempRPM1;
                        } else {
                            motor_data.leftRPM = motor_data.minRPM;
                        }
                    } else {
                        motor_data.leftRPM = motor_data.maxRPM;
                    }
                    
                    if (tempRPM2 < motor_data.maxRPM){
                        if (tempRPM2 > motor_data.minRPM ){
                            motor_data.rightRPM = tempRPM2;
                        } else {
                            motor_data.rightRPM = motor_data.minRPM;
                        }
                    } else {
                        motor_data.rightRPM = motor_data.maxRPM;
                    }
                    
                    //in order to reset timer interrupt for the new income of message 
                    T2CONbits.TON = 0; //stop the timer
                    IFS0bits.T2IF = 0; // resetting the flag
                    IEC0bits.T2IE = 1; //enabling the timer interrupt
                    T2CONbits.TON = 1;  //start the timer
                    break;
                }
                
                if (strcmp(msg_type, "HLSAT") == 0) {
                    sscanf(msg_payload, "%d,%d", &tempMINrpm, &tempMAXrpm);
                    if(tempMAXrpm>tempMINrpm && tempMAXrpm<MAX_PROPELLER && tempMINrpm>MIN_PROPELLER && tempMAXrpm>=0 && tempMINrpm<=0){ //checking for the propeller allowable values 
                        char msg_3[50];
                        sprintf(msg_3,"MCACK,SAT,1"); //sending the ack message 
                        send_string_UART(msg_3);
                        motor_data.minRPM = tempMINrpm;//new saturation values are set 
                        motor_data.maxRPM = tempMAXrpm;      
                    } else {
                        char msg_4[50];
                        sprintf(msg_4,"MCACK,SAT,0");//sending the ack message 
                        send_string_UART(msg_4);
                    }
                }
                break;
            }
        }
    }        
}

//functions for updating the duty cycles and PDC
void rpm(int left, int right){
    
    motor_data.dutycycle1 = ((double)left - (double)MIN_MOTOR_RPM)/ ((double)MAX_MOTOR_RPM - (double)MIN_MOTOR_RPM);
    
    motor_data.dutycycle2 = ((double)right - (double)MIN_MOTOR_RPM)/ ((double)MAX_MOTOR_RPM - (double)MIN_MOTOR_RPM);
    
    PDC2 = 2*PTPER*motor_data.dutycycle1;
    PDC3 = 2*PTPER*motor_data.dutycycle2;
}




//interrupts
//UART interrupt for receiving messages 
void __attribute__((__interrupt__,__auto_psv__)) _U2RXInterrupt(){
    IFS1bits.U2RXIF = 0; //turning off the flag
    while (U2STAbits.URXDA == 1) {
        write_buffer(&cb, U2RXREG);
    }
    //putting into buffer
}

//timer 2 interrupt
void __attribute__((__interrupt__, __auto_psv__)) _T2Interrupt() {
    IFS0bits.T2IF = 0; //resetting the flag
    IEC0bits.T2IE = 0; //disabling the interrupt
   
    state = TIMEOUT_MODE; // moving the state to time out state
    motor_data.leftRPM = 0; //setting the rpms values to zero (the duty cycle and PDC refresh later in the PWM refresh routine task)
    motor_data.rightRPM = 0;
    T2CONbits.TON = 0; //stops the timer
    TMR2 = 0; //resetting 
}

//s5 button interrupt
void __attribute__((__interrupt__, __auto_psv__)) _INT0Interrupt() {
    IFS0bits.INT0IF = 0; //resetting the flag
    state = SAFE_MODE;  // moving the state to safe mode
    motor_data.leftRPM = 0; //to reset the rpm and stop the motor
    motor_data.rightRPM = 0;
    //update the duty cycle and PDC immediately in the safe mode 
    rpm(0,0);    
    
    
    //stopping the time out mode 
    T2CONbits.TON = 0; //stops the timer
    TMR2 = 0; //resetting 
    IFS0bits.T2IF = 0; //resetting the flag 
    IEC0bits.T2IE = 0; //disable the interrupt of timer 2 timout mode 
    
    
    //no need of debouncing, because pressing once move to the safe mode so noise is not considered here  
}

//s6 button interrupt 
void __attribute__((__interrupt__, __auto_psv__)) _INT1Interrupt() {
    IFS1bits.INT1IF = 0; // resetting the flag
    s6status = (s6status +1 ) % 2;
    tmr_setup_period(TIMER3, 10); // debouncing time period 
    IEC1bits.INT1IE = 0; // disabling the interrupt
    IEC0bits.T3IE = 1; //deboucing timer interrupt 
}

//debouncing timer interrupt
void __attribute__((__interrupt__, __auto_psv__)) _T3Interrupt() {
    IFS0bits.T3IF = 0; // flag reset
    T3CONbits.TON = 0; // stopping the timer 
    IEC0bits.T3IE = 0; //deboucing timer interrupt disabling 
    IFS1bits.INT1IF = 0; // re setting the flag for s6 (precaution)
    IEC1bits.INT1IE = 1; // enabling the interrupt 
}



//ADC configuration for temperature 
void adc_configuration(){
    ADCON3bits.ADCS = 8; //TAD value suggested to the board
    ADCON1bits.ASAM = 1; // automatic sampling 
    ADCON1bits.SSRC = 7; //automatic conversion 
    ADCON3bits.SAMC = 1; //auto
    ADCON2bits.CHPS = 0; //CH0 only
    ADCHSbits.CH0SA = 3; //  AN3 connected to CH0
    ADPCFGbits.PCFG3 = 0; // AN3 as analogue
    ADCHSbits.CH0NA = 0;  //negative input in VREF, to ground
    ADPCFG = 0xFFFF; 
    ADCON1bits.ADON = 1; // ADC enabling
}




int main(void) {

    //configuring for 2 motors
    PTCONbits.PTMOD = 0; //free running mode 
    PTCONbits.PTCKPS = 0; //1:1 pre scaler
    PWMCON1bits.PEN2H = 1;  //PWM2H PORT
    PWMCON1bits.PEN3H = 1; //PWM2H PORT
    // Tcy = 543 ns
    DTCON1bits.DTA = 6; // dead time of 3ms (3000/543 = 6 appr.)
    DTCON1bits.DTAPS = 0;
    PTPER = 1842; //(1843200/(1000*1)) - 1
    PTCONbits.PTEN = 1; //enable PWM
    
    
    //UART2 configuring 
    //U2BRG = 47; //(2400 bpms)
    U2BRG = 23; //(4800 bpms) //with this baud rate it does not miss any message
    //U2BRG = 11; // baud rate ((7372800 / 4) / (16 * 9600)-1)
    
    //U2MODEbits.STSEL = 0; // 1 stop bit                                       
    //U2MODEbits.PDSEL = 0; // 8 bit no parity   
    U2MODEbits.UARTEN = 1; // enable UART
    U2STAbits.UTXEN = 1; // enable U1TX
    U2STAbits.URXISEL = 2; //when Receive Buffer is 3/4 full
    IEC1bits.U2RXIE = 1; //enabling the interrupt for reception
    
    //ADC configuration and enabling  
    adc_configuration();    
    
    //SPI configuration and enabling 
    spi_config();
   
    //initialising the LCD 
    TRISBbits.TRISB0 = 0; // D3 LED initialisation 
    TRISBbits.TRISB1 = 0; // D4 LED initialisation
    
     // assuming starting with the controlled mode (because they will be in one of the three states)
    s6status = s6case_1; //starting as not pressed mode  
    
    
    //all the buffers are initialised
    temperatureBuffer tempbuf;
    cb.writeIndex = 0;
    cb.readIndex = 0; 
    tempbuf.write_temp = 0;
    tempbuf.average_temp = 0;
        
    //motor data initialisation 
    motor_data.leftRPM = 0;
    motor_data.rightRPM = 0;
    motor_data.dutycycle1 = 0.5; //starting with the 0 rpm, hence 0.50 duty cycle for the specified RPM
    motor_data.dutycycle2 = 0.5;            
    motor_data.maxRPM = MAX_PROPELLER;
    motor_data.minRPM = MIN_PROPELLER;       
    
    //parser initialisation 
    parser_state pstate;
    pstate.state = STATE_DOLLAR;
    pstate.index_type = 0;
    pstate.index_payload = 0;
    

    //button initialisation 
    TRISEbits.TRISE8 = 1; //s5 button enabling
    TRISDbits.TRISD0 = 1; //s6 button enabling 
    
    //configuring the interrupts
    IEC0bits.T3IE = 1; // Enable interrupt of debouncing timer t3
    IFS0bits.INT0IF = 0; // resetting the flag of s5 button 
    IFS1bits.INT1IF = 0; // s6 button flag reset 
    IEC0bits.INT0IE = 1; // enabling interrupt for s5 button 
    IEC1bits.INT1IE = 1; // enabling interrupt for s6 button 
   
    tmr_setup_period(TIMER2, 5000); //timeout timer
    IEC0bits.T2IE = 1; //enabling the timeout interrupt 
    
    schedInfo[0].task = &UARTparse; //task to parse the UART incoming data
    schedInfo[1].task = &PWM; //task for PWM refresh
    schedInfo[2].task = &feedback; //task to send the feedback message at 5 H
    schedInfo[3].task = &temperature_acq; //task for acquisition of temperature and putting into temperature buffer
    schedInfo[4].task = &temperature_send; ////task for sending the average temperature message to UART 
    schedInfo[5].task = &LED; //task to handle the LED D3 and D4
    schedInfo[6].task = &LCD; //task to print in LCD
    
    schedInfo[0].params = &pstate;        
    schedInfo[1].params = NULL;
    schedInfo[2].params = NULL;   
    schedInfo[3].params = &tempbuf;  
    schedInfo[4].params = &tempbuf; 
    schedInfo[5].params = NULL; 
    schedInfo[6].params = &tempbuf; 
    
    schedInfo[0].n = 0;
    schedInfo[1].n = 0;
    schedInfo[2].n = 0;      
    schedInfo[3].n = 0;     
    schedInfo[4].n = 0;   
    schedInfo[5].n = 0;
    schedInfo[6].n = 0;  
    
    schedInfo[0].N = 2;  //100 Hz to read circular buffer (in order to make it faster and read the data from the buffer)
    schedInfo[1].N = 1;  //200 Hz for PWM (faster than the UART task to process and send the signal faster without missing them)
    schedInfo[2].N = 40;  //feedback message need to be 5Hz  
    schedInfo[3].N = 20; //temperature acquisition need to be 10Hz
    schedInfo[4].N = 200; // temperature send need to be 1Hz
    schedInfo[5].N = 100; //LED need to blink 1Hz (500ms on and 500 ms off, so 2Hz)
    schedInfo[6].N = 200;  //LCD refresh has been set to 1Hz
            
    tmr_setup_period(TIMER1, 5); //heartbeat 5ms  (200 Hz)
     
    while(1){        
        scheduler(&schedInfo);
        tmr_wait_period(TIMER1);       
    }
    return 0;
}
