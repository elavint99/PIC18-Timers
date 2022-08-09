// Microcontrollers Laboratory
// Practice 8
// Esteban Lavín Treviño

// Libraries and Headers
#include "Config_header.h" // Include the config header file

// Directives
#define _XTAL_FREQ 1000000

#define LCD_DATA_R      PORTD
#define LCD_DATA_W      LATD
#define LCD_DATA_DIR    TRISD
#define LCD_RS          LATCbits.LATC2
#define LCD_RS_DIR      TRISCbits.TRISC2
#define LCD_RW          LATCbits.LATC1
#define LCD_RW_DIR      TRISCbits.TRISC1
#define LCD_E           LATCbits.LATC0
#define LCD_E_DIR       TRISCbits.TRISC0
#define LED_D1 PORTAbits.RA5

// Constants
        
        
// Global Variables - Function Prototypes
void Ports_configuration(void);                     // Ports configuration.
void release(void);                                 // Release function
void LCD_rdy(void);                                 // Function checks if LCD is ready
void LCD_init(void);                                // Function to initialize LCD
void LCD_cmd(char);                                 // Function that sends command to LCD
void send2LCD(char);                                // Function that sends data to LCD
void init_osc(void);
void delay_1s(void);
void counter(void);
void __interrupt (high_priority) high_isr(void);    // High priority Interrupt Service Request.
void __interrupt (low_priority) low_isr(void);      // Low priority Interrupt Service Request.

// Main Function
int frequency = 0;
int fc = 0;
int hundreds = 0;
int tens = 0;
int units = 0;
int x = 0;

void main(void) {
    Ports_configuration();                          // Ports configuration
    LCD_init();                                     // LCD initialization
   
    while(1){                                       // Infinite Loop
        delay_1s();
        if (x==0){
            send2LCD('F');                     // frequency title
            send2LCD('r');                     // frequency title
            send2LCD('e');                     // frequency title
            send2LCD('q');                     // frequency title
            send2LCD('u');                     // frequency title
            send2LCD('e');                     // frequency title
            send2LCD('n');                     // frequency title
            send2LCD('c');                     // frequency title
            send2LCD('y');                     // frequency title
            send2LCD(':');                     // frequency title
            x = 1;
        } 
        LCD_cmd(0xC0);                              // send cursor to second line
        TMR0L = 0x00;                               // reset timer 0 count
        T0CON = 0b11111000;                         // timer 0 configuration; 8 bit; external clock (T0CKI); no prescaler
        delay_1s();                                 // 1 second delay function
        T0CON = 0x00;                               // turn off timer 1
        LED_D1 = LED_D1 ^ 0x01;                     // LED test for 1 second delay (optional)
        frequency = TMR0L;                          // frequency = Timer 0 counter
        hundreds = frequency/100;                   // hundreds value
        send2LCD(hundreds+'0');                     // send hundreds value to LCD
        fc = frequency - hundreds*100;              // frequency without hundreds
        tens = fc/10;                               // tens value
        send2LCD(tens+'0');                         // send tens value to LCD
        units = frequency - tens*10 - hundreds*100; // units value
        send2LCD(units+'0');                        // send units value to LCD
        send2LCD(' ');
        send2LCD('H');                              // Hertz suffix
        send2LCD('z');                              // Hertz suffix
    }
}
 

// Function Definitions
void Ports_configuration(void){         // Ports configuration.
    // PORTB
    ANSELB = 0x00;                      // Digital
    TRISB = 0x0F;                       // Input
    WPUB = 0x0F;                        // Pull up resistors available
    INTCON2 = INTCON2 & 0x7F;           // Activate the pull up resistors
    
    // Oscillator
    OSCCON = 0b00110010; // set internal oscillator at 1Mhz
    
    // TEST PORTA
    TRISAbits.TRISA5=0; // Output
    ANSELAbits.ANSA5=0; // 
    
    //  PORTA
    TRISAbits.TRISA4=1; // input

}

void LCD_rdy(void){ // waits until the LCD is not busy
    char test;
    // configure LCD data bus for input
    LCD_DATA_DIR = 0b11111111;
    test = 0x80;
    while(test){
        LCD_RS = 0; // select IR register
        LCD_RW = 1; // set read mode
        LCD_E = 1; // setup to clock data
        test = LCD_DATA_R;
        Nop();
        LCD_E = 0; // complete a read cycle
        test &= 0x80; // check flag busy bit
    }
    LCD_DATA_DIR = 0b00000000;
}

void LCD_init(void){ //initializes LCD
    LATC = 0; // make sure LCD control port is low
    LCD_E_DIR = 0; // config LCD enable pin for output
    LCD_RS_DIR = 0; // config LCD RS pin for output
    LCD_RW_DIR = 0; // config LCD R/W pin for output
    LCD_cmd(0x38); // configure display to 2x40
    LCD_cmd(0x0C); // turn on display, cursor and not blinking
    LCD_cmd(0x01); // clear display and move cursor to home
}

void LCD_cmd(char cx){ // sends command to LCD
    LCD_rdy(); // wait until LCD is ready
    LCD_RS = 0; // select IR register
    LCD_RW = 0; // set write mode
    LCD_E = 1; // setup clock data
    Nop();
    LCD_DATA_W = cx; // send out command
    Nop(); // small delay
    LCD_E = 0; // complete an external write cycle
}

void send2LCD(char xy){ // outputs to LCD
    LCD_RS = 1;
    LCD_RW = 0;
    LCD_E = 1;
    LCD_DATA_W = xy;
    Nop();
    Nop();
    LCD_E = 0;
}

void delay_1s(void){
    TMR1H = 0x85;               // high byte starting value 85EE
    TMR1L = 0xEE;               // low byte starting value 85EE
    T1GCONbits.TMR1GE = 0;      // start by firmware
    PIR1bits.TMR1IF = 0;        // reset interrupt flag
    T1CON = 0b00110001;         // timer 1 configuration: use Tclock = 4/Fosc; prescaler = 8
    while(PIR1bits.TMR1IF == 0);// wait for flag
    T1CON = 0x00;               // reset timer 1
}

void __interrupt (high_priority) high_isr(void){//  High priority Interrupt Service Request.
    Nop();
}
void __interrupt (low_priority)  low_isr(void){//   Low priority Interrupt Service Request.
    Nop();
}
