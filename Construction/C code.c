// 'C' source line config statements

// CONFIG1L
#pragma config FEXTOSC = ECH    // External Oscillator mode Selection bits (EC (external clock) above 8 MHz; PFM set to high power)
#pragma config RSTOSC = EXTOSC  // Power-up default value for COSC bits (EXTOSC operating per FEXTOSC bits (device manufacturing default))

// CONFIG1H
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)

// CONFIG2L
#pragma config MCLRE = EXTMCLR  // Master Clear Enable bit (If LVP = 0, MCLR pin is MCLR; If LVP = 1, RE3 pin function is MCLR )
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (Power up timer disabled)
#pragma config LPBOREN = OFF    // Low-power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled , SBOREN bit is ignored)

// CONFIG2H
#pragma config BORV = VBOR_2P45 // Brown Out Reset Voltage selection bits (Brown-out Reset Voltage (VBOR) set to 2.45V)
#pragma config ZCD = OFF        // ZCD Disable bit (ZCD disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
#pragma config PPS1WAY = ON     // PPSLOCK bit One-Way Set Enable bit (PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle)
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config DEBUG = OFF      // Debugger Enable bit (Background debugger disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Extended Instruction Set and Indexed Addressing Mode disabled)

// CONFIG3L
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled)

// CONFIG3H
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4L
#pragma config WRT0 = OFF       // Write Protection Block 0 (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection Block 1 (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection Block 2 (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection Block 3 (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG4H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-30000Bh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)
#pragma config SCANE = ON       // Scanner Enable bit (Scanner module is available for use, SCANMD bit can control the module)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored)

// CONFIG5L
#pragma config CP = OFF         // UserNVM Program Memory Code Protection bit (UserNVM code protection disabled)
#pragma config CPD = OFF        // DataNVM Memory Code Protection bit (DataNVM code protection disabled)

// CONFIG5H

// CONFIG6L
#pragma config EBTR0 = OFF      // Table Read Protection Block 0 (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection Block 1 (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection Block 2 (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection Block 3 (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG6H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

//---------------------------------------------------------------------------

#include <stdio.h> 
#include <xc.h>
#include <stdbool.h>
#include <pic18f46k40.h>
#define BUFFERLENGTH 8   

//Se declaran las variables globales a usar.

unsigned int ADC1,ADC2;              //Se usa para guardar el valor de los ADC
unsigned int enable_ADC;
unsigned int ADC_Counter;
unsigned int enable_Timer_0;        
unsigned int Contador_USART;              
char Envio_Dato=0;                   //Se usa para transmitir los datos en orden
volatile int x[BUFFERLENGTH];        //Entrada al filtro
int k=0;
int z = 0;
int i = 0;
unsigned int Canal_A = 0b00010000;   //Prender la salida A del DAC
unsigned int Canal_B = 0b10010000;   //Prender la salida B del DAC
int *apuntador_in;
int *start_buffer;
int *final_buffer;

//Coeficientes filtro FIR
const int coeficientes[BUFFERLENGTH] = {120,   1200,   4920,   9000,   9000,   4920,   1200,    120};  //Coeficientes  de orden 7 FIR
//const int coeficientes[BUFFERLENGTH] = { 0,    0,   -240,   -120,   4200,   11640,   11640,   4200,   -120,   -240,    0,    0};   //Coeficientes de orden 11 FIR


void Port_Configuration();
void Oscillator_Configuration();
void ADC_Configuration();
void Timer_0_Configuration();
void USART_Configuration();
void Envio_DAC(int value, int channel);
void Filtro_FIR(int ADC_FIR);


void Port_Configuration(){
    
    //Latch configuration
    LATA = 0x00; 
    LATB = 0x00; 
    LATC = 0x00; 
    LATD = 0x00; 
    LATE = 0x00; 
    
    //Configuración de pines como entradas y salidas
    TRISA = 0x00;       //(0000000) Port A as outputs
    TRISB = 0x0E;       //(00001110)RB0 and RB4-RB7 as outputs
    TRISC = 0x50;       //(01010000)RC6 and RC4 as outputs
    TRISD = 0b10111111; //(10111111)RD6 as TX
    TRISE = 0x08;       //(00001000)MCLR 
    
    //Configuracion de puertos análogos o digitales
    
    ANSELA = 0x00;       //(00000000)PORT A as digitals
    ANSELB = 0x00;       //(00000000)PORT B as digitals
    ANSELC = 0x00;       //(00000000)PORT C as digitals
    ANSELD = 0xBF;       //(10111111)RD6 as digital, the rest analog
    ANSELE = 0x00;       //(00000000) PORT E as digitals
 
    //Weak internal pull-up disables
    WPUA = 0x00;
    WPUB = 0x00;
    WPUC = 0x00;
    WPUD = 0x00;
    WPUE = 0x00;
    
    //Open Drain
    ODCONA = 0x00;
    ODCONB = 0x00;
    ODCONC = 0x00;
    ODCOND = 0x00;
    ODCONE = 0x00;
    
    //Slew rate Control
    SLRCONA = 0xFF;
    SLRCONB = 0xFF;
    SLRCONC = 0xFF;
    SLRCOND = 0xFF;
    SLRCONE = 0x07;
    
    //Input Threshold Control
    INLVLA = 0xFF;
    INLVLB = 0xFF;
    INLVLC = 0xFF;
    INLVLD = 0xFF;
    INLVLE = 0x0F;

    //Peripheral Module Disable
    PMD0 = 0x00; //Enables CLKRMD CLKR, SYSCMD SYSCLK, SCANMD SCANNER, FVRMD FVR, IOCMD IOC, CRCMD CRC, HLVDMD HLVD y NVMMD NVM.
    PMD1 = 0x00; //Enables TMR0MD TMR0, TMR1MD TMR1, TMR2MD TMR2, TMR3MD TMR3, TMR4MD TMR4, TMR5MD TMR5 y TMR6MD TMR6
    PMD2 = 0x00; //Enables ZCDMD ZCD, DACMD DAC, CMP1MD CMP1, ADCMD ADC, CMP2MD CMP2.
    PMD3 = 0x00; //Enables CCP2MD CCP2, CCP1MD CCP1, PWM4MD PWM4, PWM3MD PWM3.
    PMD4 = 0x00; //Enables CWG1MD CWG1, UART2MD EUSART2, MSSP1MD MSSP1, UART1MD EUSART, MSSP2MD MSSP2.  
    PMD5 = 0x00; //Enables DSMMD DSM,
    //Interruptions
    INTCONbits.IPEN = 0;    //Enables all unmasked interrupts and cleared by hardware for all interrupts
    INTCONbits.PEIE = 1;    // Enables all unmasked peripheral interrupts
    INTCONbits.GIE = 1;     //Global Interrupt Enable bit
    //SPI Transmission
    SSP1STAT = 0b11000000;   // //(11000000)Input data is sampled at the end of data output time,  Transmit occurs on the transition from active to Idle clock state
    SSP1CON1 = 0b00100001;   
    SSP1ADD = 0x02;          //Clock of 3us
    SSP1CLKPPS = 0x13;         //RC3 as PIN Serial Clock 1 DEFAULT PIN!
    SSP1DATPPS = 0x14;         //RC4 AS Serial Data Out pin
    RC3PPS = 0x0F;           //MSSP1 (SCK/SCL)   
    RC5PPS = 0x10;           //MSSP1 (SDO/SDA)
    PORTAbits.RA0=0;
}

void Oscillator_Configuration(void) {
    OSCCON1 = 0x70;         //External Oscilator 4MHz     
    OSCCON3 = 0x00;         //Not a  clock switch
    OSCFRQ  = 0x08;         //Nominal Frequency of 4MHz
    OSCTUNE = 0x00;         //Oscillator module is running at the calibrated frequency (default value).
    OSCEN   = 0x00;         //Oscillator is only enabled if requested by a peripheral
    CLKRCLK = 0x00;         //Clock Source is Fosc
    CLKRCON = 0x90;         //Reference clock module enabled.Clock outputs duty cycle of 50%. Base clock value
}

void ADC_Configuration() {
    ADCON0  = 0x84;          //ADC is enabled, Clock supplied by Fosc, ADRES and ADPREV data are right-justified
    ADCON1  = 0x00;          //ADPRE>0 & ADC input is internal //Both Conversion cycles use the precharge and guards specified by ADPPOL and ADGPOL
    ADCON2  = 0x00;          //ADRES is transferred to ADPREV at start-of-conversion, Clearing action is complete (or not started), Basic (Legacy) mode
    ADCON3  = 0x00;          //Never interrupt
    ADSTAT  = 0x00;          //ADC accumulator and ADERR calculation have not overflowed
    ADCLK   = 0x07;          //FOSC/16 ADC Clock 250kHz
    ADREF   = 0x00;          //VREF- is connected to AVSS, VREF+ is connected to VDD
    ADPRE   = 0x00;          //Precharge time is not included in the data conversion cycle
    ADACQ   = 0x00;          //Acquisition time is not included in the data conversion cycle
    ADRPT   = 0x00;          //Uncharged
    ADCAP   = 0x00;          //No additional capacitance
    ADACT   = 0x00;          //External Trigger Disabled
    ADLTHL  = 0x00; 
    ADLTHH  = 0x00;
    ADUTHL  = 0x00; 
    ADUTHH  = 0x00; 
    ADSTPTH = 0x00; 
    ADSTPTL = 0x00;  
    PIE1bits.ADIE = 1;       //ADC Interrupt Enable
    PIR1bits.ADIF = 0;       //ADC Interrupt Flag
    
}

void Timer_0_Configuration() {
    T0CON1 = 0x40;          //The input to the TMR0 counter is synchronized to Fosc/4 prescaler 1
    TMR0H  = 255;           
    TMR0L  = 156;           
    PIE0bits.TMR0IE = 1;    //Timer0 Interrupt Enable
    PIR0bits.TMR0IF = 0;    //Timer0 Interrupt Flag bit
    T0CON0 = 0x90;          //TMR0 Enable. 16 bits
}


void USART_Configuration()
{
    RC1STA  = 0x90;                 //Serial port enabled, Disables continuous receive               
    RX1PPS  = 0x17;                 //Se define el Pin RC6 como RX1 de EUSART1       
    RC7PPS  = 0x09;                 //Se define el Pin RC7 como TX1 de EUSART1
    SP1BRGH = 0x00;           
    SP1BRGL = 0x67;                 //Baud rate  9600
    TX1STA  = 0x24;                 //Transmit enabled
    BAUD1CONbits.BRG16 = 1;         //16-bit Baud Rate Generator
}
void __interrupt(high_priority) interrupcion_TIMER0(void) {
    if (TMR0IF==1 && TMR0IE==1) {     //Desbordamiento del timer
        TMR0H = 255;                //Se reinicia el contador
        TMR0L = 156;
        PIR0bits.TMR0IF = 0;
        enable_Timer_0=1;      
}
    if (ADIF==1 && ADIE==1) {        //Finaliza conversión   
        
        PIR1bits.ADIF = 0;          
        enable_ADC  = 1;   
    }
} 

void main(void) {
    
    //System Initialize
    Port_Configuration();
    Oscillator_Configuration();
    ADC_Configuration();
    Timer_0_Configuration();
    ADPCH = 0b011000;                    //Se lee el puerto RD0
    apuntador_in=&x[0];
    start_buffer=&x[0];
    final_buffer=&x[BUFFERLENGTH-1];
    while (1) {    
        if (enable_Timer_0 == 1) {   //Contador    
            ADC_Counter++;        //Contador del ADC
            INTCONbits.GIE  = 1;  // //Global Interrupt Enable bit
            PIE0bits.TMR0IE = 1;  //Timer0 Interruption enabled
            INTCONbits.PEIE = 1;  //Peripheral interrupts
            enable_Timer_0  = 0;  //Disabled enable Timer_0
        }
        if(ADC_Counter==892){           //Se completó el tiempo de muestreo
            
             ADCON0bits.GO_nDONE=1;    //ADC enabled
             ADC_Counter = 0;   
             RD5=1;
        }
        if(enable_ADC==1){            
            
            //Se guarda el valor del ADC
            ADC1 =ADRESH;              
            ADC1 =ADC1 <<8;             
            ADC1 =ADC1 +ADRESL;         
            Filtro_FIR(ADC1);   
            INTCONbits.GIE  = 1;                
            PIE1bits.ADIE   = 1;                
            INTCONbits.PEIE = 1;                 
            enable_ADC   = 0;              
            RD5=0;
            }          
     }   
    }

void Filtro_FIR(int ADC_FIR){
    PORTAbits.RA0=1;
   long y = 0;     //Salida filtro
   int *j;
   k=0;
   *apuntador_in=ADC_FIR;
   j=apuntador_in;
   //Se realiza el proceso de filtrado
    while(k<=(BUFFERLENGTH-1)){
        y = y + (long)coeficientes[k]*(long)(*j);
        *j--;
        k++;
        if(j<start_buffer){
          j=final_buffer;
         }
      }
    *apuntador_in++;
    if(apuntador_in>final_buffer){
        apuntador_in=start_buffer;
    }
    y=y>>16;
    PORTAbits.RA0=0;
    Envio_DAC(y,Canal_A);    //Se envía al DAC
}
//Función para esperar transmisión
unsigned char wait_SPI(void){
    while(!SSP1STATbits.BF);
    return SSP1BUF;
}
 // Función de envío los datos al DAC
void Envio_SPI(unsigned char Datos_DAC) {   
    SSP1STATbits.BF == 0;
    SSP1BUF = Datos_DAC;              //Guarda los datos para enviar al DAC
    wait_SPI();                       //Función para esperar transmisión
}

void Envio_DAC(int Dato_DAC, int Canal) {
    int Dato_1;
    Dato_1=Dato_DAC*4;           ////Para una mejor resolución 
    int temp;                     //Variable de 8 bits
    int temp2;                     
    temp = (Dato_1>>8) | Canal;      //Toma los últimos 2 bits del valor ADC  y //Activación de VoutA o VoutB 
    temp2 = Dato_1 & 0b0000000011111111;  ////Toma los primeros 8 bits del valor ADC
    PORTBbits.RB4=1;               //LDAC se pone en 1 para iniciar transmisión
    PORTEbits.RE0=0;               //CS se pone en 0
    Envio_SPI(temp);               //Envía los primeros 8bits
    Envio_SPI(temp2);              //Envía los siguientes 8bits
    PORTBbits.RB4=0;
    PORTEbits.RE0=1;                               
    PORTBbits.RB4=1;               //Para activar señal de salida
}
