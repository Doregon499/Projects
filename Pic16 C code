//Declaración de variab
#include <avr/io.h> //Inclusión de la libreria para el manejo de registros AVR
#define Canal1   0b00000000 // ADC0 
#define Canal2   0b00000001 // ADC1 
#define Canal3   0b00000010 // ADC2 

uint16_t resultado1 = 0; /*resultado definido como variable de 16 bits sin signo, 16 bits pese a ADC de 10 bits, debido a que la variable se 
divide en dos registros de 8 bits por la arquitectura de 8 bits del procesador*/
uint16_t resultado2 = 0;
uint16_t resultado3 = 0;
uint16_t  ADC8Low  = 0; // Variable de 8 bits del ADC, primeros 8 bits
uint16_t  ADC8High = 0; // Variable de 8 bits del ADC, los ultimos 8 bits
#define F_CPU 16000000UL // Defining the CPU Frequency


#define USART_BAUDRATE 9600 // Desired Baud Rate
#define BAUD_PRESCALER (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

#define ASYNCHRONOUS (0<<UMSEL00) // USART Mode Selection

#define DISABLED    (0<<UPM00)
#define EVEN_PARITY (2<<UPM00)
#define ODD_PARITY  (3<<UPM00)
#define PARITY_MODE  DISABLED // USART Parity Bit Selection

#define ONE_BIT (0<<USBS0)
#define TWO_BIT (1<<USBS0)
#define STOP_BIT ONE_BIT      // USART Stop Bit Selection

#define FIVE_BIT  (0<<UCSZ00)
#define SIX_BIT   (1<<UCSZ00)
#define SEVEN_BIT (2<<UCSZ00)
#define EIGHT_BIT (3<<UCSZ00)

#define DATA_BIT   EIGHT_BIT  // USART Data Bit Selection

uint16_t temp = 0;
    uint8_t   Ones_Position = 0;    // 8 bit variables to be send using usart
    uint8_t   Tens_Position = 0;
    uint8_t   Hundreths_Position =0;
    uint8_t   Thousands_Position =0;
void USART_Init()
{
  // Set Baud Rate
  UBRR0H = BAUD_PRESCALER >> 8;
  UBRR0L = BAUD_PRESCALER;
  
  // Set Frame Format
  UCSR0C = ASYNCHRONOUS | PARITY_MODE | STOP_BIT | DATA_BIT;
  
  // Enable Receiver and Transmitter
  UCSR0B = (1<<RXEN0) | (1<<TXEN0);
}

void USART_TransmitPolling(uint8_t DataByte)
{
  while (( UCSR0A & (1<<UDRE0)) == 0) {}; // Do nothing until UDR is ready
  UDR0 = DataByte;
}
void setup()
{
      cli();//Deshabilito todas las interrupciones.
      CLKPR  |=(1<<CLKPS0)|(1<<CLKPS2);  
  USART_Init();
  } 

//Loop principal
void loop()    {                   // Selección de la referencia de voltaje, AVCC
     ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);// Configuración del prescaler del ADC, para obtener la fs deseada
     ADMUX  = (1<<REFS0)|Canal1;                              // Selección del canal de conversión 
     ADCSRA |= (1<<ADEN)  | (1<<ADSC);              // Habilitar ADC
 
   while( !(ADCSRA & (1<<ADIF)) );       // Esperar fin conversión
    ADC8Low   = ADCL;   // Se debe leer primero ADCL (8 bits iniciales) y luego ADCH, de otro modo podemos causar un bloqueo de los registros ADCL y ADCH
    ADC8High  = ADCH;  
    resultado1 = ADC8Low | (ADC8High << 8); //la función ADC junta los dos registros internamente y devuelve el resultado como variable de 16 bits sin signo
  temp = resultado1;
    
    Ones_Position      = temp % 10; // Eg 1023 % 10  -> Remainder -> 3
    temp = temp/10;                 // Eg temp = 1023/10 ->102 
    Tens_Position      = temp % 10; // Eg 102  % 10  -> Remainder -> 2
    temp = temp/10;                 // Eg temp = 102/10 ->10 
    Hundreths_Position = temp % 10;  // Eg 10   % 10  -> Remainder -> 0
    Thousands_Position = temp / 10;  // Eg 10   / 10  -> Quotient  -> 1
    
    //ASCII Conversion
    Ones_Position += 0x30;
    Tens_Position += 0x30;
    Hundreths_Position += 0x30;
    Thousands_Position += 0x30;
    USART_TransmitPolling(Thousands_Position);
    USART_TransmitPolling(Hundreths_Position);
    USART_TransmitPolling(Tens_Position);
    USART_TransmitPolling(Ones_Position);
    USART_TransmitPolling(';');
    ADCSRA |= (1<<ADIF);   // Limpiar la bandera  
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//                      // Selección de la referencia de voltaje, AVCC 
     ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);// Configuración del prescaler del ADC, para obtener la fs deseada
     ADMUX  = (1<<REFS0)|Canal2;                              // Selección del canal de conversión 
     ADCSRA |= (1<<ADEN)  | (1<<ADSC);              // Habilitar ADC
 
   while( !(ADCSRA & (1<<ADIF)) );       // Esperar fin conversión
    ADC8Low   = ADCL;   // Se debe leer primero ADCL (8 bits iniciales) y luego ADCH, de otro modo podemos causar un bloqueo de los registros ADCL y ADCH
    ADC8High  = ADCH;  
    resultado2 = ADC8Low | (ADC8High << 8); //la función ADC junta los dos registros internamente y devuelve el resultado como variable de 16 bits sin signo
     temp = resultado2;
    
    Ones_Position      = temp % 10; // Eg 1023 % 10  -> Remainder -> 3
    temp = temp/10;                 // Eg temp = 1023/10 ->102 
    Tens_Position      = temp % 10; // Eg 102  % 10  -> Remainder -> 2
    temp = temp/10;                 // Eg temp = 102/10 ->10 
    Hundreths_Position = temp % 10;  // Eg 10   % 10  -> Remainder -> 0
    Thousands_Position = temp / 10;  // Eg 10   / 10  -> Quotient  -> 1
    
    //ASCII Conversion
    Ones_Position += 0x30;
    Tens_Position += 0x30;
    Hundreths_Position += 0x30;
    Thousands_Position += 0x30;
    USART_TransmitPolling(Thousands_Position);
    USART_TransmitPolling(Hundreths_Position);
    USART_TransmitPolling(Tens_Position);
    USART_TransmitPolling(Ones_Position);
    USART_TransmitPolling(';');
    ADCSRA |= (1<<ADIF);   // Limpiar la bandera  
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//    
     ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);// Configuración del prescaler del ADC, para obtener la fs deseada
     ADMUX  = (1<<REFS0)|Canal3;                              // Selección del canal de conversión 
     ADCSRA |= (1<<ADEN)  | (1<<ADSC);              // Habilitar ADC
 
   while( !(ADCSRA & (1<<ADIF)) );       // Esperar fin conversión
    ADC8Low   = ADCL;   // Se debe leer primero ADCL (8 bits iniciales) y luego ADCH, de otro modo podemos causar un bloqueo de los registros ADCL y ADCH
    ADC8High  = ADCH;  
    resultado3 = ADC8Low | (ADC8High << 8); //la función ADC junta los dos registros internamente y devuelve el resultado como variable de 16 bits sin signo
     temp = resultado3;
    
    Ones_Position      = temp % 10; // Eg 1023 % 10  -> Remainder -> 3
    temp = temp/10;                 // Eg temp = 1023/10 ->102 
    Tens_Position      = temp % 10; // Eg 102  % 10  -> Remainder -> 2
    temp = temp/10;                 // Eg temp = 102/10 ->10 
    Hundreths_Position = temp % 10;  // Eg 10   % 10  -> Remainder -> 0
    Thousands_Position = temp / 10;  // Eg 10   / 10  -> Quotient  -> 1
    
    //ASCII Conversion
    Ones_Position += 0x30;
    Tens_Position += 0x30;
    Hundreths_Position += 0x30;
    Thousands_Position += 0x30;
    USART_TransmitPolling(Thousands_Position);
    USART_TransmitPolling(Hundreths_Position);
    USART_TransmitPolling(Tens_Position);
    USART_TransmitPolling(Ones_Position);
    USART_TransmitPolling(' ');
    USART_TransmitPolling('\n');
    ADCSRA |= (1<<ADIF);   // Limpiar la bandera  
}
