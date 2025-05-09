//authors: James Acacio, Arvind Pagidi, Elijah Gutierrez

#include <LiquidCrystal.h>
#include <DHT.h>
#include <Stepper.h>

//–– DHT sensor ––
#define DHT_PIN   2
#define DHT_TYPE  DHT11
DHT       dht(DHT_PIN, DHT_TYPE);

//–– LCD on pins 6,7,8,9,10,11 ––
LiquidCrystal lcd(6, 7, 8, 9, 10, 11);

//–– Stepper on pins 2,3,4,5 ––
const int   STEPS_PER_REV = 2048;
Stepper     stepper(STEPS_PER_REV, 2, 3, 4, 5);

//–– AVR register addresses ––
volatile uint8_t* const ADMUX_REG  = (uint8_t*)0x7C;
volatile uint8_t* const ADCSRB_REG = (uint8_t*)0x7B;
volatile uint8_t* const ADCSRA_REG = (uint8_t*)0x7A;
volatile uint16_t* const ADC_DATA  = (uint16_t*)0x78;
enum { ADSC_BIT = 6 };

volatile uint8_t* const UCSR0A_REG = (uint8_t*)0xC0;
volatile uint8_t* const UCSR0B_REG = (uint8_t*)0xC1;
volatile uint8_t* const UCSR0C_REG = (uint8_t*)0xC2;
volatile uint16_t* const UBRR0_REG  = (uint16_t*)0xC4;
volatile uint8_t* const UDR0_REG   = (uint8_t*)0xC6;
#define UART_RDA  0x80
#define UART_TBE  0x20

volatile uint8_t* const PINB_REG  = (uint8_t*)0x20;
volatile uint8_t* const DDRB_REG  = (uint8_t*)0x21;
volatile uint8_t* const PORTB_REG = (uint8_t*)0x22;
volatile uint8_t* const DDRC_REG  = (uint8_t*)0x27;
volatile uint8_t* const PORTC_REG = (uint8_t*)0x28;

//–– Bit helpers ––
static inline void setBit(volatile uint8_t* r, uint8_t m){ *r |=  m; }
static inline void clrBit(volatile uint8_t* r, uint8_t m){ *r &= ~m; }
static inline bool testBit(volatile uint8_t* r, uint8_t m){ return (*r & m); }

//–– PORTC bit‐masks ––
const uint8_t LED_MASK    = 0b01000000; // PC6
const uint8_t SEL_MASK    = 0b00010000; // PC4
const uint8_t BTN1_MASK   = 0b00000100; // PC2
const uint8_t BTN2_MASK   = 0b00000001; // PC0

float temperature, humidity;

void setup() {
  initUART(9600);
  initADC();

  // configure DDRC exactly as before (mask 0xA8)
  *DDRC_REG &= 0xA8;

  // enable pull-ups on PORTB bits B0/B2/B4 (mask 0x15)
  setBit(PORTB_REG, 0x15);

  dht.begin();
  lcd.begin(16,2);
}

void loop() {
  // read DHT
  humidity    = dht.readHumidity();
  temperature = dht.readTemperature();

  // ADC channel 0
  uint16_t level = readADC(0);
  if (level >= 300) setBit(PORTC_REG, LED_MASK);
  else              clrBit(PORTC_REG, LED_MASK);

  bool left  = testBit(PORTC_REG, SEL_MASK);
  bool right = !left;

  // mirror PC2 or PC0 to PINB bit4
  if ( testBit(PORTC_REG, BTN1_MASK) ||
       testBit(PORTC_REG, BTN2_MASK) )
    setBit(PINB_REG, 0x10);
  else
    clrBit(PINB_REG, 0x10);

  // if either PC2 or PC0, set PINB bit7
  if ( testBit(PORTC_REG, BTN1_MASK) ||
       testBit(PORTC_REG, BTN2_MASK) )
    setBit(PINB_REG, 0x80);
  else
    clrBit(PINB_REG, 0x80);

  // stepper
  if (left || right) {
    stepper.setSpeed(5);
    if (left)  stepper.step(-STEPS_PER_REV);
    if (right) stepper.step( STEPS_PER_REV);
  }

  // update LCD
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.print("C");
  lcd.setCursor(0,1);
  lcd.print("Hum:  ");
  lcd.print(humidity);
  lcd.print("%");
}

//––––– ADC –––––
void initADC(){
  // free-running, prescaler = 128
  setBit(ADCSRA_REG, (1<<7)|(1<<2)|(1<<1)|(1<<0)); // ADEN + ADPS2/1/0
  clrBit(ADCSRA_REG,  (1<<5)|(1<<4));              // ADLAR=0, MUX5=0
  clrBit(ADCSRB_REG,  (1<<3)|(1<<2)|(1<<1)|(1<<0));// all ADTS = 0
}

uint16_t readADC(uint8_t ch){
  *ADMUX_REG = (1<<6) | (ch & 0x07); // REFS0 + channel
  *ADCSRA_REG |= (1<<ADSC);          // start
  while ( testBit(ADCSRA_REG, ADSC_BIT) );
  uint16_t v = *ADC_DATA;
  *ADCSRA_REG = 0;                   // turn off
  return v;
}

//––––– UART –––––
void initUART(uint32_t baud){
  const uint32_t F_CPU = 16000000UL;
  uint16_t ubrr = (F_CPU/(16UL*baud)) - 1;
  *UCSR0A_REG = (1<<1);               // U2X0
  *UBRR0_REG = ubrr;
  *UCSR0B_REG = (1<<4)|(1<<3);        // RXEN0 | TXEN0
  *UCSR0C_REG = (1<<1)|(1<<2);        // 8-bit, no parity, 1 stop
}

bool u0kbhit() { return testBit(UCSR0A_REG, UART_RDA); }
uint8_t u0getChar(){ return *UDR0_REG; }
void    u0putChar(uint8_t c){
  while(!testBit(UCSR0A_REG, UART_TBE));
  *UDR0_REG = c;
}
