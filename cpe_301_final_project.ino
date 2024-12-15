//Names: Nicholas Katsaros and Zackary Troutman

#include <Keypad.h>
#include <LiquidCrystal.h>
#include <Stepper.h>
#include <DHT.h>
#include <DHT_U.h>
#include <RTClib.h>

//Pins
#define FAN 8
#define WATER_LEVEL 0 //This is Analog 0
#define YELLOW 40
#define GREEN 41
#define RED 42
#define BLUE 43
#define DHTPIN 50
#define ISR_PIN 18


#define DHTTYPE DHT11

//States
#define DISABLED 0
#define IDLE 1
#define ERROR 2
#define RUNNING 3

//UART
#define RDA 0x80
#define TBE 0x20  
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

//ADC
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;


void U0putchar(unsigned char U0pdata);
void U0puts(const char* str);
void U0putint(int num, int digits);
void setup_pin(uint8_t pin, uint8_t mode);
char get_key_press();
int get_water_level();
float get_temperature();
void initialize_rtc();
void print_time();
int adc_read(int channel);
void adc_init();
void get_start();


const int ROW_NUM = 4;
const int COL_NUM = 4;

char keys[ROW_NUM][COL_NUM] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

byte rowPins[ROW_NUM] =  {ISR_PIN, 25, 27, 29};
byte colPins[COL_NUM] = {31, 33, 35, 37}; //last pin handles ISR
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROW_NUM, COL_NUM);

const int RS = 11, EN = 12, D4 = 2, D5 = 3, D6 = 4, D7 = 5;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);


int state;
int previous_state = DISABLED;
DHT dht(DHTPIN, DHTTYPE);
RTC_DS1307 rtc;
unsigned long last_check_time;
int stepsPerRevolution = 2048; // # steps for full 360-degree rotation
int rpm = 10; // speed for the stepper motor
bool rtcInitialized = false;

// initialize stepper library on pins 8 - 11
// pin order IN1, IN3, IN2, IN4
Stepper myStepper (stepsPerRevolution, 47, 51, 49, 53);

void safe_rtc_adjust() {
  if (!rtc.isrunning()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
}

void initialize_rtc() {
  if (!rtc.begin()) {
    U0puts("Error: Couldn't find RTC.");
    U0puts("\n");
    rtcInitialized = false;
  } else {
    safe_rtc_adjust();
    rtcInitialized = true;
  }
}

void U0init(unsigned long U0baud) {
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}

void setup_pin(uint8_t pin, uint8_t mode) {
  if (pin >= 0 && pin <= 7) {              // PORTD: Pins 0–7
    if (mode == OUTPUT){
      DDRD |= (1 << pin);
    } else {
      DDRD &= ~(1 << pin);
    }
  } else if (pin >= 8 && pin <= 13) {      // PORTH: Pins 8–13
    if (mode == OUTPUT){
      DDRH |= (1 << (pin - 8));
    } else {
      DDRH &= ~(1 << (pin - 8));
    }
  } else if (pin >= 14 && pin <= 15) {     // PORTJ: Pins 14–15
    if (mode == OUTPUT){
      DDRJ |= (1 << (pin - 14));
    } else {
      DDRJ &= ~(1 << (pin - 14));
    }
  } else if (pin >= 16 && pin <= 17) {     // PORTH: Pins 16–17
    if (mode == OUTPUT){
      DDRH |= (1 << (pin - 16 + 2));
    } else {
      DDRH &= ~(1 << (pin - 16 + 2));
    }
  } else if (pin >= 22 && pin <= 29) {     // PORTA: Pins 22–29
    if (mode == OUTPUT){
        DDRA |= (1 << (pin - 22));
    } else {
      DDRA &= ~(1 << (pin - 22));
    }
  } else if (pin >= 30 && pin <= 37) {     // PORTC: Pins 30–37
    if (mode == OUTPUT) { 
      DDRC |= (1 << (pin - 30));
    } else {
      DDRC &= ~(1 << (pin - 30));
    }
  } else if (pin >= 38 && pin <= 41) {     // PORTD: Pins 38–41
    if (mode == OUTPUT) {
        DDRD |= (1 << (pin - 38 + 4));
    } else {
      DDRD &= ~(1 << (pin - 38 + 4));
    }
  } else if (pin >= 42 && pin <= 49) {     // PORTL: Pins 42–49
    if (mode == OUTPUT){
      DDRL |= (1 << (pin - 42));
    } else {
      DDRL &= ~(1 << (pin - 42));
    }
  } else if (pin >= 50 && pin <= 53) {     // PORTB: Pins 50–53
    if (mode == OUTPUT){
      DDRB |= (1 << (pin - 50));
    } else {
      DDRB &= ~(1 << (pin - 50));
    }
  } else if (pin >= 54 && pin <= 61) {     // PORTF: Analog pins A0–A7 (54–61)
    if (mode == OUTPUT){
      DDRF |= (1 << (pin - 54));
    } else {
      DDRF &= ~(1 << (pin - 54));
    }
  } else if (pin >= 62 && pin <= 69) {     // PORTK: Analog pins A8–A15 (62–69)
    if (mode == OUTPUT){
      DDRK |= (1 << (pin - 62));
    } else {
      DDRK &= ~(1 << (pin - 62));
    }
  }
}

void adc_init()
{
  ADMUX = (1 << REFS0);
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

int adc_read(int channel) {
  ADMUX = (ADMUX & 0xF8) | (channel & 0x07);

  ADCSRA |= (1 << ADSC);

  // Wait for the conversion to complete
  while (ADCSRA & (1 << ADSC));

  // Return the ADC value as an int
  return (int)ADC;
}


volatile bool start_pressed = false;
void setup() {
  U0init(9600);

  myStepper.setSpeed(rpm);

  setup_pin(FAN, OUTPUT);
  setup_pin(WATER_LEVEL, INPUT);
  setup_pin(YELLOW, OUTPUT);
  setup_pin(GREEN, OUTPUT);
  setup_pin(RED, OUTPUT);
  setup_pin(BLUE, OUTPUT);
  
  DDRD &= ~(1 << PD3); //Setting up pin 18 for ISR
  PORTD|= (1 << PD3);
  attachInterrupt(digitalPinToInterrupt(ISR_PIN), get_start, FALLING);

  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.write("Temp: ");
  lcd.setCursor(0, 1);
  lcd.write("Humidity: ");

  dht.begin();

  initialize_rtc();

  adc_init();

  state = DISABLED;
  previous_state = DISABLED;

  last_check_time = 0;
}

void get_start() { //ISR
  if (get_key_press() == '3'){
    start_pressed = true;
  }
}

void turn_fan_on(){
  PORTB |= (1 << PORTB0);
}

void turn_fan_off(){
  PORTB &= ~(1 << PORTB0);
}

void turn_off_led(int color){
  if (color == YELLOW){
    PORTG &= ~(1 << PORTG1);
  } else if (color == GREEN){
    PORTG &= ~(1 << PORTG0);
  } else if (color == RED){
    PORTL &= ~(1 << PORTL7);
  } else if (color == BLUE){
    PORTL &= ~(1 << PORTL6);
  }
}

void turn_on_led(int color){
  if (color != YELLOW){
    turn_off_led(YELLOW);
  }
  if (color != GREEN){
    turn_off_led(GREEN);
  }
  if (color != RED){
    turn_off_led(RED);
  }
  if (color != BLUE){
    turn_off_led(BLUE);
  }

  if (color == YELLOW){
    PORTG |= (1 << PORTG1);
  } else if (color == GREEN){
    PORTG |= (1 << PORTG0);
  } else if (color == RED){
    PORTL |= (1 << PORTL7);
  } else if (color == BLUE){
    PORTL |= (1 << PORTL6);
  }
}

int disabled(){
  int next_state = DISABLED;
  turn_fan_off();
  lcd.clear();
  turn_on_led(YELLOW);

  if (start_pressed){
    next_state = IDLE;
    start_pressed = false;
  }
  return next_state;
}

int idle(){
  int next_state = IDLE;
  turn_fan_off();
  turn_on_led(GREEN);
  int water_level = get_water_level();
  float temperature = get_temperature();
  if (water_level < 200){
    next_state = ERROR;
  } else if (temperature >= 23.5){
    next_state = RUNNING;
  }
  return next_state;
}

int error(){
  int next_state = ERROR;
  turn_fan_off();
  turn_on_led(RED);
  char input = get_key_press();
  if (input == '0'){
    int water_level = get_water_level();
    if (water_level >= 200){
      next_state = IDLE;
    }
  }

  return next_state;
}

int running(){
  int next_state = RUNNING;
  turn_on_led(BLUE);

  turn_fan_on();
  int temperature = get_temperature();
  float water_level = get_water_level();
  if (temperature < 23.5) {
    next_state = IDLE;
  } else if (water_level < 200){
    next_state = ERROR;
  }
  return next_state;
}

int get_water_level(){
  return adc_read(WATER_LEVEL);
}

float get_humidity(){ //Percentage
  return dht.readHumidity();
}

float get_temperature(){ //°C
  return dht.readTemperature();
}

char get_key_press(){
  return keypad.getKey();
}

void U0putint(int number, int numDigits) {
  int divisor = 1;
  for (int i = 1; i < numDigits; i++) {
    divisor *= 10;
  }

  // Print each digit
  for (int i = 0; i < numDigits; i++) {
    int digit = number / divisor;
    U0putchar('0' + digit);
    number = number % divisor;
    divisor = divisor/10;
  }
}


void print_time(){
  DateTime now = rtc.now();
  U0putint(now.year(), 4);
  U0puts("/");
  U0putint(now.month(), 2);
  U0puts("/");
  U0putint(now.day(), 2);
  U0puts(" ");
  U0putint(now.hour(), 2);
  U0puts(":");
  U0putint(now.minute(), 2);
  U0puts(":");
  U0putint(now.second(), 2);
  U0puts("\n");
}

void rotate_motor_counter_clockwise(){
  myStepper.step(-stepsPerRevolution/4);
}

void rotate_motor_clockwise(){
  myStepper.step(stepsPerRevolution/4);
}

void U0putchar(unsigned char U0pdata)
{
  while (!(*myUCSR0A & TBE)); // Wait until the Transmit Buffer Empty (TBE) bit is set
  *myUDR0 = U0pdata;
}

void U0puts(const char *str) {
  for (int i = 0; str[i] != '\0'; ++i) {
    U0putchar(str[i]);
  }
}

void loop() {
  if (state != previous_state){
    U0puts("State transition to ");
    if (state == DISABLED){
      U0puts("DISABLED");
      U0puts("\n");
    } else if (state == IDLE){
      U0puts("IDLE");
      U0puts("\n");
    } else if (state == ERROR){
      U0puts("ERROR");
      U0puts("\n");
    } else if (state == DISABLED){
      U0puts("RUNNING");
      U0puts("\n");
    }

    print_time();

    previous_state = state; //Update previous state
    if (state == ERROR){
      //display error message to LCD
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Error: Water");
      lcd.setCursor(0, 1);
      lcd.print("level too low.");
    }
    if (state == RUNNING || state == IDLE){
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.print(get_temperature());
      lcd.print("*C"); // Add unit for clarity
      lcd.setCursor(0, 1);
      lcd.print("Humidity: ");
      lcd.print(get_humidity());
      lcd.print("%");
    }
  }

  if (state != DISABLED){
    unsigned long current_time = millis();

    if (current_time - last_check_time >= 60000) {
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.print(get_temperature());
      lcd.print("*C"); // Add unit for clarity
      lcd.setCursor(0, 1);
      lcd.print("Humidity: ");
      lcd.print(get_humidity());
      lcd.print("%");
      last_check_time = current_time;
    }

    //Also manipulate vent
    char input = get_key_press();
    if (input){
      if (input == '1'){
        rotate_motor_counter_clockwise();
        U0puts("Vent turned counter-clockwise.");
        U0puts("\n");
      }
      if (input == '2'){
        rotate_motor_clockwise();
        U0puts("Vent turned clockwise.");
        U0puts("\n");
      }

      //STOP BUTTON = '*'
      if (input == '*'){
        state = DISABLED;
        turn_fan_off();
      }
    }
  }  

  if (state == DISABLED){
    state = disabled();
  } else if (state == IDLE){
    state = idle();
  } else if (state == ERROR){
    state = error();
  } else if (state == RUNNING){
    state = running();
  }
}