# seminar-arduino

## Распиновка ATMEGA328
![ATMEGA328](img/atmega328.png)


Дока на [гугл диске](https://docs.google.com/document/d/1E5u_tU_30d35mxYcNSVe8VHLDNzwZsriz4J4xWrx2XM/edit#heading=h.uh7x8owb0an)
## Лампочка и кнопка чепрез UART 
```c
#define BTN 8
#define LED 13
void setup() {
  pinMode(BTN, INPUT);
  pinMode(LED, OUTPUT);
  
  digitalWrite(LED, LOW);

  Serial.begin(9600);
}

void loop() {
  if(Serial.available()) {
    char data_rcvd = Serial.read();

    if(data_rcvd == '1') digitalWrite(LED, HIGH);
    if(data_rcvd == '0') digitalWrite(LED, LOW);
  }

  if (digitalRead(BTN) == HIGH) Serial.write('1'); 
  else Serial.write('0'); 

}

```
## Прерывание по кнопке
```c
volatile int counter = 0;  // переменная-счётчик

void setup() {
  Serial.begin(9600); // открыли порт для связи

  // подключили кнопку на D2 и GND
  pinMode(2, INPUT_PULLUP);

  // FALLING - при нажатии на кнопку будет сигнал 0, его и ловим
  attachInterrupt(0, btnIsr, FALLING);
}

void btnIsr() {
  counter++;  // + нажатие
}

void loop() {
  Serial.println(counter);  // выводим
  delay(1000);              // ждём
}
```


## АЦП
```c
#define adc0Pin A0
int val = 0;

void setup() {
    Serial.begin(9600);
    pinMode(adc0Pin, INPUT);
}

void loop() {
    val = analogRead(adc0Pin);
    Serial.println(val);
    delay(1000);
}
```

## Битовые операции
Примеры ниже в 100 раз быстрее функции digitalWrite из библиотеки ардуино 
```c
void setup() 
{ 
  DDRB |= (1<<5); // аналог pinMode(13, OUTPUT), только быстрый
} 
 
void loop() 
{  
  PORTB |= (1 << 5); // аналог digitalWrite(13, HIGH) 
  delay(1000);
  
  PORTB &= ~(1 << 5); // аналог digitalWrite(13, LOW)
  delay(1000); 
  
  //PORTB ^= (1 << 5); // аналог digitalWrite(13, !digitalRead(13)), только быстрее в 200 раз
}
```

## 7 сегментный индикатор
```c
#define LED_NUMBER_COUNT = 10
int ledNumber[] = { 
    0b00111111,
    0b00000110,
    0b01011011,
    0b01001111,
    0b01100110,
    0b01101101,
    0b01111101,
    0b00000111,
    0b01111111,
    0b01101111
  }; 
 
int i=0; 
 
void setup() 
{ 
   DDRD = 0xFF; // аналог DDRD = 0b11111111, но короче
} 
 
void loop() 
{ 
  int isDotOn = i % 2;
  PORTD = ledNumber[i] | (isDotOn << 7); 
  i = (i+1) % LED_NUMBER_COUNT; 
  delay(1000); 
}
```

## Прерывание таймера (1 сек)
[Ликбез](https://habr.com/ru/post/453276/) по таймерам 

[Datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf)
```c
#include <avr/io.h>
#include <avr/interrupt.h>
#define LEDPIN 13

void setup()
{
    pinMode(LEDPIN, OUTPUT);

    // инициализация Timer1
    cli();  // отключить глобальные прерывания
    TCCR1A = 0;   // установить регистры в 0
    TCCR1B = 0;

    OCR1A = 15624; // установка регистра совпадения

    TCCR1B |= (1 << WGM12);  // включить CTC режим 
    TCCR1B |= (1 << CS10); // Установить биты на коэффициент деления 1024
    TCCR1B |= (1 << CS12);

    TIMSK1 |= (1 << OCIE1A);  // включить прерывание по совпадению таймера 
    sei(); // включить глобальные прерывания
}

void loop()
{
    // основная программа
}

ISR(TIMER1_COMPA_vect)
{
    digitalWrite(LEDPIN, !digitalRead(LEDPIN));
}
```

## Опрос датчика каждые 10 сек
```c
#include <avr/io.h>
#include <avr/interrupt.h>
#define LEDPIN 13
#define BTN_PIN 2 // INT0

volatile byte seconds = 0;
volatile boolean canReadSensor = false;

volatile int counter = 0;

void setup()
{
    pinMode(LEDPIN, OUTPUT);
    pinMode(BTN_PIN, INPUT_PULLUP);
    attachInterrupt(0, btnIsr, FALLING);
    Serial.begin(9600);


    // инициализация Timer1
    cli();  // отключить глобальные прерывания
    TCCR1A = 0;   // установить регистры в 0
    TCCR1B = 0;

    OCR1A = 15624; // установка регистра совпадения

    TCCR1B |= (1 << WGM12);  // включить CTC режим 
    TCCR1B |= (1 << CS10); // Установить биты на коэффициент деления 1024
    TCCR1B |= (1 << CS12);

    TIMSK1 |= (1 << OCIE1A);  // включить прерывание по совпадению таймера 
    sei(); // включить глобальные прерывания
    
}

void btnIsr() {
  counter++;
}

void readSensor()
{
  digitalWrite(LEDPIN, HIGH);
  delay(1000);
  digitalWrite(LEDPIN, LOW);
}

void loop()
{
    if (canReadSensor) {
      readSensor();             // OK
    }
    
    Serial.println(counter); 
    delay(200);  
}

ISR(TIMER1_COMPA_vect)
{
    seconds++;
    if(seconds >= 10)
    {
        seconds = 0;
        canReadSensor = true;
        // readSensor();          // NO OK
    }
}
```

## Таймер и АЦП
```c
#define ledPin 7
#define adc0Pin A0
float value = 3035;                   //Preload timer value (3035 for 4 seconds)
void setup()
{
  pinMode(adc0Pin, INPUT);
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
  
  noInterrupts();  
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << CS10)|(1 << CS12);    // 1024 prescaler 
  TIMSK1 |= (1 << TOIE1);               // enable timer overflow interrupt ISR
  TCNT1 = value;                        // preload timer
  interrupts();                         // enable all interrupts

}

ISR(TIMER1_OVF_vect)
{
  TCNT1 = value;                                   // Preload timer
  digitalWrite(ledPin, !digitalRead(ledPin));      // Turns LED ON and OFF
  //digitalWrite(ledPin, digitalRead(ledPin) ^ 1); // Turns LED ON and OFF
}

void loop()
{ 
  //value = analogRead(adc0Pin);
  //Serial.println(value);
}
```


