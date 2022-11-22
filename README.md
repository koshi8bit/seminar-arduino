# seminar-arduino

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
