#include <Wire.h>
#include <elapsedMillis.h>

int buzzer = 12;//the pin of the active buzzer

void setup() {
  Serial.begin(115200);
  pinMode(buzzer,OUTPUT);//initialize the buzzer pin as an output    
  while (!Serial); // STM32F767ZI: wait for serial monitor
  Serial.println("Active Buzzer");
}

void loop() {

  unsigned int i;
  elapsedMillis milliSeconds;
  
  milliSeconds = 0;

  //output an frequency
  for(i=0;i<500;i++) {
    digitalWrite(buzzer,HIGH);
    delay(1);//wait for 1ms
    digitalWrite(buzzer,LOW);
    delay(1);//wait for 1ms
  } 

  Serial.print("Change Frequency after ");
  Serial.println(milliSeconds);

  //output another frequency
  for(i=0;i<25;i++) {
    digitalWrite(buzzer,HIGH);
    delay(20);//wait for 2ms
    digitalWrite(buzzer,LOW);
    delay(20);//wait for 2ms
  } 
  Serial.print("Finished after ");
  Serial.println(milliSeconds);
}