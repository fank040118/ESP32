#include <Arduino.h>


int loopCount = 0;
const int loopCountMax = 10;
void testRGBPins();

void setup() {
  pinMode(32, OUTPUT);
  pinMode(33, OUTPUT);
}


void loop() {
  if(loopCount == 0 || loopCount ==10){
    digitalWrite(32, HIGH);
    digitalWrite(33, HIGH);
    delay(1000);
    digitalWrite(32, LOW);
    digitalWrite(33, LOW);
    delay(1000);
  }
  if(loopCount == 1 || loopCount == 4 || loopCount == 7){
    digitalWrite(32, HIGH);
    digitalWrite(33, LOW);
    delay(1000);
    digitalWrite(32, LOW);
    delay(1000);
  }
  else if(loopCount == 2 || loopCount == 5 || loopCount == 8){
    digitalWrite(32, LOW);
    digitalWrite(33, HIGH);
    delay(1000);
    digitalWrite(33, LOW);
    delay(1000);
  }
  else if(loopCount == 3 || loopCount == 6 || loopCount == 9){
    digitalWrite(32, HIGH);
    digitalWrite(33, HIGH);
    delay(1000);
    digitalWrite(32, LOW);
    digitalWrite(33, LOW);
    delay(1000);
  }

  // 如果循环次数达到最大值，则进入无限循环
  if(loopCount == loopCountMax){
    while(true){
      // 无限循环
    }
  }
  loopCount++;
}

