#include <Arduino.h>
#include <ESP32Servo.h>

// 定义舵机对象和引脚
Servo myServo;
const int servoPin = 13;  // GPIO13引脚

void setup() {
  // 初始化串口监视器
  Serial.begin(115200);
  Serial.println("MG996R舵机测试开始");
  
  // 配置舵机
  myServo.attach(servoPin);
  
  // 初始化舵机到中间位置
  myServo.write(90);
  Serial.println("舵机初始化到90度位置");
  delay(1000);
}

void loop() {
  // 测试1: 缓慢从0度转到180度
  Serial.println("测试1: 0度 -> 180度");
  for (int pos = 0; pos <= 180; pos += 1) {
    myServo.write(pos);
    Serial.print("当前角度: ");
    Serial.println(pos);
    delay(20);  // 等待20ms
  }
  
  delay(1000);  // 在180度位置停留1秒
  
  // 测试2: 缓慢从180度转到0度
  Serial.println("测试2: 180度 -> 0度");
  for (int pos = 180; pos >= 0; pos -= 1) {
    myServo.write(pos);
    Serial.print("当前角度: ");
    Serial.println(pos);
    delay(20);  // 等待20ms
  }
  
  delay(1000);  // 在0度位置停留1秒
  
  // 测试3: 快速跳转到几个固定位置
  Serial.println("测试3: 快速跳转测试");
  int positions[] = {0, 45, 90, 135, 180, 90};
  int numPositions = sizeof(positions) / sizeof(positions[0]);
  
  for (int i = 0; i < numPositions; i++) {
    myServo.write(positions[i]);
    Serial.print("跳转到: ");
    Serial.print(positions[i]);
    Serial.println("度");
    delay(1000);  // 在每个位置停留1秒
  }
  
  Serial.println("一轮测试完成，2秒后开始下一轮");
  delay(2000);
}