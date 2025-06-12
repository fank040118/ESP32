#include <Arduino.h>

// 超声波模块引脚定义
#define TRIG_PIN 27         // 超声波模块的Trig引脚
#define ECHO_PIN 14         // 超声波模块的Echo引脚

// LED和按钮引脚定义
#define LED_PIN 22          // LED连接到GPIO22
#define BUTTON_PIN 23       // 按钮连接到GPIO23

// 测距参数
#define MAX_DISTANCE 400    // 最大测距范围(cm)
#define TIMEOUT_US 30000    // 超时时间(微秒)
#define DETECTION_THRESHOLD 10  // 检测阈值(cm)

// 按钮防抖变量
bool lastButtonState = HIGH;
bool currentButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

// 测试状态变量
bool testInProgress = false;

// 函数声明
bool readButtonWithDebounce();
float measureDistance();
void runDistanceTest();

void setup() {
  // 初始化串口监视器
  Serial.begin(115200);
  Serial.println("HC-SR04超声波测距模块测试");
  Serial.println("按下按钮开始测距...");
  
  // 配置引脚
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // 初始状态
  digitalWrite(TRIG_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  
  Serial.println("系统初始化完成");
  Serial.println("检测阈值: <= 10cm 时LED亮1秒");
}

// 按钮防抖读取函数
bool readButtonWithDebounce() {
  int reading = digitalRead(BUTTON_PIN);
  
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != currentButtonState) {
      currentButtonState = reading;
      if (currentButtonState == LOW) {
        lastButtonState = reading;
        return true;
      }
    }
  }
  
  lastButtonState = reading;
  return false;
}

// 测量距离函数
float measureDistance() {
  // 确保Trig引脚为低电平
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // 发送10微秒高电平脉冲触发测距
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // 测量Echo引脚高电平持续时间
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, TIMEOUT_US);
  
  // 如果超时，返回-1表示测距失败
  if (duration == 0) {
    return -1;
  }
  
  // 计算距离(cm)
  // 声速约340m/s，即0.034cm/μs
  // 超声波往返，所以除以2
  float distance = (duration * 0.034) / 2;
  
  return distance;
}

// 距离测试函数
void runDistanceTest() {
  if (testInProgress) return;
  
  testInProgress = true;
  Serial.println("开始测距...");
  
  // 进行3次测量取平均值以提高精度
  float totalDistance = 0;
  int validMeasurements = 0;
  
  for (int i = 0; i < 3; i++) {
    float distance = measureDistance();
    if (distance > 0 && distance <= MAX_DISTANCE) {
      totalDistance += distance;
      validMeasurements++;
    }
    delay(60); // 两次测量间隔60ms
  }
  
  if (validMeasurements > 0) {
    float avgDistance = totalDistance / validMeasurements;
    
    Serial.print("测量距离: ");
    Serial.print(avgDistance, 1);
    Serial.println(" cm");
    
    // 检查是否在阈值范围内
    if (avgDistance <= DETECTION_THRESHOLD) {
      Serial.println("检测到物体 <= 10cm，LED亮1秒");
      digitalWrite(LED_PIN, HIGH);
      delay(1000);
      digitalWrite(LED_PIN, LOW);
      Serial.println("LED已关闭");
    } else {
      Serial.println("物体距离 > 10cm，LED不亮");
    }
  } else {
    Serial.println("测距失败，未检测到有效信号");
  }
  
  Serial.println("测试完成，等待下次按键...");
  Serial.println("--------------------------------");
  testInProgress = false;
}

void loop() {
  // 检测按钮按下
  if (readButtonWithDebounce() && !testInProgress) {
    runDistanceTest();
  }
  
  delay(10);
}