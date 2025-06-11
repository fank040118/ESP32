#include <Arduino.h>
#include "BluetoothSerial.h"

// 电机1引脚定义 (Motor 1)
#define MOTOR1_IN1 32    // 电机1的IN1引脚
#define MOTOR1_IN2 33    // 电机1的IN2引脚

// 电机2引脚定义 (Motor 2)  
#define MOTOR2_IN1 25    // 电机2的IN1引脚
#define MOTOR2_IN2 26    // 电机2的IN2引脚

// 按钮引脚定义
#define BUTTON_PIN 23    // 按钮连接到GPIO23

// PWM参数设置 - 用于限制启动电流
#define PWM_FREQ 1000   // PWM频率(PWM Frequency)
#define PWM_RESOLUTION 8 // PWM分辨率(PWM Resolution)，8位，0-255

// PWM通道分配(PWM Channel Assignment)
#define PWM_CHANNEL_M1_1 0  // 电机1-IN1的PWM通道
#define PWM_CHANNEL_M1_2 1  // 电机1-IN2的PWM通道
#define PWM_CHANNEL_M2_1 2  // 电机2-IN1的PWM通道
#define PWM_CHANNEL_M2_2 3  // 电机2-IN2的PWM通道

// 创建蓝牙串口对象
BluetoothSerial SerialBT;

// 按钮防抖变量
bool lastButtonState = HIGH;
bool currentButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

// 测试状态变量
bool testRunning = false;

// 电机控制函数声明
void motorsStop();
void motorsForward();
void motorsBackward();
void motorsLeftTurn();  // 左转：电机1正转，电机2反转
void motorsRightTurn(); // 右转：电机1反转，电机2正转
void runMotorTest();
bool readButtonWithDebounce();

void setup() {
  // 初始化串口监视器
  Serial.begin(115200);

  // 初始化蓝牙
  SerialBT.begin("ESP32_Motor_Control");
  Serial.println("蓝牙已启动，设备名称：ESP32_Motor_Control");
  
  // 配置按钮引脚
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // 配置所有PWM通道 - 用于软启动减少电流冲击
  ledcSetup(PWM_CHANNEL_M1_1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_M1_2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_M2_1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_M2_2, PWM_FREQ, PWM_RESOLUTION);
  
  // 将PWM通道绑定到引脚
  ledcAttachPin(MOTOR1_IN1, PWM_CHANNEL_M1_1);
  ledcAttachPin(MOTOR1_IN2, PWM_CHANNEL_M1_2);
  ledcAttachPin(MOTOR2_IN1, PWM_CHANNEL_M2_1);
  ledcAttachPin(MOTOR2_IN2, PWM_CHANNEL_M2_2);
  
  // 初始状态：所有电机停止
  motorsStop();
  
  Serial.println("系统初始化完成");
  Serial.println("按下按钮开始电机测试...");
}

// 双电机停止函数
void motorsStop() {
  ledcWrite(PWM_CHANNEL_M1_1, 0);
  ledcWrite(PWM_CHANNEL_M1_2, 0);
  ledcWrite(PWM_CHANNEL_M2_1, 0);
  ledcWrite(PWM_CHANNEL_M2_2, 0);
}

// 双电机同步正转函数
void motorsForward() {
  // 电机1正转
  ledcWrite(PWM_CHANNEL_M1_1, 150);
  ledcWrite(PWM_CHANNEL_M1_2, 0);
  // 电机2正转
  ledcWrite(PWM_CHANNEL_M2_1, 150);
  ledcWrite(PWM_CHANNEL_M2_2, 0);
}

// 双电机同步反转函数
void motorsBackward() {
  // 电机1反转
  ledcWrite(PWM_CHANNEL_M1_1, 0);
  ledcWrite(PWM_CHANNEL_M1_2, 150);
  // 电机2反转
  ledcWrite(PWM_CHANNEL_M2_1, 0);
  ledcWrite(PWM_CHANNEL_M2_2, 150);
}

// 左转函数：电机1正转，电机2反转
void motorsLeftTurn() {
  // 电机1正转
  ledcWrite(PWM_CHANNEL_M1_1, 150);
  ledcWrite(PWM_CHANNEL_M1_2, 0);
  // 电机2反转
  ledcWrite(PWM_CHANNEL_M2_1, 0);
  ledcWrite(PWM_CHANNEL_M2_2, 150);
}

// 右转函数：电机1反转，电机2正转
void motorsRightTurn() {
  // 电机1反转
  ledcWrite(PWM_CHANNEL_M1_1, 0);
  ledcWrite(PWM_CHANNEL_M1_2, 150);
  // 电机2正转
  ledcWrite(PWM_CHANNEL_M2_1, 150);
  ledcWrite(PWM_CHANNEL_M2_2, 0);
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
      if (currentButtonState == LOW) {  // 按钮被按下（下降沿）
        lastButtonState = reading;
        return true;
      }
    }
  }
  
  lastButtonState = reading;
  return false;
}

// 电机测试函数
void runMotorTest() {
  Serial.println("开始电机测试...");
  testRunning = true;
  
  // 测试1：双电机同步正转
  Serial.println("测试1：双电机同步正转");
  motorsForward();
  delay(2000);  // 运行2秒
  motorsStop();
  delay(500);   // 停止0.5秒
  
  // 测试2：双电机同步反转
  Serial.println("测试2：双电机同步反转");
  motorsBackward();
  delay(2000);  // 运行2秒
  motorsStop();
  delay(500);   // 停止0.5秒
  
  // 测试3：左转（电机1正转，电机2反转）
  Serial.println("测试3：左转");
  motorsLeftTurn();
  delay(2000);  // 运行2秒
  motorsStop();
  delay(500);   // 停止0.5秒
  
  // 测试4：右转（电机1反转，电机2正转）
  Serial.println("测试4：右转");
  motorsRightTurn();
  delay(2000);  // 运行2秒
  motorsStop();
  delay(500);   // 停止0.5秒
  
  Serial.println("电机测试完成！按下按钮开始下一轮测试...");
  testRunning = false;
}

void loop() {
  // 检测按钮按下
  if (readButtonWithDebounce() && !testRunning) {
    runMotorTest();
  }
  
  delay(10);  // 小延迟以稳定程序执行
}



