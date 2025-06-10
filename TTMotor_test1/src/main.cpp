#include <Arduino.h>

// MX1508电机驱动器引脚定义
// 连接到电机A (Motor A)
#define MOTOR_IN1 32    // MX1508的IN1引脚
#define MOTOR_IN2 33    // MX1508的IN2引脚

// PWM参数设置 - 用于限制启动电流
#define PWM_FREQ 1000   // PWM频率
#define PWM_RESOLUTION 8 // PWM分辨率(8位，0-255)
#define PWM_CHANNEL_1 0  // PWM通道1
#define PWM_CHANNEL_2 1  // PWM通道2

void setup() {
  // 初始化串口监视器
  Serial.begin(115200);
  Serial.println("TT电机测试开始!");
  
  // 配置PWM通道 - 用于软启动减少电流冲击
  ledcSetup(PWM_CHANNEL_1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_2, PWM_FREQ, PWM_RESOLUTION);
  
  // 将PWM通道绑定到引脚
  ledcAttachPin(MOTOR_IN1, PWM_CHANNEL_1);
  ledcAttachPin(MOTOR_IN2, PWM_CHANNEL_2);
  
  // 初始状态：电机停止
  ledcWrite(PWM_CHANNEL_1, 0);
  ledcWrite(PWM_CHANNEL_2, 0);
  
  Serial.println("电机已初始化(带软启动保护)，准备开始测试...");
  delay(2000);
}

// 软启动电机正转函数 - 避免大电流冲击
void motorForward() {
  Serial.println("  -> 软启动正转");
  // 渐进启动，从30%功率开始，避免大电流
  for(int speed = 80; speed <= 200; speed += 20) {
    ledcWrite(PWM_CHANNEL_1, speed);
    ledcWrite(PWM_CHANNEL_2, 0);
    delay(100);  // 每步延时100ms
  }
}

// 软启动电机反转函数
void motorBackward() {
  Serial.println("  -> 软启动反转");
  // 渐进启动，从30%功率开始
  for(int speed = 80; speed <= 200; speed += 20) {
    ledcWrite(PWM_CHANNEL_1, 0);
    ledcWrite(PWM_CHANNEL_2, speed);
    delay(100);
  }
}

// 电机停止函数
void motorStop() {
  ledcWrite(PWM_CHANNEL_1, 0);
  ledcWrite(PWM_CHANNEL_2, 0);
}

// 快速模式(较低功率，适合无负载测试)
void motorForwardQuick() {
  ledcWrite(PWM_CHANNEL_1, 150);  // 约60%功率
  ledcWrite(PWM_CHANNEL_2, 0);
}

void loop() {
  // 测试1：正转
  Serial.println("测试1: 电机正转");
  motorForward();
  delay(3000);  // 正转3秒
  
  // 停止
  Serial.println("停止");
  motorStop();
  delay(1000);  // 停止1秒
  
  // 测试2：反转
  Serial.println("测试2: 电机反转");
  motorBackward();
  delay(3000);  // 反转3秒
  
  // 停止
  Serial.println("停止");
  motorStop();
  delay(1000);  // 停止1秒
  
  // 测试3：快速启停测试(使用较低功率)
  Serial.println("测试3: 快速启停测试(低功率模式)");
  for(int i = 0; i < 5; i++) {
    Serial.print("快速正转 ");
    Serial.println(i+1);
    motorForwardQuick();  // 使用快速低功率模式
    delay(500);
    motorStop();
    delay(500);
  }
  
  Serial.println("一轮测试完成，等待5秒后重复...");
  delay(5000);
}
